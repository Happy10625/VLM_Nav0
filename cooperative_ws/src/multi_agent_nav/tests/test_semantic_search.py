import json
import pytest

from multi_agent_nav.semantic_search import parse_selection, SelectionState, load_catalog, SemanticFurniture


@pytest.mark.parametrize('payload', [
    {'action': 'inspect_furniture', 'furniture_id': 'unknown', 'reason': 'x'},
    {'action': 'inspect_furniture', 'furniture_id': 'table_01', 'reason': 'x', 'x': 1},
    {'action': 'navigate', 'furniture_id': 'table_01', 'reason': 'x'},
    {'action': 'inspect_furniture', 'furniture_id': ['table_01'], 'reason': 'x'},
    {}, 'not json',
])
def test_rejects_untrusted_selection(payload):
    with pytest.raises(ValueError):
        parse_selection(json.dumps(payload), ['table_01'])


def test_selection_state_invalidates_old_or_cancelled_requests():
    state = SelectionState([{'id': 'p1'}, {'id': 'p2'}])
    token = state.snapshot(['p1'])
    state.update('p1', 'observed', 0)
    assert not state.accepts(token, 'p1', 0)
    token = state.snapshot(['p2'])
    state.cancel()
    assert not state.accepts(token, 'p2', 0)


def test_completed_points_excluded_and_failure_cools_down():
    state = SelectionState([{'id': 'p1'}, {'id': 'p2'}])
    state.update('p1', 'observed', 0)
    state.update('p2', 'navigation_failed', 0)
    assert state.eligible(1) == []
    assert state.eligible(31) == ['p2']
    assert state.records['p1']['status'] == 'observed'


@pytest.fixture
def catalog_config(tmp_path):
    import hashlib
    import yaml
    from multi_agent_nav.config import map_file_info
    (tmp_path/'map.yaml').write_text('image: map.pgm\n')
    (tmp_path/'map.pgm').write_bytes(b'fixture occupancy')
    info = map_file_info(tmp_path/'map.yaml')
    semantic = dict(schema_version=1, source_robot='go2', frame_id='map', units='meters',
                    source_map=dict(yaml=info['yaml_path'], image=info['image_path'],
                                    yaml_sha256=info['yaml_sha256'], image_sha256=info['image_sha256']),
                    furniture=[dict(id='t1', category='table'), dict(id='t2', category='table')])
    (tmp_path/'semantic.yaml').write_text(yaml.safe_dump(semantic))
    (tmp_path/'annotated.png').write_bytes(b'fixture annotated image')
    catalog = dict(schema_version=1, robot_id='go2', frame_id='map', map_id='fixture',
                   map_version='sha256:'+info['map_digest'], semantic_yaml='semantic.yaml',
                   annotated_image='annotated.png',
                   semantic_yaml_sha256=hashlib.sha256((tmp_path/'semantic.yaml').read_bytes()).hexdigest(),
                   annotated_image_sha256=hashlib.sha256((tmp_path/'annotated.png').read_bytes()).hexdigest(),
                   viewpoints=[dict(viewpoint_id='p'+str(i), furniture_id='t'+str(i), enabled=True,
                                    pose=dict(x=float(i), y=0., yaw=0.)) for i in (1,2)])
    path = tmp_path/'points.yaml'
    path.write_text(yaml.safe_dump(catalog))
    return dict(alignment=dict(robots=dict(go2=dict(local_map_id='fixture', map_version=catalog['map_version']))),
                scene=dict(resources={}, targets=dict(A='target', B=None, C=None)),
                robot_go2=dict(exploration_strategy='semantic_furniture', map_path=str(tmp_path/'map.yaml'),
                               map_digest=info['map_digest'], stop_margin=.1, scan_angles=[0.],
                               semantic_search=dict(observation_points=str(path))))


@pytest.mark.parametrize('fault', ['unknown', 'missing', 'map', 'image', 'duplicate', 'disabled'])
def test_catalog_trust_boundaries(catalog_config, fault):
    from pathlib import Path
    import yaml
    config = catalog_config
    path = Path(config['robot_go2']['semantic_search']['observation_points'])
    catalog = yaml.safe_load(path.read_text())
    if fault == 'unknown':catalog['viewpoints'][0]['furniture_id'] = 'absent'
    if fault == 'missing':catalog['viewpoints'][0]['pose']['x'] = None
    if fault == 'map':catalog['map_id'] = 'another_map'
    if fault == 'image':(path.parent/'annotated.png').write_bytes(b'changed')
    if fault == 'duplicate':catalog['viewpoints'][1]['viewpoint_id'] = 'p1'
    if fault == 'disabled':
        catalog['viewpoints'][0].update(enabled=False, pose=None)
    path.write_text(yaml.safe_dump(catalog))
    if fault == 'disabled':
        assert [p['id'] for p in load_catalog(config)['points']] == ['p2']
    else:
        with pytest.raises(ValueError):load_catalog(config)


def test_program_selects_cheapest_viewpoint_for_selected_furniture():
    from types import SimpleNamespace
    client = SimpleNamespace(choose_furniture=lambda snapshot: dict(
        action='inspect_furniture', furniture_id='t1', reason='unobserved'))
    snapshot = dict(candidates=[dict(id='far', furniture_id='t1', path_length=8.),
                                dict(id='near', furniture_id='t1', path_length=2.)])
    assert SemanticFurniture(client).next_viewpoint(snapshot)['id'] == 'near'


def run_semantic(config, mode='normal'):
    from types import SimpleNamespace
    from concurrent.futures import Future
    from test_go2_exploration import LocalFixture
    from multi_agent_nav.adapters.fake import completed
    from multi_agent_nav.models import Observation
    from multi_agent_nav.frame_adapter import FrameAdapter
    from multi_agent_nav.go2_exploration import explore
    adapter = LocalFixture()
    events = []
    now = [0.]
    requests = []
    def choose(snapshot):
        requests.append(snapshot)
        return dict(action='inspect_furniture', furniture_id=snapshot['candidates'][-1]['furniture_id'], reason='fixture')
    adapter.client = SimpleNamespace(choose_furniture=choose)
    pending = Future()
    def submit(fn):
        if mode in ('timeout', 'cancel', 'late', 'map_change'):
            if mode != 'timeout':pending.work = fn
            return pending
        try:return completed(fn())
        except Exception as error:
            result=Future();result.set_exception(error);return result
    adapter.vision = SimpleNamespace(submit=submit)
    if mode == 'invalid':adapter.client.choose_furniture=lambda _:dict(action='inspect_furniture',furniture_id='fake',reason='x')
    if mode == 'unreachable':adapter.no_path = {1.,2.}
    if mode == 'api':adapter.api_error = True
    if mode == 'scan_exception':
        def failed_scan(*args):
            adapter.scans += 1
            future=Future();future.set_exception(RuntimeError('fixture scan error'));return future
        adapter.start_scan = failed_scan
    def spin():
        now[0] += .05
        adapter.advance(now[0])
        if mode in ('cancel', 'late', 'map_change') and hasattr(pending,'work') and not pending.done():
            if mode == 'cancel':adapter.selection_epoch = 1
            if mode == 'late':now[0] = 2.
            if mode == 'map_change':
                from pathlib import Path
                import hashlib, yaml
                path=Path(config['robot_go2']['semantic_search']['observation_points'])
                catalog=yaml.safe_load(path.read_text())
                image=path.parent/catalog['annotated_image'];image.write_bytes(b'new image')
                catalog['annotated_image_sha256']=hashlib.sha256(image.read_bytes()).hexdigest()
                path.write_text(yaml.safe_dump(catalog))
            pending.set_result(pending.work())
    def stop():
        adapter.stop_motion('fixture stop');adapter.advance(now[0]+1);return True
    # A nonidentity frame proves catalog poses stay in the native map frame.
    frame = FrameAdapter('fixture', '1', 'lab_ref', '', [100.,100.,0.])
    try:
        outcome = explore(adapter, spin, config, frame, 10., 'target', events.append,
                          lambda: [], stop, lambda: None, clock=lambda:now[0])
        return adapter, events, requests, outcome
    except Exception:
        assert not adapter.calls
        raise


def test_fake_semantic_selection_observes_each_point_once(catalog_config):
    adapter, events, requests, outcome = run_semantic(catalog_config)
    assert outcome == 'EXHAUSTED'
    assert [c[1].x for c in adapter.calls] == [2.,1.]
    assert adapter.scans == 2 and len(requests) == 2
    records = [e for e in events if e['state']=='OBSERVATION_RECORD']
    assert all(row['status']=='observed' for row in records[-1]['records'].values())
    assert not any(e['state']=='READY' for e in events)
    assert requests[1]['records']['p2']['status'] == 'observed'


@pytest.mark.parametrize('mode', ['timeout', 'invalid', 'cancel', 'late', 'map_change'])
def test_failed_or_late_model_response_never_starts_navigation(catalog_config, mode):
    catalog_config['robot_go2']['api_timeout'] = 1.
    with pytest.raises((ValueError, RuntimeError)):
        run_semantic(catalog_config, mode)


def test_no_path_skips_model_and_waits(catalog_config):
    adapter, events, requests, outcome = run_semantic(catalog_config, 'unreachable')
    assert outcome == 'WAITING_VIEWPOINTS' and not requests and not adapter.calls


@pytest.mark.parametrize('mode', ['api', 'scan_exception'])
def test_api_failure_is_not_a_negative_observation(catalog_config, mode):
    adapter, events, requests, outcome = run_semantic(catalog_config, mode)
    assert outcome == 'WAITING_VIEWPOINTS' and adapter.scans == 2
    assert all(row['status']=='observation_failed' for row in events[-1]['records'].values())


def test_furniture_api_uses_semantic_image_and_separate_request(monkeypatch, tmp_path):
    from types import SimpleNamespace as S
    from multi_agent_nav.adapters.go2 import ABCClient
    from vlm_nav.vlm_client import OpenAICompatibleVLMClient
    image = tmp_path/'map.png';image.write_bytes(b'png')
    client = ABCClient.__new__(ABCClient)
    client.client = S(api_key='secret-key')
    client.evidence = S(request_started=lambda *args:pytest.fail('camera evidence polluted'))
    captured = []
    def completion(self, messages, max_tokens):
        captured.extend(messages)
        return S(choices=[S(message=S(content=json.dumps(dict(action='inspect_furniture',
                   furniture_id='t1', reason='secret-key unseen viewpoint'))))])
    monkeypatch.setattr(OpenAICompatibleVLMClient, '_completion', completion)
    reply = client.choose_furniture(dict(image=str(image), main_target='target',
                targets=dict(A='a',B='b',C='c'), candidates=[dict(furniture_id='t1')], records={}))
    assert 'secret-key' not in reply['reason']
    assert captured[1]['content'][1]['image_url']['url'] == 'data:image/png;base64,cG5n'
    assert json.loads(captured[1]['content'][0]['text'])['targets']['C'] == 'c'


def test_offline_reports_catalog_without_ros_or_model(monkeypatch, capsys, catalog_config):
    from multi_agent_nav import go2_check
    monkeypatch.setattr(go2_check, 'load_config', lambda *_:catalog_config)
    monkeypatch.setattr(go2_check, 'go2_local_gates', lambda *_:[])
    monkeypatch.setattr(go2_check, 'live_gates', lambda *_:[])
    import multi_agent_nav.go2_exploration as exploration
    monkeypatch.setattr(exploration, 'exploration_gates', lambda *_:[])
    monkeypatch.setattr(go2_check, 'run_ros', lambda *_:pytest.fail('offline initialized ROS'))
    assert go2_check.main(['--offline', '--strategy', 'semantic_furniture']) == 0
    report=json.loads(capsys.readouterr().out)
    assert report['semantic_search']['points'][0]['furniture_id']=='t1'
    assert report['motion_sent'] is False


def test_duplicate_json_fields_rejected():
    with pytest.raises(ValueError):
        parse_selection('{"action":"inspect_furniture","furniture_id":"t1",'
                        '"furniture_id":"t2","reason":"x"}', ['t1','t2'])


def test_semantic_entry_does_not_require_old_fixed_points(monkeypatch, tmp_path, catalog_config):
    from contextlib import nullcontext
    from multi_agent_nav import go2_check, go2_exploration
    monkeypatch.setattr(go2_check, 'load_config', lambda *_:catalog_config)
    monkeypatch.setattr(go2_check, 'require_ros_environment', lambda:None)
    monkeypatch.setattr(go2_check, 'motion_slot', nullcontext)
    monkeypatch.setattr(go2_exploration, 'exploration_gates', lambda *_:[])
    monkeypatch.setattr(go2_check, 'run_ros', lambda *args:0)
    assert go2_check.main(['--action','explore','--allow-motion','--strategy','semantic_furniture',
                          '--output-dir',str(tmp_path/'evidence')]) == 0
