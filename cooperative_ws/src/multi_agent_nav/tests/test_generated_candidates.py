"""Generated geometry, image contract and the existing serial executor, without ROS."""
import json
import math
from pathlib import Path
from types import SimpleNamespace as S

import numpy as np
from PIL import Image
import pytest
import yaml


@pytest.fixture
def generated_config(tmp_path):
    from multi_agent_nav.config import map_file_info
    Image.new('L', (100, 100), 254).save(tmp_path/'map.pgm')
    source = dict(image='map.pgm', resolution=.1, origin=[-5., -5., 0.],
                  negate=0, occupied_thresh=.65, free_thresh=.25, mode='trinary')
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(source))
    info = map_file_info(tmp_path/'map.yaml')
    semantic = dict(schema_version=1, source_robot='go2', frame_id='map', units='meters',
                    source_map=dict(yaml=info['yaml_path'], image=info['image_path'],
                                    yaml_sha256=info['yaml_sha256'], image_sha256=info['image_sha256']),
                    furniture=[dict(id='table', name='desk', category='table',
                                    polygon=[[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]])])
    (tmp_path/'semantic.yaml').write_text(yaml.safe_dump(semantic))
    return dict(alignment=dict(robots=dict(go2=dict(local_map_id='fixture', map_version='sha256:'+info['map_digest']))),
                scene=dict(resources={}, targets=dict(A='target', B='box', C='basket')),
                robot_go2=dict(exploration_strategy='generated_candidates_vlm',
                               map_path=str(tmp_path/'map.yaml'), map_digest=info['map_digest'],
                               footprint=[[.38,.18],[.38,-.18],[-.38,-.18],[-.38,.18]],
                               footprint_padding=0., stop_margin=.1, scan_angles=[0.], api_timeout=1.,
                               generated_candidates=dict(semantic_yaml=str(tmp_path/'semantic.yaml'), observation_distance=1.)))


def test_each_edge_faces_furniture_for_both_windings(generated_config):
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    for reversed_order in (False, True):
        path=Path(generated_config['robot_go2']['generated_candidates']['semantic_yaml'])
        if reversed_order:
            doc=yaml.safe_load(path.read_text());doc['furniture'][0]['polygon'].reverse()
            path.write_text(yaml.safe_dump(doc))
        catalog=GeneratedCandidates(generated_config)
        furniture=[p for p in catalog.points if p['furniture_id']]
        positions={tuple(p['pose'][:2]) for p in furniture}
        assert positions=={(0.,-2.),(2.,0.),(0.,2.),(-2.,0.)}
        for point in furniture:
            x,y,yaw=point['pose']
            assert x*math.cos(yaw)+y*math.sin(yaw)<-1.99
        assert len({p['id'] for p in furniture})==4


def test_static_filter_rejects_unknown_occupied_bounds_and_scan_corner(generated_config):
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    image=np.full((100,100),254,dtype=np.uint8)
    image[69,50]=128  # unknown at bottom candidate with this map's .25/.65 thresholds
    image[49,70]=0    # occupied at right candidate
    image[29,54]=0    # intersects rotation envelope of top candidate, beyond narrow parked body
    Image.fromarray(image).save(Path(generated_config['robot_go2']['map_path']).parent/'map.pgm')
    from multi_agent_nav.config import map_file_info
    info=map_file_info(generated_config['robot_go2']['map_path'])
    generated_config['robot_go2']['map_digest']=info['map_digest']
    generated_config['alignment']['robots']['go2']['map_version']='sha256:'+info['map_digest']
    path=Path(generated_config['robot_go2']['generated_candidates']['semantic_yaml'])
    doc=yaml.safe_load(path.read_text())
    doc['source_map'].update(yaml_sha256=info['yaml_sha256'],image_sha256=info['image_sha256'])
    path.write_text(yaml.safe_dump(doc))
    catalog=GeneratedCandidates(generated_config)
    assert [p['pose'][:2] for p in catalog.points if p['furniture_id']]==[[-2.,0.]]
    generated_config['robot_go2']['generated_candidates']['observation_distance']=10.
    assert not any(p['furniture_id'] for p in GeneratedCandidates(generated_config).points)


@pytest.mark.parametrize('fault',['distance','polygon','digest'])
def test_generated_input_validation(generated_config,fault):
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    options=generated_config['robot_go2']['generated_candidates']
    if fault=='distance':options['observation_distance']=float('nan')
    else:
        path=Path(options['semantic_yaml']);doc=yaml.safe_load(path.read_text())
        if fault=='polygon':doc['furniture'][0]['polygon']=[[0,0],[1,1],[0,1],[1,0]]
        else:doc['source_map']['image_sha256']='wrong'
        path.write_text(yaml.safe_dump(doc))
    with pytest.raises(ValueError):GeneratedCandidates(generated_config)


@pytest.mark.parametrize('origin_yaw,first_xy',[(0.,[-4.45,-4.45]),(.4,[-4.707596541,-4.279236365])])
def test_unannotated_map_generates_free_rotation_candidates(generated_config,origin_yaw,first_xy):
    from multi_agent_nav.generated_candidates import GeneratedCandidates,circle_free
    from multi_agent_nav.config import map_file_info
    robot=generated_config['robot_go2'];robot['generated_candidates']['semantic_yaml']=''
    path=Path(robot['map_path']);source=yaml.safe_load(path.read_text());source['origin'][2]=origin_yaw
    path.write_text(yaml.safe_dump(source));info=map_file_info(path)
    robot['map_digest']=info['map_digest']
    generated_config['alignment']['robots']['go2']['map_version']='sha256:'+info['map_digest']
    catalog=GeneratedCandidates(generated_config)
    assert catalog.points and catalog.furniture==[] and catalog.semantic_path is None
    assert catalog.points[0]['pose'][:2]==pytest.approx(first_xy,abs=1e-8)
    assert len({p['id'] for p in catalog.points})==len(catalog.points)
    for point in catalog.points:
        x,y=catalog.map_xy(*point['pose'][:2])
        assert circle_free(catalog.raw,x,y,catalog.radius,.1,lambda cells:cells>=253)
    assert catalog.unchanged()
    Path(info['image_path']).write_bytes(b'changed')
    assert not catalog.unchanged()


def test_missing_optional_annotation_uses_free_candidates(generated_config,tmp_path):
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    generated_config['robot_go2']['generated_candidates']['semantic_yaml']=str(tmp_path/'missing.yaml')
    catalog=GeneratedCandidates(generated_config)
    assert catalog.points and not catalog.furniture and catalog.unchanged()


def test_dual_images_have_identical_marks_and_do_not_write_map(generated_config,tmp_path):
    from multi_agent_nav.generated_candidates import GeneratedCandidates, render_selection
    from multi_agent_nav.models import Pose2D
    catalog=GeneratedCandidates(generated_config)
    original=Path(catalog.source['image']).read_bytes()
    point=dict(catalog.points[0], display_id='P1')
    images=render_selection(catalog,[point],{catalog.points[1]['id']:dict(status='observed')},
                            Pose2D(-3.,-3.,math.pi/2),tmp_path/'render')
    arrays=[np.array(Image.open(path).convert('RGB')) for path in images]
    assert arrays[0].shape==arrays[1].shape
    for color in ((30,100,230),(240,140,20),(125,125,125)):
        masks=[np.all(a==color,axis=-1) for a in arrays]
        assert masks[0].any() and np.array_equal(*masks)
    for a in arrays:
        assert tuple(a[209,150])==(240,140,20)  # bottom edge candidate (0,-2)
        assert tuple(a[149,210])==(125,125,125)  # completed right edge (2,0)
        assert tuple(a[239,60])==(30,100,230)  # robot (-3,-3)
    assert Path(catalog.source['image']).read_bytes()==original


def run_generated(config,tmp_path,mode='normal',*,robot_id='go2',target_id='A'):
    from test_go2_exploration import LocalFixture
    from multi_agent_nav.adapters.fake import completed
    from multi_agent_nav.models import Pose2D, Observation, ApproachResult
    from multi_agent_nav.frame_adapter import FrameAdapter
    from multi_agent_nav.go2_exploration import explore
    from concurrent.futures import Future
    adapter=LocalFixture();events=[];requests=[];now=[0.]
    adapter.selection_evidence_dir=tmp_path/'requests'
    adapter.observation_candidate_free=lambda goal: mode!='blocked' and not (mode=='arrived_blocked' and adapter.calls)
    def choose(snapshot):
        requests.append(snapshot)
        return dict(candidate_id='unknown' if mode=='invalid' else 'P1',reason='unobserved side')
    adapter.client=S(choose_candidate=choose)
    pending=Future()
    def submit(fn):
        if mode in ('cancel','late'):
            pending.work=fn;return pending
        return completed(fn())
    adapter.vision=S(submit=submit)
    def scan(descriptions,scan_id,*,stop_on_target=None):
        adapter.scans+=1
        return completed([Observation(target_id=t,view_index=0,
                           status='API_ERROR' if mode=='api' or mode=='partial_detection' and t!=target_id else
                               ('DETECTED' if mode in ('found','partial_detection') and t==target_id else 'NOT_DETECTED'),
                           position_valid=mode in ('found','partial_detection') and t==target_id,
                           common_position=Pose2D(5.,2.)) for t in descriptions])
    adapter.start_scan=scan
    adapter.select_approach=lambda *args,**kwargs:completed(ApproachResult(Pose2D(4.3,2.)))
    def spin():
        now[0]+=.05;adapter.advance(now[0])
        if hasattr(pending,'work') and not pending.done():
            if mode=='cancel':adapter.selection_epoch=1
            if mode=='late':now[0]=2.
            pending.set_result(pending.work())
    def stop():adapter.stop_motion('stop');adapter.advance(now[0]);return True
    try:
        outcome=explore(adapter,spin,config,FrameAdapter('fixture','1','lab_ref' if robot_id=='go2' else 'ranger_local_map','',[0,0,0]),
                        600.,'target',events.append,lambda:[],stop,lambda:None,clock=lambda:now[0],robot_id=robot_id,target_id=target_id)
    except Exception:
        if mode in ('invalid','late','cancel'):assert not adapter.calls
        raise
    return adapter,events,requests,outcome


def test_fake_select_observe_next_and_current_id_mapping(generated_config,tmp_path):
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path)
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    points=GeneratedCandidates(generated_config).points
    assert outcome=='EXHAUSTED' and adapter.scans==len(points) and len(requests)==len(points)
    assert {(c[1].x,c[1].y) for c in adapter.calls}=={tuple(p['pose'][:2]) for p in points}
    assert len({r['decision_id'] for r in requests})==len(points)
    origin=(0.,0.)
    for request in requests:
        order=[(math.hypot(p['pose'][0]-origin[0],p['pose'][1]-origin[1]),p['id']) for p in request['candidates']]
        assert order==sorted(order)
        origin=request['candidates'][0]['pose'][:2]
    assert requests[0]['task_id']==next(e['task_id'] for e in events if e['state']=='NAVIGATING')
    assert requests[1]['candidates'][0]['display_id']=='P1'
    assert requests[1]['candidates'][0]['id']!=requests[0]['candidates'][0]['id']
    assert requests[1]['history'][0]['candidate_id']=='P1'
    assert requests[1]['history'][0]['result']=='NOT_DETECTED'
    assert not any(e['state']=='READY' for e in events)


@pytest.mark.parametrize('mode',['invalid','late','cancel'])
def test_bad_decision_cannot_move(generated_config,tmp_path,mode):
    with pytest.raises((ValueError,RuntimeError)):
        run_generated(generated_config,tmp_path,mode)


def test_no_executable_candidate_never_calls_model(generated_config,tmp_path):
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path,'blocked')
    assert outcome=='WAITING_VIEWPOINTS' and not requests and not adapter.calls


def test_scan_rechecks_real_stopped_pose_before_rotating(generated_config,tmp_path):
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path,'arrived_blocked')
    assert outcome=='WAITING_VIEWPOINTS' and len(adapter.calls)==1 and adapter.scans==0
    assert not any(e.get('observation_status')=='observed' for e in events)


def test_image_api_failure_is_not_observed(generated_config,tmp_path):
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path,'api')
    assert outcome=='WAITING_VIEWPOINTS'
    assert all(row['status']=='observation_failed' for row in events[-1]['records'].values())


def test_generated_detection_uses_original_approach(generated_config,tmp_path):
    generated_config['robot_go2'].update(approach_on_detection=True,standoff_distance=.9,
        arrival_config_files=[str(Path(__file__).resolve().parents[4]/'config/robot.yaml')])
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path,'found')
    assert outcome=='TARGET_APPROACHED' and len(requests)==1
    assert [e['goal_kind'] for e in events if e['state']=='NAVIGATING']==['VIEWPOINT_NAV','PICKUP_NAV']


def test_partial_api_error_keeps_valid_target_without_completing_observation(generated_config,tmp_path):
    generated_config['robot_go2'].update(approach_on_detection=True,standoff_distance=.9,
        arrival_config_files=[str(Path(__file__).resolve().parents[4]/'config/robot.yaml')])
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path,'partial_detection')
    assert outcome=='TARGET_APPROACHED' and len(requests)==1
    rows=[e for e in events if e['state']=='OBSERVATION_RECORD']
    assert rows[-1]['observation_status']=='observation_failed'
    assert not any(e.get('observation_status')=='observed' for e in rows)


@pytest.mark.parametrize('annotated',[True,False])
def test_candidate_api_sends_only_two_images_ids_and_history(monkeypatch,tmp_path,annotated):
    from multi_agent_nav.adapters.go2 import ABCClient
    from vlm_nav.vlm_client import OpenAICompatibleVLMClient
    client=ABCClient.__new__(ABCClient);client.client=S(api_key='secret')
    images=[tmp_path/'occupancy.png',tmp_path/'semantic.png']
    for path in images:path.write_bytes(b'png')
    captured=[]
    def completion(self,messages,max_tokens):
        captured.extend(messages)
        return S(choices=[S(message=S(content='{"candidate_id":"P1","reason":"side"}'))])
    monkeypatch.setattr(OpenAICompatibleVLMClient,'_completion',completion)
    reply=client.choose_candidate(dict(images=list(map(str,images)),task_id='task',decision_id='decision',furniture_annotated=annotated,
        main_target='target',candidates=[dict(display_id='P1',id='internal',pose=[1.,2.,3.])],
        furniture=[dict(id='table',polygon=[[1.,2.],[3.,4.]])],history=[],records={'internal':{'pose':[1,2,3]}}))
    assert reply['candidate_id']=='P1'
    assert len([c for c in captured[1]['content'] if c['type']=='image_url'])==2
    text=captured[1]['content'][0]['text']
    assert 'internal' not in text and 'pose' not in text and 'polygon' not in text
    assert json.loads(text)['allowed_candidate_ids']==['P1']
    assert json.loads(text)['furniture_annotated'] is annotated


@pytest.mark.parametrize('raw',[
    '{"candidate_id":"P13","reason":"x"}',
    '{"candidate_id":"P1","reason":"x","x":1}',
    '{"candidate_id":"P1","candidate_id":"P2","reason":"x"}',
    '{"candidate_id":["P1"],"reason":"x"}', 'null', 'bad',
])
def test_candidate_parser_rejects_extra_fields_duplicate_and_foreign_id(raw):
    from multi_agent_nav.generated_candidates import parse_candidate_selection
    with pytest.raises(ValueError):parse_candidate_selection(raw,['P1','P2'])


@pytest.mark.parametrize('cost',[253,254,255])
def test_global_rotation_space_rejects_side_obstacle_and_stale_grid(cost):
    from test_field_preparation import adapter,costmap
    from multi_agent_nav.models import Pose2D
    a=adapter();a.on_costmap(costmap())
    grid=costmap();grid.header.frame_id='map'
    a.on_costmap(grid,global_goal=True)
    assert a.observation_candidate_free(Pose2D(3.,0.))  # outside local coverage is irrelevant
    grid.data[43*80+70]=cost  # y=.35, inside scan disk but outside parked body y=.18
    assert a.costmap_candidate_free(Pose2D(3.,0.),global_goal=True)
    assert not a.observation_candidate_free(Pose2D(3.,0.))
    grid.data[43*80+70]=0
    assert not a.observation_candidate_free(Pose2D(3.8,0.))
    a.node.get_clock=lambda:S(now=lambda:S(nanoseconds=12_000_000_000))
    assert not a.observation_candidate_free(Pose2D(3.,0.))


def test_repeated_geometry_disabled_furniture_and_map_change(generated_config):
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    options=generated_config['robot_go2']['generated_candidates']
    path=Path(options['semantic_yaml']);doc=yaml.safe_load(path.read_text())
    doc['furniture'].append(dict(doc['furniture'][0],id='duplicate_geometry'))
    doc['furniture'].append(dict(doc['furniture'][0],id='disabled',enabled=False))
    path.write_text(yaml.safe_dump(doc))
    catalog=GeneratedCandidates(generated_config)
    assert len([p for p in catalog.points if p['furniture_id']])==4 and catalog.unchanged()
    assert len({p['id'] for p in catalog.points})==len(catalog.points)
    path.write_text(path.read_text()+'\n')
    assert not catalog.unchanged()


def test_cap_twelve_and_no_reselect_of_observed(generated_config,tmp_path):
    options=generated_config['robot_go2']['generated_candidates']
    path=Path(options['semantic_yaml']);doc=yaml.safe_load(path.read_text())
    doc['furniture']=[]
    for i,(x,y) in enumerate([(-2,-2),(2,-2),(-2,2),(2,2)]):
        doc['furniture'].append(dict(id=f't{i}',category='table',
            polygon=[[x-.4,y-.4],[x+.4,y-.4],[x+.4,y+.4],[x-.4,y+.4]]))
    path.write_text(yaml.safe_dump(doc))
    adapter,events,requests,outcome=run_generated(generated_config,tmp_path)
    from multi_agent_nav.generated_candidates import GeneratedCandidates
    assert outcome=='EXHAUSTED' and adapter.scans==len(GeneratedCandidates(generated_config).points)
    assert len(requests[0]['candidates'])==12
    seen=set()
    for request in requests:
        assert len(request['candidates'])<=12
        assert not seen.intersection(p['id'] for p in request['candidates'])
        seen.add(request['candidates'][0]['id'])


def test_generated_strategy_requires_real_abc_descriptions(generated_config):
    from multi_agent_nav.go2_exploration import exploration_gates
    generated_config['scene']['targets']['B']='现场填写B'
    assert 'generated scans require measured A/B/C target descriptions' in exploration_gates(generated_config)


def test_offline_generated_entry_does_not_initialize_ros(monkeypatch,capsys,generated_config):
    from multi_agent_nav import go2_check
    monkeypatch.setattr(go2_check,'load_config',lambda *_:generated_config)
    monkeypatch.setattr(go2_check,'live_gates',lambda *_:[])  # fixture has no peer/alignment certification
    monkeypatch.setattr(go2_check,'run_ros',lambda *_:pytest.fail('offline initialized ROS'))
    assert go2_check.main(['--offline','--strategy','generated_candidates_vlm'])==0
    report=json.loads(capsys.readouterr().out)
    assert len([p for p in report['semantic_search']['points'] if p['furniture_id']])==4
    assert any(not p['furniture_id'] for p in report['semantic_search']['points']) and not report['motion_sent']
