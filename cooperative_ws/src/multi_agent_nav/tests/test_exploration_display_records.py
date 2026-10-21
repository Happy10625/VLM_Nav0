"""Display and bounded experiment records; no hardware or provider access."""
import base64
import json
from pathlib import Path
from types import SimpleNamespace as S

import pytest


def test_generated_display_uses_native_geometry_and_clears_previous_decision():
    from multi_agent_nav.go2_exploration import generated_exploration_markers
    from visualization_msgs.msg import Marker
    display=dict(points=[dict(id='table/edge_1',furniture_id='table',pose=[2.,3.,.5]),
                         dict(id='table/edge_2',furniture_id='table',pose=[4.,5.,1.])],
                 records={'table/edge_2':dict(status='observed')},
                 selectable={'table/edge_1':'P1'},selected='table/edge_1')
    candidates,selected=generated_exploration_markers(display)
    assert candidates.markers[0].action==selected.markers[0].action==Marker.DELETEALL
    spheres=[m for m in candidates.markers if m.type==Marker.SPHERE and m.action==Marker.ADD]
    assert [(m.pose.position.x,m.pose.position.y) for m in spheres]==[(2.,3.),(4.,5.)]
    assert spheres[0].color.r>spheres[0].color.b
    assert spheres[1].color.r==spheres[1].color.g==spheres[1].color.b
    arrows=[m for m in selected.markers if m.type==Marker.ARROW and m.action==Marker.ADD]
    assert [(m.pose.position.x,m.pose.position.y) for m in arrows]==[(2.,3.)]
    for status in ('navigation_failed','observation_failed'):
        display['records']['table/edge_1']=dict(status=status)
        retried,_=generated_exploration_markers(display)
        sphere=next(m for m in retried.markers if m.type==Marker.SPHERE and m.action==Marker.ADD)
        assert sphere.color.r>sphere.color.b  # current validated set overrides past failure
    display['selected']=None;display['selectable']={}
    _,cleared=generated_exploration_markers(display)
    assert len(cleared.markers)==1 and cleared.markers[0].action==Marker.DELETEALL


def test_retention_keeps_ten_newest_managed_experiments_and_legacy(tmp_path):
    from multi_agent_nav.experiment_records import experiment_bundle
    legacy=tmp_path/'legacy-bag';legacy.mkdir();(legacy/'data').write_text('keep')
    for index in range(12):
        with experiment_bundle(tmp_path/f'run{index:02}',tmp_path):pass
    assert not (tmp_path/'run00').exists() and not (tmp_path/'run01').exists()
    assert len(list(tmp_path.glob('run*')))==10
    assert (legacy/'data').read_text()=='keep'


def test_retention_does_not_delete_active_experiment_or_follow_symlink(tmp_path):
    from multi_agent_nav.experiment_records import experiment_bundle
    external=tmp_path.parent/(tmp_path.name+'-external');external.mkdir()
    (external/'keep').write_text('safe')
    (tmp_path/'link').symlink_to(external,target_is_directory=True)
    with experiment_bundle(tmp_path/'active',tmp_path):
        for index in range(11):
            with experiment_bundle(tmp_path/f'run{index:02}',tmp_path):pass
        assert (tmp_path/'active').exists()
    assert len(list(tmp_path.glob('run*')))==9 and (external/'keep').exists()


def test_bundle_lock_survives_late_worker_until_completion(tmp_path):
    from concurrent.futures import Future
    from multi_agent_nav.experiment_records import experiment_bundle
    job=Future()
    with experiment_bundle(tmp_path/'late',tmp_path) as lease:lease['pending']=job
    for index in range(10):
        with experiment_bundle(tmp_path/f'run{index:02}',tmp_path):pass
    assert (tmp_path/'late').exists()
    job.set_result(None)
    with experiment_bundle(tmp_path/'last',tmp_path):pass
    assert not (tmp_path/'late').exists()


def test_full_active_capacity_rejects_new_bundle_without_deleting_data(tmp_path):
    from contextlib import ExitStack
    from multi_agent_nav.experiment_records import experiment_bundle
    with ExitStack() as stack:
        for index in range(10):stack.enter_context(experiment_bundle(tmp_path/f'active{index}',tmp_path))
        with pytest.raises(RuntimeError):
            with experiment_bundle(tmp_path/'overflow',tmp_path):pytest.fail('allocated over capacity')
        assert len(list(tmp_path.glob('active*')))==10 and not (tmp_path/'overflow').exists()


def test_generated_entry_defaults_to_managed_bundle_without_ros(monkeypatch,tmp_path):
    from contextlib import nullcontext
    from multi_agent_nav import go2_check,go2_exploration
    from multi_agent_nav.config import load_config
    config=load_config();root=tmp_path/'records';config['robot_go2']['experiment_records_root']=str(root)
    monkeypatch.setattr(go2_check,'load_config',lambda *_:config)
    monkeypatch.setattr(go2_check,'require_ros_environment',lambda:None)
    monkeypatch.setattr(go2_check,'motion_slot',nullcontext)
    monkeypatch.setattr(go2_exploration,'exploration_gates',lambda *_:[])
    paths=[]
    def fake_run(config,args,events):
        paths.append(Path(args.output_dir));events.write('{"fake":true}\n');return 0
    monkeypatch.setattr(go2_check,'run_ros',fake_run)
    assert go2_check.main(['--action','explore','--allow-motion','--strategy','generated_candidates_vlm'])==0
    assert len(paths)==1 and paths[0].parent==root
    assert (paths[0]/'.experiment.json').exists() and (paths[0]/'events.jsonl').exists()


@pytest.mark.parametrize('invalid',[False,True])
def test_selection_records_exact_request_and_invalid_raw_response(monkeypatch,tmp_path,invalid):
    from multi_agent_nav.adapters.go2 import ABCClient
    from vlm_nav.vlm_client import OpenAICompatibleVLMClient
    client=ABCClient.__new__(ABCClient);client.model='stub';client.client=S(api_key='secret-key')
    directory=tmp_path/'decision';directory.mkdir()
    images=[directory/'occupancy.png',directory/'semantic.png']
    for path in images:path.write_bytes(b'png')
    sent=[]
    def completion(self,messages,max_tokens,schema=None):
        sent.extend(messages)
        raw='secret-key invalid JSON' if invalid else '{"candidate_id":"P1","reason":"side"}'
        return S(id='response',choices=[S(finish_reason='stop',message=S(content=raw))])
    monkeypatch.setattr(OpenAICompatibleVLMClient,'_completion',completion)
    snapshot=dict(images=list(map(str,images)),evidence_directory=str(directory),task_id='task',decision_id='decision',
                  main_target='target',candidates=[dict(display_id='P1')],history=[])
    if invalid:
        with pytest.raises(ValueError):client.choose_candidate(snapshot)
    else:assert client.choose_candidate(snapshot)['candidate_id']=='P1'
    request=json.loads((directory/'request.json').read_text())
    assert request['messages']==sent and len(sent[1]['content'])==3
    assert (directory/'response.txt').read_text()==('[REDACTED] invalid JSON' if invalid else '{"candidate_id":"P1","reason":"side"}')
    meta=json.loads((directory/'metadata.json').read_text())
    assert meta['api_returned'] and meta['validation_success']==(not invalid) and meta['decision_id']=='decision'


def test_selection_write_failure_prevents_upload_and_preserves_existing_file(monkeypatch,tmp_path):
    from multi_agent_nav.adapters.go2 import ABCClient
    from vlm_nav.vlm_client import OpenAICompatibleVLMClient
    client=ABCClient.__new__(ABCClient);client.model='stub';client.client=S(api_key='secret-key')
    for name in ('occupancy.png','semantic.png'):(tmp_path/name).write_bytes(b'png')
    (tmp_path/'request.json').write_text('preserve me')
    monkeypatch.setattr(OpenAICompatibleVLMClient,'_completion',lambda *_:pytest.fail('upload after disk failure'))
    snapshot=dict(images=[str(tmp_path/name) for name in ('occupancy.png','semantic.png')],
                  evidence_directory=str(tmp_path),task_id='task',decision_id='decision',
                  main_target='target',candidates=[dict(display_id='P1')],history=[])
    with pytest.raises(OSError):client.choose_candidate(snapshot)
    assert client.selection_evidence_write_failed and (tmp_path/'request.json').read_text()=='preserve me'


def test_rviz_has_generated_topics_and_best_effort_raw_rgb():
    import yaml
    root=Path(__file__).resolve().parents[4]
    config=yaml.safe_load((root/'config/go2_exploration.rviz').read_text())
    displays=config['Visualization Manager']['Displays']
    topics={d.get('Topic',{}).get('Value') for d in displays}
    assert '/vlm_nav/go2_exploration/current_point' not in topics
    assert {'/vlm_nav/go2_exploration/generated_candidates','/vlm_nav/go2_exploration/selected_candidate'}<=topics
    image=next(d for d in displays if d['Class']=='rviz_default_plugins/Image')
    assert image['Topic']['Value']=='/camera/camera/color/image_raw'
    assert image['Topic']['Reliability Policy']=='Best Effort' and image['Transport Hint']=='raw'


@pytest.mark.skipif(not (Path(__file__).resolve().parents[4]/'scripts/go2_camera_ssh.sh').exists(),reason='Go2-only SSH deployment script is outside Ranger port')
def test_ssh_camera_entry_passes_remote_environment_and_raw_profiles(tmp_path):
    import os,subprocess
    root=Path(__file__).resolve().parents[4]
    stub=tmp_path/'ssh'
    stub.write_text('#!/usr/bin/python3\nimport json,sys,os\nopen(os.environ["SSH_ARGS_FILE"],"w").write(json.dumps(sys.argv[1:]))\n')
    stub.chmod(0o755)
    capture=tmp_path/'args.json'
    env=dict(os.environ,PATH=str(tmp_path)+':'+os.environ['PATH'],SSH_ARGS_FILE=str(capture))
    subprocess.run(['bash',str(root/'scripts/go2_camera_ssh.sh')],env=env,check=True)
    args=json.loads(capture.read_text())
    assert args[:4]==['-tt','-o','ConnectTimeout=5','unitree@192.168.123.18']
    command=args[4]
    assert '/home/unitree/unitree_ros2/setup.sh' in command and '/home/unitree/ros2_ws/install/setup.bash' in command
    assert 'depth_module.depth_profile:=640x480x15' in command and 'rgb_camera.color_profile:=640x480x15' in command
    assert 'enable_sync:=true' in command and 'align_depth.enable:=false' in command
    subprocess.run(['bash','-n','-c',command],check=True)
