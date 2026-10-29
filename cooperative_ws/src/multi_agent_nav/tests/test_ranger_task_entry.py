"""Evaluate the Ranger task wiring without initializing ROS or sending goals."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as S
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from multi_agent_nav.adapters.fake import FakeAdapter
from multi_agent_nav.mock import mock_config


def test_ranger_launch_contains_only_local_agent(monkeypatch):
    path=Path(__file__).resolve().parents[1]/'launch/ranger_cooperative.launch.py'
    spec=importlib.util.spec_from_file_location('ranger_task_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    ld=module.generate_launch_description();context=LaunchContext()
    for action in ld.entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    monkeypatch.setattr(module,'Node',lambda **kwargs:kwargs)
    nodes=module.build(context)
    assert len(nodes)==1 and nodes[0]['executable']=='multi_agent_agent_node'
    assert nodes[0]['parameters'][0]['robot_id']=='ranger'


def test_live_ranger_entry_uses_own_adapter_and_config(monkeypatch):
    import multi_agent_nav.agent_node as module
    import multi_agent_nav.adapters.factory as factory
    config=mock_config();config['scene']['share_c_coordinates']=False
    config['alignment']['robots']['ranger']['transform']=None
    adapter=FakeAdapter('ranger')
    adapter.vision=S(timeout=20.)
    adapter.create_vision_client=lambda config:S()
    monkeypatch.setattr(module.Node,'__init__',lambda self,*a:None)
    monkeypatch.setattr(module,'settings',lambda node:(config,False,'/multi_agent',[]))
    monkeypatch.setattr(factory,'create_runtime',lambda robot,config,**kwargs:(S(),adapter))
    monkeypatch.setattr(module.AgentNode,'declare_parameter',lambda *a:None)
    monkeypatch.setattr(module.AgentNode,'get_parameter',lambda *a:S(value='ranger'))
    monkeypatch.setattr(module.AgentNode,'create_publisher',lambda *a:S(publish=lambda _:None))
    monkeypatch.setattr(module.AgentNode,'create_subscription',lambda *a:None)
    monkeypatch.setattr(module.AgentNode,'create_timer',lambda *a:None)
    monkeypatch.setattr(module.AgentNode,'create_client',lambda *a:S())
    monkeypatch.setattr(module.AgentNode,'get_logger',lambda self:S(info=lambda _:None))
    node=module.AgentNode()
    assert node.machine.context.robot_id=='ranger'
    assert node.runner.adapter is adapter and node.runner.config is config
    assert node.runner.success_radius==1.
    assert node.map_worker is not adapter.vision and node.map_worker.timeout==adapter.vision.timeout
    node.map_worker.close()


def test_live_map_monitor_rejects_wrong_map_and_expired_evidence(monkeypatch):
    import multi_agent_nav.agent_node as module
    import multi_agent_nav.config as config_module
    node=object.__new__(module.AgentNode)
    from concurrent.futures import Future
    jobs=[];pending=Future()
    node.simulation=False;node.adapter=S(task_map_ready=False)
    node.map_worker=S(available=True,poll=lambda:None,submit=lambda fn:(jobs.append(fn) or pending))
    node.robot='ranger';node.config={'robot_ranger':{'map_path':'/tmp/expected.yaml'}}
    node.map_probe=S(path='/tmp/wrong.yaml',poll=lambda now:None)
    node.map_ready=False;node.map_verified_at=0.;node.next_map_probe=0.
    node.map_validation=None;node.map_assets=None;node.map_digest_ready=False;node.next_map_validation=0.
    node.poll_map(1.)
    assert not node.adapter.task_map_ready
    assert node.adapter.task_map_health['observed_path']=='/tmp/wrong.yaml'
    assert node.adapter.task_map_health['path_matches'] is False
    assert len(jobs)==1 and not pending.done()
    monkeypatch.setattr(config_module,'file_signature',lambda paths:('stable',))
    pending.set_result(([],('/tmp/expected.yaml','/tmp/image.pgm'),('stable',)))
    node.map_probe=S(path='/tmp/expected.yaml',poll=lambda now:None)
    node.poll_map(2.)
    assert node.adapter.task_map_ready
    node.next_map_probe=100.
    node.poll_map(6.)
    assert not node.adapter.task_map_ready
    assert node.adapter.task_map_health['path_matches'] is True
    assert node.adapter.task_map_health['verification_age_s']==4.
    assert node.adapter.task_map_health['max_age_s']==3.


def test_map_digest_evidence_invalidates_when_image_changes(tmp_path):
    from multi_agent_nav.config import map_file_info,map_file_evidence,file_signature
    path=tmp_path/'map.yaml';image=tmp_path/'map.pgm'
    path.write_text('image: map.pgm\n')
    image.write_bytes(b'P5\n1 1\n255\n\xff')
    config={'robot_ranger':dict(map_path=str(path),map_digest=map_file_info(path)['map_digest'])}
    gates,paths,signature=map_file_evidence(config,'ranger')
    assert not gates and file_signature(paths)==signature
    image.write_bytes(b'P5\n1 1\n255\n\x00\n')
    assert file_signature(paths)!=signature
    assert map_file_evidence(config,'ranger')[0]


def test_map_monitor_revokes_health_before_slow_revalidation(monkeypatch):
    import multi_agent_nav.agent_node as module
    import multi_agent_nav.config as config_module
    from concurrent.futures import Future
    node=object.__new__(module.AgentNode)
    pending=Future();jobs=[]
    node.simulation=False;node.robot='ranger';node.config={}
    node.adapter=S(task_map_ready=True)
    node.map_worker=S(available=True,poll=lambda:None,submit=lambda fn:(jobs.append(fn) or pending))
    node.map_assets=(('/tmp/map.yaml','/tmp/map.pgm'),('old',));node.map_digest_ready=True
    node.map_validation=None;node.map_probe=None;node.next_map_probe=100.;node.next_map_validation=0.
    node.map_ready=True;node.map_verified_at=1.
    monkeypatch.setattr(config_module,'file_signature',lambda _:('changed',))
    node.poll_map(2.)
    assert not node.adapter.task_map_ready and len(jobs)==1 and not pending.done()


def test_periodic_worker_digest_detects_changes_even_with_unchanged_metadata(monkeypatch):
    import multi_agent_nav.agent_node as module
    import multi_agent_nav.config as config_module
    from concurrent.futures import Future
    node=object.__new__(module.AgentNode)
    pending=Future();jobs=[]
    node.simulation=False;node.robot='ranger';node.config={}
    node.adapter=S(task_map_ready=True)
    node.map_worker=S(available=True,poll=lambda:None,submit=lambda fn:(jobs.append(fn) or pending))
    node.map_assets=(('/tmp/map.yaml','/tmp/map.pgm'),('same',));node.map_digest_ready=True
    node.map_validation=None;node.map_probe=None;node.next_map_probe=100.;node.next_map_validation=2.
    node.map_ready=True;node.map_verified_at=1.
    monkeypatch.setattr(config_module,'file_signature',lambda _:('same',))
    node.poll_map(2.)
    assert len(jobs)==1 and node.adapter.task_map_ready
    pending.set_result((['digest mismatch'],node.map_assets[0],('same',)))
    node.map_worker.available=False
    node.poll_map(2.1)
    assert not node.adapter.task_map_ready


def test_live_keepout_requires_both_mask_and_info_and_rejects_late_invalid_data():
    from multi_agent_nav.agent_node import AgentNode
    node=object.__new__(AgentNode)
    node.adapter=S(task_keepout_ready=False)
    node.keepout_grid_ready=node.keepout_info_ready=False
    node.keepout=S(matches_grid=lambda msg:msg=='correct-mask',matches_info=lambda msg:msg=='correct-info')
    node.keepout_grid('correct-mask');assert not node.adapter.task_keepout_ready
    node.keepout_info('correct-info');assert node.adapter.task_keepout_ready
    node.keepout_grid('wrong-map');assert not node.adapter.task_keepout_ready
    assert node.adapter.task_keepout_health['mask_received'] and not node.adapter.task_keepout_health['mask_matches']
    node.keepout_grid('correct-mask');assert node.adapter.task_keepout_ready
    node.keepout_info('wrong-topic');assert not node.adapter.task_keepout_ready


def test_missing_keepout_evidence_blocks_real_adapter_readiness():
    from test_go2_health_diagnostics import healthy_adapter
    a=healthy_adapter();a.task_keepout_ready=False
    assert not a.get_health().nav_ready
    assert not a.last_health_details['checks']['active_task_keepout']
    a.task_keepout_ready=True
    assert a.get_health().nav_ready
