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


def test_live_map_monitor_rejects_wrong_map_and_expired_evidence(monkeypatch):
    import multi_agent_nav.agent_node as module
    import multi_agent_nav.config as config_module
    node=object.__new__(module.AgentNode)
    node.simulation=False;node.adapter=S(task_map_ready=False)
    node.robot='ranger';node.config={'robot_ranger':{'map_path':'/tmp/expected.yaml'}}
    node.map_probe=S(path='/tmp/wrong.yaml',poll=lambda now:None)
    node.map_ready=False;node.map_verified_at=0.;node.next_map_probe=0.
    node.poll_map(1.)
    assert not node.adapter.task_map_ready
    monkeypatch.setattr(config_module,'map_file_gates',lambda *args:[])
    node.map_probe=S(path='/tmp/expected.yaml',poll=lambda now:None)
    node.poll_map(2.)
    assert node.adapter.task_map_ready
    node.next_map_probe=100.
    node.poll_map(6.)
    assert not node.adapter.task_map_ready
