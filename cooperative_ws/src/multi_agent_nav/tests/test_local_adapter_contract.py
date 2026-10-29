from types import SimpleNamespace as S
import numpy as np
import pytest
from multi_agent_nav.adapters.nav2 import Nav2Adapter
from multi_agent_nav.models import Pose2D


def costmap_adapter():
    a=Nav2Adapter.__new__(Nav2Adapter)
    a.config=dict(footprint=[[.2,.2],[.2,-.2],[-.2,-.2],[-.2,.2]],footprint_padding=0.,costmap_blocked_cost=253)
    def grid(size):
        return S(metadata=S(resolution=.1,size_x=size,size_y=size,
            origin=S(position=S(x=0.,y=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))),
            data=np.zeros(size*size,dtype=np.uint8))
    a.costmap=grid(10);a.global_costmap=grid(100)
    a.costmap_transform=lambda global_goal=False:Pose2D()
    return a


def test_distant_candidate_uses_global_coverage_and_checks_local_overlap():
    a=costmap_adapter()
    assert a.observation_candidate_free(Pose2D(4,4))
    assert a.approach_candidate_free(Pose2D(4,4))
    assert a.observation_candidate_free(Pose2D(1.05,.5))
    a.costmap.data[5*10+9]=255
    assert not a.observation_candidate_free(Pose2D(1.05,.5))
    assert not a.approach_candidate_free(Pose2D(1.05,.5))
    a.costmap_transform=lambda global_goal=False:Pose2D() if global_goal else None
    assert not a.observation_candidate_free(Pose2D(4,4))


def test_motion_guard_runs_before_any_native_goal_is_sent():
    a=Nav2Adapter.__new__(Nav2Adapter)
    a.motions={}
    def guard():raise RuntimeError('paused')
    a.motion_guard=guard
    with pytest.raises(RuntimeError,match='paused'):
        a._motion(S(send_goal_async=lambda _:pytest.fail('must not send')),None,'t','c','SCAN_SPIN')
    assert not a.motions


def test_real_nav2_parameter_reads_are_async_and_keep_controller_planner_values():
    from concurrent.futures import Future
    from rcl_interfaces.msg import ParameterValue,ParameterType
    a=Nav2Adapter.__new__(Nav2Adapter)
    pending=[]
    def client(kind):
        def call(request):
            f=Future();pending.append((kind,request.names,f));return f
        return S(service_is_ready=lambda:True,call_async=call)
    a.node=S(create_client=lambda _,name:client(name))
    future=a.read_approach_parameters()
    assert not future.done()
    while pending:
        kind,names,reply=pending.pop(0)
        values=[]
        for name in names:
            if name in ('goal_checker_plugins','planner_plugins'):
                value=ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY,
                    string_array_value=['custom_checker' if name=='goal_checker_plugins' else 'GridBased'])
            else:
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=.05 if name.endswith('xy_goal_tolerance') else .76)
            values.append(value)
        reply.set_result(S(values=values))
    assert future.result()==dict(xy_goal_tolerance=.05,planner_tolerance=.76)


@pytest.mark.parametrize('robot_id',['go2','ranger'])
def test_private_factory_applies_clockwise_scan_to_actual_adapter_without_changing_single_defaults(monkeypatch,robot_id):
    from importlib import import_module
    from multi_agent_nav.config import load_config
    from multi_agent_nav.local_context import local_config
    from multi_agent_nav.adapters.factory import create_runtime
    config=load_config();config['scene']['share_c_coordinates']=False
    module=import_module('multi_agent_nav.adapters.'+robot_id)
    monkeypatch.setattr(module,'make_observer',lambda *args,**kwargs:S(create_subscription=lambda *a:None,
        create_publisher=lambda *a:S(publish=lambda message:None),destroy_node=lambda:None))
    original=list(config['robot_'+robot_id]['scan_angles'])
    _,adapter=create_runtime(robot_id,config,global_goal_checks=True)
    try:
        assert adapter.config['scan_angles']==pytest.approx([-.7853981633974483*i for i in range(1,9)])
        assert config['robot_'+robot_id]['scan_angles']==original
    finally:adapter.close(stop=False)
    _,adapter=create_runtime(robot_id,local_config(config,robot_id),global_goal_checks=True)
    try:assert adapter.config['scan_angles']==original
    finally:adapter.close(stop=False)
