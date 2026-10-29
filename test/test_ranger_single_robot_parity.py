"""Behavior oracle pinned to the only authorized historical revision."""
import importlib.util
import ast
import math
import subprocess
import types
from collections import deque
from concurrent.futures import Future
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
import yaml

from vlm_nav.geometry import depth_at_pixel_with_reason

OLD = '251f9fb39c25849dac00e81aec97fa5c45d98c1d'


def old_geometry():
    module = types.ModuleType('ranger_old_geometry')
    source = subprocess.check_output(['git', 'show', OLD + ':vlm_nav/geometry.py'], text=True)
    exec(compile(source, OLD + ':vlm_nav/geometry.py', 'exec'), module.__dict__)
    return module


def old_method(name):
    from vlm_nav import vlm_navigator
    source=subprocess.check_output(['git','show',OLD+':vlm_nav/vlm_navigator.py'],text=True)
    cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=='VLMNavigator')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
    namespace=dict(vars(vlm_navigator))
    exec(compile(ast.Module(body=[method],type_ignores=[]),OLD+':'+name,'exec'),namespace)
    return namespace[name]


@pytest.mark.parametrize('values', [
    [1.] * 6 + [3.] * 6,
    [1., 1.01, 1.02, 1.03, 1.04, 1.05, 1.06, 1.15],
    [float('nan')] * 12,
    [6.8] * 12,
    [.2] * 12,
    [6.] * 12,
])
def test_depth_matches_fixed_old_source(values):
    depth = np.array(values).reshape(1, -1)
    args = (depth, len(values)//2, 0)
    assert depth_at_pixel_with_reason(*args) == old_geometry().depth_at_pixel_with_reason(*args)


def test_shared_execution_is_available_to_both_entrypoints():
    assert importlib.util.find_spec('vlm_nav.ranger_single_robot_execution') is not None


def execution_class():
    assert importlib.util.find_spec('vlm_nav.ranger_single_robot_execution') is not None
    from vlm_nav.ranger_single_robot_execution import RangerSingleRobotExecution
    return RangerSingleRobotExecution


@pytest.mark.parametrize('distance,expected', [(.81, 'SUCCEEDED'), (.85, 'APPROACHING')])
def test_stop_samples_before_terminal_remain_valid(distance, expected):
    from test_navigation_gate import arrival_node, stationary_odom
    cls = execution_class()
    node = arrival_node(distance)
    node.__class__ = cls
    node.execution_adapter = types.SimpleNamespace()
    node.begin_approach_stop('distance')
    node.arrival_action_terminal = False
    for _ in range(3):
        node.on_arrival_odom(stationary_odom())
    node.advance_approach_stop()
    assert node.state == 'APPROACH_STOPPING'
    node.arrival_action_terminal = True
    node.advance_approach_stop()
    assert node.state == expected


def test_confirmation_freezes_reference_after_three_consistent_frames():
    from vlm_nav.geometry import TargetTracker
    cls = execution_class()
    node = cls.__new__(cls)
    node.state = 'SCANNING'
    node.target_tracker = TargetTracker(3, .35)
    node.target_reference_position = None
    node.frontier_generation = 0
    node.cancel_motion = lambda **_: None
    node.set_state = lambda value: setattr(node, 'state', value)
    node.publish_grounded_markers = lambda *_: None
    node.get_logger = lambda: types.SimpleNamespace(warn=lambda *_: None)
    for point in [(1., 0., 0.), (2., 0., 0.), (2., 0., 0.)]:
        node.confirm_target(point)
        assert node.target_reference_position is None
    node.confirm_target((2., 0., 0.))
    assert node.state == 'APPROACHING'
    assert tuple(node.target_reference_position) == (2., 0., 0.)
    node.confirm_target((2.2, 0., 0.))
    assert tuple(node.target_reference_position) == (2., 0., 0.)


@pytest.fixture
def local_runtime(monkeypatch):
    from multi_agent_nav.adapters.ranger import RangerAdapter
    from multi_agent_nav.frame_adapter import FrameAdapter
    from multi_agent_nav.models import Health, Pose2D
    from vlm_nav.geometry import TargetTracker
    from vlm_nav.models import FrameSnapshot, VLMResult, Pixel
    from test_navigation_gate import arrival_node
    from builtin_interfaces.msg import Time
    clock=[100.]
    monkeypatch.setattr('time.monotonic',lambda:clock[0])
    node=arrival_node();node.__class__=execution_class()
    node.get_logger=lambda:types.SimpleNamespace(**{name:lambda *args,**kwargs:None for name in ('info','warn','error')})
    node.p=types.SimpleNamespace(**yaml.safe_load((Path(__file__).parents[1]/'config/robot.yaml').read_text())['vlm_nav']['ros__parameters'])
    node.sequence=0;node.task_epoch=0;node.frontier_generation=0
    node.blocked_goals={}
    node.pending_rgbd_frames=deque();node.target_tracker=TargetTracker(3,.35)
    node.get_parameter=lambda name:types.SimpleNamespace(value=True if name=='enabled' else 'cup')
    node.publish_state=lambda:None
    for name in ('publish_debug','publish_grounded_markers','log_worker_result','clear_vlm_grounding_markers'):
        setattr(node,name,lambda *args,**kwargs:None)
    # Keep the real state transition so the confirmation timer is exercised.
    node.set_state=types.MethodType(execution_class().set_state,node)
    node.fail_safe=types.MethodType(execution_class().fail_safe,node)
    node.last_valid_rgbd_received=node.last_camera_tf_success=node.last_robot_tf_success=clock[0]
    node.accepted_results=node.rejected_results=node.api_failures=0
    node.vlm_input_ready=lambda now:True
    node.stop_pub=types.SimpleNamespace(publish=lambda _:None)
    node.get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=int(clock[0]*1e9),to_msg=lambda:Time(sec=int(clock[0]))))
    adapter=RangerAdapter.__new__(RangerAdapter)
    adapter.node=node;adapter.scan=None;adapter.motions={};adapter.results=deque();adapter.plans={}
    adapter.motion_fault=False;adapter.path=[];adapter.read_only=False
    adapter.config=dict(robot_id='ranger',scan_angles=[-.785,-1.57],api_timeout=15.,
        arrival_config_files=[str(Path(__file__).parents[1]/'config/robot.yaml')])
    adapter.frame=FrameAdapter('r','v','ranger_local_map','local_only',[0,0,0])
    adapter.get_health=lambda:Health(pose=Pose2D(*node.robot_pose()),localization_valid=True,nav_ready=True,stopped=True,motion_terminal=not adapter.motions)
    adapter._motion_event=lambda *args,**kwargs:None
    terminals=[];requests=[]
    def ready(value):
        f=Future();f.set_result(value);return f
    def send(goal):
        terminal=Future();terminals.append(terminal);requests.append(goal)
        return ready(types.SimpleNamespace(accepted=True,goal_id=types.SimpleNamespace(uuid=bytes(16)),
            get_result_async=lambda:terminal,cancel_goal_async=lambda:ready(True)))
    node.spinner=node.navigator=types.SimpleNamespace(wait_for_server=lambda **_:True,send_goal_async=send)
    plans=[]
    def plan(goal):
        plans.append(goal)
        poses=[types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x,y=0.))) for x in (0.,1.)]
        wrapped=types.SimpleNamespace(status=4,result=types.SimpleNamespace(path=types.SimpleNamespace(poses=poses)))
        return ready(types.SimpleNamespace(accepted=True,get_result_async=lambda:ready(wrapped),cancel_goal_async=lambda:ready(True)))
    node.path_planner=types.SimpleNamespace(wait_for_server=lambda **_:True,send_goal_async=plan)
    result=[VLMResult(True,True,True,True,.9,Pixel(5,5),Pixel(5,5))]
    adapter.client=types.SimpleNamespace(infer_abc=lambda *_:{'B':('DETECTED',result[0])})
    adapter.create_vision_client=lambda config:adapter.client
    adapter.vision=types.SimpleNamespace(available=True,poll=lambda:None,cancel=lambda:None,
        submit=lambda fn:ready(fn()),discard=lambda:None,timeout=20.)
    adapter.begin_local_execution('task','command','B','cup')
    def frame(depth=2.,sequence=None):
        clock[0]+=.1
        node.sequence=node.sequence+1 if sequence is None else sequence
        node.last_valid_rgbd_received=node.last_camera_tf_success=node.last_robot_tf_success=clock[0]
        transform=np.eye(4);transform[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
        node.latest_snapshot=FrameSnapshot(node.sequence,node.task_epoch,'cup',clock[0],types.SimpleNamespace(sec=int(clock[0]),nanosec=0),'camera',
            np.zeros((11,11,3),dtype=np.uint8),np.full((11,11),depth),(100.,100.,5.,5.),transform)
    return types.SimpleNamespace(node=node,adapter=adapter,clock=clock,frame=frame,result=result,
        requests=requests,terminals=terminals,plans=plans)


def test_actual_scan_requires_three_new_model_frames(local_runtime):
    r=local_runtime
    future=r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    for index in range(3):
        r.frame();r.adapter.tick();r.adapter.tick()
        assert future.done()==(index==2)
        if index<2:
            for _ in range(4):r.adapter.tick()
            assert not future.done()
    assert future.result()[0].common_position.x==2.
    assert r.node.target_tracker.progress==3
    assert not r.requests


@pytest.mark.parametrize('field,value', [('confidence',.59),('object_match',False),('qualifier_match',False),('relation_match',False),('evidence_pixel',None)])
def test_detected_label_cannot_bypass_old_matching(local_runtime,field,value):
    r=local_runtime;r.result[0]=replace(r.result[0],**{field:value})
    future=r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    r.frame();r.adapter.tick();r.adapter.tick()
    assert future.done()
    assert future.result()[0].status=='NOT_DETECTED'
    assert r.node.target_reference_position is None


def test_sparse_depth_uses_old_rotation_then_fresh_reobservation(local_runtime):
    r=local_runtime
    future=r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    r.frame(float('nan'));r.adapter.tick();r.adapter.tick()
    assert not future.done()
    assert r.requests[-1].target_yaw==pytest.approx(np.deg2rad(10.))
    assert r.node.depth_reobserve_attempts==1
    r.terminals[-1].set_result(types.SimpleNamespace(status=4))
    assert not future.done()
    r.clock[0]+=.4
    for _ in range(3):
        r.frame();r.adapter.tick();r.adapter.tick()
    assert future.done() and future.result()[0].status=='DETECTED'


def start_approach(r,distance=2.):
    from multi_agent_nav.models import Pose2D
    r.node.set_confirmed_target_reference_position((distance,0.,0.))
    r.node.set_state('APPROACHING')
    r.steps=r.adapter.approach_steps(Pose2D(distance,0.),'task','command','PICKUP_NAV',lambda _:None,lambda:r.clock[0])
    next(r.steps)
    return r.adapter.approach


def advance_approach(r):
    r.clock[0]+=.51
    try:next(r.steps)
    except StopIteration as done:return done.value


@pytest.mark.parametrize('distance', [.89,.81,.80])
def test_already_near_reference_stops_without_new_action(local_runtime,distance):
    from test_navigation_gate import stationary_odom
    r=local_runtime;a=start_approach(r,distance)
    assert a.state=='APPROACH_STOPPING'
    assert not r.requests and not r.plans
    for _ in range(3):r.node.on_arrival_odom(stationary_odom())
    r.clock[0]+=.21;next(r.steps)
    assert a.state==('SUCCEEDED' if distance<=.81 else 'APPROACHING')
    if distance<=.81:assert advance_approach(r)=='TARGET_APPROACHED'
    r.steps.close()


@pytest.mark.parametrize('terminal', [4,5,6])
def test_distance_cancel_waits_for_terminal_and_rejects_abort(local_runtime,terminal):
    from test_navigation_gate import stationary_odom
    r=local_runtime;a=start_approach(r)
    assert len(r.requests)==1 and len(r.plans)==1
    assert r.requests[0].pose.pose.position.x==2.
    r.node.robot_pose=lambda:(1.2,0.,0.)
    advance_approach(r)
    assert a.state=='APPROACH_STOPPING'
    for _ in range(3):r.node.on_arrival_odom(stationary_odom())
    r.clock[0]+=.21;next(r.steps)
    assert a.state=='APPROACH_STOPPING'
    r.terminals[0].set_result(types.SimpleNamespace(status=terminal))
    r.clock[0]+=.21;next(r.steps) if terminal!=6 else None
    assert a.state==('FAILED' if terminal==6 else 'SUCCEEDED')
    assert not r.adapter.results
    r.steps.close()


def test_early_nav_success_retries_fixed_reference_once(local_runtime):
    from test_navigation_gate import stationary_odom
    r=local_runtime;a=start_approach(r)
    r.node.robot_pose=lambda:(1.15,0.,.7)
    r.terminals[0].set_result(types.SimpleNamespace(status=4))
    assert a.state=='APPROACH_STOPPING'
    for _ in range(3):r.node.on_arrival_odom(stationary_odom())
    r.clock[0]+=.21;next(r.steps)
    advance_approach(r)
    assert len(r.requests)==2 and r.requests[1].pose.pose.position.x==2.
    assert a.arrival_retry_count==1 and a.approach_stop_radius()==.81
    r.terminals[1].set_result(types.SimpleNamespace(status=4))
    for _ in range(3):r.node.on_arrival_odom(stationary_odom())
    r.clock[0]+=.21;next(r.steps)
    assert a.state=='FAILED'
    r.steps.close()


def test_external_cancel_late_success_cannot_complete_local_task(local_runtime):
    r=local_runtime;a=start_approach(r)
    r.adapter.stop_motion('external cancellation')
    r.terminals[0].set_result(types.SimpleNamespace(status=4))
    r.node.local_tick()
    assert a.state=='FAILED' and r.node.state=='DISARMED'
    assert not r.adapter.local_arrival_verified and not r.adapter.results
    r.steps.close()


def test_sensor_wait_invalidates_inflight_result_and_has_old_timeout(local_runtime):
    r=local_runtime
    future=r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    r.frame();r.adapter.tick()
    epoch=r.node.task_epoch
    r.clock[0]+=2.01;r.adapter.tick()
    assert r.node.state=='SENSOR_WAITING' and r.node.task_epoch==epoch+1
    r.adapter.tick()
    assert not future.done() and r.node.target_reference_position is None
    r.clock[0]+=30.01;r.adapter.tick()
    assert r.node.state=='FAILED'


@pytest.mark.parametrize('cooperative',[False,True])
@pytest.mark.parametrize('target',['B','C'])
def test_both_entry_modes_use_same_local_arrival_without_another_stop(local_runtime,cooperative,target):
    from multi_agent_nav.go2_exploration import explore_steps
    from multi_agent_nav.models import Observation, Pose2D
    from test_navigation_gate import stationary_odom
    r=local_runtime
    r.adapter.costmap_transform=lambda **_:Pose2D()
    health=r.adapter.get_health
    r.adapter.get_health=lambda:replace(health(),footprint=[[.1,.1],[.1,-.1],[-.1,-.1],[-.1,.1]])
    config={'scene':{'resources':{},'targets':dict(A='a',B='cup',C='cup'),
        'viewpoints':{'ranger':[dict(id='point',pose=[0.,0.,0.])]}},
        'robot_ranger':dict(stop_margin=.2,exploration_strategy='fixed_viewpoints',approach_on_detection=True)}
    events=[]
    steps=explore_steps(r.adapter,config,r.adapter.frame,200.,'cup',events.append,lambda:[],
        lambda:pytest.fail('local arrival must not add adapter stationary-duration verification'),lambda:None,
        clock=lambda:r.clock[0],robot_id='ranger',target_id=target,require_navigation_success=cooperative,
        initial_observation=Observation(target_id=target,status='DETECTED',position_valid=True,common_position=Pose2D(.8,0.)))
    next(steps);next(steps)
    assert r.adapter.approach.state=='APPROACH_STOPPING'
    for _ in range(3):r.node.on_arrival_odom(stationary_odom())
    r.clock[0]+=.21;next(steps)
    with pytest.raises(StopIteration) as done:next(steps)
    assert done.value.value=='TARGET_APPROACHED'
    assert r.adapter.local_arrival_verified
    assert not r.requests and r.node.target_tracker.progress==0
    assert any(e['state']=='APPROACH_VERIFIED' for e in events)
    assert sum(e['state']=='TARGET_APPROACHED' for e in events)==1


def test_sensor_recovery_requires_three_new_image_time_transforms(local_runtime):
    from builtin_interfaces.msg import Time
    r=local_runtime;r.node.set_confirmed_target_reference_position((2.,0.,0.))
    r.node.set_state('APPROACHING');r.node.enter_sensor_wait('rgbd gap')
    r.node.image_tf_queue_drops=r.node.image_tf_failures=0
    r.node.tf_buffer=types.SimpleNamespace(lookup_transform=lambda *a,**k:types.SimpleNamespace(
        transform=types.SimpleNamespace(translation=types.SimpleNamespace(x=0.,y=0.,z=0.),
        rotation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.))))
    for index in range(3):
        r.node.pending_rgbd_frames.append(types.SimpleNamespace(received_monotonic=r.clock[0],
            stamp=Time(sec=100,nanosec=index),frame_id='camera',rgb=np.zeros((11,11,3),dtype=np.uint8),
            depth_m=np.ones((11,11)),intrinsics=(100.,100.,5.,5.)))
        r.node.resolve_pending_rgbd()
        assert r.node.state==('SCANNING' if index==2 else 'SENSOR_WAITING')
    assert r.node.target_reference_position is None
    assert not r.adapter.scan['current_view_only']
    assert r.adapter.scan['headings'][0]==pytest.approx(np.pi/4)


def test_confirmation_timeout_is_old_twenty_seconds(local_runtime):
    r=local_runtime
    future=r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    r.frame();r.adapter.tick();r.adapter.tick()
    assert r.node.state=='TARGET_CONFIRMING'
    r.clock[0]+=20.01;r.frame();r.adapter.tick()
    assert r.node.state=='FAILED' and future.done()


@pytest.mark.parametrize('interruption',['cancel','health','timeout'])
def test_depth_recovery_remains_interruptible(local_runtime,interruption):
    r=local_runtime
    future=r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    r.frame(float('nan'));r.adapter.tick();r.adapter.tick()
    assert r.node.state=='TARGET_REOBSERVING'
    if interruption=='cancel':r.adapter.stop_motion('external cancellation')
    elif interruption=='health':
        health=r.adapter.get_health
        r.adapter.get_health=lambda:replace(health(),localization_valid=False)
    else:
        r.clock[0]+=r.node.p.goal_timeout+.01
        r.frame()
    r.adapter.tick()
    assert future.done()
    assert not r.adapter.local_arrival_verified


@pytest.mark.parametrize('u',[0,4,5,6,10])
def test_probe_candidates_and_bearings_match_old_method_for_same_input(local_runtime,u):
    from vlm_nav.models import Pixel
    r=local_runtime;r.frame()
    result=replace(r.result[0],target_pixel=Pixel(u,5));snapshot=r.node.latest_snapshot
    for pose in [(0.,0.,0.),(3.,-2.,.7)]:
        assert r.node.target_relative_bearing(snapshot,result.target_pixel,pose)==old_method('target_relative_bearing')(r.node,snapshot,result.target_pixel,pose)
        assert r.node.build_target_probe_candidates(snapshot,result,pose)==old_method('build_target_probe_candidates')(r.node,snapshot,result,pose)


@pytest.mark.parametrize('distance,retry,age,terminal', [(.81,0,0.,True),(.85,0,0.,True),(.85,1,0.,True),(.80,0,.51,True),(.80,0,0.,False)])
def test_arrival_event_outcome_matches_fixed_old_method(distance,retry,age,terminal):
    from test_navigation_gate import arrival_node,stationary_odom
    outcomes=[]
    for advance in [old_method('advance_approach_stop'),execution_class().advance_approach_stop]:
        node=arrival_node(distance)
        node.begin_approach_stop('distance')
        node.arrival_retry_count=retry
        node.arrival_action_terminal=terminal
        for _ in range(3):node.on_arrival_odom(stationary_odom())
        now=node.arrival_last_odom_received+age
        advance(node,now=now)
        outcomes.append((node.state,node.arrival_retry_count,getattr(node,'last_failure_reason',None)))
    assert outcomes[0]==outcomes[1]


def test_out_of_range_depth_dispatches_old_truncated_probe(local_runtime):
    r=local_runtime
    future=r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    r.frame(7.);r.adapter.tick();r.adapter.tick()
    assert not future.done() and r.node.state=='TARGET_REOBSERVING'
    assert r.plans[0].goal.pose.position.x==2.
    assert r.requests[0].pose.pose.position.x==1.
    assert r.node.target_probe_attempts==1 and r.node.depth_reobserve_attempts==0
    r.terminals[0].set_result(types.SimpleNamespace(status=4))
    assert r.adapter.scan['phase']=='FRAME' and not future.done()


def test_approach_has_one_safety_owner_and_preserves_fixed_target(local_runtime):
    r=local_runtime;a=start_approach(r)
    r.node.safety_tick=lambda:pytest.fail('observer must not run a second approach safety loop')
    r.node.local_tick();r.adapter.tick()
    assert a.target_reference_position==(2.,0.) and len(r.requests)==len(r.plans)==1
    r.steps.close()


@pytest.mark.parametrize('kind,flag',[('APPROACH_PICKUP','ready'),('APPROACH_DEPOSIT','deposited')])
@pytest.mark.parametrize('external_stop',[False,True])
def test_runner_consumes_local_verification_without_replacing_it(local_runtime,kind,flag,external_stop):
    from multi_agent_nav.runner import AgentRunner
    from test_navigation_gate import stationary_odom
    r=local_runtime
    a=start_approach(r,.8)
    for _ in range(3):r.node.on_arrival_odom(stationary_odom())
    r.clock[0]+=.21;next(r.steps)
    assert advance_approach(r)=='TARGET_APPROACHED'
    health=r.adapter.get_health
    r.adapter.get_health=lambda:replace(health(),stopped=False)
    failures=[]
    state=types.SimpleNamespace(ready=False,deposited=False)
    command=types.SimpleNamespace(task_id='task',command_id='command',kind=kind)
    machine=types.SimpleNamespace(paused=external_stop,local_suspended=False,command=command,peer_received=100.,
        context=types.SimpleNamespace(robot_id='ranger',task_id='task'),_state=state,stop=failures.append,fail=failures.append)
    def completed():
        yield from ()
        return 'TARGET_APPROACHED'
    runner=AgentRunner.__new__(AgentRunner)
    runner.machine=machine;runner.adapter=r.adapter;runner.local_steps=completed()
    runner.local_identity=('task','command');runner.local_completion=None;runner.owners=lambda:[]
    runner._advance_local()
    if runner.local_completion:runner._complete_local()
    assert not getattr(state,flag)
    r.adapter.get_health=health
    if runner.local_completion:runner._complete_local()
    assert getattr(state,flag) is (not external_stop)
    assert not failures


@pytest.mark.parametrize('attempt,u',[(0,5),(1,5),(2,5),(0,0),(0,10)])
def test_depth_rotation_matches_old_method_exactly(local_runtime,attempt,u):
    from vlm_nav.models import Pixel
    r=local_runtime;r.frame()
    snapshot=r.node.latest_snapshot
    result=replace(r.result[0],target_pixel=Pixel(u,5))
    outcomes=[]
    for recover in [old_method('recover_target_depth'),r.node.recover_target_depth.__func__]:
        r.node.depth_reobserve_attempts=attempt;r.node.target_probe_attempts=0
        r.node.target_reference_position=None;r.node.state='SCANNING'
        spins=[];r.node.send_spin=lambda angle,kind:spins.append((angle,kind))
        applied=recover(r.node,snapshot,result,'mixed_depth_samples:0/8')
        outcomes.append((applied,spins,r.node.state,r.node.depth_reobserve_attempts))
    assert outcomes[0]==outcomes[1]


def test_sensor_frames_can_recover_before_canceled_navigation_is_terminal(local_runtime):
    r=local_runtime
    r.node.set_confirmed_target_reference_position((2.,0.,0.))
    r.node.send_navigation_goal(2.,0.,0.,'target_probe')
    r.node.enter_sensor_wait('gap')
    assert r.node.recover_from_sensor_wait()
    assert r.adapter.scan is None and len(r.requests)==1
    r.terminals[0].set_result(types.SimpleNamespace(status=5))
    r.adapter.tick()
    assert r.adapter.scan is not None and r.node.state=='SCANNING'
    assert len(r.requests)==2
    assert not r.adapter.local_arrival_verified


def test_local_confirmation_uses_old_timeouts_not_shared_scan_deadline(local_runtime):
    r=local_runtime
    r.adapter.config['scan_timeout']=.01
    future=r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    r.frame();r.adapter.tick();r.adapter.tick()
    assert not future.done() and r.node.state=='TARGET_CONFIRMING'


def test_arrival_retry_does_not_require_path_messages(local_runtime):
    from multi_agent_nav.go2_exploration import explore_steps
    from multi_agent_nav.models import Observation,Pose2D
    from test_navigation_gate import stationary_odom
    r=local_runtime
    r.adapter.costmap_transform=lambda **_:Pose2D()
    health=r.adapter.get_health
    r.adapter.get_health=lambda:replace(health(),pose=Pose2D(*r.node.robot_pose()),
        footprint=[[.1,.1],[.1,-.1],[-.1,-.1],[-.1,.1]],path=list(r.adapter.path),path_valid=bool(r.adapter.path))
    config={'scene':{'resources':{},'targets':dict(B='cup'),
        'viewpoints':{'ranger':[dict(id='next',pose=[1.,0.,0.])]}},
        'robot_ranger':dict(stop_margin=.2,exploration_strategy='fixed_viewpoints',approach_on_detection=True)}
    steps=explore_steps(r.adapter,config,r.adapter.frame,200.,'cup',lambda _:None,lambda:[],None,lambda:None,
        clock=lambda:r.clock[0],robot_id='ranger',target_id='B',require_navigation_success=True,
        initial_observation=Observation(target_id='B',status='DETECTED',position_valid=True,common_position=Pose2D(2.,0.)))
    next(steps);next(steps);r.node.local_tick()
    r.adapter.path=[Pose2D(),Pose2D(1.,0.)]
    next(steps)
    r.node.robot_pose=lambda:(1.15,0.,0.)
    r.terminals[0].set_result(types.SimpleNamespace(status=4))
    for _ in range(3):r.node.on_arrival_odom(stationary_odom())
    r.clock[0]+=.21;next(steps)
    r.clock[0]+=.51;next(steps)
    assert len(r.requests)==2 and not r.adapter.path
    r.clock[0]+=6.;next(steps)
    assert r.adapter.approach.state=='APPROACHING'
    steps.close()


def test_depth_probe_keeps_path_acquired_state_across_scan_ticks(local_runtime):
    from multi_agent_nav.go2_exploration import explore_steps,ExplorationStopped
    from multi_agent_nav.models import Pose2D
    r=local_runtime
    r.adapter.costmap_transform=lambda **_:Pose2D()
    health=r.adapter.get_health
    r.adapter.get_health=lambda:replace(health(),footprint=[[.1,.1],[.1,-.1],[-.1,-.1],[-.1,.1]],
        path=list(r.adapter.path),path_valid=bool(r.adapter.path))
    config={'scene':{'resources':{},'targets':dict(B='cup'),
        'viewpoints':{'ranger':[dict(id='next',pose=[1.,0.,0.])]}},
        'robot_ranger':dict(stop_margin=.2,exploration_strategy='fixed_viewpoints',approach_on_detection=True)}
    steps=explore_steps(r.adapter,config,r.adapter.frame,200.,'cup',lambda _:None,lambda:[],None,lambda:None,
        clock=lambda:r.clock[0],robot_id='ranger',target_id='B',require_navigation_success=True,scan_at_start=True)
    next(steps);next(steps)
    r.frame(7.);r.adapter.tick();r.adapter.tick()
    r.adapter.path=[Pose2D(),Pose2D(1.,0.)]
    next(steps)
    r.adapter.path=[]
    with pytest.raises(ExplorationStopped):next(steps)


class PendingVision:
    def __init__(self):
        self.available=True
        self.timeout=20.
        self.jobs=[]
        self.reply=None

    def poll(self):
        pass

    def submit(self, function):
        self.available=False
        self.reply=Future()
        self.jobs.append((function,self.reply))
        return self.reply

    def discard(self):
        self.available=True
        self.reply=None

    def cancel(self):
        if self.reply is not None:self.reply.cancel()


def test_scan_retries_at_eight_seconds_then_skips_only_that_view(local_runtime):
    r=local_runtime
    r.adapter.vision=PendingVision()
    r.adapter.create_vision_client=lambda config:r.adapter.client
    events=[];r.adapter._motion_event=lambda event,*args,**fields:events.append((event,fields))
    reply=r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B')
    scan=r.adapter.scan
    scan['phase']='FRAME'  # the first rotation has already completed
    snapshots=[]
    for attempt in range(2):
        r.frame();r.adapter.tick()
        snapshots.append(scan['snapshot'])
        assert scan['phase']=='INFER' and scan['index']==0
        r.clock[0]+=7.99;r.frame(sequence=r.node.sequence)
        # Keep the elapsed request just below the deadline, with current sensor health.
        r.clock[0]-=.1
        r.adapter.tick()
        assert scan['phase']=='INFER' and len(r.adapter.vision.jobs)==attempt+1
        r.clock[0]+=.02;r.adapter.tick()
        assert not reply.done() and r.node.state=='SCANNING'
        if attempt<1:assert scan['phase']=='FRAME' and scan['index']==0
    assert scan['index']==1 and scan['phase']=='SPIN'
    assert len(r.adapter.vision.jobs)==2
    assert len({s.sequence for s in snapshots})==2
    assert scan['observations'][0].status=='TIMEOUT'
    assert any(e=='SCAN_VIEW_SKIPPED' for e,_ in events)
    # A late detection from the abandoned first request must not confirm a target.
    r.adapter.vision.jobs[0][1].set_result({'B':('DETECTED',r.result[0])})
    r.adapter.tick()
    assert r.node.target_reference_position is None
    assert len(r.requests)==1  # next view's rotation, not navigation to a new candidate


@pytest.mark.parametrize('failure',['error','invalid','stale'])
def test_scan_failure_retries_same_view_without_entering_failed(local_runtime,failure):
    r=local_runtime
    reply=r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    scan=r.adapter.scan
    if failure=='error':r.adapter.client.infer_abc=lambda *_:(_ for _ in ()).throw(ValueError('invalid response'))
    if failure=='invalid':r.adapter.client.infer_abc=lambda *_:{'B':('API_ERROR',None)}
    # Fake a normal worker: provider exceptions are delivered through its future.
    def submit(fn):
        f=Future()
        try:f.set_result(fn())
        except Exception as e:f.set_exception(e)
        return f
    r.adapter.vision.submit=submit;r.adapter.vision.discard=lambda:None
    r.adapter.create_vision_client=lambda config:r.adapter.client
    for attempt in range(2):
        r.frame();r.adapter.tick()
        if failure=='stale':
            r.clock[0]+=8.01;r.frame()
        r.adapter.tick()
        assert r.node.state=='SCANNING'
        assert reply.done()==(attempt==1)
        if attempt<1:assert scan['phase']=='FRAME' and scan['index']==0
    assert reply.result()[0].status in ('TIMEOUT','API_ERROR')
    assert not r.requests  # initial view keeps its no-rotation exception


def test_fast_retry_uses_new_frame_and_still_needs_three_confirmations(local_runtime):
    r=local_runtime;r.adapter.vision=PendingVision()
    r.adapter.create_vision_client=lambda config:r.adapter.client
    reply=r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    r.frame();r.adapter.tick();old=r.adapter.scan['snapshot']
    r.clock[0]+=8.01;r.frame();r.adapter.tick()
    assert r.node.state=='SCANNING'
    for i in range(3):
        r.frame();r.adapter.tick()
        assert r.adapter.scan['snapshot'].sequence>old.sequence
        fn,f=r.adapter.vision.jobs[-1];f.set_result(fn());r.adapter.vision.available=True
        r.adapter.tick()
        assert reply.done()==(i==2)
    assert r.node.target_tracker.progress==3


def test_abandoned_scan_worker_keeps_its_own_image_targets_and_evidence(local_runtime,tmp_path):
    r=local_runtime;r.adapter.vision=PendingVision()
    r.adapter.scan_evidence_dir=tmp_path
    calls=[];clients=[]
    def client(config):
        c=types.SimpleNamespace(client=types.SimpleNamespace(close=lambda:None))
        def infer(rgb,targets):
            calls.append((rgb,targets,c.evidence.directory,config['api_timeout']))
            return {'B':('NOT_DETECTED',replace(r.result[0],target_visible=False))}
        c.infer_abc=infer;clients.append(c);return c
    r.adapter.create_vision_client=client
    r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    r.frame();r.adapter.tick();first=r.adapter.scan['snapshot']
    r.clock[0]+=8.01;r.frame();r.adapter.tick()
    r.frame();r.adapter.tick();second=r.adapter.scan['snapshot']
    # Finish the replacement first, then the late abandoned request.
    r.adapter.vision.jobs[1][0]()
    r.adapter.vision.jobs[0][0]()
    assert calls[0][0] is second.rgb and calls[1][0] is first.rgb
    assert calls[0][1]==calls[1][1]=={'B':'cup'}
    assert calls[0][2]!=calls[1][2]
    assert calls[0][3]==calls[1][3]==8.
    assert len(clients)==2 and all(c.evidence is None for c in clients)
    assert r.node.target_reference_position is None


def test_eight_views_each_exhaust_two_attempts_without_changing_candidate(local_runtime):
    r=local_runtime;r.adapter.vision=PendingVision()
    r.adapter.config['scan_angles']=[-(i+1)*math.pi/4 for i in range(8)]
    reply=r.adapter.start_scan({'B':'cup'},'candidate',stop_on_target='B')
    seen=[]
    for _ in range(1800):
        if r.terminals and not r.terminals[-1].done():
            r.terminals[-1].set_result(types.SimpleNamespace(status=4))
        r.frame();r.adapter.tick()
        if r.adapter.scan:seen.append(r.adapter.scan['id'])
        assert r.node.state=='SCANNING'
        if reply.done():break
    else:pytest.fail('eight-view scan did not finish')
    assert set(seen)=={'candidate'}
    assert len(r.adapter.vision.jobs)==16
    assert [o.view_index for o in reply.result()]==list(range(8))
    assert all(o.status=='TIMEOUT' for o in reply.result())
    assert all(hasattr(goal,'target_yaw') for goal in r.requests)


def test_independent_failure_is_not_revived_by_vlm_retry(local_runtime):
    r=local_runtime;r.adapter.vision=PendingVision()
    reply=r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    r.frame();r.adapter.tick()
    r.node.fail_safe('independent safety failure')
    r.clock[0]+=8.01;r.frame();r.adapter.tick()
    assert reply.done() and isinstance(reply.exception(),RuntimeError)
    assert r.node.state=='FAILED' and len(r.adapter.vision.jobs)==1


def test_seven_second_results_survive_all_three_confirmation_frames(local_runtime):
    r=local_runtime;r.adapter.vision=PendingVision()
    reply=r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    for i in range(3):
        r.frame();r.adapter.tick()
        r.clock[0]+=7.4;r.frame()
        fn,future=r.adapter.vision.jobs[-1]
        future.set_result(fn());r.adapter.vision.available=True
        r.adapter.tick()
        assert r.node.target_tracker.progress==i+1
        assert reply.done()==(i==2)
    assert reply.result()[0].status=='DETECTED'


def test_located_scan_results_reach_rviz_after_local_validation(local_runtime):
    r=local_runtime;seen=[];tasks=[]
    r.adapter.visualization=types.SimpleNamespace(begin=tasks.append,observe=seen.extend)
    r.adapter.begin_local_execution('next-task','next-command','B','cup')
    reply=r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    for _ in range(3):r.frame();r.adapter.tick();r.adapter.tick()
    assert tasks==['next-task']
    assert reply.done() and seen==reply.result()
    assert seen[0].navigable and seen[0].target_id=='B'
