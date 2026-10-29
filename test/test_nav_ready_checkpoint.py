from concurrent.futures import Future
from dataclasses import replace
from types import SimpleNamespace as S

import pytest

from multi_agent_nav.models import Pose2D
from test_ranger_single_robot_parity import local_runtime


def test_pause_inference_preserves_candidate_view_and_confirmation(local_runtime):
    r=local_runtime; a=r.adapter
    a.scan_viewpoint_id='candidate-3'
    reply=a.start_scan({'B':'cup'},'scan',current_view_only=True)
    a.scan['index']=4
    r.frame(); a.tick()
    assert a.scan['phase']=='INFER'
    scan=a.scan; epoch=r.node.task_epoch
    a.pause_local_execution('local health invalid: nav_ready')
    r.clock[0]+=10; a.tick()
    assert a.scan is scan and scan['index']==4 and not reply.done()
    assert a.scan_viewpoint_id=='candidate-3' and r.node.task_epoch==epoch
    a.resume_local_execution()
    assert a.scan is scan and scan['index']==4 and scan['phase']=='FRAME'
    assert scan.get('vlm_attempt',1)==1
    for _ in range(3):
        r.frame(); a.tick(); a.tick()
    assert reply.result()[0].common_position.x==2.
    assert r.node.task_epoch==epoch and a.scan_viewpoint_id=='candidate-3'


def test_pause_mid_spin_resumes_remaining_angle_and_same_view(local_runtime):
    r=local_runtime; a=r.adapter
    reply=a.start_scan({'B':'cup'},'scan')
    a.tick()
    scan=a.scan; spin_reply=scan['pending']; target=r.requests[0].target_yaw
    a.pause_local_execution('local health invalid: nav_ready')
    r.terminals[0].set_result(S(status=5))
    assert not spin_reply.done() and not reply.done()
    health=a.get_health
    a.get_health=lambda:replace(health(),pose=Pose2D(0,0,-.3))
    a.resume_local_execution()
    assert r.requests[-1].target_yaw==pytest.approx(target+.3)
    assert scan['index']==0 and scan['pending'] is spin_reply
    r.terminals[-1].set_result(S(status=4)); a.tick()
    assert scan['phase']=='FRAME' and scan['index']==0


@pytest.mark.parametrize('terminal', [4,5])
def test_pause_navigation_preserves_logical_goal_and_reports_only_resumed_result(local_runtime,terminal):
    r=local_runtime; a=r.adapter
    goal=Pose2D(3,2,.4)
    goal_id=a.start_navigation(goal,'task','command','VIEWPOINT_NAV')
    a.pause_local_execution('local health invalid: nav_ready')
    r.terminals[0].set_result(S(status=terminal))
    assert not a.results
    a.resume_local_execution()
    if terminal==5:
        assert goal_id in a.motions
        assert r.requests[-1].pose.pose.position.x==3
        assert r.requests[-1].pose.pose.position.y==2
        r.terminals[-1].set_result(S(status=4))
    result=a.results.popleft()
    assert result.goal_id==goal_id and result.status=='SUCCEEDED'
    assert not a.results


@pytest.mark.parametrize('fault', ['nav', 'heartbeat'])
def test_pause_preserves_real_runner_generator(local_runtime, fault):
    from multi_agent_nav.runner import AgentRunner
    from cooperative_ws.src.multi_agent_nav.tests.test_private_c import local_agent
    r=local_runtime; config,machine,_,command=local_agent(); a=r.adapter
    machine.accept(command,0); machine.take_effects()
    runner=AgentRunner(machine,a,config['scene']['targets'],lambda _:None,config=config)
    progressed=[]
    def steps():
        progressed.append('candidate-3/view-5')
        yield
        progressed.append('same-view-completed')
        yield
    runner.local_steps=steps(); runner.local_identity=(command.task_id,command.command_id)
    next(runner.local_steps); original=runner.local_steps
    healthy=a.get_health
    a.get_health=lambda:replace(healthy(),nav_ready=fault != 'nav')
    if fault == 'heartbeat': machine.sync_received = -4.
    runner.tick(.1)
    assert machine.paused and machine.local_suspended and runner.local_steps is original
    assert progressed==['candidate-3/view-5']
    a.get_health=healthy
    machine.sync_received = .2
    runner.tick(.2)
    assert not machine.paused and runner.local_steps is original
    assert progressed==['candidate-3/view-5','same-view-completed']


def test_planner_result_waits_until_resume(local_runtime):
    r=local_runtime; a=r.adapter
    result=Future()
    r.node.path_planner.send_goal_async=lambda _:result
    r.node.plan_then_navigate([(3.,2.,.4)],'approach')
    a.pause_local_execution('local health invalid: nav_ready')
    result.set_result(S(accepted=False))
    assert r.node.plan_pending and len(a.paused_callbacks)==1
    assert not r.requests
    a.resume_local_execution()
    assert not r.node.plan_pending and not a.paused_callbacks


def test_resume_waits_for_native_cancel_terminal(local_runtime):
    r=local_runtime; a=r.adapter
    a.start_navigation(Pose2D(3,2,0),'task','command','VIEWPOINT_NAV')
    a.pause_local_execution('local health invalid: nav_ready')
    with pytest.raises(RuntimeError,match='prerequisites'):a.resume_local_execution()
    assert a.execution_paused and len(r.requests)==1
    r.clock[0]+=6; a.tick()
    assert a.motion_fault


def test_manual_stop_discards_suspended_view_and_late_results(local_runtime):
    r=local_runtime; a=r.adapter
    reply=a.start_scan({'B':'cup'},'scan')
    a.tick(); a.pause_local_execution('local health invalid: nav_ready')
    a.stop_motion('coordinator pause/cancel')
    r.terminals[0].set_result(S(status=4))
    assert reply.cancelled() and a.scan is None and not r.node.execution_active
    assert not a.paused_callbacks and not a.paused_motions
    assert len(r.requests)==1


def test_late_motion_acceptance_is_canceled_before_resuming(local_runtime):
    r=local_runtime; a=r.adapter
    response=Future(); terminal=Future(); canceled=[]
    original=r.node.navigator.send_goal_async
    r.node.navigator.send_goal_async=lambda _:response
    goal=a.start_navigation(Pose2D(3,2,0),'task','command','VIEWPOINT_NAV')
    a.pause_local_execution('local health invalid: nav_ready')
    response.set_result(S(accepted=True,goal_id=S(uuid=bytes(16)),get_result_async=lambda:terminal,
        cancel_goal_async=lambda:canceled.append(True)))
    assert canceled and goal in a.motions
    terminal.set_result(S(status=5))
    assert not a.results
    r.node.navigator.send_goal_async=original
    a.resume_local_execution()
    assert goal in a.motions
    r.terminals[-1].set_result(S(status=4))
    assert a.results.popleft().goal_id==goal


def test_eight_views_keep_order_when_paused_at_fifth_view(local_runtime):
    import math
    from test_ranger_single_robot_parity import PendingVision
    r=local_runtime; a=r.adapter; a.vision=PendingVision()
    a.config['scan_angles']=[-(i+1)*math.pi/4 for i in range(8)]
    a.scan_viewpoint_id='candidate-3'
    reply=a.start_scan({'B':'cup'},'candidate-3',stop_on_target='B')
    paused=False
    for _ in range(1900):
        if r.terminals and not r.terminals[-1].done():r.terminals[-1].set_result(S(status=4))
        r.frame(); a.tick()
        if a.scan and a.scan['index']==4 and a.scan['phase']=='INFER' and not paused:
            old=a.scan['pending']
            a.pause_local_execution('local health invalid: nav_ready')
            r.clock[0]+=10; a.tick()
            old.set_result({'B':('DETECTED',r.result[0])})
            assert not r.node.target_reference_position
            a.resume_local_execution(); paused=True
            assert a.scan['index']==4 and a.scan['id']=='candidate-3'
        if reply.done():break
    assert paused and reply.done()
    assert [o.view_index for o in reply.result()]==list(range(8))
    assert len(a.vision.jobs)==17  # 16 attempts plus the interrupted fifth-view request
    assert a.scan_viewpoint_id=='candidate-3'
