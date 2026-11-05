"""Cooperative resume keeps checkpoints and uses real action safety deadlines."""
from types import SimpleNamespace as S

import pytest

from multi_agent_nav.models import RecoverablePause
from test_ranger_single_robot_parity import local_runtime


@pytest.mark.parametrize('enabled,safety_started',[(False,100.),(True,80.)])
def test_resume_does_not_move_native_safety_timestamps(local_runtime,enabled,safety_started):
    r=local_runtime; a=r.adapter; a.config['wait_recovery']=enabled
    a.start_scan({'B':'cup'},'scan',current_view_only=True)
    a.scan.update(deadline=150.,depth_reobserve_attempts=3)
    names=('goal_started','arrival_stop_started','arrival_cancel_requested_at','arrival_action_terminal_at')
    for name in names:setattr(r.node,name,80.)
    r.node.target_confirmation_started=90.
    a.pause_local_execution('peer heartbeat timeout')
    r.clock[0]=120.
    a.resume_local_execution()
    assert [getattr(r.node,name) for name in names]==[safety_started]*4
    assert r.node.target_confirmation_started==110.
    assert a.scan['deadline']==170. and a.scan['depth_reobserve_attempts']==3
    assert a.scan['after']==120.


def test_resume_guard_failure_retains_pending_spin(local_runtime):
    r=local_runtime; a=r.adapter; a.config['wait_recovery']=True
    a.start_scan({'B':'cup'},'scan'); a.tick(); pending=a.scan['pending']
    a.pause_local_execution('peer heartbeat timeout')
    r.terminals[-1].set_result(S(status=5))
    def unavailable():raise RecoverablePause('peer heartbeat timeout')
    a.motion_guard=unavailable
    with pytest.raises(RecoverablePause):a.resume_local_execution()
    assert a.execution_paused and len(a.paused_motions)==1 and not pending.done()
    a.motion_guard=None
    a.resume_local_execution()
    assert len(r.requests)==2 and a.scan['pending'] is pending
    r.terminals[-1].set_result(S(status=4)); a.tick()
    assert a.scan['phase']=='FRAME'


def test_failed_resume_callback_repauses_new_motion_and_keeps_remaining_work(local_runtime):
    r=local_runtime; a=r.adapter; a.config['wait_recovery']=True
    a.start_scan({'B':'cup'},'scan'); a.tick(); pending=a.scan['pending']
    a.scan['depth_reobserve_attempts']=3
    r.node.target_confirmation_started=90.
    a.pause_local_execution('peer heartbeat timeout')
    r.terminals[-1].set_result(S(status=5))
    r.clock[0]=120.
    attempts=[]
    def callback():
        attempts.append(1)
        if len(attempts)==1:raise RecoverablePause('local execution temporarily unavailable')
    a.paused_callbacks.append(callback)
    with pytest.raises(RecoverablePause):a.resume_local_execution()
    assert a.execution_paused and a.motions and all(m['cancel'] for m in a.motions.values())
    assert a.paused_callbacks==[callback] and not pending.done()
    r.terminals[-1].set_result(S(status=5))
    r.clock[0]=150.
    a.resume_local_execution()
    assert len(r.requests)==3 and attempts==[1,1] and not a.paused_callbacks
    assert r.node.target_confirmation_started==140.
    assert a.scan['depth_reobserve_attempts']==3
    r.frame(); r.terminals[-1].set_result(S(status=4)); a.tick()
    assert a.scan['phase']=='FRAME'


@pytest.mark.parametrize('fault,reason',[
    ('cancel','cancel_timeout'),('evidence','evidence_write_failed'),
    ('unknown_after_cancel','action_result_error'),
])
def test_wait_recovery_health_exposes_hard_fault_even_after_earlier_reason(fault,reason):
    from test_go2_health_diagnostics import healthy_adapter
    a=healthy_adapter(); a.config['wait_recovery']=True
    a._set_motion_fault('cancel_timeout')
    if fault=='evidence':a.evidence_write_failed=True
    elif fault=='unknown_after_cancel':a._set_motion_fault('action_result_error')
    health=a.get_health()
    assert health.hard_fault==reason and not health.nav_ready
    a.config['wait_recovery']=False
    assert not a.get_health().hard_fault


def test_ranger_rgbd_exception_cannot_hide_hard_fault():
    from test_go2_health_diagnostics import healthy_adapter
    from multi_agent_nav.adapters.ranger import RangerAdapter
    a=healthy_adapter(); a.__class__=RangerAdapter
    a.config['wait_recovery']=True
    a.node.execution_active=True
    a.evidence_write_failed=True
    health=a.get_health()
    assert health.hard_fault=='evidence_write_failed' and not health.nav_ready


@pytest.mark.parametrize('rejected',[False,True])
def test_restarted_native_action_starts_its_own_real_deadline(local_runtime,rejected):
    from concurrent.futures import Future
    r=local_runtime; a=r.adapter; a.config['wait_recovery']=True
    r.node.send_spin(.1,'depth_reobserve')
    original=r.node.goal_started
    r.node.arrival_cancel_requested_at=90.
    a.pause_local_execution('peer heartbeat timeout')
    r.terminals[-1].set_result(S(status=5))
    r.clock[0]=700.
    if rejected:
        def reject(_):
            future=Future(); future.set_result(S(accepted=False)); return future
        r.node.spinner.send_goal_async=reject
    a.resume_local_execution()
    assert r.node.goal_started==(original if rejected else 700.)
    if not rejected:assert r.node.arrival_cancel_requested_at==90.


def test_resume_does_not_refresh_unfinished_planner_deadline(local_runtime):
    r=local_runtime; a=r.adapter; a.config['wait_recovery']=True
    r.node.plan_pending=True; r.node.goal_started=80.
    a.pause_local_execution('peer heartbeat timeout')
    r.clock[0]=700.
    a.resume_local_execution()
    assert r.node.plan_pending and r.node.goal_started==80.


@pytest.mark.parametrize('enabled,task,command,keep_budget,keep_target',[
    (True,'task','command',True,True),
    (True,'task','next-command',True,False),
    (True,'new-task','command',False,False),
    (False,'task','command',False,False),
])
def test_execution_reentry_preserves_only_owned_recovery_state(local_runtime,enabled,task,command,keep_budget,keep_target):
    r=local_runtime; a=r.adapter; a.config['wait_recovery']=enabled
    a.depth_reobserve_budgets={'P1':3}
    saved=dict(target=(2.,0.),retry=1,blocked={(2.,0.):120.})
    a.recovery_approach=saved
    r.node.set_confirmed_target_reference_position((2.,0.,0.))
    r.node.end_execution()
    a.begin_local_execution(task,command,'B','cup')
    assert a.depth_reobserve_budgets==({'P1':3} if keep_budget else {})
    assert a.recovery_approach==(saved if keep_budget else None)
    assert (r.node.target_reference_position is not None)==keep_target
    if keep_target:assert tuple(r.node.target_reference_position)==(2.,0.,0.)


@pytest.mark.parametrize('operation',['base_stop','pause','vision_cancel','approach_close','end_execution'])
def test_stop_publishes_zero_even_when_cancellation_raises(local_runtime,operation):
    from multi_agent_nav.adapters.nav2 import Nav2Adapter
    r=local_runtime; a=r.adapter; published=[]
    r.node.stop_pub.publish=published.append
    def broken(*args,**kwargs):raise RuntimeError('cancel transport failed')
    if operation=='base_stop':
        a.plans[1]=S(cancel_goal_async=broken)
        run=lambda:Nav2Adapter.stop_motion(a,'test failure')
    elif operation=='pause':
        a.start_scan({'B':'cup'},'scan'); a.tick()
        a.cancel_navigation=broken
        run=lambda:a.pause_local_execution('peer heartbeat timeout')
    elif operation=='vision_cancel':
        reply=a.start_scan({'B':'cup'},'scan',current_view_only=True)
        a.vision.cancel=broken
        run=lambda:a.stop_motion('user canceled')
    elif operation=='approach_close':
        a.approach=S(close=broken)
        run=lambda:a.stop_motion('user canceled')
    else:
        r.node.end_execution=broken
        run=lambda:a.stop_motion('user canceled')
    with pytest.raises(RuntimeError,match='cancel transport failed'):run()
    assert published and published[-1].linear.x==0. and published[-1].angular.z==0.
    if operation=='vision_cancel':assert a.scan is None and reply.cancelled()
    if operation in ('approach_close','end_execution'):assert not r.node.execution_active
