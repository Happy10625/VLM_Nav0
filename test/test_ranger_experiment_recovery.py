"""Runtime waits retain the experiment and never bypass stop/health conditions."""
from dataclasses import replace
from types import SimpleNamespace as S
from concurrent.futures import Future
import pytest
from test_ranger_single_robot_parity import local_runtime


def test_wait_keeps_scan_budget_until_health_and_stop_recover(local_runtime):
    r=local_runtime;r.frame()
    r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    r.adapter.scan.update(index=2,initial=.7,depth_reobserve_attempts=3)
    r.node.depth_reobserve_active=True
    health=r.adapter.get_health;flags={'ready':False,'stopped':False}
    r.adapter.get_health=lambda:replace(health(),nav_ready=flags['ready'],stopped=flags['stopped'])
    r.adapter.costmap_transform=lambda **_:S()
    events=[]
    steps=r.adapter.wait_for_experiment_recovery('temporary costmap',events.append,lambda:[],lambda:r.clock[0])
    next(steps);r.clock[0]+=40;r.frame();next(steps)
    assert not r.requests
    flags['ready']=True;next(steps)
    flags['stopped']=True
    with pytest.raises(StopIteration):next(steps)
    assert r.node.execution_identity==('task','command') and r.node.execution_active
    assert r.adapter.resume_scan['index']==2 and r.adapter.resume_scan['depth_reobserve_attempts']==3
    assert any(e['state']=='WAITING_RECOVERY' for e in events)
    assert events[-1]['state']=='RECOVERED'


def test_manual_stop_cannot_resume_wait(local_runtime):
    r=local_runtime;r.frame();r.adapter.costmap_transform=lambda **_:S()
    steps=r.adapter.wait_for_experiment_recovery('failure',lambda _:None,lambda:[],lambda:r.clock[0])
    next(steps);r.node.end_execution()
    with pytest.raises(RuntimeError,match='execution.*ended'):next(steps)
    assert not r.node.execution_active and not r.requests


def test_unknown_action_terminal_fault_is_never_cleared_by_empty_motion_dict(local_runtime):
    r=local_runtime;r.frame();r.adapter.costmap_transform=lambda **_:S()
    r.adapter.motion_fault=True;r.adapter.motion_fault_reason='action_result_error'
    steps=r.adapter.wait_for_experiment_recovery('failure',lambda _:None,lambda:[],lambda:r.clock[0])
    next(steps);r.clock[0]+=40;r.frame();next(steps)
    assert r.adapter.motion_fault and not r.requests
    steps.close()


def test_initial_observation_waits_without_restarting_experiment(local_runtime):
    from multi_agent_nav.go2_exploration import explore_steps
    from multi_agent_nav.models import Pose2D
    r=local_runtime;r.frame();health=r.adapter.get_health;stopped=[False]
    r.adapter.get_health=lambda:replace(health(),stopped=stopped[0],footprint=[[.1,.1],[.1,-.1],[-.1,-.1],[-.1,.1]])
    r.adapter.costmap_transform=lambda **_:Pose2D()
    config={'scene':{'resources':{},'targets':{'B':'cup'},'viewpoints':{'ranger':[dict(id='one',pose=[1.,0.,0.])]}},
        'robot_ranger':dict(stop_margin=.2,exploration_strategy='fixed_viewpoints',approach_on_detection=True)}
    events=[]
    steps=explore_steps(r.adapter,config,r.adapter.frame,101.,'cup',events.append,lambda:[],None,lambda:None,
        clock=lambda:r.clock[0],robot_id='ranger',target_id='B')
    for _ in range(4):next(steps)
    assert r.adapter.scan is None and not r.requests
    r.clock[0]+=100;r.frame();stopped[0]=True
    for _ in range(3):next(steps)
    assert r.adapter.scan is not None, events
    assert any(e['state']=='WAITING_RECOVERY' for e in events)
    steps.close()


def test_runtime_exception_resumes_same_task_and_preserves_target(local_runtime,monkeypatch):
    import multi_agent_nav.go2_exploration as exploration
    from multi_agent_nav.models import Pose2D
    r=local_runtime;r.frame();r.adapter.costmap_transform=lambda **_:Pose2D()
    identities=[]
    def run(*args,**kwargs):
        identities.append((kwargs['task_id'],kwargs['command_id']))
        yield
        if len(identities)==1:
            r.node.set_confirmed_target_reference_position((2.,0.,0.))
            raise RuntimeError('planner timeout')
        assert r.node.target_reference_position is not None
        return 'TARGET_APPROACHED'
    monkeypatch.setattr(exploration,'_explore_steps',run)
    events=[]
    steps=exploration.explore_steps(r.adapter,{},r.adapter.frame,101.,'cup',events.append,lambda:[],None,lambda:None,
        clock=lambda:r.clock[0],robot_id='ranger',target_id='B')
    next(steps);next(steps)
    r.clock[0]+=40;r.frame();next(steps)
    with pytest.raises(StopIteration) as done:next(steps)
    assert done.value.value=='TARGET_APPROACHED' and identities[0]==identities[1]
    assert any(e.get('reason')=='planner timeout' for e in events)


def test_cancel_timeout_only_clears_after_native_terminal(local_runtime):
    r=local_runtime;r.frame();r.adapter.costmap_transform=lambda **_:S()
    r.node.send_spin(.1,'depth_reobserve')
    r.adapter.motion_fault=True;r.adapter.motion_fault_reason='cancel_timeout'
    steps=r.adapter.wait_for_experiment_recovery('cancel timeout',lambda _:None,lambda:[],lambda:r.clock[0])
    next(steps);r.clock[0]+=40;r.frame();next(steps)
    assert r.adapter.motion_fault and len(r.requests)==1
    r.terminals[-1].set_result(S(status=5))
    with pytest.raises(StopIteration):next(steps)
    assert not r.adapter.motion_fault and len(r.requests)==1


def test_sensor_recovery_requires_original_fresh_frame_count(local_runtime):
    r=local_runtime;r.adapter.costmap_transform=lambda **_:S()
    r.node.latest_snapshot=None
    r.node.p.sensor_recovery_frames=3
    r.node.vlm_input_ready=lambda _:r.node.latest_snapshot is not None
    steps=r.adapter.wait_for_experiment_recovery('Sensor unavailable',lambda _:None,lambda:[],lambda:r.clock[0])
    next(steps);r.clock[0]+=40
    for _ in range(2):r.frame();next(steps)
    r.frame()
    with pytest.raises(StopIteration):next(steps)


def test_recovery_resume_scan_retains_original_view_and_budget(local_runtime):
    r=local_runtime;r.frame();r.adapter.costmap_transform=lambda **_:S()
    r.adapter.scan_viewpoint_id='P1'
    r.adapter.start_scan({'B':'cup'},'scan',stop_on_target='B',current_view_only=True)
    r.adapter.scan.update(index=2,initial=.7,depth_reobserve_attempts=3,current_view_only=False,headings=[0.,1.,2.,3.])
    steps=r.adapter.wait_for_experiment_recovery('spin failed',lambda _:None,lambda:[],lambda:r.clock[0])
    next(steps);r.clock[0]+=40;r.frame()
    with pytest.raises(StopIteration):next(steps)
    r.adapter.start_scan({'B':'cup'},'resume',stop_on_target='B',current_view_only=True)
    assert r.adapter.scan['index']==2 and r.adapter.scan['initial']==.7
    assert r.adapter.scan['depth_reobserve_attempts']==3 and not r.adapter.scan['current_view_only']


def test_rebuilding_failed_approach_keeps_used_retry_budget(local_runtime):
    from multi_agent_nav.models import Pose2D
    r=local_runtime
    def approach():return r.adapter.approach_steps(Pose2D(2.,0.),'task','command','PICKUP_NAV',lambda _:None,lambda:r.clock[0])
    steps=approach();next(steps)
    a=r.adapter.approach;a.arrival_retry_count=1
    a.fail_safe('Navigation timed out')
    with pytest.raises(StopIteration):next(steps)
    r.terminals[-1].set_result(S(status=5))
    steps=approach();next(steps)
    assert r.adapter.approach.arrival_retry_count==1
    steps.close()


@pytest.mark.parametrize('phase',['SPIN','SPIN_WAIT'])
def test_scan_resume_never_skips_pending_heading(local_runtime,phase):
    r=local_runtime;r.frame();r.adapter.costmap_transform=lambda **_:S()
    r.adapter.start_scan({'B':'cup'},'scan',current_view_only=True)
    r.adapter.scan.update(phase=phase,index=2,initial=.7,headings=[0.,1.,2.],current_view_only=False)
    steps=r.adapter.wait_for_experiment_recovery('spin failed',lambda _:None,lambda:[],lambda:r.clock[0])
    next(steps);r.clock[0]+=40;r.frame()
    with pytest.raises(StopIteration):next(steps)
    r.adapter.start_scan({'B':'cup'},'resume',current_view_only=True)
    assert r.adapter.scan['index']==2 and r.adapter.scan['phase']=='SPIN'


def test_scan_spin_abort_preserves_failed_scan_checkpoint(local_runtime):
    r=local_runtime;r.frame()
    reply=r.adapter.start_scan({'B':'cup'},'scan',current_view_only=False)
    r.adapter.scan.update(index=2,headings=[0.,.5,1.],initial=0.)
    r.adapter.tick()
    r.terminals[-1].set_result(S(status=6));r.adapter.tick()
    assert reply.done() and reply.exception() is not None
    assert r.adapter.failed_scan['index']==2


def test_changed_task_identity_does_not_restart_or_cancel_new_task(local_runtime,monkeypatch):
    import multi_agent_nav.go2_exploration as exploration
    r=local_runtime
    def run(*args,**kwargs):
        yield
        r.node.execution_identity=('new-task','new-command')
        raise RuntimeError('new owner')
    monkeypatch.setattr(exploration,'_explore_steps',run)
    steps=exploration.explore_steps(r.adapter,{},r.adapter.frame,101.,'cup',lambda _:None,lambda:[],None,lambda:None,
        robot_id='ranger',target_id='B')
    next(steps)
    with pytest.raises(RuntimeError,match='new owner'):next(steps)
    assert r.node.execution_active and r.node.execution_identity==('new-task','new-command')


def test_stop_setup_error_retries_cancellation_and_always_publishes_zero(local_runtime):
    r=local_runtime;r.frame();r.adapter.costmap_transform=lambda **_:S()
    calls=[];original=r.node.cancel_motion
    def cancel(**kwargs):
        calls.append('cancel')
        if calls.count('cancel')==1:raise RuntimeError('cancel transport failed')
        original(**kwargs)
    r.node.cancel_motion=cancel
    r.node.publish_stop=lambda:calls.append('zero')
    wait=lambda:r.adapter.wait_for_experiment_recovery('fault',lambda _:None,lambda:[],lambda:r.clock[0])
    with pytest.raises(RuntimeError,match='cancel transport'):next(wait())
    steps=wait();next(steps)
    assert calls.count('cancel')==2 and 'zero' in calls
    steps.close()


def test_result_error_after_cancel_timeout_never_fakes_terminal(local_runtime):
    r=local_runtime;r.frame();r.adapter.costmap_transform=lambda **_:S()
    r.node.send_spin(.1,'depth_reobserve')
    r.adapter._set_motion_fault('cancel_timeout')
    r.terminals[-1].set_exception(RuntimeError('lost result'))
    steps=r.adapter.wait_for_experiment_recovery('failure',lambda _:None,lambda:[],lambda:r.clock[0])
    next(steps);r.clock[0]+=40;r.frame();next(steps)
    assert r.adapter.motion_fault and len(r.requests)==1
    steps.close()


def test_recovery_callback_errors_yield_to_ros_instead_of_busy_loop(local_runtime,monkeypatch):
    import multi_agent_nav.go2_exploration as exploration
    r=local_runtime;calls=[]
    def run(*args,**kwargs):
        yield
        raise RuntimeError('original fault')
    def wait(*args,**kwargs):
        calls.append(1)
        if len(calls)<3:raise RuntimeError('health callback failed')
        yield
    monkeypatch.setattr(exploration,'_explore_steps',run)
    r.adapter.wait_for_experiment_recovery=wait
    steps=exploration.explore_steps(r.adapter,{},r.adapter.frame,101.,'cup',lambda _:None,lambda:[],None,lambda:None,
        robot_id='ranger',target_id='B')
    next(steps);next(steps)
    assert len(calls)==1
    steps.close()


def test_new_scan_at_same_point_does_not_reset_three_rotation_budget(local_runtime):
    r=local_runtime;r.frame()
    r.adapter.scan_viewpoint_id='same-point'
    first=r.adapter.start_scan({'B':'cup'},'first',current_view_only=True)
    r.node.rotate_depth_reobservation('depth invalid')
    r.terminals[-1].set_result(S(status=4))
    r.adapter.hold_motion('temporary interruption')
    r.adapter.start_scan({'B':'cup'},'second',current_view_only=True)
    assert r.adapter.scan['depth_reobserve_attempts']==1


def test_scan_without_local_execution_keeps_existing_entry_supported(local_runtime):
    r=local_runtime;r.frame();r.node.end_execution()
    del r.adapter.depth_reobserve_budgets
    r.adapter.start_scan({'B':'cup'},'read-only-scan',current_view_only=True)
    assert r.adapter.scan['depth_reobserve_attempts']==0


@pytest.mark.parametrize('fault',['localization','evidence'])
def test_internal_scan_fault_preserves_execution_for_recovery(local_runtime,fault):
    r=local_runtime;r.frame();r.adapter.experiment_recovery_enabled=True
    r.adapter.start_scan({'B':'cup'},'scan',current_view_only=True)
    r.adapter.scan['index']=2
    if fault=='localization':
        health=r.adapter.get_health
        r.adapter.get_health=lambda:replace(health(),localization_valid=False)
    else:
        r.adapter._scan_evidence=S(write_failed=True)
    with pytest.raises(RuntimeError):r.adapter.tick()
    assert r.node.execution_active and r.adapter.failed_scan['index']==2
    r.adapter.stop_motion('user canceled')
    assert not r.node.execution_active


@pytest.mark.parametrize('phase',['FRAME','SPIN_WAIT'])
def test_short_sensor_wait_retains_scan_direction(local_runtime,phase):
    r=local_runtime;r.adapter.experiment_recovery_enabled=True;r.frame()
    r.adapter.start_scan({'B':'cup'},'scan',current_view_only=True)
    r.adapter.scan.update(index=4,initial=.7,phase=phase,current_view_only=False)
    r.node.enter_sensor_wait('RGB-D gap')
    assert r.node.recover_from_sensor_wait()
    assert r.adapter.scan['index']==4 and r.adapter.scan['initial']==.7
    assert r.adapter.scan['phase']==('SPIN' if phase=='SPIN_WAIT' else 'FRAME')


def test_ros_callback_exception_is_routed_into_same_experiment_recovery(local_runtime,monkeypatch):
    import multi_agent_nav.go2_exploration as exploration
    r=local_runtime;attempts=[];spins=[]
    def run(*args,**kwargs):
        attempts.append(kwargs['task_id'])
        yield
        return 'TARGET_APPROACHED'
    def wait(reason,*args):
        assert reason=='temporary callback failure'
        yield
    def spin():
        spins.append(1)
        if len(spins)==1:raise RuntimeError('temporary callback failure')
    monkeypatch.setattr(exploration,'_explore_steps',run)
    r.adapter.wait_for_experiment_recovery=wait
    outcome=exploration.explore(r.adapter,spin,{},r.adapter.frame,101.,'cup',lambda _:None,lambda:[],None,lambda:None,
        clock=lambda:r.clock[0],robot_id='ranger',target_id='B')
    assert outcome=='TARGET_APPROACHED' and len(attempts)==2 and attempts[0]==attempts[1]
