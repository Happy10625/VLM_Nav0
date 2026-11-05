from dataclasses import replace
import pytest
from multi_agent_nav.models import Health, TaskClock, is_recoverable_pause
from multi_agent_nav.mock import MockMission, make_machine
from test_private_c import local_agent, private_config


def enabled_agent():
    config,m,a,c=local_agent()
    m.context=replace(m.context,wait_recovery=True)
    m._task_clock.reset(0.)
    m.accept(c,0.);m.take_effects()
    return config,m,a,c


def test_profile_switch_reaches_machine_and_rejects_non_boolean():
    config=private_config();config['robot_ranger']['wait_recovery']=True
    m,_=make_machine(config,'ranger',True)
    assert m.context.wait_recovery
    config['robot_ranger']['wait_recovery']='false'
    with pytest.raises(ValueError,match='boolean'):make_machine(config,'ranger',True)


@pytest.mark.parametrize('stage',['STOPPING','PAUSED','awaiting_peer_resume'])
def test_task_budget_freezes_until_both_agents_resume(stage):
    _,m,a,c=enabled_agent();h=a.get_health();m.task_timeout=1.
    m.tick(.1,replace(h,nav_ready=False,stopped=False,motion_terminal=False))
    if stage=='PAUSED':m.tick(.2,h)
    if stage=='awaiting_peer_resume':
        m.tick(.2,h);m.paused=False;m.awaiting_peer_resume=True
    m.sync_received=m.peer_received=1270.
    m.tick(1270.,replace(h,stopped=False,motion_terminal=False))
    assert m.state().reason!='task timeout'
    assert m.task_clock(1270.)-m.task_started_at<1.


def test_temporary_localization_can_resume_but_hard_fault_wins():
    _,m,a,c=enabled_agent();h=a.get_health()
    m.tick(.1,replace(h,localization_valid=False))
    assert m.state().reason=='local localization temporarily unavailable'
    m.sync_received=m.peer_received=.2
    m.tick(.2,h);m.tick(.3,h)
    assert not m.paused
    m.tick(.4,replace(h,localization_valid=False,hard_fault='action_result_error'))
    assert m.state().reason=='action_result_error'
    m.tick(.5,h);m.tick(.6,h)
    assert m.paused


def test_received_hard_peer_state_wins_over_local_soft_fault():
    _,m,a,c=enabled_agent();h=a.get_health()
    m.peer_state=replace(m.peer_state,run_state='PAUSED',reason='manual pause',localization_valid=False)
    m.tick(.1,replace(h,localization_valid=False))
    assert not is_recoverable_pause(m.state().reason)
    m.peer_state=replace(m.peer_state,run_state='IDLE',reason='',localization_valid=True)
    m.tick(.2,h);m.tick(.3,h)
    assert m.paused


def test_resume_requires_sync_newer_than_pause():
    _,m,a,c=enabled_agent();h=a.get_health()
    m.tick(.1,replace(h,nav_ready=False));m.tick(.2,h);m.tick(.3,h)
    assert m.paused
    m.sync_received=.4;m.tick(.4,h)
    assert not m.paused


def test_long_dual_pause_and_localization_recover_same_task():
    config=private_config();config['scene'].update(wait_recovery=True,task_timeout=30.)
    for robot in ('go2','ranger'):config['robot_'+robot]['wait_recovery']=True
    mission=MockMission(config);assert mission.start().accepted;mission.run(4)
    task=mission.coordinator.task_id
    adapter=mission.adapters['ranger'];get_health=adapter.get_health
    adapter.get_health=lambda:replace(get_health(),localization_valid=False)
    mission.run(12700)
    assert mission.coordinator.phase=='PAUSED' and mission.coordinator.nav_pause_phase
    assert all(m.state().reason!='task timeout' for m in mission.machines.values())
    adapter.get_health=get_health;mission.run(300)
    assert mission.coordinator.task_id==task and mission.coordinator.phase=='DONE'

@pytest.mark.parametrize('message,allowed',[
    ('APPROACH_FAILED: approach goal failed with status 6',True),
    ('APPROACH_FAILED: approach goal failed with status 5',False),
    ('scan spin failed: view 2, action ABORTED',True),
    ('scan spin failed: view 2, action CANCELED',False),
    ('local map/candidate assets changed during task',False),
    ('candidate planning timeout',True),
    ('unclassified defect',False),
])
def test_execution_failure_classification_is_explicit(message,allowed):
    from multi_agent_nav.go2_exploration import ExplorationStopped,is_recoverable_execution_error
    error=RuntimeError(message) if message.startswith('scan spin') else ExplorationStopped(message)
    assert is_recoverable_execution_error(error) is allowed
    error.__cause__=TypeError('programming error')
    assert not is_recoverable_execution_error(error)


def test_motion_boundary_blocks_pending_peer_ack_and_received_hard_fault():
    from multi_agent_nav.runner import AgentRunner
    config,m,a,c=enabled_agent()
    runner=AgentRunner(m,a,{'A':'A','B':'B','C':'C'},lambda _:None,config=config)
    runner.now=.1;m.awaiting_peer_resume=True
    with pytest.raises(Exception,match='peer.*recovery'):runner.motion_guard()
    m.awaiting_peer_resume=False
    m.peer_state=replace(m.peer_state,run_state='PAUSED',reason='manual pause')
    with pytest.raises(RuntimeError,match='peer paused'):runner.motion_guard()


@pytest.mark.parametrize('kind',['scan','depth_reobserve'])
def test_ranger_confirmed_spin_abort_is_retryable_but_unknown_result_is_not(kind):
    from multi_agent_nav.go2_exploration import is_recoverable_execution_error
    assert is_recoverable_execution_error(RuntimeError(f'{kind} goal failed with status 6'))
    assert not is_recoverable_execution_error(RuntimeError(f'{kind} goal failed with status 5'))
    assert not is_recoverable_execution_error(RuntimeError(f'Cannot obtain {kind} goal result: lost'))

@pytest.mark.parametrize('fault',['local_hard','peer_identity','peer_manual'])
def test_hard_fault_revokes_first_execute_effect_before_dispatch(fault):
    config,m,a,c=local_agent();m.context=replace(m.context,wait_recovery=True)
    assert m.accept(c,0.).accepted
    h=a.get_health()
    assert any(e.kind=='LOCAL_EXECUTE' for e in m.effects)
    if fault=='local_hard':h=replace(h,hard_fault='other navigation owner present')
    else:
        peer=replace(m.peer_state,**({'map_version':'wrong'} if fault=='peer_identity' else
            {'run_state':'PAUSED','reason':'operator stop'}))
        m.receive_peer(peer,config['alignment']['robots']['go2'],.1)
    effects=m.tick(.1,h)
    assert [e.kind for e in effects]==['STOP']
    assert m.paused and not is_recoverable_pause(m.state().reason)
