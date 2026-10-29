from dataclasses import replace

import pytest

from multi_agent_nav.mock import MockMission
from test_private_c import local_agent, private_config


def test_nav_ready_resume_requires_stop_terminal_and_valid_localization():
    _, machine, adapter, command = local_agent()
    assert machine.accept(command, 0).accepted
    machine.take_effects()
    health = adapter.get_health()
    effects = machine.tick(.1, replace(health, nav_ready=False, stopped=False, motion_terminal=False))
    assert [e.kind for e in effects] == ['STOP']
    for stopped, terminal in [(False, True), (True, False)]:
        assert not machine.tick(.2, replace(health, stopped=stopped, motion_terminal=terminal))
        assert machine.paused
    machine.tick(.3, health)
    effects = machine.tick(.4, health)
    assert not machine.paused
    assert [e.kind for e in effects] == ['LOCAL_EXECUTE']
    assert machine.command == command and machine.state().reason == ''
    assert not machine.tick(.5, health)


@pytest.mark.parametrize('fault', ['localization', 'manual'])
def test_new_fault_during_nav_pause_prevents_automatic_resume(fault):
    _, machine, adapter, command = local_agent()
    machine.accept(command, 0); machine.take_effects()
    health = adapter.get_health()
    machine.tick(.1, replace(health, nav_ready=False))
    if fault == 'localization':
        machine.tick(.2, replace(health, localization_valid=False))
    else:
        machine.accept(replace(command, action='PAUSE'), .2)
    machine.sync_received = machine.peer_received = 4.1
    machine.tick(4.1, health)
    machine.tick(4.2, health)
    assert machine.paused


@pytest.mark.parametrize('manual', [False, True])
def test_private_mission_nav_pause_recovers_same_task_or_obeys_manual_pause(manual):
    mission = MockMission(private_config())
    assert mission.start().accepted
    mission.run(4)
    task_id = mission.coordinator.task_id
    adapter = mission.adapters['ranger']
    get_health = adapter.get_health
    adapter.get_health = lambda: replace(get_health(), nav_ready=False)
    mission.run(5)
    assert mission.coordinator.phase == 'PAUSED'
    assert all(m.paused for m in mission.machines.values())
    if manual:
        mission.pause('explicit local pause')
    adapter.get_health = get_health
    mission.run(200)
    assert mission.coordinator.task_id == task_id
    if manual:
        assert mission.coordinator.phase == 'PAUSED'
        assert all(m.paused for m in mission.machines.values())
    else:
        assert mission.coordinator.phase == 'DONE'
        assert all(m.state().deposited and not m.paused for m in mission.machines.values())


def test_resume_does_not_repeat_completed_command():
    _, machine, adapter, command = local_agent()
    machine.accept(command, 0); machine.take_effects()
    machine._state.reply='SUCCEEDED'; machine._state.run_state='IDLE'; machine._state.ready=True
    health=adapter.get_health()
    machine.tick(.1, replace(health,nav_ready=False))
    assert not machine.tick(.2,health)
    assert not machine.paused and machine.state().ready
    assert machine.state().reply=='SUCCEEDED' and machine.state().run_state=='IDLE'


@pytest.mark.parametrize('fault', ['localization', 'timeout'])
def test_coordinator_second_fault_during_nav_pause_is_latched(fault):
    mission=MockMission(private_config()); assert mission.start().accepted
    mission.run(4)
    adapter=mission.adapters['ranger']; get_health=adapter.get_health
    adapter.get_health=lambda:replace(get_health(),nav_ready=False)
    mission.run(5)
    if fault=='localization':
        adapter.get_health=lambda:replace(get_health(),localization_valid=False)
        mission.run(2)
    else:
        mission.coordinator.task_timeout=mission.now-.1; mission.step()
    adapter.get_health=get_health
    mission.run(30)
    assert mission.coordinator.phase=='PAUSED'
    assert all(m.paused for m in mission.machines.values())


def test_c_approach_recovers_with_original_observation_and_task():
    mission=MockMission(private_config()); assert mission.start().accepted
    for _ in range(200):
        mission.step()
        if mission.coordinator.phase=='DEPOSIT' and mission.coordinator.commands:break
    assert mission.coordinator.phase=='DEPOSIT'
    commands=dict(mission.coordinator.commands)
    adapter=mission.adapters['ranger']; get_health=adapter.get_health
    adapter.get_health=lambda:replace(get_health(),nav_ready=False)
    mission.run(5)
    assert mission.coordinator.phase=='PAUSED'
    assert mission.coordinator.commands==commands
    adapter.get_health=get_health
    mission.run(200)
    assert mission.coordinator.phase=='DONE'


def test_shared_coordinate_nav_pause_remains_manual():
    mission=MockMission(); assert mission.start().accepted
    mission.run(4)
    adapter=mission.adapters['ranger']; get_health=adapter.get_health
    adapter.get_health=lambda:replace(get_health(),nav_ready=False)
    mission.run(5)
    adapter.get_health=get_health
    mission.run(50)
    assert mission.coordinator.phase=='PAUSED'
    assert all(m.paused for m in mission.machines.values())


def test_resumed_navigation_waits_for_new_path_without_changing_candidate():
    from multi_agent_nav.go2_exploration import explore_steps, ExplorationStopped
    from multi_agent_nav.frame_adapter import FrameAdapter
    from test_go2_exploration import LocalFixture
    adapter=LocalFixture(); events=[]; now=[0.]
    config={'robot_go2':{'stop_margin':.1,'scan_angles':[0.],'path_start_timeout':1.},
            'scene':{'resources':{},'viewpoints':{'go2':[{'id':'p1','pose':[1.,0.,0.]}]}}}
    steps=explore_steps(adapter,config,FrameAdapter('g','v','lab_ref','a',[0,0,0]),
        30.,'cup',events.append,lambda:[],lambda:True,lambda:None,clock=lambda:now[0])
    for _ in range(10):
        next(steps)
        if adapter.active:break
    next(steps)  # original executing path has been consumed
    now[0]=5.
    adapter.navigation_resume_sequence=1
    health=adapter.get_health
    adapter.get_health=lambda:replace(health(),stopped=True,path_valid=False,path=[])
    next(steps)
    assert [e['viewpoint_id'] for e in events if e['state']=='SELECTED']==['p1']
    now[0]=6.1
    with pytest.raises(ExplorationStopped):next(steps)


def test_explicit_new_task_discards_old_suspended_execution():
    _, machine, adapter, command=local_agent()
    machine.accept(command,0); machine.take_effects()
    machine.tick(.1,replace(adapter.get_health(),nav_ready=False))
    machine.local_suspended=True
    sync=replace(command,task_id='new-task',action='SYNC',phase='SYNCING',kind='SYNC_LOCAL_C',
                 command_id='',sequence=0,target_id='')
    assert machine.accept(sync,.2).accepted
    effects=machine.take_effects()
    assert len(effects)==1 and effects[0].kind=='STOP'
    assert machine.context.task_id=='new-task' and machine.command is None


@pytest.mark.parametrize('new_fault',['nav','stopping','manual','moving'])
def test_resumed_agent_waits_for_peer_state_ack_without_pause_loop(new_fault):
    _, machine, adapter, command=local_agent()
    machine.accept(command,0); machine.take_effects()
    health=adapter.get_health()
    machine.tick(.1,replace(health,nav_ready=False))
    machine.peer_state=replace(machine.peer_state,run_state='PAUSED',reason='coordinator nav_ready pause')
    machine.tick(.2,health)
    assert not machine.paused
    machine.tick(.3,health)
    assert not machine.paused  # peer's post-resume state has not arrived yet
    changes={'nav':dict(nav_ready=False),'stopping':dict(run_state='STOPPING',stopped=False),
             'manual':dict(reason='coordinator pause/cancel'),'moving':dict(stopped=False)}
    machine.peer_state=replace(machine.peer_state,**changes[new_fault])
    machine.tick(.4,health)
    assert machine.paused  # a fresh health loss is still fatal to motion


def test_nav_resume_waits_for_peer_to_finish_stopping():
    _, machine, adapter, command=local_agent()
    machine.accept(command,0); machine.take_effects()
    health=adapter.get_health()
    machine.tick(.1,replace(health,nav_ready=False))
    machine.peer_state=replace(machine.peer_state,run_state='STOPPING',stopped=False,
                               reason='coordinator nav_ready pause')
    machine.tick(.2,health)
    assert machine.paused
    machine.peer_state=replace(machine.peer_state,run_state='PAUSED',stopped=True)
    machine.tick(.3,health)
    assert not machine.paused


@pytest.mark.parametrize('lost', ['peer', 'coordinator', 'both'])
def test_heartbeat_pause_requires_both_fresh_heartbeats_before_resume(lost):
    config, machine, adapter, command = local_agent()
    assert machine.accept(command, 0).accepted
    machine.take_effects()
    health = adapter.get_health()
    if lost == 'peer': machine.sync_received = 4.
    if lost == 'coordinator': machine.peer_received = 4.
    effects = machine.tick(4., health)
    assert machine.paused and [e.kind for e in effects] == ['STOP']
    machine.tick(4.1, health)
    assert machine.paused  # Healthy local sensors cannot bypass missing heartbeats.
    machine.accept(replace(command, action='SYNC', kind='SYNC_LOCAL_C',
                           command_id='', sequence=0, target_id=''), 4.2)
    if lost != 'coordinator':
        machine.tick(4.2, health)
        assert machine.paused
    peer = replace(machine.peer_state, run_state='PAUSED', reason='peer heartbeat timeout')
    machine.receive_peer(peer, config['alignment']['robots']['go2'], 4.3)
    effects = machine.tick(4.3, health)
    assert not machine.paused and machine.command == command
    assert not effects
    machine.receive_peer(replace(peer, run_state='IDLE', reason=''),
                         config['alignment']['robots']['go2'], 4.4)
    assert [e.kind for e in machine.tick(4.4, health)] == ['LOCAL_EXECUTE']
    assert not machine.tick(4.5, health)  # Resumption dispatches the command only once.


@pytest.mark.parametrize('outage', ['ranger', 'go2', 'both', 'sync'])
def test_private_mission_heartbeat_loss_pauses_both_and_resumes_same_task(outage):
    mission = MockMission(private_config()); assert mission.start().accepted
    mission.run(4)
    task_id = mission.coordinator.task_id
    if outage == 'sync':
        accepts = {r: m.accept for r, m in mission.machines.items()}
        for robot, machine in mission.machines.items():
            accept = accepts[robot]
            machine.accept = lambda c, t, accept=accept: None if c.action == 'SYNC' else accept(c, t)
    else:
        for robot in mission.online:
            mission.online[robot] = outage != 'both' and robot != outage
    mission.run(45)
    assert all(m.paused for m in mission.machines.values())
    assert mission.coordinator.nav_pause_phase is not None
    assert all(not m.state().goal_id for m in mission.machines.values())
    if outage == 'sync':
        for robot, machine in mission.machines.items(): machine.accept = accepts[robot]
    else:
        mission.online = {r: True for r in mission.online}
    mission.run(200)
    assert mission.coordinator.task_id == task_id
    assert mission.coordinator.phase == 'DONE'
    assert all(m.state().deposited and not m.paused for m in mission.machines.values())


@pytest.mark.parametrize('fault', ['nav', 'localization', 'manual'])
def test_heartbeat_recovery_does_not_override_other_faults(fault):
    config, machine, adapter, command = local_agent()
    machine.accept(command, 0); machine.take_effects()
    health = adapter.get_health()
    machine.tick(4., health)
    machine.accept(replace(command, action='SYNC', kind='SYNC_LOCAL_C',
                           command_id='', sequence=0, target_id=''), 4.1)
    machine.receive_peer(machine.peer_state, config['alignment']['robots']['go2'], 4.1)
    if fault == 'manual': machine.accept(replace(command, action='PAUSE'), 4.1)
    current = replace(health, **({'nav_ready': False} if fault == 'nav' else
                                {'localization_valid': False} if fault == 'localization' else {}))
    machine.tick(4.2, current)
    assert machine.paused
    machine.tick(4.3, health)
    assert machine.paused == (fault != 'nav')


def test_heartbeat_outage_during_nav_pause_remains_recoverable():
    mission = MockMission(private_config()); assert mission.start().accepted
    mission.run(4)
    adapter = mission.adapters['ranger']; healthy = adapter.get_health
    adapter.get_health = lambda: replace(healthy(), nav_ready=False)
    mission.run(5)
    mission.online['ranger'] = False
    mission.run(40)
    assert mission.coordinator.phase == 'PAUSED'
    mission.online['ranger'] = True
    mission.run(5)
    assert all(m.paused for m in mission.machines.values())
    adapter.get_health = healthy
    mission.run(200)
    assert mission.coordinator.phase == 'DONE'


def test_resume_publishes_active_state_before_restarting_motion():
    config, machine, adapter, command = local_agent()
    machine.accept(command, 0); machine.take_effects()
    healthy = adapter.get_health()
    machine.tick(.1, replace(healthy, nav_ready=False))
    peer = replace(machine.peer_state, run_state='PAUSED', stopped=True,
                   reason='peer heartbeat lost for current task')
    machine.receive_peer(peer, config['alignment']['robots']['go2'], .2)
    assert not machine.tick(.2, healthy)  # No native restart until the peer sees us active.
    assert not machine.paused and machine.state().run_state not in ('PAUSED', 'STOPPING')
    assert machine.state().stopped
    assert not machine.tick(.3, healthy)
    machine.receive_peer(replace(peer, run_state='LOCAL_EXECUTING', reason=''),
                         config['alignment']['robots']['go2'], .4)
    assert [e.kind for e in machine.tick(.4, healthy)] == ['LOCAL_EXECUTE']
    assert not machine.tick(.5, healthy)


@pytest.mark.parametrize('fault', ['peer', 'coordinator'])
def test_one_way_heartbeat_loss_cannot_resume_from_other_side_ready(fault):
    config, machine, adapter, command = local_agent()
    machine.accept(command, 0); machine.take_effects()
    healthy = adapter.get_health()
    machine.tick(4., healthy)
    if fault == 'peer': machine.sync_received = 4.2
    else: machine.receive_peer(machine.peer_state, config['alignment']['robots']['go2'], 4.2)
    assert not machine.tick(4.2, healthy)
    assert machine.paused


def test_localization_failure_during_heartbeat_outage_is_latched():
    config, machine, adapter, command = local_agent()
    machine.accept(command, 0); machine.take_effects()
    healthy = adapter.get_health()
    machine.tick(4., healthy)
    machine.tick(4.1, replace(healthy, localization_valid=False))
    machine.sync_received = 4.2
    machine.receive_peer(machine.peer_state, config['alignment']['robots']['go2'], 4.2)
    assert not machine.tick(4.2, healthy)
    assert machine.paused and 'localization_valid' in machine.state().reason


def test_next_command_waits_for_peer_resume_ack_after_completed_command():
    config, machine, adapter, command = local_agent()
    machine.accept(command, 0); machine.take_effects()
    machine._state.reply='SUCCEEDED'; machine._state.run_state='IDLE'; machine._state.ready=True
    healthy=adapter.get_health()
    machine.tick(.1, replace(healthy, nav_ready=False))
    peer=replace(machine.peer_state,run_state='PAUSED',reason='coordinator recoverable pause')
    machine.receive_peer(peer,config['alignment']['robots']['go2'],.2)
    machine.tick(.2,healthy)
    machine.phase='SEARCH_C'
    command=replace(command,phase='SEARCH_C',kind='SEARCH_VIEWPOINT',target_id='C',sequence=2,command_id='c')
    assert not machine.accept(command,.3).accepted
    machine.receive_peer(replace(peer,run_state='IDLE',reason=''),config['alignment']['robots']['go2'],.4)
    assert machine.accept(command,.4).accepted


@pytest.mark.parametrize('fault', ['nav', 'costmap', 'localization', 'owner'])
def test_health_first_lost_inside_exploration_has_correct_pause_class(fault):
    from multi_agent_nav.runner import AgentRunner
    from multi_agent_nav.models import is_recoverable_pause
    config, machine, adapter, command = local_agent()
    runner=AgentRunner(machine,adapter,config['scene']['targets'],lambda _:None,config=config)
    machine.accept(command,0)
    runner.tick(0)  # The first yield precedes exploration's own health sampling.
    healthy=adapter.get_health
    adapter.get_health=lambda:replace(healthy(),nav_ready=fault!='nav',localization_valid=fault!='localization')
    if fault=='costmap':adapter.costmap_transform=lambda **kwargs:None
    if fault=='owner':runner.owners=lambda:['other_owner']
    for _ in range(10):
        runner._advance_local()
        if machine.paused:break
    assert machine.paused
    assert is_recoverable_pause(machine.state().reason)==(fault in ('nav','costmap'))
    assert not adapter.calls


@pytest.mark.parametrize('fault', ['nav', 'peer', 'coordinator', 'localization', 'owner'])
def test_health_first_lost_at_motion_guard_has_correct_pause_class(fault):
    from multi_agent_nav.runner import AgentRunner
    from multi_agent_nav.models import is_recoverable_pause
    config, machine, adapter, command = local_agent()
    runner=AgentRunner(machine,adapter,config['scene']['targets'],lambda _:None,config=config)
    machine.accept(command,0);machine.take_effects()
    def steps():
        runner.motion_guard()
        yield
    runner.local_steps=steps();runner.local_identity=(command.task_id,command.command_id)
    healthy=adapter.get_health
    adapter.get_health=lambda:replace(healthy(),nav_ready=fault!='nav',localization_valid=fault!='localization')
    if fault=='owner':runner.owners=lambda:['other_owner']
    if fault=='peer':machine.peer_received=-4.
    if fault=='coordinator':machine.sync_received=-4.
    runner._advance_local()
    assert machine.paused
    assert is_recoverable_pause(machine.state().reason)==(fault in ('nav','peer','coordinator'))
    assert not adapter.calls


def test_recovery_waits_for_active_peer_to_stop_too():
    config, machine, adapter, command = local_agent()
    machine.accept(command,0);machine.take_effects()
    healthy=adapter.get_health()
    machine.tick(.1,replace(healthy,nav_ready=False))
    peer=replace(machine.peer_state,run_state='NAVIGATING',stopped=False)
    machine.receive_peer(peer,config['alignment']['robots']['go2'],.2)
    assert not machine.tick(.2,healthy)
    assert machine.paused
    machine.receive_peer(replace(peer,run_state='IDLE',stopped=True),config['alignment']['robots']['go2'],.3)
    assert [e.kind for e in machine.tick(.3,healthy)]==['LOCAL_EXECUTE']
