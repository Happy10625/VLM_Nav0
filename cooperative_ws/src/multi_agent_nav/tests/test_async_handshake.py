"""Asynchronous private startup and current-task acknowledgments, without ROS."""
from dataclasses import replace

import pytest

from multi_agent_nav.mock import make_machine, MockMission
from multi_agent_nav.models import AgentState, Command, Health, Pose2D, ROBOTS
from test_private_c import private_config


def pair():
    cfg=private_config()
    machines={r:make_machine(cfg,r,True)[0] for r in ROBOTS}
    health=Health(Pose2D(),True,True,True,motion_terminal=True)
    return cfg,machines,health


def exchange(cfg,machines,now):
    states={r:m.state() for r,m in machines.items()}
    for robot,machine in machines.items():
        peer='ranger' if robot=='go2' else 'go2'
        machine.receive_peer(states[peer],cfg['alignment']['robots'][peer],now)


def sync(robot,task='t',phase='SYNCING'):
    return Command(task_id=task,recipient_id=robot,action='SYNC_LOCAL_C',phase=phase,
        common_frame_id='',alignment_id='')


@pytest.mark.parametrize('first',ROBOTS)
def test_either_agent_can_wait_sixty_seconds_for_peer(first):
    cfg,machines,health=pair()
    machine=machines[first]
    machine.tick(0,health)
    machine.tick(60,health)
    assert machine.state().run_state=='WAITING_PEER' and not machine.paused
    for m in machines.values():m.tick(60,health)
    exchange(cfg,machines,60)
    for m in machines.values():
        m.tick(60,health)
        assert m.state().run_state=='STANDBY' and not m.state().ready


def test_unhealthy_peer_is_live_and_waiting_recovers_without_restart():
    cfg,machines,health=pair()
    for m in machines.values():m.tick(0,health)
    ranger=machines['ranger'];peer=machines['go2'].state()
    assert ranger.receive_peer(replace(peer,nav_ready=False),cfg['alignment']['robots']['go2'],2)
    assert ranger.peer_received==2 and not ranger.paused
    ranger.tick(2,health)
    assert ranger.state().run_state=='WAITING_PEER'
    assert 'nav_ready' in ranger.state().reason
    assert ranger.receive_peer(peer,cfg['alignment']['robots']['go2'],3)
    ranger.tick(3,health)
    assert ranger.state().run_state=='STANDBY'


def test_local_health_does_not_depend_on_peer_and_waiting_loss_is_recoverable():
    cfg,machines,health=pair()
    for m in machines.values():m.tick(0,health)
    exchange(cfg,machines,0)
    ranger=machines['ranger'];ranger.tick(0,health)
    ranger.tick(1,replace(health,nav_ready=False))
    assert ranger.state().run_state=='PREPARING' and not ranger.paused
    ranger.tick(5,health)
    assert ranger.state().run_state=='WAITING_PEER' and ranger.state().nav_ready
    exchange(cfg,machines,5);ranger.tick(5,health)
    assert ranger.state().run_state=='STANDBY'


def test_task_sync_publishes_identity_before_waiting_for_peer_and_rejects_early_execute():
    cfg,machines,health=pair()
    for m in machines.values():m.tick(0,health)
    exchange(cfg,machines,0)
    for m in machines.values():m.tick(0,health)
    ranger=machines['ranger']
    assert ranger.accept(sync('ranger'),0).accepted
    assert ranger.state().task_id=='t' and ranger.state().run_state=='SYNCING'
    command=replace(sync('ranger',phase='SEARCH_AB'),action='EXECUTE',kind='APPROACH_PICKUP',
        target_id='B',command_id='b',sequence=1)
    assert not ranger.accept(command,0).accepted
    assert ranger.state().reply!='SYNCED'
    assert machines['go2'].accept(sync('go2'),0).accepted
    exchange(cfg,machines,.1)
    for m in machines.values():
        m.tick(.1,health)
        assert m.state().reply=='SYNCED' and not m.state().ready
    assert ranger.accept(sync('ranger',phase='SEARCH_AB'),.2).accepted
    assert ranger.accept(command,.2).accepted


def test_coordinator_waits_for_both_current_synced_replies():
    mission=MockMission(private_config())
    assert mission.start().accepted
    c=mission.coordinator
    assert c.phase=='SYNCING'
    assert not any(x.action=='EXECUTE' for x in c.tick(0))
    for robot in ROBOTS:
        c.ingest_state(replace(c.states[robot],task_id=c.task_id,run_state='SYNCED',reply='SYNCED'),.1)
    c.ingest_state(replace(c.states['ranger'],reply=''),.1)
    assert not any(x.action=='EXECUTE' for x in c.tick(.1))
    c.ingest_state(replace(c.states['ranger'],reply='SYNCED'),.2)
    assert {x.recipient_id for x in c.tick(.2) if x.action=='EXECUTE'}==set(ROBOTS)


def test_old_task_ack_and_boot_state_do_not_unlock_or_refresh_active_task():
    mission=MockMission(private_config());assert mission.start().accepted
    c=mission.coordinator
    for robot in ROBOTS:
        c.ingest_state(replace(c.states[robot],task_id='old',reply='SYNCED'),.1)
    assert not any(x.action=='EXECUTE' for x in c.tick(.1))
    mission.run(4)
    assert c.phase=='SEARCH_AB'
    received=c.received_at['ranger']
    c.ingest_state(replace(c.states['ranger'],task_id=''),mission.now+.1)
    assert c.received_at['ranger']==received
    assert c.phase=='PAUSED' and 'restart' in c.reason


def test_active_health_loss_is_not_misreported_as_heartbeat_loss_and_stays_paused():
    mission=MockMission(private_config());assert mission.start().accepted
    mission.step();mission.step();mission.step();mission.step()
    ranger=mission.machines['ranger'];go2=mission.machines['go2']
    assert ranger.command is not None
    now=mission.now
    ranger.receive_peer(replace(go2.state(),nav_ready=False),mission.config['alignment']['robots']['go2'],now)
    assert ranger.peer_received==now and ranger.paused
    assert 'nav_ready' in ranger.state().reason
    ranger.receive_peer(go2.state(),mission.config['alignment']['robots']['go2'],now+.1)
    ranger.tick(now+.1,mission.adapters['ranger'].get_health())
    assert ranger.paused


def test_handshake_waits_past_three_seconds_until_total_budget_expires():
    mission=MockMission(private_config());assert mission.start().accepted
    c=mission.coordinator
    for robot in ROBOTS:c.ingest_state(c.states[robot],4.)
    output=c.tick(4.)
    assert c.phase=='SYNCING'
    assert not any(x.action=='EXECUTE' for x in output)
    output=c.tick(600.01)
    assert c.phase=='PAUSED' and c.reason=='task timeout'
    assert not any(x.action=='EXECUTE' for x in output)


def test_active_heartbeat_loss_stays_paused_after_messages_recover():
    mission=MockMission(private_config());assert mission.start().accepted
    mission.run(4)
    ranger=mission.machines['ranger'];health=mission.adapters['ranger'].get_health()
    ranger.sync_received=4.
    ranger.tick(4.,health)
    assert ranger.paused and ranger.state().reason=='peer heartbeat timeout'
    peer=replace(mission.machines['go2'].state(),task_id=ranger.context.task_id)
    ranger.receive_peer(peer,mission.config['alignment']['robots']['go2'],4.1)
    assert ranger.paused


def test_peer_boot_state_during_execution_does_not_refresh_task_heartbeat():
    mission=MockMission(private_config());assert mission.start().accepted
    mission.run(4)
    ranger=mission.machines['ranger'];received=ranger.peer_received
    peer=replace(mission.machines['go2'].state(),task_id='')
    assert not ranger.receive_peer(peer,mission.config['alignment']['robots']['go2'],1.)
    assert ranger.peer_received==received and ranger.paused
    assert 'restart' in ranger.state().reason


def test_one_agent_restart_can_join_explicit_new_task_without_restarting_other_agent():
    mission=MockMission(private_config());assert mission.start().accepted
    mission.run(4)
    previous=mission.coordinator.task_id
    robot='go2';peer='ranger'
    replacement,frame=make_machine(mission.config,robot,True)
    from multi_agent_nav.runner import AgentRunner
    adapter=mission.adapters[robot]
    adapter.active.clear()  # Fake process replacement; no live hardware is involved.
    replacement.tick(mission.now,adapter.get_health())
    mission.machines[robot]=replacement
    mission.runners[robot]=AgentRunner(replacement,adapter,mission.config['scene']['targets'],
        lambda o:mission.coordinator.ingest_observation(o,mission.now),config=mission.config)
    mission.run(10)
    assert mission.coordinator.phase=='PAUSED' and mission.machines[peer].paused
    assert replacement.state().run_state=='WAITING_PEER'
    assert mission.start().accepted
    assert mission.coordinator.task_id!=previous
    mission.run(100)
    assert mission.coordinator.phase=='DONE',mission.coordinator.reason


@pytest.mark.parametrize('peer_moves_before_phase_sync',[False,True])
@pytest.mark.parametrize('late_old_state',[False,True])
def test_committed_handshake_survives_peer_starting_first(peer_moves_before_phase_sync,late_old_state):
    cfg,machines,health=pair()
    for m in machines.values():m.tick(0,health)
    for robot,m in machines.items():assert m.accept(sync(robot),0).accepted
    exchange(cfg,machines,.1)
    for m in machines.values():m.tick(.1,health)
    ranger=machines['ranger'];go2=machines['go2']
    if late_old_state:
        ranger.receive_peer(replace(go2.state(),task_id='old'),cfg['alignment']['robots']['go2'],.15)
    if not peer_moves_before_phase_sync:
        assert ranger.accept(sync('ranger',phase='SEARCH_AB'),.2).accepted
    moving=replace(go2.state(),stopped=False,run_state='NAVIGATING',reply='EXECUTING',command_id='go2/1')
    ranger.receive_peer(moving,cfg['alignment']['robots']['go2'],.2)
    if peer_moves_before_phase_sync:
        assert ranger.accept(sync('ranger',phase='SEARCH_AB'),.2).accepted
    command=replace(sync('ranger',phase='SEARCH_AB'),action='EXECUTE',kind='APPROACH_PICKUP',target_id='B',command_id='b',sequence=1)
    assert ranger.accept(command,.2).accepted


@pytest.mark.parametrize('phase',['SEARCH_AB','DEPOSIT','PAUSED','DONE'])
def test_booted_agent_does_not_adopt_old_task_from_non_handshake_sync(phase):
    _,machines,health=pair();ranger=machines['ranger'];ranger.tick(0,health)
    assert not ranger.accept(sync('ranger',task='old',phase=phase),.1).accepted
    assert ranger.context.task_id=='' and not ranger.paused


def test_health_loss_before_commit_is_still_latched_when_commit_arrives():
    cfg,machines,health=pair()
    for m in machines.values():m.tick(0,health)
    for robot,m in machines.items():assert m.accept(sync(robot),0).accepted
    exchange(cfg,machines,.1)
    ranger=machines['ranger'];go2=machines['go2']
    ranger.receive_peer(replace(go2.state(),nav_ready=False),cfg['alignment']['robots']['go2'],.2)
    assert not ranger.paused
    ranger.accept(sync('ranger',phase='SEARCH_AB'),.3)
    ranger.tick(.3,health)
    assert ranger.paused and 'nav_ready' in ranger.state().reason


@pytest.mark.parametrize('changes',[dict(run_state='IDLE'),dict(run_state='SYNCING'),
    dict(reply=''),dict(sequence=1),dict(command_id='old'),dict(goal_id='old'),dict(ready=True),
    dict(deposited=True),dict(exited=True),dict(resource_id='old'),dict(requested_resource='old')])
def test_coordinator_requires_canonical_clean_ack(changes):
    mission=MockMission(private_config());assert mission.start().accepted
    c=mission.coordinator
    for robot in ROBOTS:
        c.ingest_state(replace(c.states[robot],task_id=c.task_id,run_state='SYNCED',reply='SYNCED'),.1)
    c.ingest_state(replace(c.states['ranger'],**changes),.2)
    assert not any(x.action=='EXECUTE' for x in c.tick(.2))
    assert c.phase=='SYNCING'


@pytest.mark.parametrize('changes',[dict(run_state='IDLE'),dict(reply='SYNCED'),dict(ready=True),dict(goal_id='old')])
def test_dirty_boot_peer_cannot_produce_standby(changes):
    cfg,machines,health=pair()
    for m in machines.values():m.tick(0,health)
    ranger=machines['ranger']
    ranger.receive_peer(replace(machines['go2'].state(),**changes),cfg['alignment']['robots']['go2'],.1)
    assert ranger.state().run_state=='WAITING_PEER'


def test_transport_receipt_of_old_task_cannot_refresh_task_receipt():
    cfg,machines,health=pair();ranger=machines['ranger'];ranger.tick(0,health)
    ranger.accept(sync('ranger'),0)
    peer=replace(machines['go2'].state(),task_id='old')
    assert not ranger.receive_peer(peer,cfg['alignment']['robots']['go2'],4)
    assert ranger.peer_transport_received==4 and ranger.peer_received is None
    ranger.tick(60,health)
    assert not ranger.paused and ranger.state().run_state=='SYNCING'


def test_total_budget_is_not_reset_by_repeated_sync_or_phase_commit():
    cfg,machines,health=pair()
    for robot,m in machines.items():
        m.tick(0,health);assert m.accept(sync(robot),0).accepted
    for m in machines.values():m.accept(sync(m.context.robot_id),590)
    exchange(cfg,machines,590)
    ranger=machines['ranger'];ranger.accept(sync('ranger',phase='SEARCH_AB'),590)
    ranger.sync_received=ranger.peer_received=600.01
    ranger.tick(600.01,health)
    assert ranger.paused and ranger.state().reason=='task timeout'


def test_sync_recovery_after_stale_ack_and_peer_restart():
    cfg,machines,health=pair()
    for robot,m in machines.items():m.tick(0,health);m.accept(sync(robot),0)
    exchange(cfg,machines,.1)
    ranger=machines['ranger'];assert ranger.state().run_state=='SYNCED'
    ranger.tick(4,health)
    assert ranger.state().run_state=='SYNCING' and ranger.state().reply=='' and not ranger.paused
    restarted,_=make_machine(cfg,'go2',True);restarted.tick(4,health)
    ranger.receive_peer(restarted.state(),cfg['alignment']['robots']['go2'],4)
    machines['go2']=restarted;restarted.accept(sync('go2'),5);ranger.accept(sync('ranger'),5)
    exchange(cfg,machines,5)
    assert ranger.state().run_state=='SYNCED'


def test_coordinator_retains_first_pause_reason():
    mission=MockMission(private_config());assert mission.start().accepted
    mission.coordinator.pause('first fault');mission.coordinator.pause('cascade')
    assert mission.coordinator.reason=='first fault'


def test_coordinator_total_budget_includes_long_handshake_and_execution():
    mission=MockMission(private_config());assert mission.start().accepted
    c=mission.coordinator
    for robot in ROBOTS:
        c.ingest_state(replace(c.states[robot],task_id=c.task_id,run_state='SYNCED',reply='SYNCED'),599.)
    assert any(x.action=='EXECUTE' for x in c.tick(599.))
    assert c.started==0.
    assert not any(x.action=='EXECUTE' for x in c.tick(600.01))
    assert c.reason=='task timeout'


def test_preparation_identity_mismatch_is_recoverable_and_execution_mode_change_latches():
    cfg,machines,health=pair()
    for m in machines.values():m.tick(0,health)
    ranger=machines['ranger'];peer=machines['go2'].state();entry=cfg['alignment']['robots']['go2']
    ranger.receive_peer(replace(peer,map_version='wrong'),entry,.1)
    assert not ranger.paused and 'identity' in ranger.state().reason
    ranger.receive_peer(peer,entry,.2)
    assert ranger.state().run_state=='STANDBY'
    for robot,m in machines.items():m.accept(sync(robot),.3)
    exchange(cfg,machines,.4)
    ranger.accept(sync('ranger',phase='SEARCH_AB'),.5)
    ranger.receive_peer(replace(machines['go2'].state(),simulation=False),entry,.6)
    assert ranger.paused and 'mode' in ranger.state().reason


def test_syncing_coordinator_waits_for_corrected_identity_without_cached_ack():
    mission=MockMission(private_config());assert mission.start().accepted
    c=mission.coordinator
    for robot in ROBOTS:c.ingest_state(replace(c.states[robot],task_id=c.task_id,run_state='SYNCED',reply='SYNCED'),.1)
    valid=c.states['ranger']
    c.ingest_state(replace(valid,map_version='wrong'),.2)
    assert not any(x.action=='EXECUTE' for x in c.tick(.2)) and c.phase=='SYNCING'
    assert 'identity' in c.reason
    c.ingest_state(valid,.3)
    assert any(x.action=='EXECUTE' for x in c.tick(.3))


def test_coordinator_separates_valid_transport_from_current_task_receipts():
    mission=MockMission(private_config());c=mission.coordinator
    assert c.transport_received_at and not c.task_received_at
    assert mission.start().accepted
    peer=replace(c.states['ranger'],task_id=c.task_id,nav_ready=False,run_state='SYNCING')
    c.ingest_state(peer,4.)
    assert c.transport_received_at['ranger']==c.task_received_at['ranger']==4.
    c.ingest_state(replace(peer,task_id='old'),5.)
    assert c.transport_received_at['ranger']==5. and c.task_received_at['ranger']==4.
    assert not any(x.action=='EXECUTE' for x in c.tick(5.)) and c.phase=='SYNCING'
