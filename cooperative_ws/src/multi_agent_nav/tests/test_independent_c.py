"""Private C tasks progress independently and finish at C, without EXIT."""
from concurrent.futures import Future
from dataclasses import replace

import pytest

from multi_agent_nav.mock import MockMission
from multi_agent_nav.models import Observation, Pose2D, ROBOTS, validate_command
from multi_agent_nav.runner import AgentRunner
from test_private_c import private_config, local_agent


def independent_mission():
    cfg = private_config()
    cfg['scene']['resources'] = {}
    cfg['scene']['exit_points'] = {}
    return MockMission(cfg)


def cached_c(coordinator, robot, oid=None, **changes):
    state = coordinator.states[robot]
    return replace(Observation(task_id=coordinator.task_id, robot_id=robot,
        local_map_id=state.local_map_id, map_version=state.map_version,
        common_frame_id='', alignment_id='', observation_id=oid or robot + '/C',
        target_id='C', status='CACHED', simulation=True), **changes)


def ready_coordinator():
    mission = independent_mission()
    assert mission.start().accepted
    coordinator = mission.coordinator
    for robot in ROBOTS:
        coordinator.ingest_state(replace(coordinator.states[robot], task_id=coordinator.task_id, ready=True), 0.)
    return coordinator


def test_private_c_completion_stays_at_c_without_exit_or_permits():
    mission = independent_mission()
    assert mission.start().accepted
    mission.run(400)
    assert mission.coordinator.phase == 'DONE', mission.coordinator.reason
    assert {r for r, c in mission.executed if c.kind == 'APPROACH_DEPOSIT'} == set(ROBOTS)
    assert all(c.kind != 'EXIT' and not c.resource_id and not c.token for _, c in mission.executed)
    for robot in ROBOTS:
        state = mission.machines[robot].state()
        health = mission.adapters[robot].get_health()
        assert state.ready and state.deposited and state.stopped and not state.exited
        assert not state.token and not state.requested_resource
        assert health.motion_terminal
        assert ((health.pose.x - 9.) ** 2 + (health.pose.y - 1.) ** 2) ** .5 <= 1.


@pytest.mark.parametrize('delayed_robot', ROBOTS)
def test_peer_search_does_not_delay_own_approach(delayed_robot):
    mission = independent_mission()
    adapter = mission.adapters[delayed_robot]
    del adapter.detections['C']
    original_scan = adapter.start_scan
    pending = Future()
    calls = []
    def delayed_scan(descriptions, scan_id, **options):
        if mission.coordinator.phase in ('SEARCH_C', 'DEPOSIT'):
            calls.append((descriptions, scan_id, options))
            return pending
        return original_scan(descriptions, scan_id, **options)
    adapter.start_scan = delayed_scan
    assert mission.start().accepted
    mission.run(180)
    other = 'ranger' if delayed_robot == 'go2' else 'go2'
    assert calls and mission.machines[other].state().deposited
    assert not mission.machines[delayed_robot].state().deposited
    assert mission.coordinator.phase != 'DONE'
    adapter.detections['C'] = Pose2D(9., 1.)
    descriptions, scan_id, options = calls[0]
    pending.set_result(original_scan(descriptions, scan_id, **options).result())
    mission.run(180)
    assert mission.coordinator.phase == 'DONE', mission.coordinator.reason
    assert len([c for r, c in mission.executed if r == delayed_robot and c.kind == 'SEARCH_VIEWPOINT']) == 1


def test_both_c_approaches_can_be_outstanding_together():
    mission = independent_mission()
    for adapter in mission.adapters.values():
        adapter.delay = 1.
    assert mission.start().accepted
    together = False
    for _ in range(400):
        mission.step()
        commands = mission.coordinator.commands
        together |= all(r in commands and commands[r].kind == 'APPROACH_DEPOSIT' for r in ROBOTS)
    assert together and mission.coordinator.phase == 'DONE'


def test_search_in_progress_survives_transition_to_deposit():
    mission = independent_mission()
    pending = {r: Future() for r in ROBOTS}
    calls = {}
    scans = {}
    for robot in ROBOTS:
        adapter = mission.adapters[robot]
        del adapter.detections['C']
        scans[robot] = adapter.start_scan
        def delayed(descriptions, scan_id, robot=robot, **options):
            if mission.coordinator.phase in ('SEARCH_C', 'DEPOSIT'):
                calls[robot] = (descriptions, scan_id, options)
                return pending[robot]
            return scans[robot](descriptions, scan_id, **options)
        adapter.start_scan = delayed
    assert mission.start().accepted
    mission.run(100)
    assert set(calls) == set(ROBOTS) and mission.coordinator.phase == 'SEARCH_C'
    def release(robot):
        mission.adapters[robot].detections['C'] = Pose2D(9., 1.)
        descriptions, scan_id, options = calls[robot]
        pending[robot].set_result(scans[robot](descriptions, scan_id, **options).result())
    release('go2')
    mission.run(100)
    assert mission.machines['go2'].state().deposited
    peer = mission.machines['ranger']
    assert peer.phase == 'DEPOSIT' and peer.command.phase == 'SEARCH_C'
    assert not pending['ranger'].done() and not peer.paused
    release('ranger')
    mission.run(100)
    assert mission.coordinator.phase == 'DONE', mission.coordinator.reason


def test_c_cache_does_not_bypass_ab_ready_barrier():
    coordinator = ready_coordinator()
    coordinator.states['ranger'].ready = False
    coordinator.ingest_observation(cached_c(coordinator, 'go2'), 0.)
    coordinator.ingest_observation(cached_c(coordinator, 'ranger'), 0.)
    actions = [c for c in coordinator.tick(0.) if c.action == 'EXECUTE']
    assert coordinator.phase == 'SEARCH_AB'
    assert actions and all(c.kind == 'APPROACH_PICKUP' and c.recipient_id == 'ranger' for c in actions)


def test_done_requires_current_matching_terminal_commands_and_both_stopped():
    coordinator = ready_coordinator()
    for robot in ROBOTS:
        coordinator.ingest_observation(cached_c(coordinator, robot), 0.)
    commands = [c for c in coordinator.tick(0.) if c.action == 'EXECUTE']
    assert len(commands) == 2 and all(c.kind == 'APPROACH_DEPOSIT' for c in commands)
    for command in commands:
        robot = command.recipient_id
        coordinator.ingest_state(replace(coordinator.states[robot], command_id=command.command_id,
            sequence=command.sequence, reply='SUCCEEDED', deposited=True, stopped=False), .1)
    coordinator.tick(.1)
    assert coordinator.phase == 'DEPOSIT' and len(coordinator.commands) == 2
    for robot in ROBOTS:
        coordinator.ingest_state(replace(coordinator.states[robot], stopped=True, command_id='old-command'), .2)
    coordinator.tick(.2)
    assert coordinator.phase == 'DEPOSIT' and len(coordinator.commands) == 2
    for command in commands:
        coordinator.ingest_state(replace(coordinator.states[command.recipient_id], command_id=command.command_id), .3)
    coordinator.tick(.3)
    assert coordinator.phase == 'DONE' and not coordinator.commands


def test_cache_upgrade_is_task_local_and_ignores_invalid_coordinates():
    coordinator = ready_coordinator()
    observation = cached_c(coordinator, 'ranger')
    coordinator.ingest_observation(replace(observation, status='NOT_DETECTED'), 0.)
    coordinator.ingest_observation(replace(observation, task_id='old'), 0.)
    coordinator.ingest_observation(replace(observation, position_valid=True, common_position=Pose2D(4, 5)), 0.)
    coordinator.ingest_observation(observation, .1)
    actions = [c for c in coordinator.tick(.1) if c.action == 'EXECUTE']
    assert any(c.recipient_id == 'ranger' and c.kind == 'APPROACH_DEPOSIT' for c in actions)
    assert any(c.recipient_id == 'go2' and c.kind == 'SEARCH_VIEWPOINT' for c in actions)
    coordinator.pause('new task')
    assert coordinator.start(coordinator.scene['scenario_id'], .1).accepted
    assert not coordinator.local_c and not coordinator.observations


def test_private_agent_accepts_c_search_during_deposit_but_rejects_exit():
    _, machine, _, command = local_agent()
    machine._state.ready = True
    machine.phase = 'DEPOSIT'
    search = replace(command, phase='DEPOSIT', kind='SEARCH_VIEWPOINT', target_id='C')
    assert not validate_command(replace(search, kind='EXIT'), machine.context).accepted
    assert not machine.accept(replace(search, kind='EXIT'), 0.).accepted
    assert machine.accept(search, 0.).accepted


@pytest.mark.parametrize('fault', ['moving', 'far', 'aborted', 'cancel'])
def test_independent_c_requires_current_success_and_actual_stopped_distance(fault):
    config, machine, adapter, command = local_agent()
    machine._state.ready = True
    machine.phase = 'DEPOSIT'
    machine.local_observations['own-c'] = Observation(task_id='t', robot_id='ranger',
        local_map_id=machine.context.local_map_id, map_version=machine.context.map_version,
        target_id='C', observation_id='own-c', status='DETECTED', position_valid=True, common_position=Pose2D(9., 1.))
    command = replace(command, phase='DEPOSIT', kind='APPROACH_DEPOSIT', target_id='C', observation_id='own-c')
    assert machine.accept(command, 0.).accepted
    runner = AgentRunner(machine, adapter, config['scene']['targets'], lambda _: None, config=config)
    original_advance = adapter.advance
    def advance(now):
        if fault == 'aborted' and any(row[2].goal_kind == 'DEPOSIT_NAV' for row in adapter.active.values()):
            adapter.aborts = 10
        original_advance(now)
    adapter.advance = advance
    injected = False
    for index in range(240):
        now = index * .05
        machine.peer_received = machine.sync_received = now
        if not injected and any(kind == 'DEPOSIT_NAV' for kind, _, _ in adapter.calls):
            injected = True
            health = adapter.get_health
            if fault == 'moving': adapter.get_health = lambda: replace(health(), stopped=False)
            if fault == 'far': adapter.get_health = lambda: replace(health(), pose=Pose2D(-100, -100))
            if fault == 'cancel': machine.stop('cancel')
        runner.tick(now)
    assert injected and not machine.state().deposited and not machine.state().exited


def test_deposit_rejects_other_task_or_robot_local_cache():
    _, machine, _, command = local_agent()
    machine._state.ready = True
    machine.phase = 'DEPOSIT'
    for changes in (dict(task_id='old', robot_id='ranger'), dict(task_id='t', robot_id='go2')):
        machine.local_observations['wrong-c'] = Observation(target_id='C', status='DETECTED',
            position_valid=True, common_position=Pose2D(9, 1), **changes)
        assert not machine.accept(replace(command, phase='DEPOSIT', kind='APPROACH_DEPOSIT',
            target_id='C', observation_id='wrong-c'), 0.).accepted
