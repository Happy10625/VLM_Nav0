"""Distance retry recovery with real Ranger control/perception and fake transport."""
from dataclasses import replace
from types import SimpleNamespace as S

import pytest

from test_ranger_single_robot_parity import local_runtime, start_approach, advance_approach
from test_navigation_gate import stationary_odom


def exhaust_distance_retry(r):
    r.node.robot_pose = lambda: (1.15, 0., 0.)
    for index in range(2):
        r.terminals[index].set_result(S(status=4))
        for _ in range(3):
            r.node.on_arrival_odom(stationary_odom())
        r.clock[0] += .21
        next(r.steps)
        if index == 0:
            advance_approach(r)


def test_distance_exhaustion_returns_to_observation_with_same_identity(local_runtime):
    r = local_runtime
    start_approach(r)
    old_epoch = r.node.task_epoch
    r.frame()
    exhaust_distance_retry(r)
    outcome = advance_approach(r)
    assert outcome == 'RESUME_SEARCH'
    assert r.node.state == 'SCANNING'
    assert r.node.execution_identity == ('task', 'command')
    assert r.node.execution_target_id == 'B' and r.node.execution_description == 'cup'
    assert r.node.target_reference_position is None and r.node.latest_snapshot is None
    assert r.node.task_epoch > old_epoch and r.node.target_tracker.progress == 0
    assert not r.adapter.local_arrival_verified and r.adapter.completed_approach is None
    assert r.adapter.approach is None and not r.adapter.motions


@pytest.mark.parametrize('target_id,initial,interruption', [
    ('B', True, None), ('C', True, None), ('B', False, None),
    ('B', True, 'cancel'), ('B', True, 'health'), ('B', True, 'deadline')])
def test_recovery_observes_before_navigating_to_same_task_target(local_runtime, target_id, initial, interruption):
    from multi_agent_nav.go2_exploration import explore_steps, ExplorationStopped
    from multi_agent_nav.models import Observation, Pose2D
    r = local_runtime
    health = r.adapter.get_health
    r.adapter.get_health = lambda: replace(health(), footprint=[[.2,.1],[.2,-.1],[-.2,-.1],[-.2,.1]])
    r.adapter.costmap_transform = lambda **_: S()
    r.adapter.client.infer_abc = lambda *_: {target_id: ('DETECTED', r.result[0])}
    config = dict(scene=dict(resources={}, targets={target_id:'cup'},
        viewpoints=dict(ranger=[dict(id='far', pose=[5.,0.,0.])])),
        robot_ranger=dict(r.adapter.config, stop_margin=.1, approach_on_detection=True))
    events = []
    observation = Observation(target_id=target_id, status='DETECTED', position_valid=True,
                              common_position=Pose2D(2.,0.), observation_id='old')
    steps = explore_steps(r.adapter, config, r.adapter.frame, 200., 'cup', events.append,
        lambda: [], None, lambda: None, clock=lambda:r.clock[0], robot_id='ranger',
        target_id=target_id, task_id='same-task', command_id='same-command',
        require_navigation_success=True, scan_at_start=not initial,
        initial_observation=observation if initial else None)
    def pump(until, frames=False):
        for _ in range(100):
            r.clock[0] += .1
            if frames:r.frame(2. if len(r.requests)<2 else 2.4)
            r.adapter.tick()
            try:next(steps)
            except StopIteration as done:pytest.fail(f'task ended during recovery: {done.value}')
            if until():return
        pytest.fail('recovery did not reach expected state')
    try:
        pump(lambda:len(r.requests)==1, frames=not initial)
        r.node.robot_pose = lambda:(1.15,0.,0.)
        for index in range(2):
            r.terminals[index].set_result(S(status=4))
            for _ in range(3):r.node.on_arrival_odom(stationary_odom())
            pump(lambda:len(r.requests)==2 if index==0 else r.adapter.scan is not None)
        assert r.node.execution_identity == ('same-task','same-command')
        assert r.node.execution_target_id == target_id
        assert r.adapter.scan['current_view_only'] is True
        assert len(r.requests)==2 and r.node.target_reference_position is None
        assert not r.adapter.local_arrival_verified
        if interruption:
            if interruption=='cancel':
                steps.close()
            else:
                if interruption=='deadline':r.clock[0]=200.
                else:r.adapter.get_health=lambda:replace(health(),nav_ready=False)
                with pytest.raises(ExplorationStopped,match='deadline|health lost'):
                    next(steps)
            assert len(r.requests)==2 and not r.node.execution_active
            assert not r.adapter.local_arrival_verified
            return
        pump(lambda:len(r.requests)==3, frames=True)
        assert r.requests[-1].pose.pose.position.x == pytest.approx(2.4)
        approach = r.adapter.approach
        assert (approach.task_id, approach.command_id) == ('same-task','same-command')
        assert approach.goal_kind_wire == ('DEPOSIT_NAV' if target_id=='C' else 'PICKUP_NAV')
        assert approach.arrival_retry_count == 0
        assert any(e['state']=='APPROACH_REOBSERVE' for e in events)
        assert not any(e['state']=='TARGET_APPROACHED' for e in events)
    finally:
        steps.close()


@pytest.mark.parametrize('status', [5, 6])
def test_uncontrolled_terminal_failure_does_not_restart_observation(local_runtime,status):
    r = local_runtime
    start_approach(r)
    r.terminals[0].set_result(S(status=status))
    assert advance_approach(r).startswith('APPROACH_FAILED: ')
    assert r.node.target_reference_position == (2.,0.,0.)
    assert not r.adapter.local_arrival_verified


def test_cancel_between_distance_failure_and_recovery_does_not_reactivate_task(local_runtime):
    r = local_runtime
    start_approach(r)
    exhaust_distance_retry(r)
    r.node.end_execution()
    assert advance_approach(r).startswith('APPROACH_FAILED: ')
    assert r.node.state == 'DISARMED' and not r.node.execution_active
