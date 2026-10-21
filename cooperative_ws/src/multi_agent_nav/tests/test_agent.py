from dataclasses import replace
import pytest
from multi_agent_nav.models import *
from multi_agent_nav.frame_adapter import FrameAdapter
from multi_agent_nav.agent import AgentMachine
from multi_agent_nav.adapters.fake import FakeAdapter

def machine():
    ctx = AgentContext('go2', 't', 'g', 'v', 'lab_ref', 'a', True)
    agent = AgentMachine(ctx, FrameAdapter('g', 'v', 'lab_ref', 'a', (0, 0, 0)))
    agent.accept(Command(task_id='t', recipient_id='go2', alignment_id='a', action='SYNC'), 0)
    agent.peer_received = 0
    return agent

def command(kind='APPROACH_PICKUP', sequence=1):
    return Command(task_id='t', command_id=f'c{sequence}', sequence=sequence,
                   recipient_id='go2', alignment_id='a', kind=kind, target_id='A',
                   common_goal=Pose2D(2, 0), object_position=Pose2D(2, 0), position_valid=True)

def navigate(agent, cmd):
    assert agent.accept(cmd, 0).accepted
    agent.take_effects()
    agent.on_path(PathResult(True, [Pose2D(), Pose2D(2, 0)]), Pose2D(2, 0), 0)
    agent.take_effects()
    agent.navigation_started('goal', 'PICKUP_NAV' if cmd.kind == 'APPROACH_PICKUP' else 'WAIT_NAV')
    return GoalResult('t', cmd.command_id, 'goal', agent.goal_kind, 'SUCCEEDED')

def test_current_goal_only_and_duplicate_execution():
    agent = machine()
    result = navigate(agent, command())
    agent.on_goal_result(replace(result, goal_id='stale'), 0)
    assert not agent.state().ready
    agent.on_goal_result(result, 0)
    assert agent.state().ready
    assert agent.accept(command(), 0).accepted
    assert not agent.take_effects()

@pytest.mark.parametrize('kind', ['WAIT', 'SEARCH_VIEWPOINT'])
def test_internal_navigation_does_not_set_ready(kind):
    agent = machine()
    result = navigate(agent, command(kind))
    agent.on_goal_result(result, 0)
    assert not agent.state().ready

def test_pause_invalidates_late_success_and_requires_stopping():
    agent = machine()
    result = navigate(agent, command())
    pause = replace(command(), action='PAUSE')
    agent.accept(pause, .1)
    agent.on_goal_result(result, .2)
    assert not agent.state().ready
    assert not agent.accept(command(sequence=2), .3).accepted
    agent.tick(.4, Health(localization_valid=True, nav_ready=True, stopped=False, motion_terminal=True))
    assert agent.state().run_state == 'STOPPING'
    agent.tick(.5, Health(localization_valid=True, nav_ready=True, stopped=True, motion_terminal=True))
    assert agent.state().run_state == 'PAUSED'
    assert not agent.accept(command(sequence=2), .6).accepted  # explicit new task required

def test_unreachable_and_sync_loss_fail_without_ready():
    agent = machine()
    assert agent.accept(command(), 0).accepted
    agent.on_path(PathResult(False, reason='unreachable'), None, 0)
    assert agent.state().reply == 'FAILED'
    agent.tick(4, Health(localization_valid=True, nav_ready=True, stopped=True))
    assert agent.state().run_state == 'PAUSED'
    assert not agent.state().ready

def test_fake_has_no_ros_or_hardware_and_supports_cancel():
    fake = FakeAdapter('go2')
    goal = fake.start_navigation(Pose2D(2, 0), 't', 'c', 'PICKUP_NAV')
    fake.cancel_navigation(goal)
    fake.advance(1)
    assert fake.results.pop().status == 'CANCELED'
    assert fake.get_health().stopped

def test_deposit_cannot_execute_before_ready_and_phase_sync():
    agent=machine()
    cmd=replace(command('APPROACH_DEPOSIT'),target_id='C',phase='DEPOSIT')
    assert not agent.accept(cmd,0).accepted
    agent._state.ready=True
    assert not agent.accept(cmd,0).accepted
    agent.accept(replace(cmd,action='SYNC'),0)
    assert agent.accept(cmd,0).accepted
