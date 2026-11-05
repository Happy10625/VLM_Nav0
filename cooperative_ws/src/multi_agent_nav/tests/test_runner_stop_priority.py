"""A secondary inspection/cancellation error must never bypass stopping."""
from dataclasses import replace
from types import SimpleNamespace as S

import pytest

from multi_agent_nav.models import Effect, RecoverablePause, is_recoverable_pause
from multi_agent_nav.runner import AgentRunner
from test_private_c import local_agent


@pytest.fixture
def run():
    config, machine, adapter, command = local_agent()
    machine.context = replace(machine.context, wait_recovery=True)
    machine.accept(command, 0.); machine.take_effects()
    runner = AgentRunner(machine, adapter, config['scene']['targets'], lambda _:None, config=config)
    stops = []
    adapter.stop_motion = stops.append
    return S(machine=machine, adapter=adapter, runner=runner, stops=stops)


def unavailable(*args):
    raise ConnectionError('transport unavailable')


@pytest.mark.parametrize('source', ['health', 'owners', 'health_only'])
def test_inspection_errors_cannot_escape_tick_or_skip_stop(run, source):
    if source != 'health_only':
        run.adapter.tick = unavailable
    if source in ('health', 'health_only'):
        run.adapter.get_health = unavailable
    else:
        run.runner.owners = unavailable
    run.runner.tick(.1)
    assert run.stops and run.machine.paused
    assert not is_recoverable_pause(run.machine.state().reason)


@pytest.mark.parametrize('fault', ['peer_identity', 'evidence', 'health'])
def test_inspection_failure_never_hides_known_hard_fault(run, fault):
    if fault == 'peer_identity':
        run.machine.peer_identity_error = 'peer map/mode identity changed'
        expected = run.machine.peer_identity_error
    elif fault == 'evidence':
        run.adapter.evidence_write_failed = True
        expected = 'evidence_write_failed'
    else:
        run.machine.health = replace(run.machine.health, hard_fault='native_action_terminal_unknown')
        expected = 'native_action_terminal_unknown'
    run.adapter.get_health = unavailable
    reason = run.runner._failure_reason(RecoverablePause('local health invalid: nav_ready'), 'adapter failure')
    assert reason == expected


def test_cancel_rpc_failure_still_calls_stop_motion(run):
    run.adapter.cancel_navigation = unavailable
    run.machine.stop('local health invalid: nav_ready')
    run.machine.take_effects()
    run.runner.dispatch([Effect('STOP', ('goal', 'local health invalid: nav_ready'))], .1)
    assert run.stops and run.machine.paused
    assert not is_recoverable_pause(run.machine.state().reason)


def test_partial_pause_failure_discards_unsafe_checkpoint_and_stops(run):
    closed = []
    run.runner.local_steps = S(close=lambda:closed.append(True))
    def failed_pause(_):
        run.adapter.execution_paused = True
        raise ConnectionError('pause cancellation unavailable')
    run.adapter.pause_local_execution = failed_pause
    run.machine.stop('local health invalid: nav_ready')
    run.runner.dispatch(run.machine.take_effects(), .1)
    assert run.stops and closed
    assert run.runner.local_steps is None and not run.machine.local_suspended
    assert not is_recoverable_pause(run.machine.state().reason)


def test_successful_pause_keeps_resume_checkpoint(run):
    checkpoint = S(close=lambda:pytest.fail('valid paused checkpoint must survive'))
    run.runner.local_steps = checkpoint
    run.adapter.pause_local_execution = lambda _:setattr(run.adapter, 'execution_paused', True)
    run.machine.stop('local health invalid: nav_ready')
    run.runner.dispatch(run.machine.take_effects(), .1)
    assert run.runner.local_steps is checkpoint and run.machine.local_suspended
    assert not run.stops


def test_stop_transport_error_is_latched_without_escape(run):
    attempts = []
    def failed_stop(reason):
        attempts.append(reason)
        raise ConnectionError('zero command unavailable')
    run.adapter.stop_motion = failed_stop
    run.machine.stop('local health invalid: nav_ready')
    run.runner.dispatch(run.machine.take_effects(), .1)
    assert attempts and run.machine.paused
    assert not is_recoverable_pause(run.machine.state().reason)
