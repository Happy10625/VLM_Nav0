"""Ranger local startup waits for live prerequisites without weakening them."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as S

import pytest

from multi_agent_nav import go2_check


def startup(blocked):
    health = S(localization_valid=True, stopped=True, motion_terminal=True, nav_ready=True)
    state = dict(rgbd=True, local=True, global_=True, owners=[])
    active_map = S(path='/configured/map.yaml')
    if hasattr(health, blocked):
        setattr(health, blocked, False)
    elif blocked == 'map_parameter':
        active_map.path = None
    elif blocked == 'owners':
        state['owners'] = ['multi_agent_task_agent']
    else:
        state[blocked] = False
    node = S(vlm_input_ready=lambda _:state['rgbd'])
    adapter = S(get_health=lambda:health, costmap_transform=lambda global_goal=False:
                object() if state['global_' if global_goal else 'local'] else None)
    return health, state, active_map, node, adapter


@pytest.mark.parametrize('blocked', ['localization_valid', 'stopped', 'motion_terminal',
    'nav_ready', 'rgbd', 'local', 'global_', 'map_parameter', 'owners'])
def test_startup_waits_for_each_live_gate_then_resumes(blocked):
    health, state, active_map, node, adapter = startup(blocked)
    events, spins = [], []
    def spin():
        spins.append(1)
        if len(spins) == 3:
            for key in vars(health):
                setattr(health, key, True)
            state.update(rgbd=True, local=True, global_=True, owners=[])
            active_map.path = '/configured/map.yaml'
        assert len(spins) <= 3
    go2_check.wait_ranger_startup(adapter, node, active_map, spin, events.append,
                                 lambda:state['owners'], '/configured/map.yaml')
    assert len(spins) == 3
    assert [event['state'] for event in events] == ['WAITING_RECOVERY', 'RECOVERED']
    assert events[0]['failed_gates'] and events[0]['motion_sent'] is False
    assert events[-1]['recovery_status'] == 'resumed'


def test_startup_preserves_operator_interrupt():
    _, _, active_map, node, adapter = startup('stopped')
    def interrupt():
        raise KeyboardInterrupt()
    with pytest.raises(KeyboardInterrupt):
        go2_check.wait_ranger_startup(adapter, node, active_map, interrupt, lambda _:None, lambda:[],
                                     '/configured/map.yaml')


def test_wrong_active_map_returns_for_configuration_rejection():
    _, _, active_map, node, adapter = startup('nav_ready')
    active_map.path = '/wrong/map.yaml'
    go2_check.wait_ranger_startup(adapter, node, active_map,
        lambda:pytest.fail('wrong map must be rejected'), lambda _:None, lambda:[],
        '/configured/map.yaml')


@pytest.mark.parametrize('source', ['get_health', 'costmap_transform', 'owners', 'spin', 'rgbd'])
def test_startup_transient_callback_errors_wait_and_retry(source, monkeypatch):
    _, state, active_map, node, adapter = startup('ready')
    events, spins, delays = [], [], []
    calls = [0]
    def spin():
        spins.append(1)
    owners = lambda:state['owners']
    original = (node.vlm_input_ready if source == 'rgbd' else spin if source == 'spin'
                else owners if source == 'owners' else getattr(adapter, source))
    def flaky(*args, **kwargs):
        calls[0] += 1
        if calls[0] <= 2:
            raise RuntimeError('temporary callback unavailable')
        return original(*args, **kwargs)
    if source == 'rgbd':
        node.vlm_input_ready = flaky
    elif source == 'spin':
        spin = flaky
    elif source == 'owners':
        owners = flaky
    else:
        setattr(adapter, source, flaky)
    monkeypatch.setattr(go2_check.time, 'sleep', delays.append)
    go2_check.wait_ranger_startup(adapter, node, active_map, spin, events.append,
                                 owners, '/configured/map.yaml')
    assert len(delays) == 2 and all(0 < delay <= .1 for delay in delays)
    assert spins, 'waiting must keep processing callbacks'
    assert events[0]['state'] == 'WAITING_RECOVERY'
    assert 'RuntimeError: temporary callback unavailable' in events[0]['reason']
    assert events[-1]['state'] == 'RECOVERED'


def test_monitor_shows_active_recovery_and_clears_old_failure():
    path = Path(__file__).resolve().parents[1] / 'scripts/ranger_monitor.py'
    spec = importlib.util.spec_from_file_location('entry_recovery_monitor', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    view = module.ExperimentView()
    view.apply(dict(event='SCAN_FAILED', failure_reason='old scan failure'))
    view.apply(dict(state='WAITING_RECOVERY', reason='initial observation requires confirmed stop',
                    recovery_status='waiting', failed_gates=['stopped']))
    text = module.render(view, {}, now=100., ros_now=100., can_delta=1)
    assert '等待恢复' in text and 'initial observation requires confirmed stop' in text
    assert 'old scan failure' not in text
    view.apply(dict(state='RECOVERED', recovery_status='resumed', reason=''))
    text = module.render(view, {}, now=100., ros_now=100., can_delta=1)
    assert '已恢复' in text and 'initial observation requires confirmed stop' not in text
    assert 'old scan failure' not in text
