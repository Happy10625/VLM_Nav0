import json
from dataclasses import replace
from types import SimpleNamespace as S

import pytest

from test_private_c import local_agent


def test_peer_history_records_acceptance_rejection_and_is_bounded():
    config, machine, _, _ = local_agent()
    entry = config['alignment']['robots']['go2']
    peer = replace(machine.peer_state, stamp=9000.)
    assert machine.receive_peer(peer, entry, 1.)
    assert not machine.receive_peer(replace(peer, task_id='other'), entry, 2.)
    assert not machine.receive_peer(replace(peer, robot_id='ranger'), entry, 2.1)
    assert not machine.receive_peer(replace(peer, map_version='bad'), entry, 2.2)
    history = list(getattr(machine, 'peer_history', []))
    assert [row['result'] for row in history[-4:]] == [
        'accepted', 'task mismatch', 'robot mismatch', 'peer map/mode identity changed']
    assert history[-4]['received_monotonic'] == 1.
    assert history[-4]['stamp'] == 9000.
    assert history[-4]['peer_received'] == 1.
    assert history[-3]['task_id'] == 'other'
    assert history[-3]['local_task_id'] == 't'
    assert machine.peer_received == 1.
    for i in range(100):
        machine.receive_peer(peer, entry, 3. + i)
    assert len(machine.peer_history) == 32
    assert machine.peer_history[-1]['received_monotonic'] == 102.
    assert machine.peer_receive_count - machine.peer_accept_count == 3


def test_missing_peer_snapshot_distinguishes_no_callback_from_rejection():
    _, machine, adapter, _ = local_agent()
    machine.peer_received = machine.peer_transport_received = None
    machine.peer_history.clear()
    machine.peer_receive_count = machine.peer_accept_count = 0
    machine.sync_received = 3.
    machine.tick(3.001, adapter.get_health())
    snapshot = machine.heartbeat_diagnostic
    assert snapshot['reason'] == 'peer heartbeat timeout'
    assert snapshot['ages']['peer_received'] is None
    assert snapshot['peer_receive_count'] == 0 and snapshot['peer_history'] == []


@pytest.mark.parametrize('kind', ['peer', 'coordinator'])
def test_timeout_snapshot_uses_check_time_and_does_not_change_after_reception(kind):
    config, machine, adapter, _ = local_agent()
    machine.peer_received = 0. if kind == 'peer' else 3.
    machine.peer_transport_received = 2.
    machine.sync_received = 3. if kind == 'peer' else None
    machine.tick(3.001, adapter.get_health())
    snapshot = getattr(machine, 'heartbeat_diagnostic', None)
    assert snapshot is not None
    assert snapshot['reason'] == f'{kind} heartbeat timeout'
    assert snapshot['now'] == 3.001 and snapshot['lost_timeout'] == 3.
    assert snapshot['peer_received'] == (0. if kind == 'peer' else 3.)
    assert snapshot['sync_received'] == (3. if kind == 'peer' else None)
    assert snapshot['ages']['peer_transport_received'] == pytest.approx(1.001)
    assert snapshot['ages']['sync_received'] == (pytest.approx(.001) if kind == 'peer' else None)
    original = json.dumps(snapshot, sort_keys=True)
    peer = replace(machine.peer_state, stamp=99.)
    machine.receive_peer(peer, config['alignment']['robots']['go2'], 4.)
    machine.tick(4., adapter.get_health())
    assert json.dumps(snapshot, sort_keys=True) == original
    assert machine.paused == (kind == 'coordinator')
    assert machine.state().reason == ('coordinator heartbeat timeout' if kind == 'coordinator' else '')


def test_node_logs_timeout_once_and_records_callback_entry_time(monkeypatch):
    from multi_agent_nav import agent_node
    config, machine, adapter, _ = local_agent()
    messages = []
    node = S(machine=machine, config=config, next_wait_log=float('inf'),
             publish_state=lambda: None, poll_map=lambda now: None,
             get_logger=lambda: S(warning=messages.append))
    clock = iter([1., 2.])
    monkeypatch.setattr(agent_node.time, 'monotonic', lambda: next(clock))
    peer = replace(machine.peer_state, stamp=500.)
    monkeypatch.setattr(agent_node, 'from_ros', lambda message: message)
    agent_node.AgentNode.peer(node, peer, 'go2')
    assert machine.peer_received == 2.
    assert getattr(machine, 'peer_history', [])
    assert machine.peer_history[-1]['callback_monotonic'] == 1.
    assert machine.peer_history[-1]['received_monotonic'] == 2.
    machine.sync_received = 5.
    node.runner = S(tick=lambda now: machine.tick(now, adapter.get_health()))
    monkeypatch.setattr(agent_node.time, 'monotonic', lambda: 5.)
    agent_node.AgentNode.tick(node)
    assert not messages  # Exactly three seconds remains healthy.
    monkeypatch.setattr(agent_node.time, 'monotonic', lambda: 5.001)
    agent_node.AgentNode.tick(node)
    agent_node.AgentNode.tick(node)
    assert len(messages) == 1
    assert messages[0].startswith('Heartbeat timeout: ')
    data = json.loads(messages[0].split(': ', 1)[1])
    assert data['reason'] == 'peer heartbeat timeout'
    assert data['peer_history'][-1]['stamp'] == 500.
    assert data['ages']['peer_received'] == pytest.approx(3.001)


def test_node_still_rejects_wrong_robot_and_retains_diagnostic(monkeypatch):
    from multi_agent_nav import agent_node
    config, machine, _, _ = local_agent()
    before = machine.state()
    node = S(machine=machine, config=config, publish_state=lambda: pytest.fail('wrong robot changed state'))
    monkeypatch.setattr(agent_node.time, 'monotonic', lambda: 1.)
    monkeypatch.setattr(agent_node, 'from_ros', lambda message: message)
    agent_node.AgentNode.peer(node, replace(machine.peer_state, robot_id='ranger'), 'go2')
    assert machine.state() == before and machine.peer_received == 0.
    assert machine.peer_history[-1]['result'] == 'robot mismatch'
    assert machine.peer_history[-1]['callback_monotonic'] == 1.
