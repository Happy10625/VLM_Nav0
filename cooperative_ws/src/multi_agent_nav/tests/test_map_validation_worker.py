"""Map evidence outlives vision cancellation; no ROS nodes or hardware."""
from types import SimpleNamespace as S

import pytest

from multi_agent_nav.agent_node import AgentNode
from multi_agent_nav.adapters.ranger import RangerAdapter
from multi_agent_nav.adapters.vision import AsyncVision
from multi_agent_nav.config import map_file_evidence, map_file_info


@pytest.fixture
def map_node(tmp_path):
    image = tmp_path / 'map.pgm'
    image.write_bytes(b'P5\n1 1\n255\n\xff')
    path = tmp_path / 'map.yaml'
    path.write_text('image: map.pgm\n')
    config = {'robot_ranger': dict(map_path=str(path), map_digest=map_file_info(path)['map_digest'])}
    gates, paths, signature = map_file_evidence(config, 'ranger')
    assert not gates
    adapter = RangerAdapter.__new__(RangerAdapter)
    adapter.vision = AsyncVision(10.)
    adapter.scan = None
    adapter.motions, adapter.plans = {}, {}
    adapter.node = S(stop_pub=S(publish=lambda _: None))
    adapter._motion_event = lambda *args, **kwargs: None
    node = object.__new__(AgentNode)
    node.simulation = False
    node.robot, node.config, node.adapter = 'ranger', config, adapter
    node.map_worker = AsyncVision(adapter.vision.timeout)
    node.map_assets = paths, signature
    node.map_digest_ready = node.map_ready = True
    node.map_validation = node.map_probe = None
    node.next_map_probe = 1000.
    node.next_map_validation = node.map_verified_at = 10.
    try:
        yield node
    finally:
        node.map_worker.close()
        adapter.vision.close()


@pytest.mark.parametrize('cancel', ['hold', 'stop', 'discard'])
def test_normal_motion_or_vision_cancellation_preserves_map_evidence(map_node, cancel):
    node = map_node
    node.poll_map(10.)
    pending = node.map_validation
    assert pending is not None and node.adapter.task_map_ready
    if cancel == 'hold':node.adapter.hold_motion('exploration stop')
    elif cancel == 'stop':node.adapter.stop_motion('operator stop')
    else:node.adapter.vision.discard()
    assert not pending.cancelled()
    assert not node.map_worker.job.result(timeout=2.)[0]
    node.poll_map(10.05)
    assert node.adapter.task_map_ready and pending.done()
    assert node.adapter.task_map_health['validation_errors'] == []


def test_map_check_runs_while_vision_reply_is_busy_and_still_detects_changed_map(map_node):
    node = map_node
    vision_reply = node.adapter.vision.submit(lambda: 'target result')
    node.poll_map(10.)
    assert node.map_validation is not None
    assert not node.map_worker.job.result(timeout=2.)[0]
    node.poll_map(10.05)
    assert node.adapter.task_map_ready and not vision_reply.done()
    node.map_assets[0][1].write_bytes(b'P5\n1 1\n255\n\x00')
    node.poll_map(10.10)
    assert not node.adapter.task_map_ready
    assert node.adapter.task_map_health['assets_unchanged'] is False
    assert node.map_worker.job.result(timeout=2.)[0]
    node.poll_map(10.15)
    assert not node.adapter.task_map_ready
    assert node.adapter.task_map_health['validation_errors'] == ['map digest changed']


@pytest.mark.parametrize('fault', ['error', 'timeout'])
def test_failed_map_validation_still_revokes_health(map_node, monkeypatch, fault):
    from multi_agent_nav import config as config_module
    node = map_node
    clock = [10.]
    monkeypatch.setattr('multi_agent_nav.adapters.vision.time.monotonic', lambda: clock[0])
    if fault == 'error':
        def fail(*args):raise OSError('unreadable map')
        monkeypatch.setattr(config_module, 'map_file_evidence', fail)
    node.poll_map(10.)
    assert node.map_worker.job is not None
    if fault == 'error':
        with pytest.raises(OSError):node.map_worker.job.result(timeout=2.)
    else:
        node.map_worker.job.result(timeout=2.)
        clock[0] += node.map_worker.timeout + .01
    node.map_verified_at = clock[0]
    node.poll_map(clock[0])
    assert not node.adapter.task_map_ready
    assert node.adapter.task_map_health['validation_errors'] == [
        'RuntimeError' if fault == 'error' else 'TimeoutError']


def test_destroy_node_closes_map_worker(map_node, monkeypatch):
    from rclpy.node import Node
    node = map_node
    pending = node.map_worker.submit(lambda: None)
    monkeypatch.setattr(Node, 'destroy_node', lambda self: 'destroyed')
    assert node.destroy_node() == 'destroyed'
    assert pending.cancelled()
    with pytest.raises(RuntimeError, match='shutdown'):
        node.map_worker.executor.submit(lambda: None)
