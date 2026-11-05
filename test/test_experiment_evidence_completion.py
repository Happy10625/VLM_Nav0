"""Required evidence controls acceptance; optional RViz output cannot block stop."""
import json
from pathlib import Path
from types import SimpleNamespace as S

import pytest

from multi_agent_nav import go2_check as check
from multi_agent_nav import go2_exploration
from multi_agent_nav.adapters import factory, go2
from multi_agent_nav.adapters.fake import completed
from multi_agent_nav.models import Health


@pytest.fixture
def experiment(monkeypatch, tmp_path):
    import rclpy
    import rclpy.executors
    from builtin_interfaces.msg import Time
    state = S(fault='', stopped=False, closed=False, destroyed=False, now=0., events=[], report=None)
    map_path = str(tmp_path / 'map.yaml')
    config = dict(alignment=dict(common_frame_id='map', alignment_id='fixture', robots={
        robot:dict(local_map_id='map', map_version='1', transform=[0., 0., 0.])
        for robot in ('ranger', 'go2')}), scene={})
    for robot in ('ranger', 'go2'):
        config['robot_'+robot] = dict(map_path=map_path, path_topic='/plan', cancel_timeout=1.)
    health = Health(localization_valid=True, nav_ready=True, stopped=True, motion_terminal=True)
    def publish(_):
        if state.fault in ('marker', 'marker_events'):
            raise RuntimeError('RViz unavailable')
    def publisher(*args):
        if state.fault == 'marker_create':
            raise RuntimeError('RViz unavailable')
        return S(publish=publish)
    def timer(*args):
        if state.fault == 'marker_timer':
            raise RuntimeError('RViz unavailable')
    node = S(depth_units_verified=True, vlm_input_ready=lambda _:True, get_node_names=lambda:[],
        create_publisher=publisher, create_timer=timer,
        get_clock=lambda:S(now=lambda:S(to_msg=lambda:Time())),
        create_client=lambda *a:S(service_is_ready=lambda:True,
            call_async=lambda _:completed(S(values=[S(string_value=map_path)]))),
        destroy_node=lambda:setattr(state, 'destroyed', True))
    adapter = S(get_health=lambda:health, costmap=None, costmap_transform=lambda **k:object(),
        plans={}, vision=S(poll=lambda:None, job=None), tick=lambda:None,
        stop_motion=lambda _:setattr(state, 'stopped', True),
        close=lambda **k:setattr(state, 'closed', True), evidence_write_failed=False,
        motion_fault=False, local_arrival_verified=False)
    state.adapter = adapter
    def spin(**_):
        state.now += .1
    monkeypatch.setattr(rclpy, 'init', lambda **k:None)
    monkeypatch.setattr(rclpy, 'ok', lambda:False)
    monkeypatch.setattr(rclpy.executors, 'SingleThreadedExecutor', lambda:S(
        add_node=lambda _:None, spin_once=spin, shutdown=lambda:None))
    monkeypatch.setattr(check, 'time', S(monotonic=lambda:state.now, time=lambda:state.now))
    monkeypatch.setattr(check, 'require_ros_environment', lambda *a:None)
    monkeypatch.setattr(check, 'config_report', lambda *a, **k:dict(robot_local_gates=[]))
    monkeypatch.setattr(check, 'robot_local_gates', lambda *a, **k:[])
    monkeypatch.setattr(go2_exploration, 'exploration_gates', lambda *a:[])
    monkeypatch.setattr(factory, 'create_runtime', lambda *a, **k:(node, adapter))
    monkeypatch.setattr(go2, 'ABCClient', lambda _:None)
    monkeypatch.setattr(check, 'emit', lambda report:setattr(state, 'report', dict(report)))
    def explore(adapter, spin, config, frame, deadline, target, record, owners, stop, before_motion, **k):
        before_motion()
        record(dict(state='TARGET_APPROACHED', event='APPROACH_VERIFIED'))
        return 'TARGET_APPROACHED'
    monkeypatch.setattr(go2_exploration, 'explore', explore)
    write_text = Path.write_text
    def result_write(path, *args, **kwargs):
        if path.name == 'result.json' and state.fault == 'result':
            raise OSError('result disk full')
        return write_text(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'write_text', result_write)
    class Events:
        def __init__(self, stream):
            self.stream = stream
        def write(self, line):
            row = json.loads(line)
            if state.fault in ('events', 'marker_events') or (state.fault == 'session_end' and row.get('event') == 'SESSION_END'):
                raise OSError('events disk full')
            state.events.append(row)
            return self.stream.write(line)
        def flush(self):
            return self.stream.flush()
        def fileno(self):
            return self.stream.fileno()
    def run(robot):
        args = check.parser().parse_args(['--action', 'explore', '--allow-motion', '--approach-target',
            '--target', 'test target', '--timeout', '10', '--output-dir', str(tmp_path)])
        with (tmp_path / 'events.jsonl').open('w') as stream:
            return check.run_ros(config, args, Events(stream), robot_id=robot)
    state.run = run
    state.output = tmp_path
    return state


@pytest.mark.parametrize('robot', ['ranger', 'go2'])
@pytest.mark.parametrize('fault', ['events', 'session_end', 'result', 'marker_events'])
def test_missing_required_evidence_cannot_return_success(experiment, robot, fault):
    experiment.fault = fault
    assert experiment.run(robot) == 1
    assert experiment.stopped and experiment.closed and experiment.destroyed
    assert experiment.report['stop_confirmed'] is True
    assert experiment.report['evidence_write_failed'] is True
    assert experiment.report['result'] == 'EVIDENCE_WRITE_FAILED'
    assert experiment.report['motion_result'] == 'TARGET_APPROACHED'
    if fault != 'result':
        saved = json.loads((experiment.output / 'result.json').read_text())
        assert saved['result'] == 'EVIDENCE_WRITE_FAILED' and saved['evidence_write_failed']
    else:
        assert experiment.events[-1]['event'] == 'SESSION_END'
        assert experiment.events[-1]['result'] == 'EVIDENCE_WRITE_FAILED'


@pytest.mark.parametrize('robot', ['ranger', 'go2'])
@pytest.mark.parametrize('fault', ['marker', 'marker_create', 'marker_timer'])
def test_marker_failure_does_not_gate_experiment_or_stop(experiment, robot, fault):
    experiment.fault = fault
    assert experiment.run(robot) == 0
    assert experiment.stopped and experiment.closed and experiment.destroyed
    assert not experiment.adapter.motion_fault
    assert experiment.report['result'] == 'TARGET_APPROACHED'
    assert experiment.report['stop_confirmed'] is True
    assert 'RViz unavailable' in experiment.report['visualization_error']
    assert experiment.events[-1]['event'] == 'SESSION_END'


@pytest.mark.parametrize('fault', ['events', 'result'])
def test_unconfirmed_stop_takes_priority_over_evidence_error(experiment, monkeypatch, fault):
    experiment.fault = fault
    monkeypatch.setattr(check, 'stop_owned', lambda *a:False)
    with pytest.raises(check.StopUnconfirmed):
        experiment.run('ranger')
    assert experiment.closed and experiment.destroyed
    assert experiment.report['stop_confirmed'] is False
    assert experiment.report['evidence_write_failed'] is True
    if experiment.events:
        assert experiment.events[-1]['state'] == 'STOP_UNCONFIRMED'


@pytest.mark.parametrize('robot', ['ranger', 'go2'])
def test_pending_vlm_evidence_cannot_complete_acceptance(experiment, robot):
    experiment.adapter.vision.job = S(done=lambda:False)
    assert experiment.run(robot) == 1
    assert experiment.stopped and experiment.closed and experiment.destroyed
    assert experiment.report['result'] == 'EVIDENCE_INCOMPLETE'
    assert experiment.report['motion_result'] == 'TARGET_APPROACHED'
    assert experiment.report['evidence_complete'] is False
    assert experiment.report['evidence_write_failed'] is False
    saved = json.loads((experiment.output / 'result.json').read_text())
    assert saved['vlm_evidence_pending'] and saved['evidence_complete'] is False
    assert experiment.events[-1]['result'] == 'EVIDENCE_INCOMPLETE'


@pytest.mark.parametrize('confirmed', [True, False])
def test_terminal_output_error_never_masks_stop_result(experiment, monkeypatch, confirmed):
    def broken_terminal(report):
        experiment.report = report
        raise BrokenPipeError('terminal closed')
    monkeypatch.setattr(check, 'emit', broken_terminal)
    if confirmed:
        assert experiment.run('ranger') == 0
        assert experiment.stopped
    else:
        monkeypatch.setattr(check, 'stop_owned', lambda *a:False)
        with pytest.raises(check.StopUnconfirmed):
            experiment.run('ranger')
    assert experiment.closed and experiment.destroyed
    assert experiment.report['stop_confirmed'] is confirmed
    assert not experiment.adapter.motion_fault
