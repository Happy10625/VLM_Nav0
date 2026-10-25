"""Diagnostic snapshots must not mix experiments or mistake stale data for stop."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as S

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def monitor():
    script = ROOT / 'scripts/ranger_monitor.py'
    assert script.exists(), '缺少简明诊断终端'
    spec = importlib.util.spec_from_file_location('ranger_monitor', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def append(path, **row):
    with path.open('a') as stream:
        stream.write(json.dumps(row, ensure_ascii=False) + '\n')


def test_partial_event_waits_for_complete_line_and_follow_resets_old_result(monitor, tmp_path):
    first = tmp_path / 'first'; first.mkdir()
    events = first / 'events.jsonl'
    events.write_text('{"state":"NAVIGATING"')
    view = monitor.ExperimentView()
    view.poll(first)
    assert view.latest == {}
    with events.open('a') as stream: stream.write(',"viewpoint_id":"p1"}\n')
    view.poll(first)
    assert view.latest['state'] == 'NAVIGATING'
    (first / 'result.json').write_text('{"result":"OLD_FAILURE","stop_confirmed":true}')
    view.poll(first)
    assert view.result['result'] == 'OLD_FAILURE'
    second = tmp_path / 'second'; second.mkdir()
    view.poll(second)
    assert view.latest == {} and view.result == {}
    assert view.path_count == 0


def test_last_motion_is_not_overwritten_by_old_goal_or_session_fields(monitor, tmp_path):
    events = tmp_path / 'events.jsonl'
    append(events, state='NAVIGATING', event='MOTION_REQUESTED', goal_id='nav', goal_kind='VIEWPOINT_NAV')
    append(events, state='NAVIGATING', event='MOTION_TERMINAL', goal_id='nav', status='SUCCEEDED')
    append(events, state='SCANNING', event='MOTION_REQUESTED', goal_id='spin', goal_kind='SCAN_SPIN')
    append(events, state='SCANNING', event='MOTION_ACCEPTED', goal_id='spin')
    append(events, state='STOPPED', event='SESSION_END', goal_id='nav', goal_kind='VIEWPOINT_NAV', reason='RuntimeError')
    view = monitor.ExperimentView(); view.poll(tmp_path)
    assert view.motion['kind'] == 'SCAN_SPIN'
    assert view.last_stage == 'SCANNING'
    text = monitor.render(view, {}, now=100., ros_now=100., can_delta=10)
    assert 'STOPPED' in text and 'SCANNING' in text and 'RuntimeError' in text
    assert 'SCAN_SPIN' in text
    assert '{' not in text and len(text.splitlines()) <= 9


def test_compact_velocity_arrays_mark_stale_data_and_can_loss(monitor):
    view = monitor.ExperimentView()
    telemetry = {'odom': {'values': [.1, -.2, .3], 'at': 99.9, 'stamp': 99.9},
                 'cmd': {'values': [.2, 0., -.1], 'at': 99.8}}
    text = monitor.render(view, telemetry, now=100., ros_now=100., can_delta=12)
    assert '[+0.100, -0.200, +0.300]' in text
    assert '[+0.200, +0.000, -0.100]' in text
    assert 'CAN:+12' in text
    stale = monitor.render(view, telemetry, now=103., ros_now=103., can_delta=0)
    assert '速度过期' in stale and 'CAN无接收增长' in stale
    assert '停稳' not in stale


def test_missing_and_nonfinite_velocity_never_print_fake_zero(monitor):
    view = monitor.ExperimentView()
    text = monitor.render(view, {}, now=100., ros_now=100., can_delta=None)
    assert '[--, --, --]' in text and '[+0.000, +0.000, +0.000]' not in text
    bad = {'odom': {'values': [float('nan'), 0., 0.], 'at': 100., 'stamp': 100.}}
    assert '非法速度' in monitor.render(view, bad, now=100., ros_now=100., can_delta=1)


def test_chassis_command_direction_and_fast_reversals_survive_screen_refresh(monitor):
    telemetry = {}
    for at, wz in ((99., .2), (99.1, 0.), (99.2, -.2), (99.3, .01), (99.4, .2)):
        message = S(linear=S(x=.1, y=0.), angular=S(z=wz))
        monitor.receive_velocity(telemetry, 'cmd', message, at)
    text = monitor.render(monitor.ExperimentView(), telemetry, now=99.5, ros_now=99.5, can_delta=1)
    assert '/cmd_vel' in text and '[+0.100, +0.000, +0.200]' in text
    assert '逆时针' in text and '10s换向:2' in text
    assert len(text.splitlines()) == 9
    monitor.receive_velocity(telemetry, 'cmd', S(linear=S(x=0., y=0.), angular=S(z=-.2)), 99.6)
    text = monitor.render(monitor.ExperimentView(), telemetry, now=99.7, ros_now=99.7, can_delta=1)
    assert '顺时针' in text and '10s换向:3' in text
    text = monitor.render(monitor.ExperimentView(), telemetry, now=111., ros_now=111., can_delta=1)
    assert '指令过期' in text and '10s换向:0' in text
    assert '顺时针' not in text and '逆时针' not in text


@pytest.mark.parametrize('wz,expected', [(0., '零/微小角速度'), (float('nan'), '非法速度')])
def test_command_zero_and_invalid_values_are_not_counted_as_reversals(monitor, wz, expected):
    telemetry = {}
    monitor.receive_velocity(telemetry, 'cmd', S(linear=S(x=0., y=0.), angular=S(z=wz)), 100.)
    text = monitor.render(monitor.ExperimentView(), telemetry, now=100., ros_now=100., can_delta=1)
    assert expected in text and '10s换向:0' in text
    assert '顺时针' not in text and '逆时针' not in text


def test_command_history_is_bounded_and_old_samples_do_not_add_reversals(monitor):
    telemetry = {}
    for i in range(400):
        monitor.receive_velocity(telemetry, 'cmd',
            S(linear=S(x=0., y=0.), angular=S(z=.2 if i % 2 else -.2)), i / 20.)
    assert len(telemetry['cmd_history']) <= 201
    text = monitor.render(monitor.ExperimentView(), telemetry, now=30., ros_now=30., can_delta=1)
    assert '10s换向:0' in text


def test_odometry_receipt_keeps_header_stamp_and_does_not_add_command_history(monitor):
    telemetry = {}
    message = S(twist=S(twist=S(linear=S(x=.1, y=0.), angular=S(z=-.2))),
                header=S(stamp=S(sec=99, nanosec=500000000)))
    monitor.receive_velocity(telemetry, 'odom', message, 100.)
    assert telemetry['odom']['stamp'] == 99.5
    text = monitor.render(monitor.ExperimentView(), telemetry, now=100.1, ros_now=100.1, can_delta=1)
    assert '实测速度[vx,vy,wz]=[+0.100, +0.000, -0.200]' in text
    assert '速度过期' in text and '未收到底盘指令' in text


def test_path_buffer_acceptance_and_clear_are_distinct(monitor, tmp_path):
    events = tmp_path / 'events.jsonl'
    append(events, state='NAVIGATING', event='PATH_BUFFERED', poses=25, wall_time=99.)
    view = monitor.ExperimentView(); view.poll(tmp_path)
    assert '暂存待接受' in monitor.render(view, {}, now=100., ros_now=100., can_delta=1)
    append(events, state='NAVIGATING', event='PATH_RECEIVED', poses=25, wall_time=100.)
    view.poll(tmp_path)
    assert '25点' in monitor.render(view, {}, now=100., ros_now=100., can_delta=1)
    assert '路径接收过期' in monitor.render(view, {}, now=106., ros_now=106., can_delta=1)
    append(events, state='NAVIGATING', event='PATH_CLEARED', reason='navigation canceled')
    view.poll(tmp_path)
    assert view.path_count == 0


def test_broken_line_is_reported_and_log_replacement_resets_state(monitor, tmp_path):
    events = tmp_path / 'events.jsonl'
    events.write_text('bad json\n{"state":"SCANNING"}\n')
    view = monitor.ExperimentView(); view.poll(tmp_path)
    assert view.latest['state'] == 'SCANNING'
    assert '日志坏行' in monitor.render(view, {}, now=100., ros_now=100., can_delta=1)
    replacement = tmp_path / 'new'
    replacement.write_text('{"state":"WAITING_INPUTS"}\n')
    replacement.replace(events)
    view.poll(tmp_path)
    assert view.latest['state'] == 'WAITING_INPUTS' and view.errors == 0


@pytest.mark.parametrize('failure', [
    {'event': 'SCAN_FAILED', 'reason': 'scan spin failed: view 2, action ABORTED'},
    {'failure_reason': 'scan spin failed: view 2, action ABORTED'},
])
def test_failure_detail_survives_cleanup_and_includes_vlm_and_native_reason(monitor, tmp_path, monkeypatch, failure):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    (tmp_path / 'behavior_server_42_1000.log').write_text(
        '[WARN] [88.0] [behavior_server]: old collision\n'
        '[WARN] [102.0] [behavior_server]: Collision Ahead - Exiting Spin\n'
        '[WARN] [102.1] [behavior_server]: spin failed\n'
        '[WARN] [105.0] [behavior_server]: unrelated later failure\n')
    events = tmp_path / 'events.jsonl'
    append(events, state='SCANNING', event='VLM_VIEW_RESULT', view_index=0,
           observations=[{'status': 'NOT_DETECTED'}], error_type=None)
    append(events, state='SCANNING', event='MOTION_REQUESTED', goal_id='spin',
           goal_kind='SCAN_SPIN', wall_time=100.)
    append(events, state='SCANNING', event='MOTION_TERMINAL', goal_id='spin',
           status='ABORTED', wall_time=102.2)
    append(events, state='SCANNING', **failure)
    append(events, state='STOPPING', reason='RuntimeError')
    append(events, state='STOPPING', event='STOP_COMMAND_SENT', reason='independent acceptance completion/cancel')
    append(events, state='STOPPED', event='SESSION_END', reason='RuntimeError')
    view = monitor.ExperimentView(); view.poll(tmp_path)
    text = monitor.render(view, {}, now=103., ros_now=103., can_delta=1)
    assert 'view 2, action ABORTED' in text
    assert 'Collision Ahead - Exiting Spin' in text
    assert 'NOT_DETECTED' in text
    assert 'old collision' not in text and 'unrelated later failure' not in text
    assert len(text.splitlines()) <= 9


def test_native_failure_without_log_never_claims_collision_and_resets_on_new_action(monitor, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    events = tmp_path / 'events.jsonl'
    append(events, event='MOTION_REQUESTED', goal_id='spin', goal_kind='SCAN_SPIN', wall_time=100.)
    append(events, event='MOTION_TERMINAL', goal_id='spin', status='ABORTED', wall_time=102.)
    view = monitor.ExperimentView(); view.poll(tmp_path)
    text = monitor.render(view, {}, now=103., ros_now=103., can_delta=1)
    assert '未取得具体原因' in text and 'Collision Ahead' not in text
    append(events, event='MOTION_REQUESTED', goal_id='new', goal_kind='SCAN_SPIN', wall_time=104.)
    view.poll(tmp_path)
    assert '未取得具体原因' not in monitor.render(view, {}, now=105., ros_now=105., can_delta=1)


def test_result_only_failure_and_failed_checks_are_visible(monitor, tmp_path):
    (tmp_path / 'result.json').write_text(json.dumps({
        'result': 'RuntimeError', 'failure_reason': 'camera TF unavailable',
        'health_details': {'checks': {'rgbd_ready': False, 'odom_fresh': True}},
    }))
    view = monitor.ExperimentView(); view.poll(tmp_path)
    text = monitor.render(view, {}, now=100., ros_now=100., can_delta=1)
    assert 'camera TF unavailable' in text and 'rgbd_ready' in text


def test_operational_reason_remains_visible_after_cleanup_events(monitor, tmp_path):
    events = tmp_path / 'events.jsonl'
    append(events, state='PLAN_FAILED', reason='planner returned no permitted path')
    append(events, event='STOP_COMMAND_SENT', reason='independent acceptance completion/cancel')
    view = monitor.ExperimentView(); view.poll(tmp_path)
    text = monitor.render(view, {}, now=100., ros_now=100., can_delta=1)
    assert 'PLAN_FAILED/planner returned no permitted path' in text
    assert 'independent acceptance completion/cancel' not in text


def test_successful_progress_clears_old_waiting_reason(monitor, tmp_path):
    events = tmp_path / 'events.jsonl'
    append(events, state='WAITING_INPUTS', reason='waiting for RGB-D/image-time TF')
    append(events, state='SCANNING', reason='')
    view = monitor.ExperimentView(); view.poll(tmp_path)
    text = monitor.render(view, {}, now=100., ros_now=100., can_delta=1)
    assert 'waiting for RGB-D' not in text
