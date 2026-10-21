"""Human-readable acceptance must not conceal failed or missing evidence."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'ranger_check.py'


@pytest.fixture
def check():
    assert SCRIPT.is_file(), '缺少可读的 Ranger 检查脚本'
    spec = importlib.util.spec_from_file_location('ranger_check', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def inspection():
    return {
        'mode': 'local_ranger_inspect', 'robot_id': 'ranger', 'motion_sent': False,
        'robot_local_gates': [], 'single_robot_experiment_gates': [],
        'active_map_matches': True, 'costmap_valid': True, 'rgbd_ready': True,
        'depth_units_verified': True, 'local_observation_ready': True,
        'health': {'localization_valid': True, 'nav_ready': True, 'stopped': True,
                   'motion_terminal': True, 'path_valid': False},
        'health_details': {
            'checks': {k: True for k in ('navigate_server', 'planner_server', 'spin_server',
                                       'motion_fault_clear', 'odom_fresh')},
            'pose_age_s': .02, 'rgbd_frame_age_s': .04, 'odom_age_s': .01,
            'local_costmap': {'reason': 'ok'},
        },
        'health_window': {'non_motion_inputs_stable': True,
                          'post_input_ready_observed_s': 9.5},
    }


def output(check, rows):
    return '\n'.join(check.format_check(*row) for row in rows)


def test_healthy_readonly_inspection_does_not_require_execution_path(check):
    rows = check.inspection_checks(inspection(), 0)
    assert all(passed for passed, _ in rows)
    assert '9.50' in output(check, rows)
    assert 'JSON' not in output(check, rows)


def test_rgbd_tf_and_costmap_failures_are_explained_without_json(check):
    report = inspection()
    report.update(rgbd_ready=False, costmap_valid=False, local_observation_ready=False)
    report['health']['localization_valid'] = False
    report['health_details']['local_costmap']['reason'] = 'tf_unavailable'
    rows = check.inspection_checks(report, 1)
    text = output(check, rows)
    assert sum(not passed for passed, _ in rows) == 3
    assert 'RGB-D' in text and 'TF' in text and 'costmap' in text
    assert '[失败]' in text and '{' not in text


def test_single_map_failure_is_counted_once(check):
    report = inspection()
    report.update(active_map_matches=False, active_map_yaml=None, local_observation_ready=False)
    rows = check.inspection_checks(report, 1)
    assert sum(not passed for passed, _ in rows) == 1
    assert '地图参数未取得' in output(check, rows)


def test_real_map_mismatch_reports_both_paths(check):
    report = inspection()
    report.update(active_map_matches=False, active_map_yaml='/tmp/other.yaml',
                  map_files={'yaml_path': '/tmp/expected.yaml'}, local_observation_ready=False)
    rows = check.inspection_checks(report, 1)
    text = output(check, rows)
    assert sum(not passed for passed, _ in rows) == 1
    assert '/tmp/other.yaml' in text and '/tmp/expected.yaml' in text


def test_unexplained_not_ready_is_not_hidden_by_merged_summary(check):
    report = inspection()
    report['local_observation_ready'] = False
    assert sum(not passed for passed, _ in check.inspection_checks(report, 1)) == 1


@pytest.mark.parametrize('report,code', [({}, 0), (inspection(), 1),
    ({**inspection(), 'motion_sent': True}, 0)])
def test_incomplete_report_failed_process_or_motion_cannot_pass(check, report, code):
    assert any(not passed for passed, _ in check.inspection_checks(report, code))


def test_configuration_gates_are_translated_and_missing_fields_fail(check):
    report = {'mode': 'configuration_only', 'robot_id': 'ranger', 'motion_sent': False,
              'robot_local_gates': [], 'map_files': {'map_digest': 'abc'},
              'single_robot_experiment_gates': ['measured positive stop margin missing',
                                              'ranger camera_calibrated field verification required']}
    text = output(check, check.configuration_checks(report, 1))
    assert '停车余量' in text and '相机外参' in text
    assert '[失败]' in text
    assert any(not passed for passed, _ in check.configuration_checks({}, 0))


def test_can_no_feedback_fails_despite_up_and_healthy_ros_odom(check):
    before = {'up': True, 'state': 'ERROR-ACTIVE', 'bitrate': 500000,
              'rx': 0, 'tx': 0, 'backlog': 11, 'dropped': 148}
    rows = check.can_checks(before, dict(before))
    text = output(check, rows)
    assert any(not passed for passed, _ in rows)
    assert 'CAN' in text and '11' in text and '供电' in text


def test_can_receiving_frames_is_distinct_from_interface_presence(check):
    before = {'up': True, 'state': 'ERROR-ACTIVE', 'bitrate': 500000,
              'rx': 100, 'tx': 1, 'backlog': 0, 'dropped': 0}
    assert all(ok for ok, _ in check.can_checks(before, {**before, 'rx': 200}))
    assert any(not ok for ok, _ in check.can_checks({}, {}))


def test_unknown_send_queue_is_not_reported_as_empty(check):
    before = {'up': True, 'state': 'ERROR-ACTIVE', 'bitrate': 500000, 'rx': 100}
    text = output(check, check.can_checks(before, {**before, 'rx': 200}))
    assert '[失败] CAN 发送队列' in text
    assert '未知' in text


def test_tc_command_failure_preserves_unknown_queue(check, monkeypatch):
    link = [{'flags': ['UP'], 'linkinfo': {'info_data': {
        'state': 'ERROR-ACTIVE', 'bittiming': {'bitrate': 500000}}},
        'stats64': {'rx': {'packets': 100}, 'tx': {'packets': 1}}}]

    def external_command(command, **kwargs):
        if command[0] == 'ip':
            return subprocess.CompletedProcess(command, 0, json.dumps(link), '')
        return subprocess.CompletedProcess(command, 1, '', 'tc error')

    monkeypatch.setattr(check.subprocess, 'run', external_command)
    snapshot = check.can_snapshot('can0')
    assert snapshot.get('backlog') is None


def test_report_parser_handles_ros_noise_but_rejects_truncated_output(check):
    report = inspection()
    raw = '[WARN] startup\n' + json.dumps(report) + '\n[ros2run]: Process exited with failure 1\n'
    assert check.parse_report(raw) == report
    assert check.parse_report('{"health": {') is None
    assert check.parse_report('[ERROR] no report') is None


def test_subprocess_failures_and_timeouts_keep_logs_off_console(check, tmp_path, capsys):
    log = tmp_path / 'failed.log'
    report, code = check.run_report(
        [sys.executable, '-c', 'import json;print(json.dumps({"mode":"configuration_only"}));raise SystemExit(3)'],
        log, 5)
    assert code == 3 and report['mode'] == 'configuration_only'
    assert 'configuration_only' in log.read_text()
    report, code = check.run_report(
        [sys.executable, '-c', 'import time;time.sleep(5)'], tmp_path / 'timeout.log', .05)
    assert code == 124 and report is None
    assert not capsys.readouterr().out


def test_readonly_cli_rejects_motion_flags_before_creating_output(tmp_path):
    assert SCRIPT.is_file(), '缺少可读的 Ranger 检查脚本'
    destination = tmp_path / 'must-not-exist'
    result = subprocess.run([sys.executable, str(SCRIPT), '--allow-motion',
                             '--output-dir', str(destination)], capture_output=True, text=True)
    assert result.returncode == 2
    assert not destination.exists()


def test_shell_entry_does_not_pass_check_options_to_environment_loader():
    result = subprocess.run(['bash', str(SCRIPT.with_name('check_ranger.sh')), '--help'],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert '--hardware-only' in result.stdout
    assert '用法：source' not in result.stdout
