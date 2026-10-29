import copy
import json

import pytest

from multi_agent_nav.go2_check import emit


def test_console_omits_coordinate_arrays_but_keeps_diagnostics(capsys):
    report = {
        'health': {'footprint': [[1, 2], [3, 4]], 'path': [], 'path_valid': False},
        'preflight': {'path': [{'x': 1, 'y': 2}], 'reason': 'blocked'},
        'rejected': ['fresh costmap required'], 'result': 'REJECTED',
        'stop_confirmed': False, 'evidence_write_failed': True,
        'vlm_evidence_root': '/tmp/explore.20261011_120000/vlm',
    }
    original = copy.deepcopy(report)
    emit(report)
    output = json.loads(capsys.readouterr().out)
    assert output['health'] == {'footprint_count': 2, 'path_count': 0, 'path_valid': False}
    assert output['preflight'] == {'path_count': 1, 'reason': 'blocked'}
    for key in ('rejected', 'result', 'stop_confirmed', 'evidence_write_failed', 'vlm_evidence_root'):
        assert output[key] == report[key]
    assert report == original  # Full result.json must retain the original coordinates.


@pytest.mark.parametrize('outcome', [
    {'result': 'TARGET_APPROACHED', 'motion_sent': True, 'stop_confirmed': True,
     'last_native_action_uuid': 'nav-goal'},
    {'result': 'operator interrupt', 'motion_sent': True, 'stop_confirmed': False,
     'stop_error': 'RuntimeError', 'evidence_write_failed': True},
    {'rejected': ['active map_server YAML does not match configured map_path'],
     'motion_sent': False},
])
def test_ranger_explore_console_keeps_only_outcome_and_motion(outcome, capsys):
    report = dict(mode='local_ranger_explore', robot_id='ranger', target_id='B',
        map_files={'yaml_path': '/tmp/map.yaml', 'resolution': .05},
        active_map_yaml='/tmp/map.yaml', costmap_shape=[100, 100],
        arrival_contract={'target_success_radius': .81},
        configured_nav2_tolerances={'planner_tolerance': .76},
        semantic_search={'points': [{'id': 'p1', 'pose': [1, 2, 0]}]},
        health={'pose': {'x': 0, 'y': 0, 'yaw': 0}, 'motion_terminal': True},
        health_details={'checks': {'local_costmap': False}},
        future_configuration_field={'value': 42}, **outcome)
    original = copy.deepcopy(report)
    emit(report)
    assert json.loads(capsys.readouterr().out) == dict(target_id='B', **outcome)
    assert report == original


@pytest.mark.parametrize('mode', ['configuration_only', 'local_ranger_inspect', 'local_go2_explore'])
def test_diagnostic_and_go2_reports_keep_their_fields(mode, capsys):
    report = dict(mode=mode, robot_id='ranger' if 'go2' not in mode else 'go2',
                  map_files={'yaml_path': '/tmp/map.yaml'}, motion_sent=False)
    emit(report)
    assert json.loads(capsys.readouterr().out) == report


def test_ranger_entry_rejection_keeps_full_evidence(monkeypatch, tmp_path, capsys):
    from multi_agent_nav import go2_check, ranger_check
    from multi_agent_nav.config import load_config
    config = load_config()
    config['robot_ranger']['stop_margin'] = None
    config['scene']['viewpoints']['ranger'] = [{'id': 'p1', 'pose': [1., 0., 0.]}]
    monkeypatch.setattr(go2_check, 'load_config', lambda _: config)
    def forbidden(*args, **kwargs):
        pytest.fail('rejected configuration must not acquire motion or start ROS')
    monkeypatch.setattr(go2_check, 'motion_slot', forbidden)
    monkeypatch.setattr(go2_check, 'run_ros', forbidden)
    output = tmp_path / 'evidence'
    assert ranger_check.main(['--action', 'explore', '--allow-motion', '--target', 'cup',
        '--strategy', 'fixed_viewpoints', '--output-dir', str(output)]) == 1
    console = json.loads(capsys.readouterr().out)
    saved = json.loads((output / 'result.json').read_text())
    assert console == dict(target_id='B', rejected=saved['rejected'], motion_sent=False)
    assert any('stop margin' in reason for reason in console['rejected'])
    assert 'map_files' in saved and 'arrival_contract' in saved and 'viewpoints' in saved
    event = json.loads((output / 'events.jsonl').read_text())
    assert event['state'] == 'REJECTED' and event['reasons'] == console['rejected']
