import copy
import json

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
