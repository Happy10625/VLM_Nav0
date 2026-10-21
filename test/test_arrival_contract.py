import math
from pathlib import Path
import pytest
from vlm_nav.arrival import arrival_decision, target_distance, stop_radius, load_contract


@pytest.mark.parametrize('distance,expected', [(.81,'SUCCESS'),(.81001,'FAILED'),(1.13,'FAILED'),(math.nan,'FAILED'),(math.inf,'FAILED')])
def test_success_requires_finite_distance_inside_original_boundary(distance, expected):
    assert arrival_decision(distance,.81,1,1)==expected


def test_original_config_and_retry_boundary():
    root=Path(__file__).resolve().parents[1]
    contract=load_contract([root/'config/robot.yaml',root/'config/robot_go2.yaml'])
    assert contract==dict(target_success_radius=.81,approach_cancel_radius=.89,arrival_retry_limit=1)
    assert stop_radius(.81,.89,0)==.89
    assert stop_radius(.81,.89,1)==.81
    assert arrival_decision(1.13,.81,0,1)=='RETRY'
    assert target_distance(None,(1.,0.))==math.inf
    assert target_distance((math.nan,0.),(1.,0.))==math.inf


def test_bad_config_is_rejected(tmp_path):
    file=tmp_path/'bad.yaml'
    file.write_text('vlm_nav:\n  ros__parameters:\n    target_success_radius: 0.81\n    approach_cancel_radius: 0.5\n    arrival_retry_limit: 1\n')
    with pytest.raises(ValueError):load_contract([file])
