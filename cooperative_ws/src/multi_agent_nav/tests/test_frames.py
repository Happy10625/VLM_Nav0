import math
import numpy as np
import pytest
from multi_agent_nav.models import Pose2D
from multi_agent_nav.frame_adapter import FrameAdapter
from multi_agent_nav.calibration import fit_alignment

def test_rotated_map_roundtrip_vertices_and_identity():
    frame = FrameAdapter('ranger_map', 'v2', 'lab_ref', 'a', (2, -3, .8))
    pose = Pose2D(4, 5, 3)
    out = frame.to_common(pose)
    assert -math.pi <= out.yaw <= math.pi
    back = frame.to_local(out)
    assert (back.x, back.y, back.yaw) == pytest.approx((4, 5, 3))
    polygon = [(0, 0), (1, 0), (1, 1)]
    assert frame.polygon_to_local(polygon) == [(frame.to_local(Pose2D(*p)).x,
                                               frame.to_local(Pose2D(*p)).y) for p in polygon]
    identity = FrameAdapter('go2_map', 'v1', 'lab_ref', 'a', (0, 0, 0))
    assert identity.to_common(pose) == pose
    with pytest.raises(ValueError):
        frame.validate_ids('ranger_map', 'rebuilt', 'lab_ref', 'a')

def test_fit_known_transform_and_held_out_error():
    src = [(0, 0), (3, 0), (0, 2), (2, 2)]
    frame = FrameAdapter('r', 'v', 'lab_ref', 'a', (2, -3, .8))
    dst = [(frame.to_common(Pose2D(*p)).x, frame.to_common(Pose2D(*p)).y) for p in src]
    hold = [((4, 1), (frame.to_common(Pose2D(4, 1)).x, frame.to_common(Pose2D(4, 1)).y))]
    report = fit_alignment(src, dst, hold, {'fit_max': .01, 'validation_max': .01, 'geometry_ratio_min': .02})
    assert report['validated']
    assert report['transform'] == pytest.approx([2, -3, .8])
    assert not fit_alignment(src, dst, [((4, 1), (999, 1))], report['thresholds'])['validated']
    assert not fit_alignment(src, dst, [], report['thresholds'])['validated']

@pytest.mark.parametrize('points', [[(0, 0)]*3, [(0, 0), (1, 0), (2, 0)],
                                  [(0, 0), (1, .00001), (2, 0)]])
def test_degenerate_reference_geometry_rejected(points):
    report = fit_alignment(points, points, [((0, 1), (0, 1))],
                           {'fit_max': .01, 'validation_max': .01, 'geometry_ratio_min': .02})
    assert not report['validated']

def test_reflection_not_accepted_as_rigid_alignment():
    src = [(0, 0), (2, 0), (0, 2)]
    report = fit_alignment(src, [(0, 0), (-2, 0), (0, 2)], [((1, 1), (-1, 1))],
                           {'fit_max': .01, 'validation_max': .01, 'geometry_ratio_min': .02})
    assert not report['validated']
