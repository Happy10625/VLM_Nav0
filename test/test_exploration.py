import numpy as np
import pytest

from vlm_nav.exploration import (
    clip_polyline_to_length,
    clip_polyline_to_radius,
    direct_standoff_goal,
    max_polyline_deviation,
    render_frontier_map,
    sample_polyline,
    scan_montage,
)
from vlm_nav.models import FrontierCandidate


def test_scan_montage_is_fixed_size_and_does_not_modify_inputs():
    images = [np.zeros((720, 1280, 3), dtype=np.uint8) for _ in range(8)]
    montage = scan_montage(images, [index * 0.785 for index in range(8)])
    assert montage.shape == (360, 1280, 3)
    assert np.count_nonzero(montage) > 0
    assert all(np.count_nonzero(image) == 0 for image in images)


def test_frontier_map_contains_candidate_annotations():
    grid = np.zeros((100, 100), dtype=np.int16)
    grid[:, 80:] = -1
    candidate = FrontierCandidate(1, 2.0, 0.0, 0.0, 2.0, 20)
    rendered = render_frontier_map(
        grid,
        (-5.0, -5.0),
        0.1,
        (0.0, 0.0, 0.0),
        (0.0, 0.0),
        3.0,
        [candidate],
    )
    assert rendered.shape == (720, 720, 3)
    assert len(np.unique(rendered.reshape(-1, 3), axis=0)) > 4


def test_polyline_sampling_uses_arc_length_and_returns_16_segments():
    samples, total = sample_polyline([(0, 0), (1, 0), (1, 3)], 16)
    assert len(samples) == 17
    assert total == 4.0
    assert samples[0] == (0.0, 0.0)
    assert samples[8] == (1.0, 1.0)
    assert samples[-1] == (1.0, 3.0)


def test_polyline_is_interpolated_at_first_rolling_radius_exit():
    clipped, was_clipped, travelled = clip_polyline_to_radius(
        [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)],
        center=(0.0, 0.0),
        radius=3.0,
    )

    assert was_clipped
    assert travelled == pytest.approx(3.0)
    assert clipped[-1] == pytest.approx((3.0, 0.0))


def test_polyline_arc_length_prefix_interpolates_inside_an_edge():
    clipped, travelled = clip_polyline_to_length(
        [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0)],
        2.0,
    )

    assert travelled == pytest.approx(2.0)
    assert clipped[-1] == pytest.approx((1.0, 1.0))


def test_easy_case_direct_standoff_goal_faces_target():
    goal = direct_standoff_goal((0.0, 0.0, 1.2), (2.0, 0.0), 0.81)

    assert goal == pytest.approx((1.19, 0.0, 0.0, 2.0))


def test_easy_case_path_deviation_measures_lateral_detour():
    deviation = max_polyline_deviation(
        [(0.0, 0.0), (1.0, 0.20), (2.0, 0.0)],
        (0.0, 0.0),
        (2.0, 0.0),
    )

    assert deviation == pytest.approx(0.20)
