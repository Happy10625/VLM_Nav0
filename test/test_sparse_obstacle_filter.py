import numpy as np
import pytest

pytest.importorskip("cv2")

from vlm_nav.sparse_obstacle_filter import filter_sparse_obstacle_points


def apply(points, **overrides):
    parameters = {
        "resolution": 0.05,
        "footprint_length": 0.72,
        "footprint_width": 0.50,
        "min_obstacle_points": 3,
        "min_height": 0.05,
        "max_height": 1.50,
        "max_range": 20.0,
    }
    parameters.update(overrides)
    return filter_sparse_obstacle_points(np.asarray(points), **parameters)


def test_one_or_two_obstacle_cells_inside_body_window_are_noise():
    points = [(1.00, 0.00, 0.5), (1.25, 0.10, 0.5)]

    keep, removed = apply(points)

    assert removed == 2
    assert not keep.any()


def test_three_obstacle_cells_inside_body_window_are_retained():
    points = [
        (1.00, 0.00, 0.5),
        (1.15, 0.10, 0.5),
        (1.30, -0.10, 0.5),
    ]

    keep, removed = apply(points)

    assert removed == 0
    assert keep.all()


def test_points_outside_body_window_do_not_combine_into_an_obstacle():
    points = [(0.0, 0.0, 0.5), (1.0, 0.0, 0.5), (2.0, 0.0, 0.5)]

    keep, removed = apply(points)

    assert removed == 3
    assert not keep.any()


def test_duplicate_returns_in_one_xy_cell_count_as_one_obstacle_point():
    points = [(1.0, 0.0, 0.2), (1.0, 0.0, 0.4), (1.0, 0.0, 0.6)]

    keep, removed = apply(points)

    assert removed == 3
    assert not keep.any()


def test_non_obstacle_height_points_pass_through_without_affecting_count():
    points = [(1.0, 0.0, 0.0), (1.1, 0.0, 0.5)]

    keep, removed = apply(points)

    assert removed == 1
    assert keep.tolist() == [True, False]
