import math

import numpy as np

from vlm_nav.geometry import (
    classify_standoff_cell,
    depth_at_pixel,
    depth_at_pixel_with_reason,
    project_pixel,
    select_frontier,
    snap_to_free_cell,
    TargetTracker,
)


def test_depth_uses_near_median_and_rejects_sparse_pixels():
    depth = np.full((21, 21), np.nan, dtype=np.float32)
    depth[7:14, 7:14] = 2.0
    depth[8, 8] = 4.0
    assert math.isclose(
        depth_at_pixel(depth, 10, 10, radius=4, min_samples=10), 2.0
    )
    assert depth_at_pixel(np.full((5, 5), np.nan), 2, 2) is None
    assert depth_at_pixel(depth, -1, 2) is None


def test_depth_rejection_reports_actionable_reason():
    depth = np.full((11, 11), np.nan, dtype=np.float32)
    value, reason = depth_at_pixel_with_reason(
        depth, 5, 5, radius=3, min_samples=8
    )
    assert value is None
    assert reason == "insufficient_valid_depth_samples:0/8"

    value, reason = depth_at_pixel_with_reason(depth, 20, 5)
    assert value is None
    assert reason.startswith("pixel_out_of_bounds:")


def test_depth_rejection_distinguishes_target_beyond_reliable_range():
    depth = np.full((11, 11), 6.8, dtype=np.float32)

    value, reason = depth_at_pixel_with_reason(
        depth,
        5,
        5,
        radius=3,
        max_depth=6.0,
        min_samples=8,
    )

    assert value is None
    assert reason == "depth_out_of_range:median_m=6.800,limit_m=6.000"


def test_pixel_projection_uses_intrinsics_and_timestamp_transform():
    transform = np.eye(4)
    transform[:3, 3] = [1.0, 2.0, 3.0]
    point = project_pixel(320, 240, 2.0, (500.0, 500.0, 320.0, 240.0), transform)
    assert np.allclose(point, [1.0, 2.0, 5.0])


def test_snap_to_nearest_free_cell_and_reject_outside_map():
    grid = np.full((10, 10), 100, dtype=np.int16)
    grid[5, 6] = 0
    snapped = snap_to_free_cell((0.51, 0.51), grid, 0.0, 0.0, 0.1, 0.2)
    assert np.allclose(snapped, (0.65, 0.55))
    assert snap_to_free_cell((5.0, 5.0), grid, 0.0, 0.0, 0.1, 0.2) is None


def test_unknown_standoff_is_provisional_when_no_known_obstacle_is_nearby():
    grid = np.full((40, 40), -1, dtype=np.int16)
    status, point = classify_standoff_cell(
        (2.05, 2.05),
        grid,
        0.0,
        0.0,
        0.1,
        free_snap_distance=0.2,
        footprint_length=0.72,
        footprint_width=0.50,
        allow_unknown=True,
    )

    assert status == "unknown"
    assert np.allclose(point, (2.05, 2.05))


def test_one_known_obstacle_is_sparse_noise_for_unknown_standoff():
    grid = np.full((40, 40), -1, dtype=np.int16)
    grid[20, 23] = 100
    status, point = classify_standoff_cell(
        (2.05, 2.05),
        grid,
        0.0,
        0.0,
        0.1,
        free_snap_distance=0.2,
        footprint_length=0.72,
        footprint_width=0.50,
        min_occupied_cells=3,
        occupied_threshold=50,
        allow_unknown=True,
    )

    assert status == "unknown"
    assert np.allclose(point, (2.05, 2.05))


def test_two_known_obstacles_are_sparse_noise_for_unknown_standoff():
    grid = np.full((40, 40), -1, dtype=np.int16)
    grid[20, 22:24] = 100
    status, point = classify_standoff_cell(
        (2.05, 2.05),
        grid,
        0.0,
        0.0,
        0.1,
        free_snap_distance=0.2,
        footprint_length=0.72,
        footprint_width=0.50,
        min_occupied_cells=3,
        occupied_threshold=50,
        allow_unknown=True,
    )

    assert status == "unknown"
    assert np.allclose(point, (2.05, 2.05))


def test_three_known_obstacles_veto_standoff_footprint():
    grid = np.full((40, 40), -1, dtype=np.int16)
    grid[20, 21:24] = 100
    status, point = classify_standoff_cell(
        (2.05, 2.05),
        grid,
        0.0,
        0.0,
        0.1,
        free_snap_distance=0.2,
        footprint_yaw=0.0,
        footprint_length=0.72,
        footprint_width=0.50,
        min_occupied_cells=3,
        occupied_threshold=50,
        allow_unknown=True,
    )

    assert status == "occupied"
    assert point is None


def test_known_free_standoff_is_preferred_over_unknown():
    grid = np.full((40, 40), -1, dtype=np.int16)
    grid[20, 21] = 0
    status, point = classify_standoff_cell(
        (2.05, 2.05),
        grid,
        0.0,
        0.0,
        0.1,
        free_snap_distance=0.2,
        footprint_length=0.72,
        footprint_width=0.50,
        allow_unknown=True,
    )

    assert status == "known_free"
    assert np.allclose(point, (2.15, 2.05))


def test_unknown_standoff_rejects_map_edge_that_cannot_fit_robot_clearance():
    grid = np.full((20, 20), -1, dtype=np.int16)
    status, point = classify_standoff_cell(
        (0.15, 1.05),
        grid,
        0.0,
        0.0,
        0.1,
        footprint_length=0.72,
        footprint_width=0.50,
        allow_unknown=True,
    )

    assert status == "outside"
    assert point is None


def test_frontier_selection_only_uses_reachable_free_space():
    grid = np.full((20, 20), -1, dtype=np.int16)
    grid[5:15, 5:15] = 0
    grid[9:11, 9:11] = 100
    selected, clusters = select_frontier(grid, (6, 6), min_cells=3)
    assert selected is not None
    x, y = selected["goal"]
    assert grid[y, x] == 0
    assert clusters


def test_target_requires_three_spatially_consistent_observations():
    tracker = TargetTracker(required_frames=3, confirmation_radius=0.35)
    assert tracker.update((1.0, 2.0, 0.4)) is None
    assert tracker.update((1.1, 2.0, 0.5)) is None
    confirmed = tracker.update((0.9, 2.1, 0.45))
    assert np.allclose(confirmed, (1.0, 2.0, 0.45))
    # A distant observation starts a new confirmation window.
    assert tracker.update((3.0, 3.0, 0.4)) is None


def test_target_tracker_reports_progress_and_inconsistent_reset_distance():
    """Catch silent confirmation-window resets that look like a hung state."""
    tracker = TargetTracker(required_frames=3, confirmation_radius=0.35)

    tracker.update((1.0, 2.0, 0.4))
    tracker.update((1.1, 2.0, 0.5))
    tracker.update((2.0, 2.0, 0.4))

    assert tracker.progress == 1
    assert tracker.reset_count == 1
    assert math.isclose(tracker.last_jump_distance, 0.95)
