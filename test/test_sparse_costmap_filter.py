import numpy as np
import pytest

pytest.importorskip("cv2")

from vlm_nav.sparse_costmap_filter import filter_sparse_lethal_cells


def test_isolated_and_two_cell_obstacles_are_removed():
    grid = np.zeros((11, 11), dtype=np.uint8)
    grid[2, 2] = 254
    grid[7, 7] = 254
    grid[7, 8] = 254

    filtered, removed = filter_sparse_lethal_cells(
        grid,
        resolution=0.05,
        neighborhood_radius=0.20,
        min_occupied_cells=3,
    )

    assert removed == 3
    assert np.count_nonzero(filtered == 254) == 0


def test_dense_obstacle_is_retained():
    grid = np.zeros((11, 11), dtype=np.uint8)
    grid[5, 5] = 254
    grid[5, 6] = 254
    grid[6, 5] = 254

    filtered, removed = filter_sparse_lethal_cells(
        grid,
        resolution=0.05,
        neighborhood_radius=0.20,
        min_occupied_cells=3,
    )

    assert removed == 0
    assert np.array_equal(filtered, grid)


def test_metric_radius_excludes_distant_cells():
    grid = np.zeros((15, 15), dtype=np.uint8)
    grid[7, 3] = 254
    grid[7, 7] = 254
    grid[7, 11] = 254

    filtered, removed = filter_sparse_lethal_cells(
        grid,
        resolution=0.05,
        neighborhood_radius=0.15,
        min_occupied_cells=2,
    )

    assert removed == 3
    assert np.count_nonzero(filtered == 254) == 0


def test_unknown_and_inflation_costs_are_preserved():
    grid = np.zeros((5, 5), dtype=np.uint8)
    grid[1, 1] = 255
    grid[2, 2] = 253
    grid[3, 3] = 254

    filtered, removed = filter_sparse_lethal_cells(
        grid,
        resolution=0.05,
        neighborhood_radius=0.20,
        min_occupied_cells=3,
    )

    assert removed == 1
    assert filtered[1, 1] == 255
    assert filtered[2, 2] == 253
    assert filtered[3, 3] == 0


@pytest.mark.parametrize(
    ("resolution", "radius", "threshold"),
    [(0.0, 0.2, 3), (0.05, -0.1, 3), (0.05, 0.2, 0)],
)
def test_invalid_filter_parameters_are_rejected(resolution, radius, threshold):
    with pytest.raises(ValueError):
        filter_sparse_lethal_cells(
            np.zeros((3, 3), dtype=np.uint8),
            resolution=resolution,
            neighborhood_radius=radius,
            min_occupied_cells=threshold,
        )
