import numpy as np

from vlm_nav.geometry import depth_at_pixel, project_pixel, snap_to_free_cell
from vlm_nav.vlm_client import validate_vlm_result


def test_synthetic_vlm_pixel_reaches_a_free_map_candidate():
    payload = {
        "target_visible": True,
        "confidence": 0.9,
        "target_pixel": {"u": 4, "v": 4},
        "waypoints": [{"u": 4, "v": 6}],
    }
    result = validate_vlm_result(payload, width=9, height=9)
    depth_image = np.full((9, 9), 2.0, dtype=np.float32)
    depth = depth_at_pixel(depth_image, result.waypoints[0].u, result.waypoints[0].v)
    transform = np.eye(4)
    point = project_pixel(
        result.waypoints[0].u,
        result.waypoints[0].v,
        depth,
        (4.0, 4.0, 4.0, 4.0),
        transform,
    )
    grid = np.zeros((100, 100), dtype=np.int16)
    snapped = snap_to_free_cell(point[:2], grid, -5.0, -5.0, 0.1, 0.25)
    assert snapped is not None
