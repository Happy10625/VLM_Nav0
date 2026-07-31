"""Pure geometry, occupancy-grid, and exploration helpers."""

import math
from collections import deque
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np


class TargetTracker:
    """Require spatially consistent observations before confirming a target."""

    def __init__(self, required_frames: int, confirmation_radius: float):
        self.required_frames = max(1, int(required_frames))
        self.confirmation_radius = float(confirmation_radius)
        self.positions = []

    def reset(self):
        self.positions.clear()

    def update(self, point: Sequence[float]) -> Optional[Tuple[float, ...]]:
        candidate = np.asarray(point, dtype=float)
        if candidate.size < 2 or not np.all(np.isfinite(candidate)):
            return None
        if self.positions:
            center = np.median(np.asarray(self.positions), axis=0)
            if np.linalg.norm(candidate[:2] - center[:2]) > self.confirmation_radius:
                self.positions.clear()
        self.positions.append(tuple(float(value) for value in candidate))
        self.positions = self.positions[-self.required_frames :]
        if len(self.positions) < self.required_frames:
            return None
        center = np.median(np.asarray(self.positions), axis=0)
        return tuple(float(value) for value in center)


def depth_at_pixel(
    depth_m: np.ndarray,
    u: int,
    v: int,
    radius: int = 5,
    min_depth: float = 0.20,
    max_depth: float = 5.0,
    min_samples: int = 8,
    max_deviation: float = 0.20,
) -> Optional[float]:
    """Return a robust local depth, rejecting sparse and mixed-depth regions."""
    depth, _ = depth_at_pixel_with_reason(
        depth_m,
        u,
        v,
        radius=radius,
        min_depth=min_depth,
        max_depth=max_depth,
        min_samples=min_samples,
        max_deviation=max_deviation,
    )
    return depth


def depth_at_pixel_with_reason(
    depth_m: np.ndarray,
    u: int,
    v: int,
    radius: int = 5,
    min_depth: float = 0.20,
    max_depth: float = 5.0,
    min_samples: int = 8,
    max_deviation: float = 0.20,
) -> Tuple[Optional[float], str]:
    """Return robust local depth plus an actionable rejection reason."""
    if depth_m.ndim != 2:
        return None, f"invalid_depth_shape:{tuple(depth_m.shape)}"
    if not (0 <= v < depth_m.shape[0] and 0 <= u < depth_m.shape[1]):
        return (
            None,
            f"pixel_out_of_bounds:u={u},v={v},"
            f"width={depth_m.shape[1]},height={depth_m.shape[0]}",
        )
    x0, x1 = max(0, u - radius), min(depth_m.shape[1], u + radius + 1)
    y0, y1 = max(0, v - radius), min(depth_m.shape[0], v + radius + 1)
    values = np.asarray(depth_m[y0:y1, x0:x1], dtype=float).reshape(-1)
    finite_positive = values[np.isfinite(values) & (values > 0.0)]
    beyond_range = finite_positive[finite_positive > max_depth]
    values = finite_positive[
        (finite_positive >= min_depth) & (finite_positive <= max_depth)
    ]
    if values.size < min_samples:
        if beyond_range.size >= min_samples:
            return (
                None,
                f"depth_out_of_range:median_m={float(np.median(beyond_range)):.3f},"
                f"limit_m={max_depth:.3f}",
            )
        return (
            None,
            f"insufficient_valid_depth_samples:{values.size}/{min_samples}",
        )
    median = float(np.median(values))
    near = values[np.abs(values - median) <= max_deviation]
    if near.size < min_samples:
        return (
            None,
            f"mixed_depth_samples:{near.size}/{min_samples},"
            f"median_m={median:.3f},max_deviation_m={max_deviation:.3f}",
        )
    return float(np.median(near)), "ok"


def project_pixel(
    u: int,
    v: int,
    depth: float,
    intrinsics: Sequence[float],
    transform_matrix: np.ndarray,
) -> np.ndarray:
    """Back-project an optical-frame pixel and transform it to the map frame."""
    fx, fy, cx, cy = [float(value) for value in intrinsics]
    if fx <= 0.0 or fy <= 0.0 or depth <= 0.0:
        raise ValueError("invalid camera intrinsics or depth")
    camera = np.array(
        [(u - cx) * depth / fx, (v - cy) * depth / fy, depth, 1.0],
        dtype=float,
    )
    transform = np.asarray(transform_matrix, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError("transform_matrix must be 4x4")
    return (transform @ camera)[:3]


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    result = np.eye(4, dtype=float)
    result[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )
    return result


def transform_matrix(translation: Sequence[float], quaternion: Sequence[float]) -> np.ndarray:
    result = quaternion_matrix(*[float(value) for value in quaternion])
    result[:3, 3] = [float(value) for value in translation]
    return result


def world_to_grid(x: float, y: float, origin_x: float, origin_y: float, resolution: float):
    return int(math.floor((x - origin_x) / resolution)), int(
        math.floor((y - origin_y) / resolution)
    )


def grid_to_world(x: int, y: int, origin_x: float, origin_y: float, resolution: float):
    return origin_x + (x + 0.5) * resolution, origin_y + (y + 0.5) * resolution


def snap_to_free_cell(
    point_xy: Sequence[float],
    grid: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    max_distance: float = 0.25,
) -> Optional[Tuple[float, float]]:
    """Snap a map point to the nearest known-free occupancy cell."""
    gx, gy = world_to_grid(point_xy[0], point_xy[1], origin_x, origin_y, resolution)
    radius = max(0, int(math.ceil(max_distance / resolution)))
    candidates = []
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            x, y = gx + dx, gy + dy
            if 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1] and grid[y, x] == 0:
                candidates.append((dx * dx + dy * dy, x, y))
    if not candidates:
        return None
    _, x, y = min(candidates)
    return grid_to_world(x, y, origin_x, origin_y, resolution)


def classify_standoff_cell(
    point_xy: Sequence[float],
    grid: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    free_snap_distance: float = 0.25,
    footprint_yaw: float = 0.0,
    footprint_length: float = 0.72,
    footprint_width: float = 0.50,
    min_occupied_cells: int = 3,
    occupied_threshold: int = 50,
    allow_unknown: bool = True,
):
    """Classify a target standoff pose against the latest occupancy map.

    Known-free cells are preferred and may be snapped within
    ``free_snap_distance``. An unknown endpoint can be returned as a
    provisional candidate when its full, oriented robot footprint is inside
    the map. Occupied cells inside that footprint are treated as sparse noise
    when fewer than ``min_occupied_cells`` are present; the threshold or more
    still vetoes the candidate.
    """
    if resolution <= 0.0 or grid.ndim != 2:
        return "outside", None

    snapped = snap_to_free_cell(
        point_xy,
        grid,
        origin_x,
        origin_y,
        resolution,
        max(0.0, float(free_snap_distance)),
    )
    probe_xy = snapped if snapped is not None else point_xy
    gx, gy = world_to_grid(
        float(probe_xy[0]),
        float(probe_xy[1]),
        origin_x,
        origin_y,
        resolution,
    )
    height, width = grid.shape
    if not (0 <= gx < width and 0 <= gy < height):
        return "outside", None

    length = float(footprint_length)
    footprint_width = float(footprint_width)
    minimum = int(min_occupied_cells)
    if length <= 0.0 or footprint_width <= 0.0 or minimum < 1:
        return "outside", None
    half_length = 0.5 * length
    half_width = 0.5 * footprint_width
    search_radius = int(
        math.ceil(math.hypot(half_length, half_width) / resolution)
    ) + 1
    yaw = float(footprint_yaw)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    occupied_cells = 0
    footprint_cells = 0
    for dy in range(-search_radius, search_radius + 1):
        for dx in range(-search_radius, search_radius + 1):
            x, y = gx + dx, gy + dy
            cell_x, cell_y = grid_to_world(
                x, y, origin_x, origin_y, resolution
            )
            world_dx = cell_x - float(probe_xy[0])
            world_dy = cell_y - float(probe_xy[1])
            local_x = cosine * world_dx + sine * world_dy
            local_y = -sine * world_dx + cosine * world_dy
            if (
                abs(local_x) > half_length + 1e-9
                or abs(local_y) > half_width + 1e-9
            ):
                continue
            footprint_cells += 1
            if not (0 <= x < width and 0 <= y < height):
                return "outside", None
            if int(grid[y, x]) >= int(occupied_threshold):
                occupied_cells += 1
                if occupied_cells >= minimum:
                    return "occupied", None
    if footprint_cells <= 0:
        return "outside", None

    if snapped is not None:
        return "known_free", snapped

    value = int(grid[gy, gx])
    if value == -1:
        if not allow_unknown:
            return "unknown_rejected", None
        return "unknown", grid_to_world(
            gx, gy, origin_x, origin_y, resolution
        )
    if value >= int(occupied_threshold):
        # The endpoint itself is one of fewer-than-threshold occupied cells,
        # so expose it as usable sparse noise rather than vetoing parking.
        return "known_free", grid_to_world(
            gx, gy, origin_x, origin_y, resolution
        )
    return "uncertain", None


def reachable_free(grid: np.ndarray, robot_cell: Sequence[int]) -> np.ndarray:
    height, width = grid.shape
    start_x, start_y = int(robot_cell[0]), int(robot_cell[1])
    reachable = np.zeros_like(grid, dtype=bool)
    if not (0 <= start_x < width and 0 <= start_y < height) or grid[start_y, start_x] != 0:
        return reachable
    queue = deque([(start_x, start_y)])
    reachable[start_y, start_x] = True
    while queue:
        x, y = queue.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if (
                0 <= nx < width
                and 0 <= ny < height
                and not reachable[ny, nx]
                and grid[ny, nx] == 0
            ):
                reachable[ny, nx] = True
                queue.append((nx, ny))
    return reachable


def frontier_clusters(grid: np.ndarray, robot_cell: Sequence[int], min_cells: int = 8):
    reachable = reachable_free(grid, robot_cell)
    frontier = np.zeros_like(reachable)
    height, width = grid.shape
    for y, x in zip(*np.nonzero(reachable)):
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height and grid[ny, nx] == -1:
                frontier[y, x] = True
                break
    seen = np.zeros_like(frontier)
    clusters = []
    for y, x in zip(*np.nonzero(frontier)):
        if seen[y, x]:
            continue
        queue = deque([(x, y)])
        seen[y, x] = True
        cells = []
        while queue:
            cx, cy = queue.popleft()
            cells.append((cx, cy))
            for dx, dy in (
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ):
                nx, ny = cx + dx, cy + dy
                if (
                    0 <= nx < width
                    and 0 <= ny < height
                    and frontier[ny, nx]
                    and not seen[ny, nx]
                ):
                    seen[ny, nx] = True
                    queue.append((nx, ny))
        if len(cells) < min_cells:
            continue
        center = np.mean(np.asarray(cells), axis=0)
        goal = min(cells, key=lambda cell: np.linalg.norm(np.asarray(cell) - center))
        distance = min(
            math.hypot(cell[0] - robot_cell[0], cell[1] - robot_cell[1])
            for cell in cells
        )
        clusters.append({"cells": cells, "goal": goal, "distance_cells": distance})
    return clusters


def select_frontier(grid: np.ndarray, robot_cell: Sequence[int], min_cells: int = 8):
    clusters = frontier_clusters(grid, robot_cell, min_cells)
    if not clusters:
        return None, []
    selected = max(
        clusters,
        key=lambda cluster: len(cluster["cells"]) - 0.35 * cluster["distance_cells"],
    )
    return selected, clusters


def approach_goal_radius(clearance: float, front_extent: float, margin: float) -> float:
    return max(0.05, float(clearance) + float(front_extent) - float(margin))


def standoff_candidates(
    target_xy: Sequence[float],
    robot_xy: Sequence[float],
    radius: float,
    samples: int = 16,
):
    """Return poses on a target-facing ring, nearest-to-robot first."""
    poses = []
    for index in range(max(4, int(samples))):
        angle = 2.0 * math.pi * index / max(4, int(samples))
        x = float(target_xy[0]) + radius * math.cos(angle)
        y = float(target_xy[1]) + radius * math.sin(angle)
        yaw = math.atan2(float(target_xy[1]) - y, float(target_xy[0]) - x)
        distance = math.hypot(x - float(robot_xy[0]), y - float(robot_xy[1]))
        poses.append((x, y, yaw, distance))
    return sorted(poses, key=lambda pose: pose[3])


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def scan_yaws(initial_yaw: float, steps: int) -> Iterable[float]:
    count = max(1, int(steps))
    return [initial_yaw + 2.0 * math.pi * (index + 1) / count for index in range(count)]
