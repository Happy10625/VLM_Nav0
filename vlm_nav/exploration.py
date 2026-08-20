"""Pure helpers for VLM-selected frontier exploration and rolling paths."""

import math
from typing import Iterable, Sequence

import cv2
import numpy as np


def max_polyline_deviation(
    points: Sequence[Sequence[float]],
    start_xy: Sequence[float],
    end_xy: Sequence[float],
) -> float:
    """Maximum lateral distance of a path from its requested line segment."""
    if not points:
        return math.inf
    start = np.asarray((float(start_xy[0]), float(start_xy[1])), dtype=float)
    end = np.asarray((float(end_xy[0]), float(end_xy[1])), dtype=float)
    segment = end - start
    length_squared = float(np.dot(segment, segment))
    if length_squared <= 1e-12:
        return max(
            float(np.linalg.norm(np.asarray(point[:2], dtype=float) - start))
            for point in points
        )
    maximum = 0.0
    for point in points:
        value = np.asarray((float(point[0]), float(point[1])), dtype=float)
        fraction = float(np.dot(value - start, segment) / length_squared)
        projection = start + min(1.0, max(0.0, fraction)) * segment
        maximum = max(maximum, float(np.linalg.norm(value - projection)))
    return maximum


def scan_montage(images: Sequence[np.ndarray], headings: Sequence[float]) -> np.ndarray:
    """Build a compact 4x2 north-referenced contact sheet."""
    tiles = []
    for index, image in enumerate(images[:8]):
        tile = cv2.resize(np.ascontiguousarray(image), (320, 180))
        heading = math.degrees(headings[index]) % 360.0 if index < len(headings) else 0.0
        cv2.rectangle(tile, (0, 0), (320, 27), (0, 0, 0), -1)
        cv2.putText(
            tile,
            f"view {index + 1} heading {heading:.0f} deg",
            (8, 19),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 230, 40),
            1,
            cv2.LINE_AA,
        )
        tiles.append(tile)
    if not tiles:
        raise ValueError("at least one scan image is required")
    while len(tiles) < 8:
        tiles.append(np.zeros_like(tiles[0]))
    return np.vstack((np.hstack(tiles[:4]), np.hstack(tiles[4:8])))


def render_frontier_map(
    grid: np.ndarray,
    map_origin_xy: Sequence[float],
    resolution: float,
    robot_pose: Sequence[float],
    session_origin: Sequence[float],
    travel_radius: float,
    candidates: Iterable,
    size: int = 720,
    return_annotations: bool = False,
) -> np.ndarray:
    """Render a north-up local occupancy map with numbered VLM candidates."""
    candidates = list(candidates)
    margin = max(0.5, float(travel_radius) * 0.15)
    candidate_extent = max(
        (
            max(
                abs(float(candidate.x) - float(session_origin[0])),
                abs(float(candidate.y) - float(session_origin[1])),
            )
            for candidate in candidates
        ),
        default=0.0,
    )
    half_extent = max(float(travel_radius), candidate_extent) + margin
    min_x = float(session_origin[0]) - half_extent
    max_x = float(session_origin[0]) + half_extent
    min_y = float(session_origin[1]) - half_extent
    max_y = float(session_origin[1]) + half_extent
    origin_x, origin_y = map(float, map_origin_xy)
    height, width = grid.shape

    def cell_x(world_x):
        return int(math.floor((world_x - origin_x) / resolution))

    def cell_y(world_y):
        return int(math.floor((world_y - origin_y) / resolution))

    x0, x1 = max(0, cell_x(min_x)), min(width, cell_x(max_x) + 1)
    y0, y1 = max(0, cell_y(min_y)), min(height, cell_y(max_y) + 1)
    if x1 <= x0 or y1 <= y0:
        raise ValueError("travel-radius crop does not intersect occupancy map")
    crop = grid[y0:y1, x0:x1]
    raster = np.full((*crop.shape, 3), 128, dtype=np.uint8)
    raster[crop == 0] = (238, 238, 238)
    raster[crop > 0] = (20, 20, 20)
    raster = np.flipud(raster)
    canvas = cv2.resize(raster, (size, size), interpolation=cv2.INTER_NEAREST)

    crop_min_x = origin_x + x0 * resolution
    crop_min_y = origin_y + y0 * resolution
    crop_max_x = origin_x + x1 * resolution
    crop_max_y = origin_y + y1 * resolution

    def pixel(x, y):
        px = int(round((float(x) - crop_min_x) / (crop_max_x - crop_min_x) * (size - 1)))
        py = int(round((crop_max_y - float(y)) / (crop_max_y - crop_min_y) * (size - 1)))
        return px, py

    center = pixel(session_origin[0], session_origin[1])
    radius_px = int(round(float(travel_radius) / (crop_max_x - crop_min_x) * size))
    cv2.circle(canvas, center, radius_px, (180, 80, 180), 2, cv2.LINE_AA)

    robot_px = pixel(robot_pose[0], robot_pose[1])
    arrow_length = 34
    tip = (
        int(robot_px[0] + arrow_length * math.cos(robot_pose[2])),
        int(robot_px[1] - arrow_length * math.sin(robot_pose[2])),
    )
    cv2.arrowedLine(canvas, robot_px, tip, (0, 180, 255), 4, cv2.LINE_AA, tipLength=0.35)
    cv2.putText(canvas, "ROBOT", (robot_px[0] + 8, robot_px[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 120, 255), 2, cv2.LINE_AA)

    candidate_pixels = []
    for candidate in candidates:
        point = pixel(candidate.x, candidate.y)
        candidate_pixels.append((candidate.candidate_id, point))
        cv2.circle(canvas, point, 17, (255, 80, 30), -1, cv2.LINE_AA)
        cv2.circle(canvas, point, 19, (255, 255, 255), 2, cv2.LINE_AA)
        label = str(candidate.candidate_id)
        cv2.putText(canvas, label, (point[0] - 8, point[1] + 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, "N", (size - 38, 35), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (40, 40, 255), 2, cv2.LINE_AA)
    cv2.arrowedLine(canvas, (size - 25, 80), (size - 25, 43),
                    (40, 40, 255), 3, cv2.LINE_AA, tipLength=0.35)
    if return_annotations:
        return canvas, robot_px, tuple(candidate_pixels)
    return canvas


def sample_polyline(points: Sequence[Sequence[float]], segments: int):
    """Return equally spaced XY points including both path endpoints."""
    if len(points) < 2:
        return [], 0.0
    xy = np.asarray([(float(item[0]), float(item[1])) for item in points], dtype=float)
    lengths = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    total = float(np.sum(lengths))
    if total <= 1e-6:
        return [tuple(xy[0]), tuple(xy[-1])], total
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    samples = []
    for distance in np.linspace(0.0, total, max(1, int(segments)) + 1):
        edge = min(int(np.searchsorted(cumulative, distance, side="right") - 1), len(lengths) - 1)
        edge_length = lengths[edge]
        fraction = 0.0 if edge_length <= 1e-9 else (distance - cumulative[edge]) / edge_length
        point = xy[edge] + fraction * (xy[edge + 1] - xy[edge])
        samples.append((float(point[0]), float(point[1])))
    return samples, total


def clip_polyline_to_radius(
    points: Sequence[Sequence[float]],
    center: Sequence[float],
    radius: float,
):
    """Keep the path prefix inside a circle and interpolate its first exit."""
    if len(points) < 2:
        return list(points), False, 0.0
    radius = float(radius)
    if radius <= 0.0:
        raise ValueError("radius must be positive")
    xy = np.asarray(
        [(float(item[0]), float(item[1])) for item in points], dtype=float
    )
    center_xy = np.asarray((float(center[0]), float(center[1])), dtype=float)
    clipped = [tuple(xy[0])]
    travelled = 0.0
    for start, end in zip(xy, xy[1:]):
        edge = end - start
        edge_length = float(np.linalg.norm(edge))
        end_radius = float(np.linalg.norm(end - center_xy))
        if end_radius <= radius + 1e-9:
            clipped.append(tuple(end))
            travelled += edge_length
            continue

        # The accepted prefix starts inside the rolling circle.  Solve the
        # segment/circle intersection and stop at the first outward crossing.
        relative = start - center_xy
        a = float(np.dot(edge, edge))
        b = 2.0 * float(np.dot(relative, edge))
        c = float(np.dot(relative, relative)) - radius * radius
        discriminant = max(0.0, b * b - 4.0 * a * c)
        roots = (
            (
                (-b - math.sqrt(discriminant)) / (2.0 * a),
                (-b + math.sqrt(discriminant)) / (2.0 * a),
            )
            if a > 1e-12
            else ()
        )
        fractions = [
            value for value in roots if -1e-9 <= value <= 1.0 + 1e-9
        ]
        fraction = min(fractions) if fractions else 0.0
        fraction = min(1.0, max(0.0, float(fraction)))
        intersection = start + fraction * edge
        if float(np.linalg.norm(intersection - np.asarray(clipped[-1]))) > 1e-6:
            clipped.append(tuple(intersection))
        travelled += fraction * edge_length
        return clipped, True, travelled
    return clipped, False, travelled


def clip_polyline_to_length(
    points: Sequence[Sequence[float]], maximum_length: float
):
    """Return a path prefix ending at the requested arc length."""
    if len(points) < 2:
        return list(points), 0.0
    limit = max(0.0, float(maximum_length))
    xy = np.asarray(
        [(float(item[0]), float(item[1])) for item in points], dtype=float
    )
    clipped = [tuple(xy[0])]
    travelled = 0.0
    for start, end in zip(xy, xy[1:]):
        edge = end - start
        edge_length = float(np.linalg.norm(edge))
        if travelled + edge_length <= limit + 1e-9:
            clipped.append(tuple(end))
            travelled += edge_length
            continue
        remaining = max(0.0, limit - travelled)
        fraction = 0.0 if edge_length <= 1e-12 else remaining / edge_length
        point = start + min(1.0, fraction) * edge
        if float(np.linalg.norm(point - np.asarray(clipped[-1]))) > 1e-6:
            clipped.append(tuple(point))
        travelled += remaining
        return clipped, travelled
    return clipped, travelled
