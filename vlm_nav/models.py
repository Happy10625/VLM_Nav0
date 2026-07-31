"""ROS-independent data contracts used by the VLM navigation stack."""

from dataclasses import dataclass
from typing import Any, Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class Pixel:
    u: int
    v: int


@dataclass(frozen=True)
class VLMResult:
    target_visible: bool
    confidence: float
    target_pixel: Optional[Pixel]
    waypoints: Tuple[Pixel, ...]
    coordinate_mode: str = "pixels"


@dataclass(frozen=True)
class FrontierCandidate:
    candidate_id: int
    x: float
    y: float
    bearing: float
    distance: float
    cell_count: int


@dataclass(frozen=True)
class FrontierDecision:
    selected_frontier_id: int
    confidence: float
    reason: str


@dataclass(frozen=True)
class FrameSnapshot:
    """Immutable synchronized frame retained until its VLM response arrives."""

    sequence: int
    task_epoch: int
    target_description: str
    captured_monotonic: float
    stamp: Any
    frame_id: str
    rgb: np.ndarray
    depth_m: np.ndarray
    intrinsics: Tuple[float, float, float, float]
    transform_matrix: np.ndarray
    request_kind: str = "target"
    auxiliary_rgb: Optional[np.ndarray] = None
    frontier_generation: int = 0
    frontier_candidates: Tuple[FrontierCandidate, ...] = ()
    frontier_context: str = ""
    frontier_robot_pixel: Optional[Tuple[int, int]] = None
    frontier_candidate_pixels: Tuple[Tuple[int, Tuple[int, int]], ...] = ()


@dataclass(frozen=True)
class WorkerResult:
    snapshot: FrameSnapshot
    result: Optional[Any]
    latency_s: float
    error: Optional[str] = None
    raw_response: Optional[str] = None
