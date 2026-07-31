"""Retained, per-ARM visual records for VLM navigation troubleshooting."""

from datetime import datetime, timezone
import json
import os
import shutil

import cv2
import numpy as np

from .models import FrontierDecision, VLMResult
from .vlm_client import overlay_coordinate_grid


class ArmImageRecorder:
    """Save annotated VLM inputs and retain only the newest ARM sessions."""

    def __init__(self, root_path, keep_sessions=3, jpeg_quality=90):
        self.root_path = os.path.abspath(os.path.expanduser(str(root_path)))
        self.keep_sessions = max(1, int(keep_sessions))
        self.jpeg_quality = int(jpeg_quality)
        self.session_path = None
        self.record_index = 0
        self.last_error = "none"

    def start_arm(self, now=None):
        """Start a distinct record directory and prune older ARM directories."""
        now = now or datetime.now()
        self.session_path = None
        os.makedirs(self.root_path, exist_ok=True)
        stem = now.strftime("arm_%Y%m%d_%H%M%S_%f")
        session_path = os.path.join(self.root_path, stem)
        suffix = 1
        while os.path.exists(session_path):
            session_path = os.path.join(self.root_path, f"{stem}_{suffix:02d}")
            suffix += 1
        os.makedirs(session_path)
        self.session_path = session_path
        self.record_index = 0
        self.last_error = "none"
        self._prune()
        return session_path

    def _prune(self):
        sessions = sorted(
            (
                entry
                for entry in os.scandir(self.root_path)
                if entry.is_dir(follow_symlinks=False) and entry.name.startswith("arm_")
            ),
            key=lambda entry: entry.name,
        )
        for entry in sessions[: -self.keep_sessions]:
            shutil.rmtree(entry.path)

    @staticmethod
    def draw_target_path(image, result):
        points = [(pixel.u, pixel.v) for pixel in result.waypoints]
        if result.target_pixel is not None:
            points.append((result.target_pixel.u, result.target_pixel.v))
        for start, end in zip(points, points[1:]):
            cv2.arrowedLine(
                image, start, end, (255, 255, 0), 4, cv2.LINE_AA, tipLength=0.12
            )
        for index, pixel in enumerate(result.waypoints):
            cv2.circle(image, (pixel.u, pixel.v), 9, (0, 255, 0), -1, cv2.LINE_AA)
            cv2.putText(
                image,
                f"W{index + 1}",
                (pixel.u + 11, pixel.v - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        if result.target_pixel is not None:
            target = (result.target_pixel.u, result.target_pixel.v)
            cv2.circle(image, target, 12, (255, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(
                image,
                "TARGET",
                (target[0] + 14, target[1] - 7),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 0, 0),
                2,
                cv2.LINE_AA,
            )

    @staticmethod
    def _draw_frontier_path(image, snapshot, result):
        pixels = dict(getattr(snapshot, "frontier_candidate_pixels", ()))
        robot = getattr(snapshot, "frontier_robot_pixel", None)
        selected = pixels.get(result.selected_frontier_id)
        if robot is not None and selected is not None:
            cv2.arrowedLine(
                image,
                tuple(robot),
                tuple(selected),
                (0, 255, 0),
                6,
                cv2.LINE_AA,
                tipLength=0.10,
            )
            cv2.circle(image, tuple(selected), 27, (0, 255, 0), 4, cv2.LINE_AA)
        cv2.rectangle(image, (0, 0), (image.shape[1], 38), (0, 0, 0), -1)
        cv2.putText(
            image,
            f"VLM PATH: ROBOT -> FRONTIER {result.selected_frontier_id}",
            (10, 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    def record(self, snapshot, result, disposition):
        """Write the annotated image(s) for one completed VLM request."""
        if self.session_path is None:
            return ()
        try:
            self.record_index += 1
            sequence = int(snapshot.sequence)
            prefix = (
                f"{self.record_index:06d}_seq{sequence:08d}_"
                f"{snapshot.request_kind}_{disposition}"
            )
            saved = []
            if snapshot.request_kind == "frontier":
                scene = np.ascontiguousarray(snapshot.rgb.copy())
                scene_path = os.path.join(self.session_path, prefix + "_scene.jpg")
                self._write_rgb(scene_path, scene)
                saved.append(scene_path)
                if snapshot.auxiliary_rgb is not None:
                    map_image = np.ascontiguousarray(snapshot.auxiliary_rgb.copy())
                    if isinstance(result, FrontierDecision):
                        self._draw_frontier_path(map_image, snapshot, result)
                    map_path = os.path.join(self.session_path, prefix + "_map.jpg")
                    self._write_rgb(map_path, map_image)
                    saved.append(map_path)
            else:
                normalized = (
                    isinstance(result, VLMResult)
                    and result.coordinate_mode == "normalized_1000"
                )
                image = overlay_coordinate_grid(
                    snapshot.rgb, normalized_1000=normalized
                )
                if isinstance(result, VLMResult):
                    self.draw_target_path(image, result)
                path = os.path.join(self.session_path, prefix + ".jpg")
                self._write_rgb(path, image)
                saved.append(path)
            self.last_error = "none"
            return tuple(saved)
        except (OSError, cv2.error, ValueError) as error:
            self.last_error = f"{type(error).__name__}: {error}"
            return ()

    def record_event(self, record):
        """Append a structured VLM/navigation event to the current ARM log."""
        if self.session_path is None:
            return False
        try:
            event = dict(record)
            event.setdefault(
                "logged_at", datetime.now(timezone.utc).isoformat()
            )
            path = os.path.join(self.session_path, "events.jsonl")
            with open(path, "a", encoding="utf-8") as stream:
                stream.write(json.dumps(event, ensure_ascii=False) + "\n")
            return True
        except (OSError, TypeError, ValueError) as error:
            self.last_error = f"{type(error).__name__}: {error}"
            return False

    def _write_rgb(self, path, image):
        ok = cv2.imwrite(
            path,
            cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if not ok:
            raise OSError(f"cannot write image: {path}")
