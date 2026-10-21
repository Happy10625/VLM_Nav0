"""Pure validation helpers shared by Go2 preflight tools and unit tests."""

from __future__ import annotations

from dataclasses import dataclass, field
from bisect import bisect_left
import math
from statistics import median
from typing import Dict, Iterable, List, Mapping, Optional, Sequence


GO2_SUBNET_PREFIX = "192.168.123."
TBD = "TBD / requires measurement"


class InterfaceSelectionError(RuntimeError):
    pass


class CalibrationError(RuntimeError):
    pass


class FormalPreflightError(RuntimeError):
    pass


def validate_depth_units_report(
    report: Mapping[str, object],
    *,
    expected_width: int,
    expected_height: int,
    expected_fps: int,
    expected_depth_topic: str,
    expected_device_scale: float,
) -> float:
    """Return the verified ROS raw-depth scale or fail closed."""

    if report.get("verified") is not True or report.get("passed") is not True:
        raise CalibrationError("raw-depth unit measurement is not verified")
    profile = report.get("profile")
    expected_profile = {
        "width": int(expected_width),
        "height": int(expected_height),
        "fps": int(expected_fps),
        "enable_sync": True,
        "align_depth": False,
    }
    if not isinstance(profile, Mapping) or any(
        profile.get(key) != value for key, value in expected_profile.items()
    ):
        raise CalibrationError("raw-depth unit report profile does not match runtime")
    if report.get("depth_topic") != expected_depth_topic:
        raise CalibrationError("raw-depth unit report topic does not match runtime")
    try:
        device_scale = float(report["device_scale"])
        meters_per_unit = float(report["meters_per_unit"])
        raw_value = int(report["raw_value"])
        measured_distance = float(report["measured_z_m"])
        converted_distance = float(report["converted_m"])
        device_converted_distance = float(report["raw_times_device_scale_m"])
        absolute_error = float(report["absolute_error_m"])
        allowed_error = float(report["allowed_error_m"])
    except (KeyError, TypeError, ValueError) as error:
        raise CalibrationError("raw-depth unit report metadata is incomplete") from error
    if not all(
        math.isfinite(value)
        for value in (
            device_scale,
            meters_per_unit,
            measured_distance,
            converted_distance,
            device_converted_distance,
            absolute_error,
            allowed_error,
        )
    ) or min(
        device_scale,
        meters_per_unit,
        measured_distance,
        converted_distance,
        device_converted_distance,
        allowed_error,
        raw_value,
    ) <= 0.0:
        raise CalibrationError("raw-depth unit report metadata is invalid")
    if not math.isclose(
        device_scale, float(expected_device_scale), rel_tol=1e-6, abs_tol=1e-12
    ):
        raise CalibrationError("raw-depth device scale does not match runtime profile")
    if not math.isclose(
        meters_per_unit,
        float(expected_device_scale),
        rel_tol=1e-6,
        abs_tol=1e-12,
    ):
        raise CalibrationError("raw-depth conversion scale does not match device metadata")
    expected_converted = raw_value * meters_per_unit
    expected_device_converted = raw_value * device_scale
    expected_error = abs(expected_converted - measured_distance)
    expected_allowed_error = max(0.05, measured_distance * 0.05)
    if not all(
        (
            math.isclose(converted_distance, expected_converted, rel_tol=1e-9, abs_tol=1e-9),
            math.isclose(
                device_converted_distance,
                expected_device_converted,
                rel_tol=1e-9,
                abs_tol=1e-9,
            ),
            math.isclose(absolute_error, expected_error, rel_tol=1e-9, abs_tol=1e-9),
            math.isclose(
                allowed_error,
                expected_allowed_error,
                rel_tol=1e-9,
                abs_tol=1e-9,
            ),
        )
    ):
        raise CalibrationError("raw-depth unit report conversion metadata is inconsistent")
    if absolute_error < 0.0 or absolute_error > allowed_error:
        raise CalibrationError("raw-depth unit measurement exceeds allowed error")
    if not str(report.get("snapshot_sha256", "")).strip():
        raise CalibrationError("raw-depth unit report has no snapshot fingerprint")
    return meters_per_unit


def validate_formal_preflight_report(
    report: Mapping[str, object], *, now_unix_s: float, max_age_s: float
) -> None:
    """Require an explicit, successful, recent non-diagnostic sensor report."""

    if report.get("healthy") is not True:
        raise FormalPreflightError("formal sensor preflight is not healthy")
    formal_status = str(report.get("formal_preflight", ""))
    if formal_status != "PASS":
        if "NOT_EVALUATED" in formal_status or "diagnostic" in formal_status.lower():
            raise FormalPreflightError("diagnostic report is not a formal preflight")
        raise FormalPreflightError("formal preflight PASS marker is missing")
    generated = report.get("generated_at_unix_s")
    if not isinstance(generated, (int, float)) or not math.isfinite(float(generated)):
        raise FormalPreflightError("formal preflight generation time is missing")
    if not math.isfinite(now_unix_s) or not math.isfinite(max_age_s) or max_age_s <= 0:
        raise FormalPreflightError("invalid formal preflight freshness parameters")
    age = now_unix_s - float(generated)
    if age < -5.0:
        raise FormalPreflightError("formal preflight timestamp is in the future")
    if age > max_age_s:
        raise FormalPreflightError(
            f"formal preflight is stale ({age:.1f}s > {max_age_s:.1f}s)"
        )


def nearest_stream_offsets(
    reference_stamps: Sequence[float], high_rate_stamps: Sequence[float]
) -> List[float]:
    """Return reference-minus-nearest offsets within the shared time window."""

    high_rate = sorted(float(value) for value in high_rate_stamps)
    if not high_rate:
        raise ValueError("high-rate timestamp stream is empty")
    offsets = []
    for reference in reference_stamps:
        reference = float(reference)
        if reference < high_rate[0] or reference > high_rate[-1]:
            continue
        index = bisect_left(high_rate, reference)
        if index == 0:
            nearest = high_rate[0]
        elif index == len(high_rate):
            nearest = high_rate[-1]
        else:
            before = high_rate[index - 1]
            after = high_rate[index]
            nearest = before if reference - before <= after - reference else after
        offsets.append(reference - nearest)
    if not offsets:
        raise ValueError("timestamp streams have no shared coverage")
    return offsets


def rgbd_sync_health(
    rgb_stamps: Sequence[float],
    depth_stamps: Sequence[float],
    *,
    slop_s: float,
    minimum_samples: int,
    minimum_rate: float,
) -> Dict[str, object]:
    """Measure one-to-one RGB/depth matches across a bounded sample window."""

    if slop_s < 0.0 or minimum_samples <= 0 or not 0.0 <= minimum_rate <= 1.0:
        raise ValueError("invalid RGB-D synchronization thresholds")
    rgb = sorted(float(value) for value in rgb_stamps)
    depth = sorted(float(value) for value in depth_stamps)
    rgb_index = depth_index = matched = 0
    while rgb_index < len(rgb) and depth_index < len(depth):
        delta = rgb[rgb_index] - depth[depth_index]
        if abs(delta) <= slop_s:
            matched += 1
            rgb_index += 1
            depth_index += 1
        elif delta < 0.0:
            rgb_index += 1
        else:
            depth_index += 1
    total = max(len(rgb), len(depth))
    rate = matched / total if total else 0.0
    return {
        "healthy": (
            len(rgb) >= minimum_samples
            and len(depth) >= minimum_samples
            and rate >= minimum_rate
        ),
        "matched": matched,
        "total": total,
        "rate": rate,
    }


def analyze_cloud_points(points: Iterable[Sequence[float]]) -> Dict[str, object]:
    """Analyze x/y/z/time/ring tuples; ring is diagnostic, never a blocker."""

    finite_xyz = True
    finite_time = True
    times: List[float] = []
    rings: Dict[int, int] = {}
    point_count = 0
    for point in points:
        if len(point) < 5:
            raise ValueError("cloud point must contain x, y, z, time, ring")
        x, y, z, point_time, ring = point[:5]
        point_count += 1
        finite_xyz = finite_xyz and all(math.isfinite(float(value)) for value in (x, y, z))
        if math.isfinite(float(point_time)):
            times.append(float(point_time))
        else:
            finite_time = False
        ring_value = int(ring)
        rings[ring_value] = rings.get(ring_value, 0) + 1

    if point_count == 0:
        raise ValueError("cloud has no points")
    point_time_span = max(times) - min(times) if times else 0.0
    point_time_varies = finite_time and len(times) == point_count and point_time_span > 0.0
    return {
        "healthy": finite_xyz and finite_time and point_time_varies,
        "point_count": point_count,
        "finite_xyz": finite_xyz,
        "finite_time": finite_time,
        "point_time_varies": point_time_varies,
        "point_time_span_s": point_time_span,
        "ring_distribution": rings,
        "ring_single_value_diagnostic": len(rings) == 1,
    }


def _has_go2_address(addresses: Iterable[str]) -> bool:
    return any(address.split("/", 1)[0].startswith(GO2_SUBNET_PREFIX) for address in addresses)


def select_go2_interface(
    interfaces: Mapping[str, Sequence[str]], override: Optional[str] = None
) -> str:
    """Select exactly one interface on 192.168.123.0/24, failing closed."""

    if override:
        if override not in interfaces:
            raise InterfaceSelectionError(
                f"explicit interface {override!r} does not exist"
            )
        if not _has_go2_address(interfaces[override]):
            raise InterfaceSelectionError(
                f"explicit interface {override!r} has no 192.168.123.x address"
            )
        return override

    matches = [name for name, addresses in interfaces.items() if _has_go2_address(addresses)]
    if not matches:
        raise InterfaceSelectionError("no interface has a 192.168.123.x address")
    if len(matches) > 1:
        raise InterfaceSelectionError(
            "multiple interfaces have 192.168.123.x addresses; set GO2_NET_IFACE"
        )
    return matches[0]


def _is_numeric_vector(value: object, length: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == length
        and all(isinstance(item, (int, float)) and math.isfinite(item) for item in value)
    )


def require_calibrated(name: str, config: Mapping[str, object]) -> None:
    """Reject missing, TBD, or unverified identity hardware transforms."""

    if config.get("calibrated") is not True:
        raise CalibrationError(f"{name} is not calibrated")
    source = str(config.get("source", "")).strip()
    if not source or source == TBD or source.lower() == "unverified":
        raise CalibrationError(f"{name} has no verified source")

    if "translation" in config or "rotation_matrix" in config:
        translation = config.get("translation")
        rotation = config.get("rotation_matrix")
        if not _is_numeric_vector(translation, 3) or not _is_numeric_vector(rotation, 9):
            raise CalibrationError(f"{name} has invalid numeric LiDAR-to-IMU values")
        identity = translation == [0.0, 0.0, 0.0] and rotation == [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
    else:
        xyz = config.get("xyz")
        rpy = config.get("rpy")
        if not _is_numeric_vector(xyz, 3) or not _is_numeric_vector(rpy, 3):
            raise CalibrationError(f"{name} has invalid numeric rigid transform values")
        identity = xyz == [0.0, 0.0, 0.0] and rpy == [0.0, 0.0, 0.0]

    if identity and config.get("identity_verified") is not True:
        raise CalibrationError(
            f"{name} is identity but identity_verified is not true"
        )


@dataclass
class TimestampAccumulator:
    offsets: List[float] = field(default_factory=list)
    previous_header_stamp: Optional[float] = None
    timestamp_backward_count: int = 0

    def add(self, *, header_stamp: float, local_receive_time: float) -> None:
        if not math.isfinite(header_stamp) or not math.isfinite(local_receive_time):
            raise ValueError("timestamps must be finite")
        if (
            self.previous_header_stamp is not None
            and header_stamp < self.previous_header_stamp
        ):
            self.timestamp_backward_count += 1
        self.previous_header_stamp = header_stamp
        self.offsets.append(local_receive_time - header_stamp)

    def summary(self) -> Dict[str, float | int]:
        if not self.offsets:
            raise ValueError("no timestamp samples")
        return {
            "minimum_offset_s": min(self.offsets),
            "median_offset_s": median(self.offsets),
            "jitter_s": max(self.offsets) - min(self.offsets),
            "timestamp_backward_count": self.timestamp_backward_count,
            "sample_count": len(self.offsets),
        }


class CameraHealthAccumulator:
    """Accumulate RGB, raw depth, and both CameraInfo streams."""

    def __init__(
        self,
        *,
        minimum_samples: int,
        sync_slop_s: float,
        minimum_sync_rate: float = 1.0,
        minimum_rate_hz: Optional[float] = None,
        maximum_rate_hz: Optional[float] = None,
        maximum_duplicate_rate: float = 0.01,
    ) -> None:
        if (
            minimum_samples <= 0
            or sync_slop_s < 0.0
            or not 0.0 <= minimum_sync_rate <= 1.0
            or not 0.0 <= maximum_duplicate_rate <= 1.0
        ):
            raise ValueError("invalid camera preflight thresholds")
        self.minimum_samples = int(minimum_samples)
        self.sync_slop_s = float(sync_slop_s)
        self.minimum_sync_rate = float(minimum_sync_rate)
        self.minimum_rate_hz = minimum_rate_hz
        self.maximum_rate_hz = maximum_rate_hz
        self.maximum_duplicate_rate = float(maximum_duplicate_rate)
        self.rgb_clock = TimestampAccumulator()
        self.depth_clock = TimestampAccumulator()
        self.color_info_clock = TimestampAccumulator()
        self.depth_info_clock = TimestampAccumulator()
        self.info_clock = self.color_info_clock
        self.rgb_stamps: List[float] = []
        self.depth_stamps: List[float] = []
        self.color_info_stamps: List[float] = []
        self.depth_info_stamps: List[float] = []
        self.info_stamps = self.color_info_stamps
        self.rgb_dimensions = set()
        self.depth_dimensions = set()
        self.color_info_dimensions = set()
        self.depth_info_dimensions = set()
        self.info_dimensions = self.color_info_dimensions
        self.rgb_metadata_healthy = True
        self.depth_metadata_healthy = True
        self.color_info_metadata_healthy = True
        self.depth_info_metadata_healthy = True

    @staticmethod
    def _valid_image(
        width: int,
        height: int,
        encoding: str,
        step: int,
        data_size: int,
        allowed_encodings: Sequence[str],
    ) -> bool:
        return bool(
            width > 0
            and height > 0
            and step > 0
            and data_size >= height * step
            and encoding in allowed_encodings
        )

    def add_rgb(
        self,
        *,
        stamp: float,
        local_receive_time: float,
        width: int,
        height: int,
        encoding: str,
        step: int,
        data_size: int,
    ) -> None:
        self.rgb_clock.add(header_stamp=stamp, local_receive_time=local_receive_time)
        self.rgb_stamps.append(float(stamp))
        self.rgb_dimensions.add((int(width), int(height)))
        self.rgb_metadata_healthy = self.rgb_metadata_healthy and self._valid_image(
            width, height, encoding, step, data_size, ("rgb8", "bgr8")
        )

    def add_depth(
        self,
        *,
        stamp: float,
        local_receive_time: float,
        width: int,
        height: int,
        encoding: str,
        step: int,
        data_size: int,
    ) -> None:
        self.depth_clock.add(header_stamp=stamp, local_receive_time=local_receive_time)
        self.depth_stamps.append(float(stamp))
        self.depth_dimensions.add((int(width), int(height)))
        self.depth_metadata_healthy = (
            self.depth_metadata_healthy
            and self._valid_image(
                width, height, encoding, step, data_size, ("16UC1", "mono16")
            )
        )

    def _add_camera_info(
        self,
        stream: str,
        *,
        stamp: float,
        local_receive_time: float,
        width: int,
        height: int,
        frame_id: str,
        k: Sequence[float],
    ) -> None:
        clock = getattr(self, f"{stream}_info_clock")
        stamps = getattr(self, f"{stream}_info_stamps")
        dimensions = getattr(self, f"{stream}_info_dimensions")
        clock.add(header_stamp=stamp, local_receive_time=local_receive_time)
        stamps.append(float(stamp))
        dimensions.add((int(width), int(height)))
        healthy = bool(
            width > 0
            and height > 0
            and frame_id.strip()
            and len(k) == 9
            and all(math.isfinite(float(value)) for value in k)
            and float(k[0]) > 0.0
            and float(k[4]) > 0.0
        )
        attribute = f"{stream}_info_metadata_healthy"
        setattr(self, attribute, getattr(self, attribute) and healthy)

    def add_color_camera_info(self, **values) -> None:
        self._add_camera_info("color", **values)

    def add_depth_camera_info(self, **values) -> None:
        self._add_camera_info("depth", **values)

    def add_camera_info(self, **values) -> None:
        """Backward-compatible color CameraInfo entry point."""
        self.add_color_camera_info(**values)

    @staticmethod
    def _clock_summary(accumulator: TimestampAccumulator) -> Dict[str, object]:
        if not accumulator.offsets:
            return {"sample_count": 0, "timestamp_backward_count": 0}
        return accumulator.summary()

    def _stream_summary(self, stamps: Sequence[float], clock) -> Dict[str, object]:
        unique = sorted(set(stamps))
        duplicates = len(stamps) - len(unique)
        duration = unique[-1] - unique[0] if len(unique) > 1 else 0.0
        frequency = (len(unique) - 1) / duration if duration > 0.0 else 0.0
        duplicate_rate = duplicates / len(stamps) if stamps else 0.0
        rate_healthy = bool(
            self.minimum_rate_hz is None
            or (
                frequency >= float(self.minimum_rate_hz)
                and (
                    self.maximum_rate_hz is None
                    or frequency <= float(self.maximum_rate_hz)
                )
            )
        )
        return {
            "unique_hz": frequency,
            "callbacks": len(stamps),
            "unique_frames": len(unique),
            "duplicates": duplicates,
            "duplicate_rate": duplicate_rate,
            "rollback": clock.timestamp_backward_count,
            "healthy": (
                len(stamps) >= self.minimum_samples
                and rate_healthy
                and duplicate_rate <= self.maximum_duplicate_rate
                and clock.timestamp_backward_count == 0
            ),
        }

    def summary(self) -> Dict[str, object]:
        rgb_clock = self._clock_summary(self.rgb_clock)
        depth_clock = self._clock_summary(self.depth_clock)
        color_info_clock = self._clock_summary(self.color_info_clock)
        depth_info_clock = self._clock_summary(self.depth_info_clock)
        rgb_stream = self._stream_summary(self.rgb_stamps, self.rgb_clock)
        depth_stream = self._stream_summary(self.depth_stamps, self.depth_clock)
        profiles_match_camera_info = bool(
            len(self.rgb_dimensions) == 1
            and self.rgb_dimensions == self.color_info_dimensions
            and len(self.depth_dimensions) == 1
            and self.depth_dimensions == self.depth_info_dimensions
        )
        offset_summary = None
        sync_healthy = False
        if self.rgb_stamps and self.depth_stamps:
            offsets = [
                stamp - min(self.depth_stamps, key=lambda value: abs(stamp - value))
                for stamp in self.rgb_stamps
            ]
            absolute = [abs(value) for value in offsets]
            offset_summary = {
                "minimum": min(offsets),
                "median": median(offsets),
                "maximum": max(offsets),
                "maximum_absolute": max(absolute),
                "sample_count": len(offsets),
            }
            sync_window = rgbd_sync_health(
                self.rgb_stamps,
                self.depth_stamps,
                slop_s=self.sync_slop_s,
                minimum_samples=self.minimum_samples,
                minimum_rate=self.minimum_sync_rate,
            )
            sync_healthy = sync_window["healthy"]
        else:
            sync_window = {
                "healthy": False,
                "matched": 0,
                "total": max(len(self.rgb_stamps), len(self.depth_stamps)),
                "rate": 0.0,
            }

        counts_healthy = all(
            count >= self.minimum_samples
            for count in (
                len(self.rgb_stamps),
                len(self.depth_stamps),
                len(self.color_info_stamps),
                len(self.depth_info_stamps),
            )
        )
        healthy = all(
            (
                counts_healthy,
                rgb_stream["healthy"],
                depth_stream["healthy"],
                self.rgb_metadata_healthy,
                self.depth_metadata_healthy,
                self.color_info_metadata_healthy,
                self.depth_info_metadata_healthy,
                profiles_match_camera_info,
                sync_healthy,
            )
        )
        return {
            "healthy": healthy,
            "rgb_count": len(self.rgb_stamps),
            "depth_count": len(self.depth_stamps),
            "camera_info_count": len(self.color_info_stamps),
            "color_camera_info_count": len(self.color_info_stamps),
            "depth_camera_info_count": len(self.depth_info_stamps),
            "rgb_clock": rgb_clock,
            "depth_clock": depth_clock,
            "camera_info_clock": color_info_clock,
            "color_camera_info_clock": color_info_clock,
            "depth_camera_info_clock": depth_info_clock,
            "rgb_metadata_healthy": self.rgb_metadata_healthy,
            "depth_metadata_healthy": self.depth_metadata_healthy,
            "camera_info_metadata_healthy": self.color_info_metadata_healthy,
            "color_camera_info_metadata_healthy": self.color_info_metadata_healthy,
            "depth_camera_info_metadata_healthy": self.depth_info_metadata_healthy,
            "profiles_match_camera_info": profiles_match_camera_info,
            "dimensions_aligned": profiles_match_camera_info,
            "rgb_depth_offset_s": offset_summary,
            "rgb_stream": rgb_stream,
            "depth_stream": depth_stream,
            "sync_window": sync_window,
        }


@dataclass
class ImuHealthAccumulator:
    gravity_min: float
    gravity_max: float
    stationary_gyro_max: float
    acceleration_abs_max: float
    gyro_abs_max: float
    jump_max: float
    sample_count: int = 0
    nonfinite_count: int = 0
    extreme_count: int = 0
    gravity_out_of_range_count: int = 0
    gyro_out_of_range_count: int = 0
    jump_count: int = 0
    previous_acceleration: Optional[tuple[float, float, float]] = None

    def add(self, ax: float, ay: float, az: float, gx: float, gy: float, gz: float) -> None:
        values = (ax, ay, az, gx, gy, gz)
        self.sample_count += 1
        if not all(math.isfinite(value) for value in values):
            self.nonfinite_count += 1
            return

        acceleration = (ax, ay, az)
        gyro = (gx, gy, gz)
        if any(abs(value) > self.acceleration_abs_max for value in acceleration) or any(
            abs(value) > self.gyro_abs_max for value in gyro
        ):
            self.extreme_count += 1

        acceleration_norm = math.sqrt(sum(value * value for value in acceleration))
        gyro_norm = math.sqrt(sum(value * value for value in gyro))
        if not self.gravity_min <= acceleration_norm <= self.gravity_max:
            self.gravity_out_of_range_count += 1
        if gyro_norm > self.stationary_gyro_max:
            self.gyro_out_of_range_count += 1

        if self.previous_acceleration is not None:
            jump = math.sqrt(
                sum(
                    (current - previous) ** 2
                    for current, previous in zip(
                        acceleration, self.previous_acceleration
                    )
                )
            )
            if jump > self.jump_max:
                self.jump_count += 1
        self.previous_acceleration = acceleration

    def summary(self) -> Dict[str, bool | int]:
        errors = (
            self.nonfinite_count
            + self.extreme_count
            + self.gravity_out_of_range_count
            + self.gyro_out_of_range_count
            + self.jump_count
        )
        return {
            "healthy": self.sample_count > 0 and errors == 0,
            "sample_count": self.sample_count,
            "nonfinite_count": self.nonfinite_count,
            "extreme_count": self.extreme_count,
            "gravity_out_of_range_count": self.gravity_out_of_range_count,
            "gyro_out_of_range_count": self.gyro_out_of_range_count,
            "jump_count": self.jump_count,
        }


@dataclass
class RawImuAccumulator:
    """Collect measurements without assigning unverified health thresholds."""

    sample_count: int = 0
    nonfinite_count: int = 0
    acceleration_norms: List[float] = field(default_factory=list)
    gyro_norms: List[float] = field(default_factory=list)
    maximum_acceleration_abs: float = 0.0
    maximum_gyro_abs: float = 0.0
    maximum_acceleration_jump: float = 0.0
    previous_acceleration: Optional[tuple[float, float, float]] = None

    def add(self, ax: float, ay: float, az: float, gx: float, gy: float, gz: float) -> None:
        self.sample_count += 1
        values = (ax, ay, az, gx, gy, gz)
        if not all(math.isfinite(value) for value in values):
            self.nonfinite_count += 1
            return
        acceleration = (ax, ay, az)
        gyro = (gx, gy, gz)
        self.acceleration_norms.append(
            math.sqrt(sum(value * value for value in acceleration))
        )
        self.gyro_norms.append(math.sqrt(sum(value * value for value in gyro)))
        self.maximum_acceleration_abs = max(
            self.maximum_acceleration_abs, *(abs(value) for value in acceleration)
        )
        self.maximum_gyro_abs = max(
            self.maximum_gyro_abs, *(abs(value) for value in gyro)
        )
        if self.previous_acceleration is not None:
            jump = math.sqrt(
                sum(
                    (current - previous) ** 2
                    for current, previous in zip(
                        acceleration, self.previous_acceleration
                    )
                )
            )
            self.maximum_acceleration_jump = max(
                self.maximum_acceleration_jump, jump
            )
        self.previous_acceleration = acceleration

    @staticmethod
    def _distribution(values: Sequence[float]) -> Dict[str, float]:
        if not values:
            return {"minimum": math.nan, "median": math.nan, "maximum": math.nan}
        return {
            "minimum": min(values),
            "median": median(values),
            "maximum": max(values),
        }

    def summary(self) -> Dict[str, object]:
        return {
            "sample_count": self.sample_count,
            "nonfinite_count": self.nonfinite_count,
            "acceleration_norm_mps2": self._distribution(self.acceleration_norms),
            "gyro_norm_radps": self._distribution(self.gyro_norms),
            "maximum_acceleration_abs_mps2": self.maximum_acceleration_abs,
            "maximum_gyro_abs_radps": self.maximum_gyro_abs,
            "maximum_acceleration_jump_mps2": self.maximum_acceleration_jump,
        }
