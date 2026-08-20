"""ROS 2 node that grounds VLM pixels in RGB-D and delegates motion to Nav2."""

from collections import deque
from dataclasses import replace
import json
import math
import queue
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped, Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import (
    depth_at_pixel_with_reason,
    grid_to_world,
    normalize_angle,
    project_pixel,
    scan_yaws,
    select_frontier,
    TargetTracker,
    transform_matrix,
    world_to_grid,
)
from .exploration import (
    clip_polyline_to_length,
    clip_polyline_to_radius,
    max_polyline_deviation,
    render_frontier_map,
    sample_polyline,
    scan_montage,
)
from .arm_image_recorder import ArmImageRecorder
from .latest_frame_worker import LatestFrameWorker
from .models import (
    FrameSnapshot,
    FrontierCandidate,
    FrontierDecision,
    Pixel,
    VLMResult,
    WorkerResult,
)
from .vlm_client import OpenAICompatibleVLMClient


DISARMED = "DISARMED"
SEARCHING = "SEARCHING"
SCANNING = "SCANNING"
FRONTIER_SELECTING = "FRONTIER_SELECTING"
EXPLORING = "EXPLORING"
TARGET_CONFIRMING = "TARGET_CONFIRMING"
TARGET_REOBSERVING = "TARGET_REOBSERVING"
TARGET_ALIGNING = "TARGET_ALIGNING"
APPROACHING = "APPROACHING"
APPROACH_STOPPING = "APPROACH_STOPPING"
SENSOR_WAITING = "SENSOR_WAITING"
SUCCEEDED = "SUCCEEDED"
API_ERROR = "API_ERROR"
FAILED = "FAILED"


def yaw_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class VLMNavigator(Node):
    def __init__(self):
        super().__init__("vlm_nav")
        defaults = {
            "enabled": False,
            "target_description": "chair",
            "global_frame": "map",
            "base_frame": "base_link",
            "camera_frame": "camera_color_optical_frame",
            "map_topic": "/map",
            "rgb_topic": "/camera/color/image_raw",
            "depth_topic": "/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/color/camera_info",
            "navigate_action": "/navigate_to_pose",
            "compute_path_action": "/compute_path_to_pose",
            "spin_action": "/spin",
            "clear_costmap_on_arm": True,
            "global_costmap_clear_service": (
                "/global_costmap/clear_entirely_global_costmap"
            ),
            "local_costmap_clear_service": (
                "/local_costmap/clear_entirely_local_costmap"
            ),
            "behavior_costmap_topic": "/local_costmap/costmap_raw",
            "initial_costmap_refresh_timeout": 3.0,
            "stop_cmd_topic": "/cmd_vel",
            "vlm_sample_rate": 5.0,
            "vlm_timeout": 4.0,
            "max_result_age": 3.0,
            "confidence_threshold": 0.60,
            "image_detail": "high",
            "jpeg_quality": 85,
            "vlm_image_record_path": "~/.ros/vlm_nav/arm_records",
            "vlm_image_record_keep_arms": 3,
            "api_failure_limit": 3,
            "confirm_frames": 3,
            "confirmation_radius": 0.35,
            "target_confirmation_timeout": 20.0,
            "target_lost_timeout": 10.0,
            "depth_neighborhood_radius": 5,
            "min_depth_samples": 8,
            "min_depth": 0.20,
            "max_depth": 6.0,
            "max_depth_deviation": 0.20,
            "depth_reobserve_angle_deg": 10.0,
            "depth_reobserve_attempt_limit": 3,
            "target_probe_distance": 2.0,
            "target_probe_attempt_limit": 3,
            "target_probe_min_distance": 0.30,
            "max_ground_height": 0.35,
            "target_success_radius": 0.81,
            "approach_cancel_radius": 0.89,
            "arrival_odom_topic": "/fastlio/odom",
            "arrival_linear_speed_tolerance": 0.03,
            "arrival_angular_speed_tolerance": 0.05,
            "arrival_stationary_samples": 3,
            "arrival_odom_max_age": 0.5,
            "arrival_cancel_timeout": 3.0,
            "arrival_stop_timeout": 5.0,
            "arrival_retry_limit": 1,
            "easy_case_mode": False,
            "easy_case_timeout": 120.0,
            "easy_case_alignment_tolerance": 0.10,
            "easy_case_max_path_deviation": 0.10,
            "scan_first": True,
            "scan_steps": 8,
            "scan_settle_time": 0.30,
            "scan_result_retry_limit": 3,
            "spin_time_allowance": 20.0,
            "allow_frontier_after_scan": True,
            "min_frontier_cells": 8,
            "max_frontier_candidates": 16,
            "frontier_confidence_threshold": 0.5,
            "frontier_decision_failure_limit": 3,
            "path_horizon_segments": 16,
            "path_execute_segments": 8,
            "frontier_full_commit_distance": 1.0,
            "rescan_at_frontier": True,
            "navigation_tick_period": 0.5,
            "goal_timeout": 120.0,
            "blocked_goal_seconds": 15.0,
            "rgbd_wait_timeout": 2.0,
            "sensor_failure_timeout": 30.0,
            "sensor_recovery_frames": 3,
            "tf_failure_timeout": 3.0,
            "image_tf_wait_timeout": 5.0,
            "image_tf_queue_size": 16,
            "max_travel_radius": 3.0,
            "publish_stop_command": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = SimpleNamespace(
            **{name: self.get_parameter(name).value for name in defaults}
        )
        self.add_on_set_parameters_callback(self.on_parameters_changed)

        self.state = DISARMED
        self.image_recorder = ArmImageRecorder(
            self.p.vlm_image_record_path,
            keep_sessions=int(self.p.vlm_image_record_keep_arms),
            jpeg_quality=int(self.p.jpeg_quality),
        )
        visualization_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(String, "~/state", 10)
        self.vlm_text_pub = self.create_publisher(
            String, "~/output_text", visualization_qos
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "~/markers", visualization_qos
        )
        self.debug_pub = self.create_publisher(Image, "~/debug_image", 2)
        self.frontier_map_pub = self.create_publisher(
            Image, "~/frontier_map_image", visualization_qos
        )
        self.scan_montage_pub = self.create_publisher(
            Image, "~/scan_montage_image", visualization_qos
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "~/diagnostics", 10
        )
        self.stop_pub = self.create_publisher(Twist, self.p.stop_cmd_topic, 10)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.p.map_topic, self.on_map, map_qos)
        self.create_subscription(
            Costmap,
            self.p.behavior_costmap_topic,
            self.on_behavior_costmap,
            map_qos,
        )
        self.create_subscription(
            CameraInfo, self.p.camera_info_topic, self.on_camera_info, 10
        )
        self.create_subscription(
            Odometry,
            self.p.arrival_odom_topic,
            self.on_arrival_odom,
            qos_profile_sensor_data,
        )
        self.perception_group = MutuallyExclusiveCallbackGroup()
        self.rgb_sub = Subscriber(
            self,
            Image,
            self.p.rgb_topic,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.perception_group,
        )
        self.depth_sub = Subscriber(
            self,
            Image,
            self.p.depth_topic,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.perception_group,
        )
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.10
        )
        self.sync.registerCallback(self.on_rgbd)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = ActionClient(
            self, NavigateToPose, self.p.navigate_action
        )
        self.path_planner = ActionClient(
            self, ComputePathToPose, self.p.compute_path_action
        )
        self.spinner = ActionClient(self, Spin, self.p.spin_action)
        self.costmap_clear_clients = [
            (
                "global",
                self.create_client(
                    ClearEntireCostmap,
                    self.p.global_costmap_clear_service,
                ),
            ),
            (
                "local",
                self.create_client(
                    ClearEntireCostmap,
                    self.p.local_costmap_clear_service,
                ),
            ),
        ]

        self.map_message = None
        self.grid = None
        self.map_revision = 0
        self.camera_info = None
        self.latest_snapshot = None
        self.last_submitted_sequence = -1
        self.sequence = 0
        self.task_epoch = 0
        self.worker = None
        self.worker_results = queue.SimpleQueue()
        self.api_failures = 0
        self.last_api_latency = 0.0
        self.last_result_age = math.inf
        self.last_api_error = "none"
        self.last_failure_reason = "none"
        self.accepted_results = 0
        self.rejected_results = 0
        self.last_rgbd_pair_received = 0.0
        self.last_valid_rgbd_received = 0.0
        self.last_camera_tf_success = 0.0
        self.last_image_tf_error = "none"
        self.last_robot_tf_success = 0.0
        self.enabled_since = 0.0
        self.invalid_rgbd_frames = 0
        self.image_tf_failures = 0
        self.image_tf_queue_drops = 0
        self.pending_rgbd_frames = deque()
        self.last_rgbd_queued = 0.0
        self.sensor_wait_started = 0.0
        self.sensor_wait_reason = "none"
        self.sensor_recovery_count = 0

        self.target_tracker = TargetTracker(
            int(self.p.confirm_frames), float(self.p.confirmation_radius)
        )
        self.target_reference_position = None
        self.target_seen_time = 0.0
        self.target_confirmation_started = 0.0
        self.easy_started = 0.0
        self.easy_alignment_complete = False
        self.easy_target_distance = math.inf
        self.easy_alignment_error = math.inf
        self.easy_direct_goal = None
        self.easy_path_deviation = math.inf
        self.last_debug_pixels = None
        self.last_vlm_confidence = 0.0
        self.last_vlm_disposition = "none"

        self.goal_handle = None
        self.goal_pending = False
        self.goal_kind = None
        self.goal_pose = None
        self.goal_started = 0.0
        self.goal_token = 0
        self.arrival_stop_started = 0.0
        self.arrival_cancel_requested_at = 0.0
        self.arrival_cancel_acknowledged = False
        self.arrival_action_terminal = False
        self.arrival_action_terminal_at = 0.0
        self.arrival_stationary_count = 0
        self.arrival_last_odom_received = 0.0
        self.arrival_linear_speed = math.inf
        self.arrival_angular_speed = math.inf
        self.arrival_retry_count = 0
        self.plan_handle = None
        self.plan_pending = False
        self.plan_candidates = []
        self.plan_kind = None
        self.plan_token = 0
        self.current_plan_pose = None
        self.plan_attempt_count = 0
        self.last_plan_kind = "none"
        self.last_plan_status = -1
        self.last_plan_pose_count = 0
        self.last_plan_path_length = 0.0
        self.last_plan_planning_time = 0.0
        self.last_plan_endpoint_radius = math.inf
        self.last_plan_max_radius = math.inf
        self.last_plan_candidate = None
        self.last_plan_rejection_reason = "none"
        self.plan_window_origin = None
        self.last_plan_commit_length = 0.0
        self.last_plan_radius_clipped = False
        self.rolling_goal_is_final = True
        self.active_motion_origin = None
        self.blocked_goals = {}

        self.session_origin = None
        self.initial_yaw = 0.0
        self.scan_headings = []
        self.scan_index = 0
        self.scan_observations = []
        self.scan_observation_headings = []
        self.scan_settle_until = 0.0
        self.scan_capture_after_sequence = -1
        self.scan_waiting_for_vlm = False
        self.scan_request_sequence = -1
        self.scan_retry_count = 0
        self.last_target_grounding_error = "none"
        self.depth_reobserve_attempts = 0
        self.target_probe_attempts = 0
        self.last_depth_recovery_action = "none"
        self.initial_costmap_clear_required = False
        self.initial_costmap_clear_requested = False
        self.initial_costmap_clear_pending = 0
        self.initial_costmap_clear_token = 0
        self.initial_costmap_clear_errors = []
        self.initial_costmap_clear_baseline_revision = 0
        self.initial_costmap_clear_completed_at = 0.0
        self.initial_costmap_refresh_deadline = 0.0
        self.last_initial_costmap_clear_status = "not_required"
        self.behavior_costmap_revision = 0
        self.last_behavior_costmap_received = 0.0

        self.frontier_generation = 0
        self.frontier_candidates = []
        self.frontier_rejected_ids = set()
        self.frontier_request = None
        self.frontier_request_pending = False
        self.frontier_decision_failures = 0
        self.frontier_selected_id = None
        self.frontier_selected_reason = ""
        self.frontier_selected_confidence = 0.0
        self.frontier_context = ""
        self.frontier_goal_is_final = False
        self.frontier_path_samples = []
        self.frontier_path_length = 0.0
        self.frontier_scene_rgb = None

        sample_period = 1.0 / max(0.1, float(self.p.vlm_sample_rate))
        self.sample_timer = self.create_timer(sample_period, self.sample_latest_frame)
        self.result_timer = self.create_timer(0.05, self.drain_worker_results)
        self.navigation_timer = self.create_timer(
            max(0.1, float(self.p.navigation_tick_period)), self.navigation_tick
        )
        self.safety_timer = self.create_timer(0.2, self.safety_tick)
        self.diagnostics_timer = self.create_timer(1.0, self.publish_diagnostics)
        self.publish_state()

    # ---------- lifecycle and safety ----------

    def log_worker_result(
        self, completed: WorkerResult, age: float, state_before: str, disposition: str
    ):
        snapshot = completed.snapshot
        stamp = snapshot.stamp
        parsed = None
        if isinstance(completed.result, VLMResult):
            parsed = {
                "target_visible": completed.result.target_visible,
                "object_match": completed.result.object_match,
                "qualifier_match": completed.result.qualifier_match,
                "relation_match": completed.result.relation_match,
                "confidence": completed.result.confidence,
                "target_pixel": (
                    {
                        "u": completed.result.target_pixel.u,
                        "v": completed.result.target_pixel.v,
                    }
                    if completed.result.target_pixel is not None
                    else None
                ),
                "evidence_pixel": (
                    {
                        "u": completed.result.evidence_pixel.u,
                        "v": completed.result.evidence_pixel.v,
                    }
                    if completed.result.evidence_pixel is not None
                    else None
                ),
                "coordinate_mode": completed.result.coordinate_mode,
            }
        elif isinstance(completed.result, FrontierDecision):
            parsed = {
                "selected_frontier_id": completed.result.selected_frontier_id,
                "confidence": completed.result.confidence,
                "reason": completed.result.reason,
            }
        candidates = [
            {
                "id": item.candidate_id,
                "x": round(item.x, 3),
                "y": round(item.y, 3),
                "bearing_rad": round(item.bearing, 4),
                "distance_m": round(item.distance, 3),
                "frontier_cells": item.cell_count,
            }
            for item in getattr(snapshot, "frontier_candidates", ())
        ]
        record = {
            "event": "vlm_response",
            "request_kind": getattr(snapshot, "request_kind", "target"),
            "sequence": snapshot.sequence,
            "task_epoch": snapshot.task_epoch,
            "frontier_generation": getattr(snapshot, "frontier_generation", 0),
            "frontier_context": getattr(snapshot, "frontier_context", ""),
            "frontier_candidates": candidates,
            "target_description": snapshot.target_description,
            "image_stamp": {
                "sec": int(getattr(stamp, "sec", 0)),
                "nanosec": int(getattr(stamp, "nanosec", 0)),
            },
            "image_frame": snapshot.frame_id,
            "latency_s": round(float(completed.latency_s), 6),
            "result_age_s": round(float(age), 6),
            "navigation_state_before": state_before,
            "navigation_state_after": self.state,
            "disposition": disposition,
            "accepted": disposition.startswith("accepted"),
            "raw_response": completed.raw_response,
            "parsed_result": parsed,
            "error": completed.error,
            "target_grounding_error": getattr(
                self, "last_target_grounding_error", "none"
            ),
        }
        recorder = getattr(self, "image_recorder", None)
        if recorder is not None:
            saved = recorder.record(snapshot, completed.result, disposition)
            if not saved and recorder.last_error != "none":
                self.get_logger().error(
                    f"Cannot save VLM ARM image record: {recorder.last_error}"
                )
            if hasattr(recorder, "record_event"):
                recorder.record_event(record)
        self.publish_vlm_output(record)

    def publish_vlm_output(self, record):
        publisher = getattr(self, "vlm_text_pub", None)
        marker_publisher = getattr(self, "marker_pub", None)
        if publisher is None or marker_publisher is None:
            return
        text_message = String()
        text_message.data = json.dumps(record, ensure_ascii=False)
        publisher.publish(text_message)

        parsed = record.get("parsed_result")
        request_kind = str(record.get("request_kind", "target"))
        visible = bool(parsed and parsed.get("target_visible"))
        confidence = float(parsed.get("confidence", 0.0)) if parsed else 0.0
        self.last_vlm_confidence = confidence
        self.last_vlm_disposition = str(record["disposition"])

        marker = Marker()
        marker.header.frame_id = self.p.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vlm_status_text"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 1.65
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.22
        marker.color.r = 1.0
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 1.0
        if request_kind == "frontier":
            selected = parsed.get("selected_frontier_id", "-") if parsed else "-"
            reason = parsed.get("reason", "") if parsed else ""
            marker.text = (
                f"VLM frontier: {selected} confidence={confidence:.2f}\n"
                f"{record['navigation_state_after']} | {record['disposition']}\n"
                f"{reason[:90]} | latency={float(record['latency_s']):.2f}s"
            )
        else:
            marker.text = (
                f"VLM: {record['target_description']}\n"
                f"{record['navigation_state_after']} | {record['disposition']}\n"
                f"visible={visible} confidence={confidence:.2f} "
                f"latency={float(record['latency_s']):.2f}s"
            )
        marker_publisher.publish(MarkerArray(markers=[marker]))

    def publish_easy_event(self, event, **details):
        if not self.easy_case_enabled():
            return
        record = {
            "event": str(event),
            "easy_case_mode": True,
            "state": getattr(self, "state", "unknown"),
            "target_description": str(
                getattr(self.p, "target_description", "chair")
            ),
            "target_reference_position": getattr(self, "target_reference_position", None),
            "direct_goal": getattr(self, "easy_direct_goal", None),
            **details,
        }
        recorder = getattr(self, "image_recorder", None)
        if recorder is not None and hasattr(recorder, "record_event"):
            recorder.record_event(record)
        publisher = getattr(self, "vlm_text_pub", None)
        if publisher is not None:
            message = String()
            message.data = json.dumps(record, ensure_ascii=False)
            publisher.publish(message)

    def easy_case_enabled(self):
        return bool(
            getattr(getattr(self, "p", None), "easy_case_mode", False)
        )

    def on_parameters_changed(self, parameters):
        values = {parameter.name: parameter.value for parameter in parameters}
        requested_enabled = bool(
            values.get("enabled", self.get_parameter("enabled").value)
        )
        if "easy_case_mode" in values:
            if requested_enabled:
                return SetParametersResult(
                    successful=False,
                    reason="change easy_case_mode only while enabled=false",
                )
        if "target_description" in values:
            description = str(values["target_description"]).strip()
            if not description:
                return SetParametersResult(
                    successful=False, reason="target_description cannot be empty"
                )
            if requested_enabled:
                return SetParametersResult(
                    successful=False,
                    reason="change target_description only while enabled=false",
                )

        if "easy_case_mode" in values:
            self.p.easy_case_mode = bool(values["easy_case_mode"])
            self.task_epoch += 1
            self.reset_task(DISARMED)
        if "target_description" in values:
            self.p.target_description = description
            self.task_epoch += 1
            self.reset_task(DISARMED)
        if "enabled" in values:
            self.p.enabled = requested_enabled
            if requested_enabled:
                try:
                    record_path = self.image_recorder.start_arm()
                    self.get_logger().info(f"VLM ARM image record: {record_path}")
                except OSError as error:
                    self.image_recorder.last_error = (
                        f"{type(error).__name__}: {error}"
                    )
                    self.get_logger().error(
                        f"Cannot start VLM ARM image record: {error}"
                    )
                self.reset_task(SCANNING)
                self.require_initial_costmap_clear()
                self.enabled_since = time.monotonic()
                self.easy_started = (
                    self.enabled_since if self.easy_case_enabled() else 0.0
                )
                self.last_rgbd_pair_received = self.enabled_since
                self.last_valid_rgbd_received = self.enabled_since
                self.last_camera_tf_success = self.enabled_since
                self.last_robot_tf_success = self.enabled_since
                self.sensor_wait_started = 0.0
                self.sensor_wait_reason = "none"
                self.sensor_recovery_count = 0
            else:
                self.initial_costmap_clear_token += 1
                self.initial_costmap_clear_required = False
                self.initial_costmap_clear_requested = False
                self.initial_costmap_clear_pending = 0
                self.initial_costmap_clear_errors = []
                self.initial_costmap_clear_completed_at = 0.0
                self.initial_costmap_refresh_deadline = 0.0
                self.last_initial_costmap_clear_status = "not_required"
                self.reset_task(DISARMED)
        return SetParametersResult(successful=True)

    def require_initial_costmap_clear(self):
        if not bool(getattr(self.p, "clear_costmap_on_arm", True)):
            self.initial_costmap_clear_token += 1
            self.initial_costmap_clear_required = False
            self.initial_costmap_clear_requested = False
            self.initial_costmap_clear_pending = 0
            self.initial_costmap_clear_errors = []
            self.initial_costmap_clear_completed_at = 0.0
            self.initial_costmap_refresh_deadline = 0.0
            self.last_initial_costmap_clear_status = "disabled"
            return
        self.initial_costmap_clear_required = True
        self.initial_costmap_clear_requested = False
        self.initial_costmap_clear_pending = 0
        self.initial_costmap_clear_errors = []
        self.initial_costmap_clear_baseline_revision = int(
            getattr(self, "behavior_costmap_revision", 0)
        )
        self.initial_costmap_clear_completed_at = 0.0
        self.initial_costmap_refresh_deadline = 0.0
        self.initial_costmap_clear_token += 1
        self.last_initial_costmap_clear_status = "pending"

    def ensure_initial_costmap_clear_done(self):
        if not bool(getattr(self, "initial_costmap_clear_required", False)):
            return True
        if self.initial_costmap_clear_pending > 0:
            self.publish_stop()
            return False
        if self.initial_costmap_clear_requested:
            completed_at = float(
                getattr(self, "initial_costmap_clear_completed_at", 0.0)
            )
            refreshed = (
                completed_at > 0.0
                and int(getattr(self, "behavior_costmap_revision", 0))
                > int(
                    getattr(
                        self, "initial_costmap_clear_baseline_revision", 0
                    )
                )
                and float(
                    getattr(self, "last_behavior_costmap_received", 0.0)
                )
                >= completed_at
            )
            if refreshed:
                self.initial_costmap_clear_required = False
                self.initial_costmap_clear_requested = False
                self.initial_costmap_clear_completed_at = 0.0
                self.initial_costmap_refresh_deadline = 0.0
                self.last_initial_costmap_clear_status = "cleared_and_refreshed"
                self.get_logger().info(
                    "Nav2 costmaps cleared and behavior costmap refreshed "
                    "before initial ARM scan"
                )
                return True
            deadline = float(
                getattr(self, "initial_costmap_refresh_deadline", 0.0)
            )
            if deadline > 0.0 and time.monotonic() > deadline:
                self.last_initial_costmap_clear_status = (
                    "failed:behavior_costmap_refresh_timeout"
                )
                self.fail_safe(
                    "Behavior costmap did not refresh after Nav2 costmap "
                    "clear; refusing initial ARM scan"
                )
            self.publish_stop()
            return False
        clients = getattr(self, "costmap_clear_clients", ())
        if not clients:
            self.fail_safe("No Nav2 costmap clear services configured")
            return False
        self.initial_costmap_clear_requested = True
        self.initial_costmap_clear_errors = []
        self.initial_costmap_clear_baseline_revision = int(
            getattr(self, "behavior_costmap_revision", 0)
        )
        token = self.initial_costmap_clear_token
        for label, client in clients:
            service_name = getattr(client, "srv_name", str(label))
            if not client.wait_for_service(timeout_sec=0.5):
                self.initial_costmap_clear_errors.append(
                    f"{label}:{service_name}:unavailable"
                )
                continue
            self.initial_costmap_clear_pending += 1
            future = client.call_async(ClearEntireCostmap.Request())
            future.add_done_callback(
                lambda done, costmap=label, generation=token: (
                    self.on_initial_costmap_clear_result(
                        done, costmap, generation
                    )
                )
            )
        if self.initial_costmap_clear_pending <= 0:
            reason = "; ".join(self.initial_costmap_clear_errors)
            self.fail_safe(
                f"Cannot clear Nav2 costmaps before ARM scan: {reason}"
            )
            return False
        self.last_initial_costmap_clear_status = "requested"
        self.get_logger().info(
            "Requested Nav2 costmap clear before initial ARM scan"
        )
        self.publish_stop()
        return False

    def on_initial_costmap_clear_result(self, future, label, token):
        if token != getattr(self, "initial_costmap_clear_token", 0):
            return
        try:
            future.result()
        except Exception as error:
            self.initial_costmap_clear_errors.append(
                f"{label}:{type(error).__name__}:{error}"
            )
        self.initial_costmap_clear_pending = max(
            0, self.initial_costmap_clear_pending - 1
        )
        if self.initial_costmap_clear_pending > 0:
            return
        if self.initial_costmap_clear_errors:
            reason = "; ".join(self.initial_costmap_clear_errors)
            self.last_initial_costmap_clear_status = f"failed:{reason}"
            self.fail_safe(
                f"Nav2 costmap clear failed before ARM scan: {reason}"
            )
            return
        completed_at = time.monotonic()
        self.initial_costmap_clear_completed_at = completed_at
        self.initial_costmap_refresh_deadline = completed_at + max(
            0.5,
            float(
                getattr(self.p, "initial_costmap_refresh_timeout", 3.0)
            ),
        )
        self.last_initial_costmap_clear_status = "waiting_behavior_refresh"
        self.get_logger().info(
            "Nav2 costmaps cleared; waiting for a fresh behavior costmap "
            "before initial ARM scan"
        )

    def reset_task(self, state):
        self.cancel_motion(publish_stop=True)
        self.reset_approach_stop(reset_retry=True)
        self.latest_snapshot = None
        self.target_tracker.reset()
        self.target_reference_position = None
        self.target_seen_time = 0.0
        self.easy_started = 0.0
        self.easy_alignment_complete = False
        self.easy_target_distance = math.inf
        self.easy_alignment_error = math.inf
        self.easy_direct_goal = None
        self.easy_path_deviation = math.inf
        self.session_origin = None
        self.scan_headings = []
        self.scan_index = 0
        self.scan_observations = []
        self.scan_observation_headings = []
        self.scan_settle_until = 0.0
        self.scan_capture_after_sequence = -1
        self.scan_waiting_for_vlm = False
        self.scan_request_sequence = -1
        self.scan_retry_count = 0
        self.last_target_grounding_error = "none"
        self.depth_reobserve_attempts = 0
        self.target_probe_attempts = 0
        self.last_depth_recovery_action = "none"
        self.frontier_generation += 1
        self.frontier_candidates = []
        self.frontier_rejected_ids = set()
        self.frontier_request = None
        self.frontier_request_pending = False
        self.frontier_decision_failures = 0
        self.frontier_selected_id = None
        self.frontier_selected_reason = ""
        self.frontier_selected_confidence = 0.0
        self.frontier_goal_is_final = False
        self.frontier_path_samples = []
        self.frontier_path_length = 0.0
        self.frontier_scene_rgb = None
        self.api_failures = 0
        self.plan_attempt_count = 0
        self.last_plan_kind = "none"
        self.last_plan_status = -1
        self.last_plan_pose_count = 0
        self.last_plan_path_length = 0.0
        self.last_plan_planning_time = 0.0
        self.last_plan_endpoint_radius = math.inf
        self.last_plan_max_radius = math.inf
        self.last_plan_candidate = None
        self.last_plan_rejection_reason = "none"
        self.plan_window_origin = None
        self.last_plan_commit_length = 0.0
        self.last_plan_radius_clipped = False
        self.rolling_goal_is_final = True
        self.active_motion_origin = None
        self.sensor_wait_started = 0.0
        self.sensor_wait_reason = "none"
        self.sensor_recovery_count = 0
        self.pending_rgbd_frames.clear()
        self.last_rgbd_queued = 0.0
        if state != FAILED:
            self.last_failure_reason = "none"
        self.task_epoch += 1
        marker_publisher = getattr(self, "marker_pub", None)
        if marker_publisher is not None:
            self.clear_task_boundary()
        self.set_state(state)

    def set_state(self, state):
        if state != self.state:
            self.state = state
            if state == TARGET_CONFIRMING:
                self.target_confirmation_started = time.monotonic()
            else:
                self.target_confirmation_started = 0.0
            self.get_logger().info(f"State -> {state}")
            self.publish_state()

    def publish_state(self):
        message = String()
        message.data = self.state
        self.state_pub.publish(message)

    def publish_stop(self):
        if not bool(self.p.publish_stop_command) or not rclpy.ok():
            return
        message = Twist()
        for _ in range(3):
            self.stop_pub.publish(message)

    def fail_safe(self, reason):
        self.last_failure_reason = str(reason)
        self.get_logger().error(reason)
        self.cancel_motion(publish_stop=True)
        self.set_state(FAILED)
        self.publish_easy_event("easy_case_failed", reason=str(reason))

    def enter_sensor_wait(self, reason):
        if self.state in (DISARMED, API_ERROR, SUCCEEDED, FAILED):
            return
        reason = str(reason)
        if self.state != SENSOR_WAITING:
            self.sensor_wait_started = time.monotonic()
            self.sensor_recovery_count = 0
            self.latest_snapshot = None
            self.task_epoch += 1
            self.cancel_motion(publish_stop=True)
            self.get_logger().warn(f"Sensor safety pause: {reason}")
            self.set_state(SENSOR_WAITING)
        elif reason != self.sensor_wait_reason:
            self.get_logger().warn(f"Sensor wait reason changed: {reason}")
        self.sensor_wait_reason = reason

    def recover_from_sensor_wait(self):
        if self.state != SENSOR_WAITING:
            return False
        pose = self.robot_pose()
        if pose is None:
            return False
        self.get_logger().info(
            "RGB-D and image-time TF recovered; restarting scan from current pose"
        )
        self.sensor_wait_started = 0.0
        self.sensor_wait_reason = "none"
        self.sensor_recovery_count = 0
        self.target_tracker.reset()
        self.target_reference_position = None
        self.target_seen_time = 0.0
        self.frontier_generation += 1
        self.frontier_candidates = []
        self.frontier_rejected_ids = set()
        self.frontier_request = None
        self.frontier_request_pending = False
        self.frontier_selected_id = None
        self.frontier_selected_reason = ""
        self.frontier_selected_confidence = 0.0
        self.frontier_path_samples = []
        self.frontier_path_length = 0.0
        self.frontier_scene_rgb = None
        self.begin_scan(pose)
        return True

    @staticmethod
    def elapsed_since(timestamp, now):
        return math.inf if timestamp <= 0.0 else max(0.0, now - timestamp)

    def safety_tick(self):
        if not self.get_parameter("enabled").value or self.state in (
            DISARMED,
            API_ERROR,
            SUCCEEDED,
            FAILED,
        ):
            return
        # Keep robot TF health independent of navigation_tick().  In
        # TARGET_CONFIRMING that timer intentionally does not call robot_pose,
        # which previously made a healthy TF look stale and caused a false
        # safety failure after tf_failure_timeout.
        pose = self.robot_pose()
        now = time.monotonic()
        valid_rgbd_age = self.elapsed_since(self.last_valid_rgbd_received, now)
        image_tf_age = self.elapsed_since(self.last_camera_tf_success, now)
        if valid_rgbd_age > float(self.p.rgbd_wait_timeout):
            self.enter_sensor_wait(
                "No valid aligned RGB-D received within safety timeout"
            )
        elif image_tf_age > float(self.p.tf_failure_timeout):
            self.enter_sensor_wait(
                "Valid RGB-D is arriving but image-time camera TF is unavailable"
            )

        if self.state == SENSOR_WAITING:
            hard_timeout = float(self.p.sensor_failure_timeout)
            if (
                hard_timeout > 0.0
                and self.sensor_wait_started > 0.0
                and now - self.sensor_wait_started > hard_timeout
            ):
                self.fail_safe(
                    f"Sensor unavailable beyond hard timeout: "
                    f"{self.sensor_wait_reason}"
                )
            return
        if now - self.last_robot_tf_success > float(self.p.tf_failure_timeout):
            self.fail_safe("Robot TF unavailable beyond safety timeout")
            return
        if pose is None:
            return
        if self.state == APPROACHING and self.target_reference_position is not None:
            if self.target_reference_distance(pose) <= self.approach_stop_radius():
                self.begin_approach_stop("distance_contract_reached")
                return
        if self.state == APPROACH_STOPPING:
            self.advance_approach_stop(now, pose)
            return
        confirmation_timeout = float(
            getattr(self.p, "target_confirmation_timeout", 20.0)
        )
        confirmation_started = float(
            getattr(self, "target_confirmation_started", 0.0)
        )
        if (
            self.state == TARGET_CONFIRMING
            and confirmation_timeout > 0.0
            and confirmation_started > 0.0
            and now - confirmation_started > confirmation_timeout
        ):
            tracker = self.target_tracker
            self.fail_safe(
                "Target confirmation did not converge within "
                f"{confirmation_timeout:.1f}s "
                f"(progress={tracker.progress}/{tracker.required_frames}, "
                f"spatial_resets={tracker.reset_count})"
            )
            return
        if (
            self.easy_case_enabled()
            and getattr(self, "easy_started", 0.0) > 0.0
            and float(getattr(self.p, "easy_case_timeout", 120.0)) > 0.0
            and now - self.easy_started
            > float(getattr(self.p, "easy_case_timeout", 120.0))
        ):
            self.fail_safe(
                "Easy-case task exceeded "
                f"{float(self.p.easy_case_timeout):.1f} second safety timeout"
            )
            return
        if (
            getattr(self, "active_motion_origin", None) is not None
            and (self.goal_handle is not None or self.goal_pending)
        ):
            distance = math.hypot(
                pose[0] - self.active_motion_origin[0],
                pose[1] - self.active_motion_origin[1],
            )
            # The command endpoint lies on or inside the rolling circle.
            # Allow normal Nav2 goal tolerance/local-controller settling, but
            # retain a hard watchdog for a genuinely runaway command.
            if distance > float(self.p.max_travel_radius) + 0.50:
                self.fail_safe("Robot exceeded active rolling-path radius")
                return
        if (
            self.target_reference_position is not None
            and self.state in (TARGET_ALIGNING, APPROACHING)
            and now - self.target_seen_time > float(self.p.target_lost_timeout)
        ):
            if self.easy_case_enabled():
                self.fail_safe("Target lost during easy-case alignment or approach")
                return
            self.fail_safe("Target lost during approach")
            return
        if (
            (self.goal_handle is not None or self.goal_pending or self.plan_pending)
            and now - self.goal_started > float(self.p.goal_timeout)
        ):
            self.fail_safe("Navigation or path-validation goal timed out")

    # ---------- sensor input and VLM worker ----------

    def on_camera_info(self, message):
        self.camera_info = message

    def on_behavior_costmap(self, _message):
        self.behavior_costmap_revision += 1
        self.last_behavior_costmap_received = time.monotonic()

    def on_arrival_odom(self, message):
        twist = message.twist.twist
        self.arrival_last_odom_received = time.monotonic()
        self.arrival_linear_speed = math.hypot(
            float(twist.linear.x), float(twist.linear.y)
        )
        self.arrival_angular_speed = abs(float(twist.angular.z))
        if self.state != APPROACH_STOPPING:
            return
        if (
            self.arrival_linear_speed
            <= float(self.p.arrival_linear_speed_tolerance)
            and self.arrival_angular_speed
            <= float(self.p.arrival_angular_speed_tolerance)
        ):
            self.arrival_stationary_count += 1
        else:
            self.arrival_stationary_count = 0

    def on_map(self, message):
        expected = int(message.info.width) * int(message.info.height)
        if expected <= 0 or len(message.data) != expected:
            self.get_logger().warn("Ignoring malformed OccupancyGrid")
            return
        self.map_message = message
        self.grid = np.asarray(message.data, dtype=np.int16).reshape(
            message.info.height, message.info.width
        )
        self.map_revision += 1

    @staticmethod
    def image_to_rgb(message):
        encoding = message.encoding.lower()
        if encoding not in ("rgb8", "bgr8"):
            raise ValueError(f"unsupported RGB encoding: {message.encoding}")
        row = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step
        )
        image = row[:, : message.width * 3].reshape(
            message.height, message.width, 3
        )
        if encoding == "bgr8":
            image = image[:, :, ::-1]
        return np.ascontiguousarray(image)

    @staticmethod
    def image_to_depth_m(message):
        row = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step
        )
        if message.encoding.upper() == "32FC1":
            return (
                row[:, : message.width * 4]
                .copy()
                .view(np.float32)
                .reshape(message.height, message.width)
            )
        if message.encoding.upper() in ("16UC1", "MONO16"):
            return (
                row[:, : message.width * 2]
                .copy()
                .view(np.uint16)
                .reshape(message.height, message.width)
                .astype(np.float32)
                / 1000.0
            )
        raise ValueError(f"unsupported depth encoding: {message.encoding}")

    def on_rgbd(self, rgb_message, depth_message):
        if (
            not self.get_parameter("enabled").value
            or self.state in (DISARMED, SUCCEEDED, FAILED)
        ):
            return
        now = time.monotonic()
        self.last_rgbd_pair_received = now
        if self.camera_info is None:
            self.sensor_recovery_count = 0
            return
        rgb_size = (int(rgb_message.height), int(rgb_message.width))
        depth_size = (int(depth_message.height), int(depth_message.width))
        info_size = (int(self.camera_info.height), int(self.camera_info.width))
        depth_frame = depth_message.header.frame_id
        if (
            rgb_size != depth_size
            or rgb_size != info_size
            or (depth_frame and depth_frame != self.p.camera_frame)
        ):
            self.invalid_rgbd_frames += 1
            self.sensor_recovery_count = 0
            self.get_logger().warn(
                "Rejecting RGB-D metadata mismatch: "
                f"rgb={rgb_size}, depth={depth_size}, camera_info={info_size}, "
                f"depth_frame={depth_frame or 'empty'}",
                throttle_duration_sec=5.0,
            )
            return
        try:
            rgb = self.image_to_rgb(rgb_message)
            depth = self.image_to_depth_m(depth_message)
        except (ValueError, TypeError) as error:
            self.invalid_rgbd_frames += 1
            self.sensor_recovery_count = 0
            self.get_logger().warn(f"Invalid RGB-D frame: {error}", throttle_duration_sec=5.0)
            return
        self.last_valid_rgbd_received = now
        # FAST_LIO and SLAM can publish a correct transform a short time after
        # the camera frame arrives.  A blocking lookup here used to discard
        # every frame whenever odometry lag exceeded 0.2 s.  Retain a bounded,
        # sampled queue and retry exact image-time lookups without blocking the
        # RGB-D callback.  We intentionally do not substitute Time() (latest
        # TF), because that would project pixels using the wrong robot pose.
        queue_period = 1.0 / max(1.0, float(self.p.vlm_sample_rate))
        if (
            not self.pending_rgbd_frames
            or now - self.last_rgbd_queued >= queue_period
        ):
            queue_limit = max(1, int(self.p.image_tf_queue_size))
            if len(self.pending_rgbd_frames) >= queue_limit:
                self.pending_rgbd_frames.popleft()
                self.image_tf_queue_drops += 1
            self.pending_rgbd_frames.append(
                SimpleNamespace(
                    received_monotonic=now,
                    stamp=depth_message.header.stamp,
                    frame_id=depth_message.header.frame_id or self.p.camera_frame,
                    rgb=rgb.copy(),
                    depth_m=depth.copy(),
                    intrinsics=(
                        float(self.camera_info.k[0]),
                        float(self.camera_info.k[4]),
                        float(self.camera_info.k[2]),
                        float(self.camera_info.k[5]),
                    ),
                )
            )
            self.last_rgbd_queued = now
        self.resolve_pending_rgbd()

    def resolve_pending_rgbd(self):
        """Resolve queued frames only when their exact image-time TF exists."""
        wait_timeout = max(0.1, float(self.p.image_tf_wait_timeout))
        while self.pending_rgbd_frames:
            frame = self.pending_rgbd_frames[0]
            if time.monotonic() - frame.received_monotonic > wait_timeout:
                self.pending_rgbd_frames.popleft()
                self.image_tf_queue_drops += 1
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.p.global_frame,
                    self.p.camera_frame,
                    Time.from_msg(frame.stamp),
                    timeout=Duration(seconds=0.0),
                )
            except TransformException as error:
                self.image_tf_failures += 1
                self.last_image_tf_error = str(error)
                self.sensor_recovery_count = 0
                self.get_logger().warn(
                    f"Image-time camera TF pending: {error}",
                    throttle_duration_sec=5.0,
                )
                return
            self.pending_rgbd_frames.popleft()
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            matrix = transform_matrix(
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
            )
            self.sequence += 1
            self.latest_snapshot = FrameSnapshot(
                sequence=self.sequence,
                task_epoch=self.task_epoch,
                target_description=str(
                    self.get_parameter("target_description").value
                ),
                captured_monotonic=frame.received_monotonic,
                stamp=frame.stamp,
                frame_id=frame.frame_id,
                rgb=frame.rgb,
                depth_m=frame.depth_m,
                intrinsics=frame.intrinsics,
                transform_matrix=matrix,
            )
            self.last_camera_tf_success = time.monotonic()
            self.last_image_tf_error = "none"
            if self.state == SENSOR_WAITING:
                self.sensor_recovery_count += 1
                if self.sensor_recovery_count >= max(
                    1, int(self.p.sensor_recovery_frames)
                ):
                    self.recover_from_sensor_wait()
                    return
            else:
                self.sensor_recovery_count = 0

    def ensure_worker(self):
        if self.worker is not None:
            return True
        try:
            client = OpenAICompatibleVLMClient(
                timeout_s=float(self.p.vlm_timeout),
                image_detail=str(self.p.image_detail),
                jpeg_quality=int(self.p.jpeg_quality),
            )
        except Exception as error:
            self.record_api_failure(f"Cannot initialize VLM client: {error}")
            return False
        self.worker = LatestFrameWorker(
            lambda snapshot: (
                client.infer_frontier(
                    snapshot.rgb,
                    snapshot.auxiliary_rgb,
                    snapshot.frontier_candidates,
                    snapshot.target_description,
                )
                if snapshot.request_kind == "frontier"
                else client.infer(snapshot.rgb, snapshot.target_description)
            ),
            self.worker_results.put,
            get_raw_response=lambda: client.last_raw_response,
            pass_snapshot=True,
        )
        self.get_logger().info(f"VLM client initialized with model {client.model}")
        return True

    def sample_latest_frame(self):
        if (
            not self.get_parameter("enabled").value
            or self.state in (DISARMED, SENSOR_WAITING, SUCCEEDED, FAILED)
        ):
            return
        frontier_request = getattr(self, "frontier_request", None)
        if self.state in (FRONTIER_SELECTING, API_ERROR) and frontier_request is not None:
            if getattr(self, "frontier_request_pending", False) and self.ensure_worker():
                # Age is measured from this concrete API attempt. Reusing the
                # original frontier-cycle timestamp made every later recovery
                # response look stale even when its own latency was healthy.
                frontier_request = replace(
                    frontier_request, captured_monotonic=time.monotonic()
                )
                self.frontier_request = frontier_request
                self.frontier_request_pending = False
                self.worker.submit(frontier_request)
            return
        snapshot = self.latest_snapshot
        if self.state in (SCANNING, TARGET_REOBSERVING):
            if (
                self.scan_waiting_for_vlm
                or self.scan_settle_until <= 0.0
                or time.monotonic() < self.scan_settle_until
                or snapshot is None
                or snapshot.sequence <= self.scan_capture_after_sequence
                or snapshot.captured_monotonic < self.scan_settle_until
            ):
                return
            if not self.ensure_worker():
                return
            self.scan_waiting_for_vlm = True
            self.scan_request_sequence = snapshot.sequence
            self.last_submitted_sequence = snapshot.sequence
            self.worker.submit(snapshot)
            observation = (
                "target depth reobservation"
                if self.state == TARGET_REOBSERVING
                else (
                    "scan heading "
                    f"{self.scan_index + 1}/{len(self.scan_headings)}"
                )
            )
            self.get_logger().info(
                "Stationary frame submitted to VLM: "
                f"{observation}, sequence={snapshot.sequence}"
            )
            return
        if snapshot is None or snapshot.sequence == self.last_submitted_sequence:
            return
        if not self.ensure_worker():
            return
        self.last_submitted_sequence = snapshot.sequence
        self.worker.submit(snapshot)

    def drain_worker_results(self):
        while True:
            try:
                completed = self.worker_results.get_nowait()
            except queue.Empty:
                return
            self.handle_worker_result(completed)

    def record_api_failure(self, reason):
        self.last_api_error = str(reason)
        self.api_failures += 1
        self.rejected_results += 1
        self.get_logger().warn(
            f"VLM failure {self.api_failures}/{int(self.p.api_failure_limit)}: {reason}"
        )
        if (
            self.get_parameter("enabled").value
            and self.state != FAILED
            and self.api_failures >= int(self.p.api_failure_limit)
        ):
            self.cancel_motion(publish_stop=True)
            self.set_state(API_ERROR)

    def record_frontier_failure(self, reason):
        self.last_api_error = str(reason)
        self.frontier_decision_failures += 1
        self.rejected_results += 1
        self.frontier_request_pending = True
        self.get_logger().warn(
            "Frontier VLM failure "
            f"{self.frontier_decision_failures}/"
            f"{int(self.p.frontier_decision_failure_limit)}: {reason}"
        )
        if self.frontier_decision_failures >= int(
            self.p.frontier_decision_failure_limit
        ):
            self.cancel_motion(publish_stop=True)
            self.set_state(API_ERROR)

    def handle_worker_result(self, completed: WorkerResult):
        self.last_api_latency = completed.latency_s
        snapshot = completed.snapshot
        age = time.monotonic() - snapshot.captured_monotonic
        self.last_result_age = age
        state_before = self.state
        scan_result = self.is_current_scan_result(snapshot)
        depth_reobserve_result = self.is_current_depth_reobserve_result(
            snapshot
        )
        if getattr(snapshot, "request_kind", "target") == "frontier":
            self.handle_frontier_worker_result(completed, age, state_before)
            return
        if self.state == SENSOR_WAITING:
            self.rejected_results += 1
            self.log_worker_result(
                completed, age, state_before, "rejected_state_sensor_waiting"
            )
            return
        if completed.error:
            self.record_api_failure(completed.error)
            self.log_worker_result(completed, age, state_before, "api_error")
            if scan_result and self.state == SCANNING:
                self.retry_stationary_scan_view(completed.error)
            elif (
                depth_reobserve_result
                and self.state == TARGET_REOBSERVING
            ):
                self.retry_depth_reobservation(completed.error)
            return
        rejection = None
        if not self.get_parameter("enabled").value:
            rejection = "rejected_disabled"
        elif self.state in (DISARMED, SENSOR_WAITING, SUCCEEDED, FAILED):
            rejection = f"rejected_state_{self.state.lower()}"
        elif snapshot.task_epoch != self.task_epoch:
            rejection = "rejected_task_epoch"
        elif snapshot.target_description != str(
            self.get_parameter("target_description").value
        ):
            rejection = "rejected_target_changed"
        elif age > float(self.p.max_result_age):
            rejection = "rejected_stale"
        if rejection is not None:
            self.rejected_results += 1
            self.log_worker_result(completed, age, state_before, rejection)
            if scan_result and self.state == SCANNING:
                self.retry_stationary_scan_view(rejection)
            elif (
                depth_reobserve_result
                and self.state == TARGET_REOBSERVING
            ):
                self.retry_depth_reobservation(rejection)
            return
        result = completed.result
        if result is None:
            self.record_api_failure("empty result")
            self.log_worker_result(completed, age, state_before, "api_error_empty")
            if scan_result and self.state == SCANNING:
                self.retry_stationary_scan_view("empty result")
            elif (
                depth_reobserve_result
                and self.state == TARGET_REOBSERVING
            ):
                self.retry_depth_reobservation("empty result")
            return
        self.last_vlm_confidence = result.confidence
        self.api_failures = 0
        self.accepted_results += 1
        if self.state == API_ERROR:
            if self.target_reference_position is None:
                self.set_state(SCANNING)
            elif (
                self.easy_case_enabled()
                and not self.easy_alignment_complete
            ):
                self.set_state(TARGET_ALIGNING)
            else:
                self.set_state(APPROACHING)
        self.publish_debug(snapshot, result)
        if (
            not result.target_visible
            or not result.object_match
            or not result.qualifier_match
            or not result.relation_match
            or result.target_pixel is None
            or result.evidence_pixel is None
            or result.confidence < float(self.p.confidence_threshold)
        ):
            if self.target_reference_position is None:
                self.target_tracker.reset()
                self.clear_vlm_grounding_markers()
                if self.state == TARGET_CONFIRMING:
                    pose = self.robot_pose()
                    if pose is not None:
                        self.begin_scan(pose)
                    else:
                        self.set_state(SCANNING)
            if depth_reobserve_result:
                pose = self.robot_pose()
                if pose is None:
                    self.fail_safe(
                        "Target disappeared during depth reobservation and "
                        "robot pose is unavailable"
                    )
                else:
                    self.target_tracker.reset()
                    self.target_reference_position = None
                    self.begin_scan(pose)
            self.log_worker_result(
                completed,
                age,
                state_before,
                (
                    "accepted_no_target_after_depth_reobserve"
                    if depth_reobserve_result
                    else "accepted_no_target"
                ),
            )
            if scan_result and self.state == SCANNING:
                self.advance_stationary_scan_view(snapshot)
            return

        target, grounding_error = self.ground_pixel_with_reason(
            snapshot, result.target_pixel, require_ground=False
        )
        if target is None:
            self.last_target_grounding_error = grounding_error
            self.rejected_results += 1
            self.get_logger().warn(
                "VLM target pixel could not be grounded: "
                f"{grounding_error}; pixel=({result.target_pixel.u},"
                f"{result.target_pixel.v})",
                throttle_duration_sec=2.0,
            )
            if self.recover_target_depth(
                snapshot,
                result,
                grounding_error,
            ):
                self.log_worker_result(
                    completed,
                    age,
                    state_before,
                    "accepted_target_depth_recovery",
                )
                return
            self.log_worker_result(
                completed,
                age,
                state_before,
                "rejected_target_grounding",
            )
            if scan_result and self.state == SCANNING:
                self.retry_stationary_scan_view(grounding_error)
            elif (
                depth_reobserve_result
                and self.state == TARGET_REOBSERVING
            ):
                self.retry_depth_reobservation(grounding_error)
            return
        self.last_target_grounding_error = "none"
        if scan_result or depth_reobserve_result:
            self.scan_waiting_for_vlm = False
            self.scan_request_sequence = -1
        self.clear_target_depth_recovery()
        self.target_seen_time = time.monotonic()
        self.confirm_target(target)
        self.publish_grounded_markers(target)
        if self.easy_case_enabled():
            self.log_worker_result(
                completed, age, state_before, "accepted_target_easy_case"
            )
            return
        self.log_worker_result(completed, age, state_before, "accepted_target")

    def handle_frontier_worker_result(self, completed, age, state_before):
        snapshot = completed.snapshot
        if self.state == SENSOR_WAITING:
            self.rejected_results += 1
            self.log_worker_result(
                completed, age, state_before, "rejected_state_sensor_waiting"
            )
            return
        if completed.error:
            self.record_frontier_failure(completed.error)
            self.log_worker_result(
                completed, age, state_before, "frontier_api_error"
            )
            return
        rejection = None
        if not self.get_parameter("enabled").value:
            rejection = "rejected_disabled"
        elif self.state in (DISARMED, SUCCEEDED, FAILED):
            rejection = f"rejected_state_{self.state.lower()}"
        elif self.state not in (FRONTIER_SELECTING, API_ERROR):
            rejection = f"rejected_state_{self.state.lower()}"
        elif snapshot.task_epoch != self.task_epoch:
            rejection = "rejected_task_epoch"
        elif snapshot.frontier_generation != self.frontier_generation:
            rejection = "rejected_frontier_generation"
        elif age > float(self.p.max_result_age):
            rejection = "rejected_stale"
        if rejection is not None:
            self.rejected_results += 1
            self.log_worker_result(completed, age, state_before, rejection)
            return
        result = completed.result
        if not isinstance(result, FrontierDecision):
            self.record_frontier_failure("empty or wrong frontier result")
            self.log_worker_result(
                completed, age, state_before, "frontier_api_error_empty"
            )
            return
        available = {
            item.candidate_id: item
            for item in self.frontier_candidates
            if item.candidate_id not in self.frontier_rejected_ids
        }
        if (
            result.selected_frontier_id not in available
            or result.confidence < float(self.p.frontier_confidence_threshold)
        ):
            if result.selected_frontier_id in available:
                self.frontier_rejected_ids.add(result.selected_frontier_id)
            self.record_frontier_failure(
                "invalid, unavailable, or low-confidence frontier selection"
            )
            self.log_worker_result(
                completed, age, state_before, "rejected_frontier_selection"
            )
            self.refresh_frontier_request()
            return
        self.frontier_decision_failures = 0
        self.api_failures = 0
        self.accepted_results += 1
        self.frontier_selected_id = result.selected_frontier_id
        self.frontier_selected_reason = result.reason
        self.frontier_selected_confidence = result.confidence
        self.frontier_request = None
        self.frontier_request_pending = False
        candidate = available[result.selected_frontier_id]
        pose = self.robot_pose()
        yaw = (
            math.atan2(candidate.y - pose[1], candidate.x - pose[0])
            if pose is not None
            else candidate.bearing
        )
        self.set_state(EXPLORING)
        self.publish_frontier_selection(candidate)
        self.log_worker_result(
            completed, age, state_before, "accepted_frontier"
        )
        self.plan_then_navigate([(candidate.x, candidate.y, yaw)], "frontier")

    def ground_pixel_with_reason(
        self, snapshot: FrameSnapshot, pixel: Pixel, require_ground: bool
    ):
        depth, depth_reason = depth_at_pixel_with_reason(
            snapshot.depth_m,
            pixel.u,
            pixel.v,
            radius=int(self.p.depth_neighborhood_radius),
            min_depth=float(self.p.min_depth),
            max_depth=float(self.p.max_depth),
            min_samples=int(self.p.min_depth_samples),
            max_deviation=float(self.p.max_depth_deviation),
        )
        if depth is None:
            return None, depth_reason
        try:
            point = project_pixel(
                pixel.u,
                pixel.v,
                depth,
                snapshot.intrinsics,
                snapshot.transform_matrix,
            )
        except ValueError as error:
            return None, f"projection_error:{error}"
        if require_ground and abs(float(point[2])) > float(self.p.max_ground_height):
            return (
                None,
                f"ground_height_rejected:z={float(point[2]):.3f},"
                f"limit={float(self.p.max_ground_height):.3f}",
            )
        return tuple(float(value) for value in point), "ok"

    def ground_pixel(self, snapshot: FrameSnapshot, pixel: Pixel, require_ground: bool):
        point, _ = self.ground_pixel_with_reason(snapshot, pixel, require_ground)
        return point

    def confirm_target(self, point):
        reset_count = int(getattr(self.target_tracker, "reset_count", 0))
        center = self.target_tracker.update(point)
        new_reset_count = int(getattr(self.target_tracker, "reset_count", 0))
        if new_reset_count > reset_count:
            self.get_logger().warn(
                "Target confirmation spatial reset: "
                f"jump={self.target_tracker.last_jump_distance:.3f}m exceeds "
                f"radius={self.target_tracker.confirmation_radius:.3f}m; "
                f"progress={self.target_tracker.progress}/"
                f"{self.target_tracker.required_frames}"
            )
        if center is None:
            if self.state in (
                SEARCHING,
                SCANNING,
                FRONTIER_SELECTING,
                EXPLORING,
                TARGET_CONFIRMING,
                TARGET_REOBSERVING,
            ):
                self.frontier_generation += 1
                self.frontier_request = None
                self.frontier_request_pending = False
                self.cancel_motion(publish_stop=True)
                self.set_state(TARGET_CONFIRMING)
            return
        first_confirmation = self.target_reference_position is None
        if first_confirmation or not self.easy_case_enabled():
            if self.easy_case_enabled():
                self.target_reference_position = center
            else:
                self.set_confirmed_target_reference_position(center)
        if first_confirmation:
            self.cancel_motion(publish_stop=True)
            if self.easy_case_enabled():
                self.easy_alignment_complete = False
                self.easy_direct_goal = None
                self.easy_path_deviation = math.inf
                self.set_state(TARGET_ALIGNING)
                self.publish_easy_event("target_confirmed")
            else:
                self.set_state(APPROACHING)
        self.publish_grounded_markers(self.target_reference_position)

    def set_confirmed_target_reference_position(self, target):
        self.target_reference_position = tuple(float(value) for value in target)

    # ---------- navigation ----------

    def robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.p.global_frame,
                self.p.base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as error:
            self.get_logger().warn(
                f"Robot TF unavailable: {error}", throttle_duration_sec=5.0
            )
            return None
        self.last_robot_tf_success = time.monotonic()
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            quaternion_yaw(transform.transform.rotation),
        )

    def begin_scan(self, pose):
        self.cancel_motion(publish_stop=True)
        self.initial_yaw = pose[2]
        self.scan_headings = list(scan_yaws(pose[2], int(self.p.scan_steps)))
        self.scan_index = 0
        self.scan_observations = []
        self.scan_observation_headings = []
        self.scan_settle_until = 0.0
        self.scan_capture_after_sequence = self.sequence
        self.scan_waiting_for_vlm = False
        self.scan_request_sequence = -1
        self.scan_retry_count = 0
        self.depth_reobserve_attempts = 0
        self.target_probe_attempts = 0
        self.last_depth_recovery_action = "none"
        self.frontier_request = None
        self.frontier_request_pending = False
        self.set_state(SCANNING)

    def is_current_scan_result(self, snapshot):
        return (
            self.state == SCANNING
            and bool(getattr(self, "scan_waiting_for_vlm", False))
            and snapshot.sequence == getattr(self, "scan_request_sequence", -1)
        )

    def is_current_depth_reobserve_result(self, snapshot):
        return (
            self.state == TARGET_REOBSERVING
            and bool(getattr(self, "scan_waiting_for_vlm", False))
            and snapshot.sequence == getattr(self, "scan_request_sequence", -1)
        )

    def prepare_stationary_observation(self):
        """Hold position and require a fresh post-motion RGB-D/VLM sample."""
        self.publish_stop()
        self.scan_capture_after_sequence = self.sequence
        self.scan_settle_until = (
            time.monotonic() + max(0.0, float(self.p.scan_settle_time))
        )
        self.scan_waiting_for_vlm = False
        self.scan_request_sequence = -1

    def retry_depth_reobservation(self, reason):
        """Retry the current held view without advancing the scan."""
        self.scan_waiting_for_vlm = False
        self.scan_request_sequence = -1
        self.scan_capture_after_sequence = self.sequence
        self.scan_settle_until = time.monotonic()
        self.scan_retry_count += 1
        limit = max(1, int(self.p.scan_result_retry_limit))
        if self.scan_retry_count >= limit:
            self.fail_safe(
                "Target depth reobservation result rejected repeatedly: "
                f"{reason}"
            )
            return
        self.publish_stop()
        self.get_logger().warn(
            "Retrying target depth reobservation frame "
            f"({self.scan_retry_count}/{limit}): {reason}"
        )

    def target_relative_bearing(self, snapshot, pixel, pose):
        """Convert the VLM target pixel into a robot-relative yaw."""
        fx, fy, cx, cy = [float(value) for value in snapshot.intrinsics]
        ray_camera = np.array(
            [(float(pixel.u) - cx) / fx, (float(pixel.v) - cy) / fy, 1.0],
            dtype=float,
        )
        rotation = np.asarray(snapshot.transform_matrix, dtype=float)[:3, :3]
        ray_map = rotation @ ray_camera
        if math.hypot(float(ray_map[0]), float(ray_map[1])) > 1e-6:
            map_yaw = math.atan2(float(ray_map[1]), float(ray_map[0]))
            return normalize_angle(map_yaw - float(pose[2]))
        # This fallback is useful for synthetic/unit-test transforms. ROS
        # optical-frame +x points right, hence the negative yaw sign.
        return -math.atan2(float(pixel.u) - cx, fx)

    def build_target_probe_candidates(self, snapshot, result, pose):
        """Build observation subgoals along the visually grounded target ray."""
        maximum = min(
            float(self.p.target_probe_distance),
            float(self.p.max_travel_radius),
        )
        minimum = float(self.p.target_probe_min_distance)
        if maximum < minimum:
            return []
        target_bearing = self.target_relative_bearing(
            snapshot, result.target_pixel, pose
        )
        map_bearing = normalize_angle(float(pose[2]) + target_bearing)
        raw_candidates = [
            (
                float(pose[0]) + distance * math.cos(map_bearing),
                float(pose[1]) + distance * math.sin(map_bearing),
            )
            for distance in (maximum, maximum * 0.75, maximum * 0.5)
            if distance >= minimum
        ]

        candidates = []
        seen = set()
        for point in raw_candidates:
            key = self.goal_key(point)
            if key in seen:
                continue
            seen.add(key)
            candidates.append((point[0], point[1], map_bearing))
        return candidates

    def clear_target_depth_recovery(self):
        self.depth_reobserve_attempts = 0
        self.target_probe_attempts = 0
        self.last_depth_recovery_action = "none"
        self.scan_retry_count = 0

    def start_target_probe(self, snapshot, result, reason):
        """Approach one Nav2-validated point, then observe the target again."""
        if self.target_reference_position is not None:
            return False
        limit = max(1, int(self.p.target_probe_attempt_limit))
        if self.target_probe_attempts >= limit:
            self.fail_safe(
                "Target remains outside reliable depth range after "
                f"{limit} intermediate approaches: {reason}"
            )
            return True
        pose = self.robot_pose()
        if pose is None:
            return False
        candidates = self.build_target_probe_candidates(
            snapshot, result, pose
        )
        if not candidates:
            self.fail_safe(
                "Target depth is unavailable and no observation subgoal "
                f"exists along the target-pixel direction: {reason}"
            )
            return True
        self.cancel_motion(publish_stop=True)
        self.task_epoch += 1
        self.target_tracker.reset()
        self.target_reference_position = None
        self.scan_waiting_for_vlm = False
        self.scan_request_sequence = -1
        self.scan_settle_until = 0.0
        self.depth_reobserve_attempts = 0
        self.target_probe_attempts += 1
        self.last_depth_recovery_action = (
            f"target_probe:{self.target_probe_attempts}/{limit}"
        )
        self.set_state(TARGET_REOBSERVING)
        self.get_logger().warn(
            "Target is outside reliable depth range; requesting Nav2 validation "
            "for an observation subgoal along the target-pixel direction "
            f"({self.target_probe_attempts}/{limit})"
        )
        self.plan_then_navigate(candidates, "target_probe")
        return True

    def recover_target_depth(self, snapshot, result, reason):
        """Select rotation or staged approach for an ungrounded VLM target."""
        if self.target_reference_position is not None:
            return False
        recoverable_prefixes = (
            "insufficient_valid_depth_samples:",
            "mixed_depth_samples:",
            "depth_out_of_range:",
        )
        if not str(reason).startswith(recoverable_prefixes):
            return False
        pose = self.robot_pose()
        if pose is None:
            return False
        rotate_limit = max(1, int(self.p.depth_reobserve_attempt_limit))
        if (
            str(reason).startswith("depth_out_of_range:")
            or self.depth_reobserve_attempts >= rotate_limit
        ):
            return self.start_target_probe(snapshot, result, reason)

        relative = self.target_relative_bearing(
            snapshot, result.target_pixel, pose
        )
        max_angle = math.radians(
            max(1.0, float(self.p.depth_reobserve_angle_deg))
        )
        relative = max(-max_angle, min(max_angle, relative))
        if abs(relative) < math.radians(2.0):
            relative = (
                max_angle
                if self.depth_reobserve_attempts % 2 == 0
                else -max_angle
            )

        self.cancel_motion(publish_stop=True)
        self.task_epoch += 1
        self.scan_waiting_for_vlm = False
        self.scan_request_sequence = -1
        self.scan_settle_until = 0.0
        self.depth_reobserve_attempts += 1
        self.last_depth_recovery_action = (
            f"rotate:{math.degrees(relative):.1f}deg;"
            f"attempt={self.depth_reobserve_attempts}/{rotate_limit}"
        )
        self.set_state(TARGET_REOBSERVING)
        self.get_logger().warn(
            "Target depth is unreliable; rotating toward the VLM target "
            f"pixel by {math.degrees(relative):.1f} degrees "
            f"({self.depth_reobserve_attempts}/{rotate_limit})"
        )
        self.send_spin(relative, kind="depth_reobserve")
        return True

    def retry_stationary_scan_view(self, reason):
        """Keep the robot stopped and request a newer frame at this heading."""
        self.scan_waiting_for_vlm = False
        self.scan_request_sequence = -1
        self.scan_capture_after_sequence = self.sequence
        self.scan_settle_until = time.monotonic()
        self.scan_retry_count += 1
        limit = max(1, int(self.p.scan_result_retry_limit))
        if self.scan_retry_count >= limit:
            self.fail_safe(
                "Stationary scan result rejected repeatedly at heading "
                f"{self.scan_index + 1}/{len(self.scan_headings)}: {reason}"
            )
            return
        self.publish_stop()
        self.get_logger().warn(
            "Retrying stationary scan frame at heading "
            f"{self.scan_index + 1}/{len(self.scan_headings)} "
            f"({self.scan_retry_count}/{limit}): {reason}"
        )

    def advance_stationary_scan_view(self, snapshot):
        """Record a completed no-target view, then allow the next rotation."""
        if self.scan_index < len(self.scan_headings):
            self.scan_observations.append(snapshot.rgb.copy())
            self.scan_observation_headings.append(
                self.scan_headings[self.scan_index]
            )
        self.scan_index += 1
        self.scan_waiting_for_vlm = False
        self.scan_request_sequence = -1
        self.scan_settle_until = 0.0
        self.scan_capture_after_sequence = self.sequence
        self.scan_retry_count = 0

    def prepare_frontier_cycle(self, pose, use_scan_context):
        cell = world_to_grid(
            pose[0],
            pose[1],
            self.map_message.info.origin.position.x,
            self.map_message.info.origin.position.y,
            self.map_message.info.resolution,
        )
        _, clusters = select_frontier(
            self.grid, cell, int(self.p.min_frontier_cells)
        )
        ordered = sorted(
            clusters,
            key=lambda cluster: (
                len(cluster["cells"]) - 0.35 * cluster["distance_cells"]
            ),
            reverse=True,
        )
        candidates = []
        for cluster in ordered:
            x, y = grid_to_world(
                cluster["goal"][0],
                cluster["goal"][1],
                self.map_message.info.origin.position.x,
                self.map_message.info.origin.position.y,
                self.map_message.info.resolution,
            )
            candidate_id = len(candidates) + 1
            candidates.append(
                FrontierCandidate(
                    candidate_id=candidate_id,
                    x=float(x),
                    y=float(y),
                    bearing=math.atan2(y - pose[1], x - pose[0]),
                    distance=math.hypot(x - pose[0], y - pose[1]),
                    cell_count=len(cluster["cells"]),
                )
            )
            if len(candidates) >= int(self.p.max_frontier_candidates):
                break
        if not candidates:
            self.fail_safe("No reachable frontier remains")
            return
        if (
            use_scan_context
            and self.scan_observations
            and len(self.scan_observations) == len(self.scan_observation_headings)
        ):
            self.frontier_scene_rgb = scan_montage(
                self.scan_observations, self.scan_observation_headings
            )
            self.frontier_context = "eight_view_scan_montage"
            self.publish_rgb_image(
                self.scan_montage_pub, self.frontier_scene_rgb, self.p.base_frame
            )
        elif self.latest_snapshot is not None:
            self.frontier_scene_rgb = self.latest_snapshot.rgb.copy()
            self.frontier_context = "current_rgb"
        else:
            return
        self.frontier_generation += 1
        self.frontier_candidates = candidates
        self.frontier_rejected_ids = set()
        self.frontier_selected_id = None
        self.frontier_selected_reason = ""
        self.frontier_selected_confidence = 0.0
        self.set_state(FRONTIER_SELECTING)
        self.refresh_frontier_request(pose)

    def refresh_frontier_request(self, pose=None):
        available = [
            item
            for item in self.frontier_candidates
            if item.candidate_id not in self.frontier_rejected_ids
        ]
        if not available:
            self.fail_safe("VLM frontier candidates exhausted")
            return
        if pose is None:
            pose = self.robot_pose()
        if pose is None or self.latest_snapshot is None or self.frontier_scene_rgb is None:
            return
        map_image, robot_pixel, candidate_pixels = render_frontier_map(
            self.grid,
            (
                self.map_message.info.origin.position.x,
                self.map_message.info.origin.position.y,
            ),
            float(self.map_message.info.resolution),
            pose,
            pose[:2],
            float(self.p.max_travel_radius),
            available,
            return_annotations=True,
        )
        self.publish_rgb_image(
            self.frontier_map_pub, map_image, self.p.global_frame
        )
        self.publish_frontiers(available)
        self.frontier_request = replace(
            self.latest_snapshot,
            rgb=self.frontier_scene_rgb.copy(),
            request_kind="frontier",
            auxiliary_rgb=map_image,
            frontier_generation=self.frontier_generation,
            frontier_candidates=tuple(available),
            frontier_context=self.frontier_context,
            captured_monotonic=time.monotonic(),
            frontier_robot_pixel=robot_pixel,
            frontier_candidate_pixels=candidate_pixels,
        )
        self.frontier_request_pending = True

    def publish_rgb_image(self, publisher, image, frame_id):
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(frame_id)
        message.height, message.width = image.shape[:2]
        message.encoding = "rgb8"
        message.is_bigendian = 0
        message.step = message.width * 3
        message.data = np.ascontiguousarray(image).tobytes()
        publisher.publish(message)

    def navigation_tick(self):
        if (
            not self.get_parameter("enabled").value
            or self.state in (
                DISARMED,
                API_ERROR,
                SUCCEEDED,
                FAILED,
                SENSOR_WAITING,
                TARGET_CONFIRMING,
                FRONTIER_SELECTING,
            )
            or self.grid is None
            or self.map_message is None
        ):
            return
        pose = self.robot_pose()
        if pose is None:
            return
        if self.state == SCANNING and not self.ensure_initial_costmap_clear_done():
            return
        if self.session_origin is None:
            self.session_origin = pose[:2]
            self.publish_task_boundary()
            self.begin_scan(pose)
        if self.goal_handle is not None or self.goal_pending or self.plan_pending:
            return

        if self.target_reference_position is not None and self.state == TARGET_ALIGNING:
            dx = self.target_reference_position[0] - pose[0]
            dy = self.target_reference_position[1] - pose[1]
            self.easy_target_distance = math.hypot(dx, dy)
            target_yaw = math.atan2(dy, dx)
            self.easy_alignment_error = normalize_angle(target_yaw - pose[2])
            self.publish_easy_case_markers(pose)
            if abs(self.easy_alignment_error) <= float(
                self.p.easy_case_alignment_tolerance
            ):
                self.easy_alignment_complete = True
                self.publish_stop()
                self.set_state(APPROACHING)
                self.publish_easy_event(
                    "target_alignment_complete",
                    alignment_error_rad=self.easy_alignment_error,
                )
            else:
                self.publish_easy_event(
                    "target_alignment_requested",
                    alignment_error_rad=self.easy_alignment_error,
                )
                self.send_spin(self.easy_alignment_error, kind="target_align")
            return

        if self.target_reference_position is not None and self.state == APPROACHING:
            target_distance = math.hypot(
                pose[0] - self.target_reference_position[0],
                pose[1] - self.target_reference_position[1],
            )
            self.easy_target_distance = target_distance
            if target_distance <= self.approach_stop_radius():
                self.begin_approach_stop("distance_contract_reached")
                return
            goal = (
                float(self.target_reference_position[0]),
                float(self.target_reference_position[1]),
                float(pose[2]),
            )
            if self.easy_case_enabled():
                self.easy_direct_goal = goal[:3]
                self.publish_easy_case_markers(pose)
                self.publish_easy_event(
                    "direct_goal_requested",
                    target_distance_m=target_distance,
                    direct_travel_m=target_distance,
                )
                self.plan_then_navigate([goal[:3]], "easy_approach")
                return
            self.plan_then_navigate([goal[:3]], "approach")
            return

        if self.state == TARGET_REOBSERVING:
            self.publish_stop()
            return

        if self.state == SCANNING and (
            self.scan_waiting_for_vlm or self.scan_settle_until > 0.0
        ):
            self.publish_stop()
            return
        if self.state == SCANNING and self.scan_index < len(self.scan_headings):
            relative = normalize_angle(self.scan_headings[self.scan_index] - pose[2])
            self.send_spin(relative)
            return
        if self.state == SCANNING and self.easy_case_enabled():
            self.fail_safe("Easy-case full scan completed without finding target")
            return
        if self.state == SCANNING and not bool(self.p.allow_frontier_after_scan):
            self.fail_safe("Full scan completed without finding target")
            return
        if self.state == SCANNING:
            self.prepare_frontier_cycle(pose, use_scan_context=True)
            return
        if self.state == EXPLORING:
            self.prepare_frontier_cycle(pose, use_scan_context=False)

    def target_reference_distance(self, pose):
        if self.target_reference_position is None or pose is None:
            return math.inf
        return math.hypot(
            float(pose[0]) - float(self.target_reference_position[0]),
            float(pose[1]) - float(self.target_reference_position[1]),
        )

    def approach_stop_radius(self):
        if int(getattr(self, "arrival_retry_count", 0)) > 0:
            return float(self.p.target_success_radius)
        return float(self.p.approach_cancel_radius)

    def reset_approach_stop(self, reset_retry=False):
        self.arrival_stop_started = 0.0
        self.arrival_cancel_requested_at = 0.0
        self.arrival_cancel_acknowledged = False
        self.arrival_action_terminal = False
        self.arrival_action_terminal_at = 0.0
        self.arrival_stationary_count = 0
        if reset_retry:
            self.arrival_retry_count = 0

    def request_approach_cancel(self):
        if self.goal_handle is None or self.arrival_cancel_requested_at > 0.0:
            return
        self.arrival_cancel_requested_at = time.monotonic()
        try:
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.on_approach_cancel_response)
        except Exception as error:
            self.fail_safe(f"Cannot request approach cancellation: {error}")

    def on_approach_cancel_response(self, future):
        try:
            future.result()
        except Exception as error:
            if self.state == APPROACH_STOPPING:
                self.fail_safe(f"Cannot confirm approach cancellation request: {error}")
            return
        self.arrival_cancel_acknowledged = True

    def begin_approach_stop(self, reason, action_terminal=False):
        if self.state == APPROACH_STOPPING:
            if action_terminal and not self.arrival_action_terminal:
                self.arrival_action_terminal = True
                self.arrival_action_terminal_at = time.monotonic()
            return
        self.set_state(APPROACH_STOPPING)
        self.arrival_stop_started = time.monotonic()
        self.arrival_cancel_requested_at = 0.0
        self.arrival_cancel_acknowledged = False
        self.arrival_action_terminal = bool(action_terminal)
        self.arrival_action_terminal_at = (
            self.arrival_stop_started if action_terminal else 0.0
        )
        self.arrival_stationary_count = 0
        self.publish_stop()
        if self.plan_handle is not None or self.plan_pending:
            self.plan_token += 1
            if self.plan_handle is not None:
                self.plan_handle.cancel_goal_async()
            self.plan_handle = None
            self.plan_pending = False
            self.plan_candidates = []
            self.plan_kind = None
        if self.goal_handle is not None:
            self.request_approach_cancel()
        elif not self.goal_pending:
            self.arrival_action_terminal = True
            self.arrival_action_terminal_at = time.monotonic()
        self.get_logger().info(f"Approach stop confirmation started: {reason}")

    def advance_approach_stop(self, now=None, pose=None):
        if self.state != APPROACH_STOPPING:
            return
        now = time.monotonic() if now is None else float(now)
        self.publish_stop()
        if not self.arrival_action_terminal:
            cancel_started = float(self.arrival_cancel_requested_at)
            terminal_wait_started = (
                cancel_started
                if cancel_started > 0.0
                else float(self.arrival_stop_started)
            )
            if now - terminal_wait_started > float(self.p.arrival_cancel_timeout):
                self.fail_safe("Approach action cancellation did not become terminal")
            return
        terminal_at = float(getattr(self, "arrival_action_terminal_at", 0.0))
        stop_elapsed = now - (
            terminal_at if terminal_at > 0.0 else float(self.arrival_stop_started)
        )
        odom_age = self.elapsed_since(self.arrival_last_odom_received, now)
        stationary = self.arrival_stationary_count >= int(
            self.p.arrival_stationary_samples
        )
        if stationary and odom_age <= float(self.p.arrival_odom_max_age):
            pose = self.robot_pose() if pose is None else pose
            distance = self.target_reference_distance(pose)
            self.easy_target_distance = distance
            if distance <= float(self.p.target_success_radius):
                self.finish_approach_success()
                return
            if self.arrival_retry_count < int(self.p.arrival_retry_limit):
                self.arrival_retry_count += 1
                self.get_logger().warn(
                    "Approach action stopped outside distance contract; "
                    "replanning target reference position"
                )
                self.reset_approach_stop(reset_retry=False)
                self.set_state(APPROACHING)
                return
            self.fail_safe(
                "Approach stopped outside target distance contract after retry"
            )
            return
        if stop_elapsed > float(self.p.arrival_stop_timeout):
            self.fail_safe(
                "Approach motion did not stop with fresh odometry before timeout"
            )

    def finish_approach_success(self):
        self.publish_stop()
        self.goal_handle = None
        self.goal_pending = False
        self.goal_kind = None
        self.goal_pose = None
        self.active_motion_origin = None
        self.reset_approach_stop(reset_retry=False)
        self.set_state(SUCCEEDED)

    @staticmethod
    def goal_key(pose):
        return round(float(pose[0]), 1), round(float(pose[1]), 1)

    def plan_then_navigate(self, candidates, kind):
        now = time.monotonic()
        self.blocked_goals = {
            key: expiry for key, expiry in self.blocked_goals.items() if expiry > now
        }
        self.plan_candidates = [
            tuple(candidate)
            for candidate in candidates
            if self.blocked_goals.get(self.goal_key(candidate), 0.0) <= now
        ]
        if not self.plan_candidates:
            if kind in (
                "approach",
                "easy_approach",
                "target_probe",
            ):
                self.fail_safe(f"No unblocked {kind} candidate")
            elif kind == "frontier":
                self.reject_selected_frontier("Selected frontier is temporarily blocked")
            return
        if not self.path_planner.wait_for_server(timeout_sec=0.5):
            if kind == "frontier":
                self.reject_selected_frontier(
                    "ComputePathToPose action server unavailable"
                )
            else:
                self.fail_safe("ComputePathToPose action server unavailable")
            return
        self.plan_token += 1
        self.plan_pending = True
        self.plan_kind = kind
        self.plan_attempt_count = 0
        self.last_plan_kind = str(kind)
        self.last_plan_status = -1
        self.last_plan_pose_count = 0
        self.last_plan_path_length = 0.0
        self.last_plan_planning_time = 0.0
        self.last_plan_endpoint_radius = math.inf
        self.last_plan_max_radius = math.inf
        self.last_plan_candidate = None
        self.last_plan_rejection_reason = "none"
        self.plan_window_origin = None
        self.last_plan_commit_length = 0.0
        self.last_plan_radius_clipped = False
        self.rolling_goal_is_final = True
        self.goal_started = now
        self.request_next_plan(self.plan_token)

    def request_next_plan(self, token):
        if token != self.plan_token:
            return
        if not self.plan_candidates:
            kind = self.plan_kind
            self.plan_pending = False
            self.plan_handle = None
            self.plan_kind = None
            if kind == "frontier":
                self.reject_selected_frontier(
                    "Nav2 cannot plan to selected frontier"
                )
            else:
                self.fail_safe(
                    f"Nav2 cannot plan to any {kind} candidate; "
                    f"last rejection: {self.last_plan_rejection_reason}"
                )
            return
        self.current_plan_pose = self.plan_candidates.pop(0)
        self.plan_attempt_count += 1
        self.last_plan_candidate = tuple(self.current_plan_pose)
        goal = ComputePathToPose.Goal()
        goal.goal = self.make_pose_stamped(*self.current_plan_pose)
        goal.use_start = False
        future = self.path_planner.send_goal_async(goal)
        future.add_done_callback(lambda done: self.on_plan_response(done, token))

    def on_plan_response(self, future, token):
        if token != self.plan_token:
            return
        try:
            handle = future.result()
        except Exception as error:
            self.reject_current_plan(f"Path request failed: {error}", token)
            return
        if not handle.accepted:
            self.reject_current_plan("Path request rejected", token)
            return
        self.plan_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(lambda done: self.on_plan_result(done, token))

    def on_plan_result(self, future, token):
        if token != self.plan_token:
            return
        try:
            wrapped = future.result()
            path = wrapped.result.path.poses
        except Exception as error:
            self.last_plan_status = -1
            self.last_plan_pose_count = 0
            self.last_plan_rejection_reason = (
                f"path_result_exception:{type(error).__name__}"
            )
            self.reject_current_plan(f"Path result failed: {error}", token)
            return
        self.last_plan_status = int(wrapped.status)
        self.last_plan_pose_count = len(path)
        self.last_plan_path_length = sum(
            math.hypot(
                second.pose.position.x - first.pose.position.x,
                second.pose.position.y - first.pose.position.y,
            )
            for first, second in zip(path, path[1:])
        )
        planning_time = getattr(wrapped.result, "planning_time", None)
        self.last_plan_planning_time = (
            float(getattr(planning_time, "sec", 0))
            + float(getattr(planning_time, "nanosec", 0)) / 1e9
        )
        points = [
            (item.pose.position.x, item.pose.position.y) for item in path
        ]
        self.plan_window_origin = points[0] if points else None
        if self.plan_window_origin is None:
            radii = []
            self.last_plan_endpoint_radius = math.inf
        else:
            radii = [
                math.hypot(
                    point[0] - self.plan_window_origin[0],
                    point[1] - self.plan_window_origin[1],
                )
                for point in points
            ]
            candidate = self.current_plan_pose
            self.last_plan_endpoint_radius = math.hypot(
                candidate[0] - self.plan_window_origin[0],
                candidate[1] - self.plan_window_origin[1],
            )
        self.last_plan_max_radius = max(radii) if radii else math.inf
        limit = float(self.p.max_travel_radius)
        kind = self.plan_kind
        rejection = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            rejection = f"planner_action_status_{int(wrapped.status)}"
        elif not path:
            rejection = "empty_path"
        elif kind != "approach" and len(path) < 2:
            rejection = "path_too_short"
        if rejection is None:
            self.last_plan_rejection_reason = "none"
            requested_pose = self.current_plan_pose
            if kind == "approach":
                horizon = max(
                    1, int(getattr(self.p, "path_horizon_segments", 16))
                )
                samples = (
                    sample_polyline(points, horizon)[0]
                    if len(points) >= 2
                    else points
                )
                self.last_plan_commit_length = self.last_plan_path_length
                self.last_plan_radius_clipped = False
                self.rolling_goal_is_final = True
                self.active_motion_origin = self.plan_window_origin
                if getattr(self, "marker_pub", None) is not None:
                    self.publish_task_boundary()
                    self.publish_rolling_path(samples, requested_pose, points)
                self.plan_pending = False
                self.plan_handle = None
                self.plan_candidates = []
                self.plan_kind = None
                self.send_navigation_goal(*requested_pose, kind)
                return
            if kind == "easy_approach":
                self.easy_path_deviation = max_polyline_deviation(
                    points, self.plan_window_origin, requested_pose[:2]
                )
                if self.last_plan_max_radius > limit + 1e-6:
                    self.last_plan_rejection_reason = (
                        "easy_path_outside_activity_radius"
                    )
                    self.reject_current_plan(
                        "Easy-case path exceeds the activity radius", token
                    )
                    return
                if self.easy_path_deviation > float(
                    self.p.easy_case_max_path_deviation
                ):
                    self.last_plan_rejection_reason = (
                        "easy_path_not_straight"
                    )
                    self.reject_current_plan(
                        "Easy-case path is not sufficiently straight: "
                        f"lateral deviation={self.easy_path_deviation:.3f}m",
                        token,
                    )
                    return
                samples, total = sample_polyline(
                    points,
                    max(1, int(self.p.path_horizon_segments)),
                )
                self.last_plan_commit_length = total
                self.last_plan_radius_clipped = False
                self.rolling_goal_is_final = True
                self.active_motion_origin = self.plan_window_origin
                self.plan_pending = False
                self.plan_handle = None
                self.plan_candidates = []
                self.plan_kind = None
                if getattr(self, "marker_pub", None) is not None:
                    self.publish_task_boundary()
                    self.publish_rolling_path(
                        samples, requested_pose, points
                    )
                    pose_now = self.robot_pose()
                    if pose_now is not None:
                        self.publish_easy_case_markers(pose_now)
                self.publish_easy_event(
                    "direct_path_accepted",
                    path_length_m=total,
                    lateral_deviation_m=self.easy_path_deviation,
                )
                self.send_navigation_goal(*requested_pose, kind)
                return
            horizon = max(
                1, int(getattr(self.p, "path_horizon_segments", 16))
            )
            samples, total = sample_polyline(points, horizon)
            _radius_prefix, radius_clipped, radius_length = (
                clip_polyline_to_radius(
                    points, self.plan_window_origin, limit
                )
            )
            requested_commit_length = total
            if kind == "frontier":
                self.frontier_path_samples = samples
                self.frontier_path_length = total
                if (
                    total > float(self.p.frontier_full_commit_distance)
                    and len(samples) >= 2
                ):
                    index = min(
                        max(1, int(self.p.path_execute_segments)),
                        len(samples) - 1,
                    )
                    requested_commit_length = total * (
                        float(index) / float(len(samples) - 1)
                    )
            commit_length = min(
                requested_commit_length,
                radius_length if radius_clipped else total,
            )
            committed_points, committed_length = clip_polyline_to_length(
                points, commit_length
            )
            if len(committed_points) < 2:
                self.last_plan_rejection_reason = "rolling_prefix_too_short"
                self.reject_current_plan(
                    "Plan rejected: rolling path prefix is too short",
                    token,
                )
                return
            execute = committed_points[-1]
            previous = committed_points[-2]
            yaw = math.atan2(
                execute[1] - previous[1], execute[0] - previous[0]
            )
            pose = (execute[0], execute[1], yaw)
            self.last_plan_commit_length = committed_length
            self.last_plan_radius_clipped = radius_clipped
            self.rolling_goal_is_final = (
                committed_length >= total - 1e-3
            )
            if kind == "frontier":
                self.frontier_goal_is_final = self.rolling_goal_is_final
            self.active_motion_origin = self.plan_window_origin
            if getattr(self, "marker_pub", None) is not None:
                self.publish_task_boundary()
                self.publish_rolling_path(
                    samples, pose, committed_points
                )
            self.plan_pending = False
            self.plan_handle = None
            self.plan_candidates = []
            self.plan_kind = None
            self.send_navigation_goal(*pose, kind)
            return
        self.last_plan_rejection_reason = rejection
        candidate = self.current_plan_pose
        self.reject_current_plan(
            "Plan rejected: "
            f"reason={rejection}, kind={self.plan_kind}, "
            f"attempt={self.plan_attempt_count}, "
            f"status={self.last_plan_status}, "
            f"poses={self.last_plan_pose_count}, "
            f"path_length={self.last_plan_path_length:.3f}m, "
            f"planning_time={self.last_plan_planning_time:.3f}s, "
            f"candidate=({candidate[0]:.3f},{candidate[1]:.3f}), "
            f"endpoint_radius={self.last_plan_endpoint_radius:.3f}m, "
            f"max_path_radius={self.last_plan_max_radius:.3f}m, "
            f"limit={limit:.3f}m",
            token,
        )

    def reject_current_plan(self, reason, token):
        if self.last_plan_rejection_reason == "none":
            self.last_plan_rejection_reason = str(reason)
        self.get_logger().warn(reason)
        if self.current_plan_pose is not None:
            self.blocked_goals[self.goal_key(self.current_plan_pose)] = (
                time.monotonic() + float(self.p.blocked_goal_seconds)
            )
        self.plan_handle = None
        self.request_next_plan(token)

    def make_pose_stamped(self, x, y, yaw):
        message = PoseStamped()
        message.header.frame_id = self.p.global_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = float(x)
        message.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_quaternion(yaw)
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        return message

    def send_navigation_goal(self, x, y, yaw, kind):
        if not self.navigator.wait_for_server(timeout_sec=0.5):
            if kind == "frontier":
                self.reject_selected_frontier(
                    "NavigateToPose action server unavailable"
                )
            else:
                self.fail_safe("NavigateToPose action server unavailable")
            return
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose_stamped(x, y, yaw)
        self.goal_token += 1
        token = self.goal_token
        self.goal_pending = True
        self.goal_kind = kind
        self.goal_pose = (float(x), float(y), float(yaw))
        self.goal_started = time.monotonic()
        future = self.navigator.send_goal_async(goal)
        future.add_done_callback(lambda done: self.on_goal_response(done, token, kind))
        self.publish_goal_marker(x, y, yaw, kind)

    def send_spin(self, relative_yaw, kind="scan"):
        if not self.spinner.wait_for_server(timeout_sec=0.5):
            self.fail_safe("Spin action server unavailable")
            return
        goal = Spin.Goal()
        goal.target_yaw = float(relative_yaw)
        allowance = max(0.0, float(self.p.spin_time_allowance))
        goal.time_allowance.sec = int(allowance)
        goal.time_allowance.nanosec = int((allowance - int(allowance)) * 1e9)
        self.goal_token += 1
        token = self.goal_token
        self.goal_pending = True
        self.goal_kind = str(kind)
        self.goal_pose = None
        self.goal_started = time.monotonic()
        future = self.spinner.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self.on_goal_response(done, token, kind)
        )

    def on_goal_response(self, future, token, kind):
        try:
            handle = future.result()
        except Exception as error:
            if token == self.goal_token:
                self.goal_pending = False
                if kind == "frontier":
                    self.reject_selected_frontier(
                        f"frontier goal request failed: {error}"
                    )
                else:
                    self.fail_safe(f"{kind} goal request failed: {error}")
            return
        if token != self.goal_token:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        self.goal_pending = False
        if not handle.accepted:
            if kind == "frontier":
                self.reject_selected_frontier("frontier goal rejected")
            else:
                self.fail_safe(f"{kind} goal rejected")
            return
        self.goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(
            lambda done: self.on_navigation_result(done, token, kind)
        )
        if kind in ("approach", "easy_approach") and self.state == APPROACH_STOPPING:
            self.request_approach_cancel()

    def on_navigation_result(self, future, token, kind):
        if token != self.goal_token:
            return
        try:
            status = future.result().status
        except Exception as error:
            if kind == "frontier":
                self.reject_selected_frontier(
                    f"Cannot obtain frontier goal result: {error}"
                )
            else:
                self.fail_safe(f"Cannot obtain {kind} goal result: {error}")
            return
        failed_pose = self.goal_pose
        self.goal_handle = None
        self.goal_pending = False
        self.goal_kind = None
        self.goal_pose = None
        self.active_motion_origin = None
        if kind in ("approach", "easy_approach"):
            if self.state == APPROACH_STOPPING:
                if status in (
                    GoalStatus.STATUS_CANCELED,
                    GoalStatus.STATUS_SUCCEEDED,
                ):
                    self.arrival_action_terminal = True
                    self.arrival_action_terminal_at = time.monotonic()
                else:
                    self.fail_safe(
                        f"{kind} goal failed while stopping with status {status}"
                    )
                return
            if status == GoalStatus.STATUS_SUCCEEDED:
                if kind == "easy_approach":
                    self.publish_easy_event(
                        "direct_navigation_action_succeeded"
                    )
                self.begin_approach_stop(
                    "nav2_action_succeeded", action_terminal=True
                )
                return
        if status != GoalStatus.STATUS_SUCCEEDED:
            if failed_pose is not None:
                self.blocked_goals[self.goal_key(failed_pose)] = (
                    time.monotonic() + float(self.p.blocked_goal_seconds)
                )
            if kind == "frontier":
                self.reject_selected_frontier(
                    f"frontier goal failed with status {status}"
                )
            elif kind in (
                "approach",
                "scan",
                "depth_reobserve",
                "target_probe",
                "target_align",
                "easy_approach",
            ):
                self.fail_safe(f"{kind} goal failed with status {status}")
            return
        if kind == "scan":
            # Do not rotate to the next heading yet.  Wait for the chassis to
            # settle, capture a frame newer than this completed Spin, and hold
            # position until that exact frame's VLM result has been handled.
            self.prepare_stationary_observation()
        elif kind == "depth_reobserve":
            self.scan_retry_count = 0
            self.prepare_stationary_observation()
        elif kind == "target_probe":
            self.depth_reobserve_attempts = 0
            self.scan_retry_count = 0
            self.last_depth_recovery_action = (
                f"observe_after_probe:{self.target_probe_attempts}"
            )
            self.set_state(TARGET_REOBSERVING)
            self.prepare_stationary_observation()
        elif kind == "target_align":
            self.easy_alignment_complete = True
            self.easy_alignment_error = 0.0
            self.publish_stop()
            self.set_state(APPROACHING)
            self.publish_easy_event("target_alignment_action_succeeded")
        elif kind == "frontier":
            pose = self.robot_pose()
            if pose is None:
                return
            if self.frontier_goal_is_final and bool(self.p.rescan_at_frontier):
                self.begin_scan(pose)
            else:
                self.frontier_generation += 1
                self.frontier_candidates = []
                self.frontier_rejected_ids = set()
                self.frontier_request = None
                self.frontier_request_pending = False
                self.set_state(EXPLORING)

    def reject_selected_frontier(self, reason):
        self.get_logger().warn(reason)
        if self.frontier_selected_id is not None:
            self.frontier_rejected_ids.add(self.frontier_selected_id)
        self.frontier_selected_id = None
        self.frontier_request = None
        self.frontier_request_pending = False
        self.set_state(FRONTIER_SELECTING)
        self.refresh_frontier_request()

    def cancel_motion(self, publish_stop=False):
        self.goal_token += 1
        self.plan_token += 1
        if self.plan_handle is not None:
            self.plan_handle.cancel_goal_async()
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.plan_handle = None
        self.plan_pending = False
        self.plan_candidates = []
        self.plan_kind = None
        self.goal_handle = None
        self.goal_pending = False
        self.goal_kind = None
        self.goal_pose = None
        self.active_motion_origin = None
        self.rolling_goal_is_final = True
        self.reset_approach_stop(reset_retry=True)
        if publish_stop:
            self.publish_stop()

    # ---------- visualization and diagnostics ----------

    def publish_debug(self, snapshot, result):
        image = np.ascontiguousarray(snapshot.rgb.copy())
        ArmImageRecorder.draw_target_annotation(image, result)
        message = Image()
        message.header.stamp = snapshot.stamp
        message.header.frame_id = snapshot.frame_id
        message.height, message.width = image.shape[:2]
        message.encoding = "rgb8"
        message.is_bigendian = 0
        message.step = message.width * 3
        message.data = np.ascontiguousarray(image).tobytes()
        self.debug_pub.publish(message)

    def marker(self, marker_id, namespace, marker_type, x, y, z=0.05):
        marker = Marker()
        marker.header.frame_id = self.p.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.20
        marker.color.a = 0.95
        return marker

    def delete_marker(self, marker_id, namespace):
        marker = Marker()
        marker.header.frame_id = self.p.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def clear_task_boundary(self):
        self.marker_pub.publish(
            MarkerArray(
                markers=[
                    self.delete_marker(0, "task_origin"),
                    self.delete_marker(0, "task_radius"),
                    self.delete_marker(0, "task_radius_label"),
                    self.delete_marker(0, "easy_direct_line"),
                    self.delete_marker(0, "easy_stage"),
                ]
            )
        )

    def publish_task_boundary(self):
        center = self.active_motion_origin or self.session_origin
        if center is None:
            self.clear_task_boundary()
            return
        origin_x, origin_y = center
        radius = float(self.p.max_travel_radius)
        markers = MarkerArray()
        boundary = self.marker(
            0, "task_radius", Marker.LINE_STRIP, 0.0, 0.0, 0.045
        )
        boundary.scale.x = 0.055
        boundary.color.r = 0.7
        boundary.color.g = 0.2
        boundary.color.b = 1.0
        boundary.color.a = 0.9
        boundary.points = [
            Point(
                x=origin_x + radius * math.cos(2.0 * math.pi * index / 72),
                y=origin_y + radius * math.sin(2.0 * math.pi * index / 72),
                z=0.045,
            )
            for index in range(73)
        ]
        markers.markers.append(boundary)

        origin = self.marker(
            0, "task_origin", Marker.CYLINDER, origin_x, origin_y, 0.07
        )
        origin.scale.x = origin.scale.y = 0.22
        origin.scale.z = 0.06
        origin.color.r = 0.7
        origin.color.g = 0.2
        origin.color.b = 1.0
        markers.markers.append(origin)

        label = self.marker(
            0,
            "task_radius_label",
            Marker.TEXT_VIEW_FACING,
            origin_x,
            origin_y,
            0.32,
        )
        label.scale.x = label.scale.y = 0.0
        label.scale.z = 0.18
        label.color.r = 0.8
        label.color.g = 0.4
        label.color.b = 1.0
        label.text = (
            f"ROLLING PATH WINDOW | radius={radius:.2f} m"
            if self.active_motion_origin is not None
            else f"TASK START | rolling radius={radius:.2f} m"
        )
        markers.markers.append(label)
        self.marker_pub.publish(markers)

    def vlm_grounding_deletes(self):
        return [
            self.delete_marker(0, "vlm_target"),
            self.delete_marker(0, "vlm_target_label"),
        ]

    def clear_vlm_grounding_markers(self):
        self.marker_pub.publish(
            MarkerArray(markers=self.vlm_grounding_deletes())
        )

    def publish_grounded_markers(self, target):
        markers = MarkerArray(markers=self.vlm_grounding_deletes())
        target_marker = self.marker(0, "vlm_target", Marker.SPHERE, target[0], target[1], target[2])
        target_marker.scale.x = 0.42
        target_marker.scale.y = 0.42
        target_marker.scale.z = 0.42
        target_marker.color.r = 1.0
        target_marker.color.g = 0.05
        target_marker.color.b = 0.05
        markers.markers.append(target_marker)

        target_label = self.marker(
            0,
            "vlm_target_label",
            Marker.TEXT_VIEW_FACING,
            target[0],
            target[1],
            target[2] + 0.38,
        )
        target_label.scale.x = 0.0
        target_label.scale.y = 0.0
        target_label.scale.z = 0.24
        target_label.color.r = 1.0
        target_label.color.g = 0.3
        target_label.color.b = 0.2
        target_label.text = (
            f"TARGET: {self.p.target_description}\n"
            f"confidence={self.last_vlm_confidence:.2f}"
        )
        markers.markers.append(target_label)

        self.marker_pub.publish(markers)

    def publish_frontiers(self, candidates):
        markers = MarkerArray()
        for index in range(int(self.p.max_frontier_candidates)):
            markers.markers.append(self.delete_marker(index, "frontiers"))
            markers.markers.append(self.delete_marker(index, "frontier_labels"))
        for candidate in candidates:
            index = int(candidate.candidate_id) - 1
            item = self.marker(index, "frontiers", Marker.SPHERE, candidate.x, candidate.y)
            item.color.b = 1.0
            markers.markers.append(item)
            label = self.marker(
                index,
                "frontier_labels",
                Marker.TEXT_VIEW_FACING,
                candidate.x,
                candidate.y,
                0.34,
            )
            label.scale.x = label.scale.y = 0.0
            label.scale.z = 0.18
            label.color.r = label.color.g = label.color.b = 1.0
            label.text = f"F{candidate.candidate_id}"
            markers.markers.append(label)
        self.marker_pub.publish(markers)

    def publish_frontier_selection(self, candidate):
        selected = self.marker(
            0,
            "vlm_selected_frontier",
            Marker.ARROW,
            candidate.x,
            candidate.y,
            0.10,
        )
        selected.scale.x, selected.scale.y, selected.scale.z = 0.55, 0.14, 0.14
        selected.color.r = 1.0
        selected.color.g = 0.55
        selected.pose.orientation.w = 1.0
        label = self.marker(
            0,
            "vlm_selected_frontier_reason",
            Marker.TEXT_VIEW_FACING,
            candidate.x,
            candidate.y,
            0.58,
        )
        label.scale.x = label.scale.y = 0.0
        label.scale.z = 0.15
        label.color.r = 1.0
        label.color.g = 0.75
        label.color.b = 0.2
        label.text = (
            f"VLM selected F{candidate.candidate_id} "
            f"({self.frontier_selected_confidence:.2f})\n"
            f"{self.frontier_selected_reason[:100]}"
        )
        self.marker_pub.publish(MarkerArray(markers=[selected, label]))

    def publish_rolling_path(
        self, samples, execution_pose, committed_points=None
    ):
        markers = MarkerArray(
            markers=[
                self.delete_marker(0, "frontier_full_path"),
                self.delete_marker(0, "frontier_committed_path"),
                self.delete_marker(0, "frontier_reassessment"),
            ]
        )
        if not samples:
            self.marker_pub.publish(markers)
            return
        full = self.marker(0, "frontier_full_path", Marker.LINE_STRIP, 0.0, 0.0)
        full.scale.x = 0.035
        full.color.b = 1.0
        full.color.g = 0.55
        full.points = [Point(x=x, y=y, z=0.08) for x, y in samples]
        markers.markers.append(full)
        committed = self.marker(
            0, "frontier_committed_path", Marker.LINE_STRIP, 0.0, 0.0
        )
        committed.scale.x = 0.075
        committed.color.r = 1.0
        committed.color.g = 0.55
        committed_xy = (
            list(committed_points)
            if committed_points is not None
            else list(samples)
        )
        committed.points = [
            Point(x=x, y=y, z=0.11) for x, y in committed_xy
        ]
        markers.markers.append(committed)
        checkpoint = self.marker(
            0,
            "frontier_reassessment",
            Marker.SPHERE,
            execution_pose[0],
            execution_pose[1],
            0.13,
        )
        checkpoint.scale.x = checkpoint.scale.y = checkpoint.scale.z = 0.28
        checkpoint.color.r = 1.0
        checkpoint.color.g = 0.9
        markers.markers.append(checkpoint)
        self.marker_pub.publish(markers)

    def publish_goal_marker(self, x, y, yaw, kind):
        item = self.marker(0, f"{kind}_goal", Marker.ARROW, x, y)
        qx, qy, qz, qw = yaw_quaternion(yaw)
        item.pose.orientation.x = qx
        item.pose.orientation.y = qy
        item.pose.orientation.z = qz
        item.pose.orientation.w = qw
        item.scale.x, item.scale.y, item.scale.z = 0.45, 0.12, 0.12
        item.color.g = 1.0 if kind != "frontier" else 0.3
        item.color.b = 1.0 if kind == "frontier" else 0.0
        self.marker_pub.publish(MarkerArray(markers=[item]))

    def publish_easy_case_markers(self, robot_pose):
        if self.target_reference_position is None:
            return
        target = self.target_reference_position
        markers = MarkerArray(
            markers=[
                self.delete_marker(0, "easy_direct_line"),
                self.delete_marker(0, "easy_stage"),
            ]
        )
        direct = self.marker(
            0, "easy_direct_line", Marker.LINE_STRIP, 0.0, 0.0, 0.12
        )
        direct.scale.x = 0.075
        direct.color.r = 1.0
        direct.color.g = 0.85
        direct.color.b = 0.0
        direct.points = [
            Point(x=float(robot_pose[0]), y=float(robot_pose[1]), z=0.12),
            Point(x=float(target[0]), y=float(target[1]), z=0.12),
        ]
        markers.markers.append(direct)
        label_x, label_y = float(target[0]), float(target[1])
        stage = self.marker(
            0,
            "easy_stage",
            Marker.TEXT_VIEW_FACING,
            label_x,
            label_y,
            0.62,
        )
        stage.scale.x = stage.scale.y = 0.0
        stage.scale.z = 0.18
        stage.color.r = 1.0
        stage.color.g = 0.9
        stage.color.b = 0.2
        stage.text = (
            f"EASY CASE: {self.state}\n"
            f"target={self.easy_target_distance:.2f}m "
            f"align={math.degrees(self.easy_alignment_error):.1f}deg"
        )
        markers.markers.append(stage)
        self.marker_pub.publish(markers)

    def compact_diagnostic_values(self, now, worker_busy):
        """Return the small set of fields needed for live fault isolation."""

        def age_value(timestamp):
            age = self.elapsed_since(timestamp, now)
            return round(age, 3) if math.isfinite(age) else "none"

        camera_age = age_value(self.last_camera_tf_success)
        camera_error = self.last_image_tf_error
        if camera_error and camera_error != "none":
            camera_status = (
                f"degraded; ready_age_s={camera_age}; error={camera_error}"
            )
        elif camera_age == "none":
            camera_status = "waiting; ready_age_s=none"
        else:
            camera_status = f"ok; ready_age_s={camera_age}"

        heading_count = len(self.scan_headings)
        progress_index = min(self.scan_index + 1, heading_count)
        progress = f"{progress_index}/{heading_count}"
        if self.state == SCANNING:
            if self.scan_waiting_for_vlm:
                scan_status = (
                    f"waiting_vlm; heading={progress}; "
                    f"retry={self.scan_retry_count}"
                )
            elif self.scan_settle_until > now:
                remaining = round(self.scan_settle_until - now, 3)
                scan_status = (
                    f"settling; heading={progress}; remaining_s={remaining}"
                )
            elif self.goal_kind == "scan":
                scan_status = f"turning; heading={progress}"
            else:
                scan_status = f"ready; heading={progress}"
        elif self.state == TARGET_REOBSERVING:
            scan_status = (
                f"depth_recovery; {self.last_depth_recovery_action}"
            )
        elif self.state in (
            TARGET_CONFIRMING,
            TARGET_ALIGNING,
            APPROACHING,
            APPROACH_STOPPING,
            SUCCEEDED,
        ):
            scan_status = "target_found"
        elif (
            self.state in (FRONTIER_SELECTING, EXPLORING)
            and heading_count > 0
            and self.scan_index >= heading_count
        ):
            scan_status = "completed_no_target"
        elif self.state == FAILED and "scan" in self.last_failure_reason.lower():
            scan_status = f"failed; {self.last_failure_reason}"
        else:
            scan_status = "inactive"

        result_age = (
            round(self.last_result_age, 3)
            if math.isfinite(self.last_result_age)
            else "none"
        )
        vlm_status = (
            f"{self.last_vlm_disposition}; "
            f"latency_s={round(self.last_api_latency, 3)}; "
            f"result_age_s={result_age}; "
            f"worker={'busy' if worker_busy else 'idle'}"
        )

        if self.target_reference_position is not None:
            x, y, z = self.target_reference_position
            target_status = (
                f"grounded; map_xyz=({x:.3f},{y:.3f},{z:.3f})"
            )
        elif self.state == TARGET_CONFIRMING:
            tracker = self.target_tracker
            confirmation_age = self.elapsed_since(
                getattr(self, "target_confirmation_started", 0.0), now
            )
            age_text = (
                round(confirmation_age, 3)
                if math.isfinite(confirmation_age)
                else "none"
            )
            target_status = (
                f"confirming; progress={tracker.progress}/"
                f"{tracker.required_frames}; age_s={age_text}; "
                f"spatial_resets={tracker.reset_count}; "
                f"last_jump_m={round(tracker.last_jump_distance, 3)}"
            )
        elif (
            self.last_target_grounding_error
            and self.last_target_grounding_error != "none"
        ):
            target_status = (
                f"rejected; {self.last_target_grounding_error}"
            )
        else:
            target_status = "none"

        if self.plan_pending:
            navigation_status = f"planning; kind={self.plan_kind}"
        elif self.goal_pending or self.goal_handle is not None:
            navigation_status = f"executing; kind={self.goal_kind}"
        elif (
            self.last_plan_rejection_reason
            and self.last_plan_rejection_reason != "none"
        ):
            navigation_status = (
                f"plan_rejected; {self.last_plan_rejection_reason}"
            )
        else:
            navigation_status = (
                f"idle; last_plan={self.last_plan_kind}; "
                f"status={self.last_plan_status}"
            )

        target_distance = float(
            getattr(self, "easy_target_distance", math.inf)
        )
        distance_text = (
            f"{target_distance:.3f}" if math.isfinite(target_distance) else "none"
        )
        parameters = getattr(self, "p", SimpleNamespace())
        approach_status = (
            f"distance_m={distance_text}; "
            f"limit_m={float(getattr(parameters, 'target_success_radius', 0.81)):.2f}; "
            f"cancel_m={float(getattr(parameters, 'approach_cancel_radius', 0.89)):.2f}; "
            f"action_terminal={bool(getattr(self, 'arrival_action_terminal', False))}; "
            f"stationary={int(getattr(self, 'arrival_stationary_count', 0))}/"
            f"{int(getattr(parameters, 'arrival_stationary_samples', 3))}; "
            f"retry={int(getattr(self, 'arrival_retry_count', 0))}/"
            f"{int(getattr(parameters, 'arrival_retry_limit', 1))}"
        )

        return {
            "state": self.state,
            "sequence": self.sequence,
            "camera_status": camera_status,
            "robot_tf_age_s": age_value(self.last_robot_tf_success),
            "scan_status": scan_status,
            "vlm_status": vlm_status,
            "target_status": target_status,
            "navigation_status": navigation_status,
            "approach_status": approach_status,
            "sensor_wait_reason": self.sensor_wait_reason,
            "last_api_error": self.last_api_error,
            "last_failure_reason": self.last_failure_reason,
        }

    def publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = "vlm_nav"
        status.hardware_id = "rgbd-vlm-nav2"
        if self.state in (FAILED, API_ERROR):
            status.level = DiagnosticStatus.ERROR
        elif (
            not self.get_parameter("enabled").value
            or self.state == SENSOR_WAITING
        ):
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.message = self.state
        worker_busy = self.worker.in_flight if self.worker is not None else False
        values = self.compact_diagnostic_values(
            time.monotonic(),
            worker_busy,
        )
        status.values = [
            KeyValue(key=key, value=str(value))
            for key, value in values.items()
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics_pub.publish(message)

    def destroy_node(self):
        if self.worker is not None:
            self.worker.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VLMNavigator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cancel_motion(publish_stop=True)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
