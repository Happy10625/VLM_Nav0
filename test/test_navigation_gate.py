"""Small ROS contract tests; skipped when the ROS environment is not sourced."""

from collections import deque
import math
import time
from types import SimpleNamespace

import pytest
import numpy as np

rclpy = pytest.importorskip("rclpy")

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker
from tf2_ros import TransformException

from vlm_nav.vlm_navigator import (
    APPROACHING,
    API_ERROR,
    FAILED,
    SCANNING,
    SEARCHING,
    SENSOR_WAITING,
    TARGET_ALIGNING,
    TARGET_CONFIRMING,
    TARGET_REOBSERVING,
    VLMNavigator,
)
from vlm_nav.models import FrameSnapshot, Pixel, VLMResult, WorkerResult


def path_pose(x, y):
    return SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=float(x), y=float(y)))
    )


def test_successful_compute_path_is_required_before_navigation_dispatch():
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 7
    node.current_plan_pose = (1.0, 0.0, 0.0)
    node.session_origin = (0.0, 0.0)
    node.p = SimpleNamespace(max_travel_radius=3.0)
    node.plan_kind = "waypoint"
    node.plan_pending = True
    node.plan_handle = object()
    node.plan_candidates = []
    dispatched = []
    node.send_navigation_goal = lambda *args: dispatched.append(args)
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            path=SimpleNamespace(poses=[path_pose(0.0, 0.0), path_pose(1.0, 0.0)])
        ),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=7)

    assert dispatched == [(1.0, 0.0, 0.0, "waypoint")]
    assert node.plan_pending is False


def test_easy_case_confirmation_enters_target_alignment():
    node = VLMNavigator.__new__(VLMNavigator)
    node.target_tracker = SimpleNamespace(
        update=lambda _point: (2.0, 0.0, 0.5)
    )
    node.target_position = None
    node.pending_waypoints = []
    node.p = SimpleNamespace(easy_case_mode=True)
    states = []
    node.cancel_motion = lambda publish_stop: None
    node.set_state = states.append
    node.publish_grounded_markers = lambda *_args: None

    node.confirm_target((2.0, 0.0, 0.5))

    assert states == [TARGET_ALIGNING]
    assert node.easy_alignment_complete is False
    assert node.target_position == (2.0, 0.0, 0.5)


def test_initial_arm_costmap_clear_blocks_scan_until_services_complete():
    callbacks = []

    class Future:
        def add_done_callback(self, callback):
            callbacks.append(callback)

        def result(self):
            return SimpleNamespace()

    class Client:
        srv_name = "/local_costmap/clear_entirely_local_costmap"

        def __init__(self):
            self.requests = []

        def wait_for_service(self, timeout_sec):
            return True

        def call_async(self, request):
            self.requests.append(request)
            return Future()

    node = VLMNavigator.__new__(VLMNavigator)
    client = Client()
    node.p = SimpleNamespace(clear_costmap_on_arm=True)
    node.costmap_clear_clients = [("local", client)]
    node.initial_costmap_clear_required = False
    node.initial_costmap_clear_token = 0
    node.behavior_costmap_revision = 4
    node.last_behavior_costmap_received = 0.0
    stopped = []
    failed = []
    node.publish_stop = lambda: stopped.append(True)
    node.fail_safe = failed.append
    node.get_logger = lambda: SimpleNamespace(info=lambda *_args: None)

    node.require_initial_costmap_clear()

    assert node.ensure_initial_costmap_clear_done() is False
    assert len(client.requests) == 1
    assert node.initial_costmap_clear_pending == 1
    assert stopped
    assert not failed

    callbacks[0](Future())

    assert node.initial_costmap_clear_required is True
    assert node.last_initial_costmap_clear_status == "waiting_behavior_refresh"
    node.on_behavior_costmap(object())

    assert node.ensure_initial_costmap_clear_done() is True
    assert node.initial_costmap_clear_required is False
    assert node.last_initial_costmap_clear_status == "cleared_and_refreshed"


def test_initial_arm_scan_spin_waits_for_costmap_clear_gate():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SCANNING
    node.grid = np.zeros((1, 1), dtype=np.int16)
    node.map_message = object()
    node.session_origin = (0.0, 0.0)
    node.goal_handle = None
    node.goal_pending = False
    node.plan_pending = False
    node.target_position = None
    node.scan_waiting_for_vlm = False
    node.scan_settle_until = 0.0
    node.scan_index = 0
    node.scan_headings = [0.5]
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.robot_pose = lambda: (0.0, 0.0, 0.0)
    node.ensure_initial_costmap_clear_done = lambda: False
    spins = []
    node.send_spin = lambda relative, kind="scan": spins.append((relative, kind))

    node.navigation_tick()

    assert spins == []


def test_initial_arm_scan_fails_if_behavior_costmap_never_refreshes():
    node = VLMNavigator.__new__(VLMNavigator)
    node.initial_costmap_clear_required = True
    node.initial_costmap_clear_requested = True
    node.initial_costmap_clear_pending = 0
    node.initial_costmap_clear_completed_at = time.monotonic() - 1.0
    node.initial_costmap_refresh_deadline = time.monotonic() - 0.1
    node.initial_costmap_clear_baseline_revision = 3
    node.behavior_costmap_revision = 3
    node.last_behavior_costmap_received = 0.0
    stopped = []
    failed = []
    node.publish_stop = lambda: stopped.append(True)
    node.fail_safe = failed.append

    assert node.ensure_initial_costmap_clear_done() is False
    assert failed == [
        "Behavior costmap did not refresh after Nav2 costmap clear; "
        "refusing initial ARM scan"
    ]
    assert stopped
    assert (
        node.last_initial_costmap_clear_status
        == "failed:behavior_costmap_refresh_timeout"
    )


def test_first_grounded_scan_detection_stops_before_three_frame_confirmation():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SCANNING
    node.target_tracker = SimpleNamespace(update=lambda _point: None)
    node.frontier_generation = 0
    node.frontier_request = object()
    node.frontier_request_pending = True
    cancelled = []
    states = []
    node.cancel_motion = lambda publish_stop: cancelled.append(publish_stop)
    node.set_state = states.append

    node.confirm_target((2.0, 0.0, 0.5))

    assert cancelled == [True]
    assert states == [TARGET_CONFIRMING]
    assert node.frontier_request is None
    assert node.frontier_request_pending is False


def test_successful_scan_spin_waits_for_stationary_vlm_frame():
    node = VLMNavigator.__new__(VLMNavigator)
    node.goal_token = 4
    node.goal_pose = None
    node.goal_handle = object()
    node.goal_pending = False
    node.goal_kind = "scan"
    node.active_motion_origin = None
    node.rolling_goal_is_final = True
    node.scan_index = 0
    node.scan_headings = [0.5, 1.0]
    node.sequence = 42
    node.p = SimpleNamespace(scan_settle_time=0.30)
    stopped = []
    node.publish_stop = lambda: stopped.append(True)
    wrapped = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)

    started = time.monotonic()
    node.on_navigation_result(
        SimpleNamespace(result=lambda: wrapped), token=4, kind="scan"
    )

    assert stopped
    assert node.scan_index == 0
    assert node.scan_capture_after_sequence == 42
    assert node.scan_settle_until >= started + 0.25
    assert node.scan_waiting_for_vlm is False
    assert node.scan_request_sequence == -1


def test_stationary_scan_submits_one_new_frame_and_holds_heading():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SCANNING
    node.scan_waiting_for_vlm = False
    node.scan_settle_until = time.monotonic() - 0.1
    node.scan_capture_after_sequence = 7
    node.scan_request_sequence = -1
    node.scan_index = 0
    node.scan_headings = [0.5, 1.0]
    node.last_submitted_sequence = -1
    snapshot = SimpleNamespace(
        sequence=8,
        captured_monotonic=time.monotonic(),
    )
    node.latest_snapshot = snapshot
    submitted = []
    node.worker = SimpleNamespace(submit=submitted.append)
    node.ensure_worker = lambda: True
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.get_logger = lambda: SimpleNamespace(info=lambda *_args: None)

    node.sample_latest_frame()
    node.sample_latest_frame()

    assert submitted == [snapshot]
    assert node.scan_waiting_for_vlm is True
    assert node.scan_request_sequence == 8
    assert node.scan_index == 0


def test_no_target_scan_result_advances_only_after_result_handling():
    node = VLMNavigator.__new__(VLMNavigator)
    node.scan_index = 0
    node.scan_headings = [0.5, 1.0]
    node.scan_observations = []
    node.scan_observation_headings = []
    node.scan_waiting_for_vlm = True
    node.scan_request_sequence = 9
    node.scan_settle_until = time.monotonic()
    node.scan_capture_after_sequence = 8
    node.scan_retry_count = 2
    node.sequence = 10
    snapshot = SimpleNamespace(rgb=np.zeros((2, 2, 3), dtype=np.uint8))

    node.advance_stationary_scan_view(snapshot)

    assert node.scan_index == 1
    assert len(node.scan_observations) == 1
    assert node.scan_observation_headings == [0.5]
    assert node.scan_waiting_for_vlm is False
    assert node.scan_settle_until == 0.0
    assert node.scan_retry_count == 0


def depth_recovery_snapshot():
    camera_to_map = np.eye(4)
    camera_to_map[:3, :3] = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ]
    )
    return SimpleNamespace(
        intrinsics=(500.0, 500.0, 320.0, 240.0),
        transform_matrix=camera_to_map,
    )


def depth_recovery_node():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SCANNING
    node.task_epoch = 4
    node.p = SimpleNamespace(
        depth_reobserve_angle_deg=10.0,
        depth_reobserve_attempt_limit=3,
        target_probe_distance=2.0,
        target_probe_attempt_limit=3,
        max_travel_radius=3.0,
        min_waypoint_distance=0.3,
    )
    node.depth_reobserve_attempts = 0
    node.target_probe_attempts = 0
    node.target_probe_candidates = []
    node.last_depth_recovery_action = "none"
    node.scan_waiting_for_vlm = True
    node.scan_request_sequence = 8
    node.scan_settle_until = 1.0
    node.target_position = None
    node.pending_waypoints = []
    node.target_tracker = SimpleNamespace(reset=lambda: None)
    node.robot_pose = lambda: (0.0, 0.0, 0.0)
    node.ground_pixel = lambda *_args, **_kwargs: None
    node.snap_free = lambda point: point
    node.cancel_motion = lambda publish_stop: None
    node.get_logger = lambda: SimpleNamespace(warn=lambda *_args: None)
    node.set_state = lambda state: setattr(node, "state", state)
    return node


def test_unreliable_target_depth_rotates_toward_vlm_pixel():
    node = depth_recovery_node()
    rotations = []
    node.send_spin = lambda angle, kind: rotations.append((angle, kind))
    result = VLMResult(
        target_visible=True,
        confidence=0.9,
        target_pixel=Pixel(420, 240),
        waypoints=(),
    )

    recovered = node.recover_target_depth(
        depth_recovery_snapshot(),
        result,
        "insufficient_valid_depth_samples:0/8",
    )

    assert recovered is True
    assert node.state == TARGET_REOBSERVING
    assert node.depth_reobserve_attempts == 1
    assert node.task_epoch == 5
    assert rotations[0][1] == "depth_reobserve"
    assert math.degrees(rotations[0][0]) == pytest.approx(-10.0)


def test_out_of_range_target_uses_nav2_validated_directional_probe():
    node = depth_recovery_node()
    planned = []
    node.plan_then_navigate = (
        lambda candidates, kind: planned.append((candidates, kind))
    )
    result = VLMResult(
        target_visible=True,
        confidence=0.9,
        target_pixel=Pixel(320, 240),
        waypoints=(),
    )

    recovered = node.recover_target_depth(
        depth_recovery_snapshot(),
        result,
        "depth_out_of_range:median_m=6.800,limit_m=6.000",
    )

    assert recovered is True
    assert node.state == TARGET_REOBSERVING
    assert node.target_probe_attempts == 1
    assert node.task_epoch == 5
    assert planned[0][1] == "target_probe"
    assert planned[0][0][0] == pytest.approx((2.0, 0.0, 0.0))


def test_target_probe_arrival_waits_for_fresh_stationary_observation():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = TARGET_REOBSERVING
    node.goal_token = 6
    node.goal_pose = (2.0, 0.0, 0.0)
    node.goal_handle = object()
    node.goal_pending = False
    node.goal_kind = "target_probe"
    node.active_motion_origin = (0.0, 0.0)
    node.rolling_goal_is_final = True
    node.sequence = 21
    node.target_probe_attempts = 1
    node.depth_reobserve_attempts = 2
    node.scan_retry_count = 1
    node.p = SimpleNamespace(scan_settle_time=0.30)
    node.publish_stop = lambda: None
    node.set_state = lambda state: setattr(node, "state", state)
    wrapped = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)

    started = time.monotonic()
    node.on_navigation_result(
        SimpleNamespace(result=lambda: wrapped),
        token=6,
        kind="target_probe",
    )

    assert node.state == TARGET_REOBSERVING
    assert node.depth_reobserve_attempts == 0
    assert node.scan_capture_after_sequence == 21
    assert node.scan_settle_until >= started + 0.25
    assert node.scan_waiting_for_vlm is False


def test_easy_case_straight_plan_is_dispatched_without_radius_clipping():
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 8
    node.plan_attempt_count = 1
    node.current_plan_pose = (1.19, 0.0, 0.0)
    node.p = SimpleNamespace(
        max_travel_radius=3.0,
        easy_case_max_path_deviation=0.10,
        path_horizon_segments=16,
    )
    node.plan_kind = "easy_approach"
    node.plan_pending = True
    node.plan_handle = object()
    node.plan_candidates = []
    dispatched = []
    node.send_navigation_goal = lambda *args: dispatched.append(args)
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            path=SimpleNamespace(
                poses=[
                    path_pose(0.0, 0.0),
                    path_pose(0.6, 0.03),
                    path_pose(1.19, 0.0),
                ]
            )
        ),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=8)

    assert dispatched == [(1.19, 0.0, 0.0, "easy_approach")]
    assert node.easy_path_deviation == pytest.approx(0.03)
    assert node.last_plan_radius_clipped is False


def test_easy_case_detouring_plan_is_rejected():
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 9
    node.plan_attempt_count = 1
    node.current_plan_pose = (1.19, 0.0, 0.0)
    node.p = SimpleNamespace(
        max_travel_radius=3.0,
        easy_case_max_path_deviation=0.10,
    )
    node.plan_kind = "easy_approach"
    rejected = []
    node.reject_current_plan = lambda reason, token: rejected.append(
        (reason, token)
    )
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            path=SimpleNamespace(
                poses=[
                    path_pose(0.0, 0.0),
                    path_pose(0.6, 0.25),
                    path_pose(1.19, 0.0),
                ]
            )
        ),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=9)

    assert rejected and rejected[0][1] == 9
    assert "not sufficiently straight" in rejected[0][0]


def test_path_leaving_rolling_radius_is_truncated_and_dispatched():
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 2
    node.plan_attempt_count = 1
    node.current_plan_pose = (4.0, 0.0, 0.0)
    node.session_origin = (0.0, 0.0)
    node.p = SimpleNamespace(
        max_travel_radius=3.0,
        path_horizon_segments=16,
        target_clearance=0.50,
        robot_front_extent=0.36,
        approach_goal_margin=0.05,
        standoff_radius_offsets=[0.0, 0.20],
    )
    node.plan_kind = "approach"
    node.plan_pending = True
    node.plan_handle = object()
    node.plan_candidates = []
    node.standoff_candidate_radii = {
        node.goal_key(node.current_plan_pose): 1.01,
    }
    node.selected_standoff_radius = 0.0
    node.selected_standoff_mode = "none"
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *_args: None,
        warn=lambda *_args: None,
    )
    node.classify_standoff_point = lambda *_args, **_kwargs: (
        "known_free",
        (4.0, 0.0),
    )
    dispatched = []
    rejected = []
    node.send_navigation_goal = lambda *args: dispatched.append(args)
    node.reject_current_plan = lambda reason, token: rejected.append((reason, token))
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            path=SimpleNamespace(poses=[path_pose(0.0, 0.0), path_pose(4.0, 0.0)])
        ),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=2)

    assert not rejected
    assert dispatched[0][0] == pytest.approx(3.0)
    assert dispatched[0][1] == pytest.approx(0.0)
    assert dispatched[0][3] == "approach"
    assert node.last_plan_radius_clipped is True
    assert node.rolling_goal_is_final is False
    assert node.last_plan_commit_length == pytest.approx(3.0)
    assert node.selected_standoff_radius == pytest.approx(1.01)
    assert node.selected_standoff_mode == "degraded"


def test_standoff_candidates_use_inner_ring_before_degraded_outer_ring():
    node = VLMNavigator.__new__(VLMNavigator)
    node.target_position = (5.0, 5.0, 0.5)
    node.p = SimpleNamespace(
        target_clearance=0.50,
        robot_front_extent=0.36,
        approach_goal_margin=0.05,
        standoff_radius_offsets=[0.0, 0.20],
    )
    node.classify_standoff_point = lambda point: ("known_free", point)

    candidates = node.evaluate_standoff_candidates(
        pose=(3.0, 5.0, 0.0), publish=False
    )

    assert len(candidates) == 32
    distances = [
        math.hypot(item[0] - 5.0, item[1] - 5.0)
        for item in candidates
    ]
    assert all(value == pytest.approx(0.81) for value in distances[:16])
    assert all(value == pytest.approx(1.01) for value in distances[16:])
    assert [item["radius"] for item in node.standoff_ring_stats] == pytest.approx(
        [0.81, 1.01]
    )


def test_outer_standoff_remains_when_target_blocks_entire_inner_ring():
    node = VLMNavigator.__new__(VLMNavigator)
    node.target_position = (5.0, 5.0, 0.5)
    node.p = SimpleNamespace(
        target_clearance=0.50,
        robot_front_extent=0.36,
        approach_goal_margin=0.05,
        standoff_radius_offsets=[0.0, 0.20],
    )

    def classify(point):
        radius = math.hypot(point[0] - 5.0, point[1] - 5.0)
        return ("occupied", None) if radius < 0.9 else ("known_free", point)

    node.classify_standoff_point = classify

    candidates = node.evaluate_standoff_candidates(
        pose=(3.0, 5.0, 0.0), publish=False
    )

    assert len(candidates) == 16
    assert all(
        math.hypot(item[0] - 5.0, item[1] - 5.0) == pytest.approx(1.01)
        for item in candidates
    )
    assert node.standoff_ring_stats[0]["rejected"] == 16
    assert node.standoff_ring_stats[1]["free"] == 16


def test_selected_outer_ring_sets_degraded_success_limit_and_can_be_reset():
    node = VLMNavigator.__new__(VLMNavigator)
    node.p = SimpleNamespace(
        target_clearance=0.50,
        robot_front_extent=0.36,
        approach_goal_margin=0.05,
        standoff_radius_offsets=[0.0, 0.20],
    )
    pose = (4.0, 5.0, 0.0)
    node.standoff_candidate_radii = {node.goal_key(pose): 1.01}
    node.selected_standoff_radius = 0.0
    node.selected_standoff_mode = "none"
    warnings = []
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *_args: None,
        warn=warnings.append,
    )

    node.select_standoff_radius(pose)

    assert node.selected_standoff_mode == "degraded"
    assert node.approach_success_limit() == pytest.approx(1.06)
    assert warnings and "degraded" in warnings[0]

    node.clear_standoff_selection()

    assert node.selected_standoff_mode == "none"
    assert node.approach_success_limit() == pytest.approx(0.86)


def test_planner_action_failure_is_distinct_from_radius_rejection():
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 5
    node.plan_attempt_count = 1
    node.current_plan_pose = (1.0, 0.0, 0.0)
    node.session_origin = (0.0, 0.0)
    node.p = SimpleNamespace(max_travel_radius=3.0)
    node.plan_kind = "approach"
    dispatched = []
    rejected = []
    node.send_navigation_goal = lambda *args: dispatched.append(args)
    node.reject_current_plan = lambda reason, token: rejected.append((reason, token))
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED,
        result=SimpleNamespace(path=SimpleNamespace(poses=[])),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=5)

    assert not dispatched
    assert rejected and rejected[0][1] == 5
    assert "reason=planner_action_status_6" in rejected[0][0]


def test_long_frontier_path_commits_only_first_half_for_reassessment():
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 3
    node.current_plan_pose = (2.0, 0.0, 0.0)
    node.session_origin = (0.0, 0.0)
    node.p = SimpleNamespace(
        max_travel_radius=3.0,
        path_horizon_segments=16,
        path_execute_segments=8,
        frontier_full_commit_distance=1.0,
    )
    node.plan_kind = "frontier"
    node.plan_pending = True
    node.plan_handle = object()
    node.plan_candidates = []
    node.frontier_generation = 5
    node.frontier_selected_id = 2
    node.publish_rolling_path = lambda *_args: None
    dispatched = []
    node.send_navigation_goal = lambda *args: dispatched.append(args)
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            path=SimpleNamespace(
                poses=[path_pose(0.0, 0.0), path_pose(2.0, 0.0)]
            )
        ),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=3)

    assert dispatched[0][0] == pytest.approx(1.0)
    assert dispatched[0][1] == pytest.approx(0.0)
    assert dispatched[0][3] == "frontier"
    assert node.frontier_goal_is_final is False
    assert len(node.frontier_path_samples) == 17


def test_short_frontier_path_commits_to_actual_frontier():
    node = VLMNavigator.__new__(VLMNavigator)
    node.plan_token = 4
    node.current_plan_pose = (0.8, 0.0, 0.0)
    node.session_origin = (0.0, 0.0)
    node.p = SimpleNamespace(
        max_travel_radius=3.0,
        path_horizon_segments=16,
        path_execute_segments=8,
        frontier_full_commit_distance=1.0,
    )
    node.plan_kind = "frontier"
    node.plan_pending = True
    node.plan_handle = object()
    node.plan_candidates = []
    node.frontier_generation = 5
    node.frontier_selected_id = 1
    node.publish_rolling_path = lambda *_args: None
    dispatched = []
    node.send_navigation_goal = lambda *args: dispatched.append(args)
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            path=SimpleNamespace(
                poses=[path_pose(0.0, 0.0), path_pose(0.8, 0.0)]
            )
        ),
    )

    node.on_plan_result(SimpleNamespace(result=lambda: wrapped), token=4)

    assert dispatched == [(0.8, 0.0, 0.0, "frontier")]
    assert node.frontier_goal_is_final is True


def test_rolling_approach_checkpoint_waits_for_a_newer_vlm_frame():
    node = VLMNavigator.__new__(VLMNavigator)
    node.goal_token = 9
    node.goal_pose = (3.0, 0.0, 0.0)
    node.goal_handle = object()
    node.goal_pending = False
    node.goal_kind = "approach"
    node.rolling_goal_is_final = False
    node.sequence = 42
    node.pending_waypoints = [(4.0, 0.0, 0.0)]
    node.active_motion_origin = (0.0, 0.0)
    node.active_standoff_pose = None
    node.active_standoff_status = "none"
    stopped = []
    node.publish_stop = lambda: stopped.append(True)
    node.get_logger = lambda: SimpleNamespace(info=lambda _message: None)
    wrapped = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)

    node.on_navigation_result(
        SimpleNamespace(result=lambda: wrapped), token=9, kind="approach"
    )

    assert stopped
    assert not node.pending_waypoints
    assert node.rolling_reassessment_required is True
    assert node.rolling_reassessment_after_sequence == 42
    assert node.active_motion_origin is None


def test_api_failure_does_not_overwrite_failed_state():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = FAILED
    node.api_failures = 3
    node.rejected_results = 0
    node.last_api_error = "none"
    node.p = SimpleNamespace(api_failure_limit=3)
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.get_logger = lambda: SimpleNamespace(warn=lambda message: None)
    node.cancel_motion = lambda publish_stop: pytest.fail(
        "FAILED state must not be replaced by API_ERROR"
    )
    node.set_state = lambda state: pytest.fail(
        "FAILED state must not be replaced by API_ERROR"
    )

    node.record_api_failure("timeout")

    assert node.state == FAILED
    assert node.last_api_error == "timeout"
    assert node.api_failures == 4


def test_vlm_result_is_published_and_forwarded_to_image_recorder():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = TARGET_CONFIRMING
    recorded = []
    published = []
    node.image_recorder = SimpleNamespace(
        record=lambda *args: recorded.append(args) or ("image.jpg",),
        last_error="none",
    )
    node.publish_vlm_output = published.append
    snapshot = SimpleNamespace(
        sequence=12,
        task_epoch=3,
        target_description="chair",
        stamp=SimpleNamespace(sec=123, nanosec=456),
        frame_id="camera_color_optical_frame",
    )
    result = VLMResult(
        target_visible=True,
        confidence=0.92,
        target_pixel=Pixel(640, 320),
        waypoints=(Pixel(620, 500),),
    )
    completed = WorkerResult(
        snapshot,
        result,
        0.75,
        raw_response=(
            '{"target_visible":true,"confidence":0.92,'
            '"target_pixel":{"u":640,"v":320},'
            '"waypoints":[{"u":620,"v":500}]}'
        ),
    )

    node.log_worker_result(
        completed, age=0.8, state_before=SEARCHING, disposition="accepted_target"
    )

    record = published[0]
    assert record["event"] == "vlm_response"
    assert record["navigation_state_before"] == SEARCHING
    assert record["navigation_state_after"] == TARGET_CONFIRMING
    assert record["disposition"] == "accepted_target"
    assert record["accepted"] is True
    assert record["parsed_result"]["target_pixel"] == {"u": 640, "v": 320}
    assert record["raw_response"].startswith('{"target_visible":true')
    assert recorded[0][0] is snapshot
    assert recorded[0][1] is result
    assert recorded[0][2] == "accepted_target"


def test_grounded_markers_include_target_label_and_ordered_path_without_deleteall():
    node = VLMNavigator.__new__(VLMNavigator)
    node.p = SimpleNamespace(global_frame="map", target_description="chair")
    node.last_vlm_confidence = 0.91
    published = []
    node.marker_pub = SimpleNamespace(publish=published.append)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time())
    )

    node.publish_grounded_markers(
        target=(2.0, 0.5, 0.1),
        waypoints=[(0.5, 0.0, 0.0), (1.2, 0.2, 0.0)],
    )

    markers = published[0].markers
    assert not any(item.action == Marker.DELETEALL for item in markers)
    namespaces = {item.ns for item in markers if item.action == Marker.ADD}
    assert {
        "vlm_target",
        "vlm_target_label",
        "vlm_waypoints",
        "vlm_path",
    }.issubset(namespaces)
    path = next(
        item
        for item in markers
        if item.ns == "vlm_path" and item.action == Marker.ADD
    )
    assert [(point.x, point.y) for point in path.points] == [
        (0.5, 0.0),
        (1.2, 0.2),
        (2.0, 0.5),
    ]


@pytest.mark.parametrize("terminal_state", [SENSOR_WAITING, FAILED])
def test_terminal_failure_does_not_submit_more_images(terminal_state):
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = terminal_state
    node.latest_snapshot = object()
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.ensure_worker = lambda: pytest.fail(
        "FAILED state must not start or submit API work"
    )

    node.sample_latest_frame()


def test_api_error_can_still_retry_for_automatic_recovery():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = API_ERROR
    node.latest_snapshot = SimpleNamespace(sequence=9)
    node.last_submitted_sequence = 8
    submitted = []
    node.worker = SimpleNamespace(submit=lambda snapshot: submitted.append(snapshot))
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.ensure_worker = lambda: True

    node.sample_latest_frame()

    assert submitted == [node.latest_snapshot]


def test_frontier_retry_refreshes_attempt_timestamp():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = API_ERROR
    snapshot = FrameSnapshot(
        sequence=9,
        task_epoch=1,
        target_description="chair",
        captured_monotonic=1.0,
        stamp=Time(),
        frame_id="camera",
        rgb=np.zeros((2, 2, 3), dtype=np.uint8),
        depth_m=np.zeros((2, 2), dtype=np.float32),
        intrinsics=(1.0, 1.0, 1.0, 1.0),
        transform_matrix=np.eye(4),
        request_kind="frontier",
    )
    node.frontier_request = snapshot
    node.frontier_request_pending = True
    submitted = []
    node.worker = SimpleNamespace(submit=submitted.append)
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.ensure_worker = lambda: True

    node.sample_latest_frame()

    assert len(submitted) == 1
    assert submitted[0].captured_monotonic > 1.0
    assert node.frontier_request is submitted[0]
    assert node.frontier_request_pending is False


def safety_test_node():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = TARGET_CONFIRMING
    node.p = SimpleNamespace(
        rgbd_wait_timeout=2.0,
        sensor_failure_timeout=30.0,
        tf_failure_timeout=3.0,
        max_travel_radius=3.0,
        target_lost_timeout=10.0,
        goal_timeout=120.0,
    )
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.session_origin = None
    node.target_position = None
    node.goal_handle = None
    node.goal_pending = False
    node.plan_pending = False
    now = time.monotonic()
    node.last_valid_rgbd_received = now
    node.last_camera_tf_success = now
    node.last_robot_tf_success = now
    node.sensor_wait_started = 0.0
    node.sensor_wait_reason = "none"
    return node


def test_safety_tick_refreshes_robot_tf_while_target_confirming():
    node = safety_test_node()
    node.last_valid_rgbd_received = time.monotonic()
    node.last_camera_tf_success = time.monotonic()
    node.last_robot_tf_success = 0.0
    node.robot_pose = lambda: (
        setattr(node, "last_robot_tf_success", time.monotonic())
        or (0.0, 0.0, 0.0)
    )
    failures = []
    node.fail_safe = failures.append

    node.safety_tick()

    assert failures == []


def test_safety_tick_pauses_for_camera_tf_without_failing():
    node = safety_test_node()
    node.last_valid_rgbd_received = time.monotonic()
    node.last_camera_tf_success = 0.0
    node.last_robot_tf_success = 0.0
    node.robot_pose = lambda: (
        setattr(node, "last_robot_tf_success", time.monotonic())
        or (0.0, 0.0, 0.0)
    )
    pauses = []
    node.enter_sensor_wait = pauses.append
    failures = []
    node.fail_safe = failures.append

    node.safety_tick()

    assert failures == []
    assert pauses == [
        "Valid RGB-D is arriving but image-time camera TF is unavailable"
    ]


def test_safety_tick_distinguishes_missing_valid_rgbd():
    node = safety_test_node()
    node.last_valid_rgbd_received = 0.0
    node.last_camera_tf_success = time.monotonic()
    node.robot_pose = lambda: (0.0, 0.0, 0.0)
    pauses = []
    node.enter_sensor_wait = pauses.append
    node.fail_safe = lambda reason: pytest.fail(reason)

    node.safety_tick()

    assert pauses == [
        "No valid aligned RGB-D received within safety timeout"
    ]


def test_sensor_wait_hard_timeout_is_the_only_sensor_failure():
    node = safety_test_node()
    node.state = SENSOR_WAITING
    now = time.monotonic()
    node.last_valid_rgbd_received = now
    node.last_camera_tf_success = now
    node.sensor_wait_started = now - 31.0
    node.sensor_wait_reason = "No valid aligned RGB-D received within safety timeout"
    node.robot_pose = lambda: (0.0, 0.0, 0.0)
    failures = []
    node.fail_safe = failures.append

    node.safety_tick()

    assert failures == [
        "Sensor unavailable beyond hard timeout: "
        "No valid aligned RGB-D received within safety timeout"
    ]


def test_rgbd_metadata_mismatch_is_rejected_before_image_conversion():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SCANNING
    node.p = SimpleNamespace(camera_frame="camera_color_optical_frame")
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    node.camera_info = SimpleNamespace(width=1280, height=720)
    node.last_rgbd_pair_received = 0.0
    node.last_valid_rgbd_received = 0.0
    node.invalid_rgbd_frames = 0
    node.sensor_recovery_count = 2
    node.get_logger = lambda: SimpleNamespace(warn=lambda *args, **kwargs: None)
    node.image_to_rgb = lambda message: pytest.fail(
        "metadata mismatch must be rejected before RGB conversion"
    )
    node.image_to_depth_m = lambda message: pytest.fail(
        "metadata mismatch must be rejected before depth conversion"
    )
    rgb = SimpleNamespace(
        width=1280,
        height=720,
        header=SimpleNamespace(frame_id="camera_color_optical_frame"),
    )
    depth = SimpleNamespace(
        width=640,
        height=480,
        header=SimpleNamespace(frame_id="camera_color_optical_frame"),
    )

    node.on_rgbd(rgb, depth)

    assert node.last_rgbd_pair_received > 0.0
    assert node.last_valid_rgbd_received == 0.0
    assert node.invalid_rgbd_frames == 1
    assert node.sensor_recovery_count == 0


def test_delayed_image_tf_is_retried_at_the_exact_original_stamp():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SCANNING
    node.p = SimpleNamespace(
        global_frame="map",
        camera_frame="camera_color_optical_frame",
        image_tf_wait_timeout=5.0,
        sensor_recovery_frames=3,
    )
    stamp = Time(sec=123, nanosec=456)
    frame = SimpleNamespace(
        received_monotonic=time.monotonic(),
        stamp=stamp,
        frame_id="camera_color_optical_frame",
        rgb=np.zeros((2, 2, 3), dtype=np.uint8),
        depth_m=np.ones((2, 2), dtype=np.float32),
        intrinsics=(1.0, 1.0, 1.0, 1.0),
    )
    node.pending_rgbd_frames = deque([frame])
    node.image_tf_queue_drops = 0
    node.image_tf_failures = 0
    node.last_image_tf_error = "none"
    node.sensor_recovery_count = 0
    node.sequence = 0
    node.task_epoch = 7
    node.latest_snapshot = None
    node.last_camera_tf_success = 0.0
    node.get_parameter = lambda name: SimpleNamespace(
        value="chair" if name == "target_description" else True
    )
    node.get_logger = lambda: SimpleNamespace(warn=lambda *args, **kwargs: None)
    requested = []

    def unavailable(_target, _source, requested_time, timeout):
        requested.append(requested_time.nanoseconds)
        raise TransformException("future extrapolation")

    node.tf_buffer = SimpleNamespace(lookup_transform=unavailable)
    node.resolve_pending_rgbd()

    assert len(node.pending_rgbd_frames) == 1
    assert node.latest_snapshot is None
    assert node.last_image_tf_error == "future extrapolation"

    transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )
    node.tf_buffer.lookup_transform = (
        lambda _target, _source, requested_time, timeout: (
            requested.append(requested_time.nanoseconds) or transform
        )
    )
    node.resolve_pending_rgbd()

    expected_stamp_ns = 123_000_000_456
    assert requested == [expected_stamp_ns, expected_stamp_ns]
    assert not node.pending_rgbd_frames
    assert node.latest_snapshot.stamp is stamp
    assert node.latest_snapshot.captured_monotonic == frame.received_monotonic
    assert node.sequence == 1
    assert node.last_image_tf_error == "none"


def test_map_update_cancels_active_standoff_that_becomes_occupied():
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = APPROACHING
    node.target_position = (2.0, 0.0, 0.5)
    node.map_revision = 3
    node.active_standoff_pose = (1.2, 0.0, 0.0)
    node.active_standoff_status = "unknown"
    node.robot_pose = lambda: (0.0, 0.0, 0.0)
    node.evaluate_standoff_candidates = lambda pose, publish: []
    node.classify_standoff_point = lambda point, free_snap_distance: (
        "occupied",
        None,
    )
    warnings = []
    node.get_logger = lambda: SimpleNamespace(
        warn=warnings.append,
        info=lambda *_args: None,
    )
    cancelled = []
    node.cancel_motion = lambda publish_stop: (
        cancelled.append(publish_stop),
        setattr(node, "active_standoff_pose", None),
        setattr(node, "active_standoff_status", "none"),
    )
    node.standoff_wait_started = 0.0
    message = SimpleNamespace(
        info=SimpleNamespace(width=2, height=2),
        data=[0, 0, 0, 100],
    )

    node.on_map(message)

    assert node.map_revision == 4
    assert cancelled == [True]
    assert node.standoff_wait_started > 0.0
    assert "became unsafe" in warnings[0]


def test_compact_diagnostics_keep_only_fault_isolation_fields():
    now = time.monotonic()
    node = VLMNavigator.__new__(VLMNavigator)
    node.state = SCANNING
    node.sequence = 42
    node.last_camera_tf_success = now - 0.25
    node.last_robot_tf_success = now - 0.10
    node.last_image_tf_error = "none"
    node.scan_headings = [0.0, 1.57, 3.14]
    node.scan_index = 1
    node.scan_waiting_for_vlm = True
    node.scan_retry_count = 1
    node.scan_settle_until = 0.0
    node.goal_kind = "scan"
    node.last_failure_reason = "none"
    node.last_result_age = 0.4
    node.last_api_latency = 1.2
    node.last_vlm_disposition = "target_rejected"
    node.target_position = None
    node.last_target_grounding_error = "invalid_depth"
    node.plan_pending = False
    node.plan_kind = None
    node.goal_pending = False
    node.goal_handle = None
    node.last_plan_rejection_reason = "none"
    node.last_plan_kind = "none"
    node.last_plan_status = -1
    node.sensor_wait_reason = "none"
    node.last_api_error = "none"

    values = node.compact_diagnostic_values(now, worker_busy=False)

    assert list(values) == [
        "state",
        "sequence",
        "camera_status",
        "robot_tf_age_s",
        "scan_status",
        "vlm_status",
        "target_status",
        "navigation_status",
        "standoff_status",
        "sensor_wait_reason",
        "last_api_error",
        "last_failure_reason",
    ]
    assert values["sequence"] == 42
    assert values["camera_status"] == "ok; ready_age_s=0.25"
    assert values["scan_status"] == (
        "waiting_vlm; heading=2/3; retry=1"
    )
    assert values["target_status"] == "rejected; invalid_depth"
