"""Approach branches extracted from VLMNavigator at
32eab6244ec4b483807f8ed68e8667f5495874bb; Ranger approach has no rolling-radius watchdog.
"""
import math
import time
from pathlib import Path
from types import SimpleNamespace

import yaml
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from .models import Pose2D

APPROACHING = 'APPROACHING'
APPROACH_STOPPING = 'APPROACH_STOPPING'
SUCCEEDED = 'SUCCEEDED'
FAILED = 'FAILED'

# Frozen merged Go2 values are checked, not silently substituted for configuration.
BASELINE = dict(target_success_radius=.81, approach_cancel_radius=.89,
    arrival_retry_limit=1, arrival_linear_speed_tolerance=.03,
    arrival_angular_speed_tolerance=.05, arrival_stationary_samples=3,
    arrival_odom_max_age=.5, arrival_cancel_timeout=3., arrival_stop_timeout=5.,
    arrival_odom_topic='/odometry', max_travel_radius=3., goal_timeout=120.,
    navigation_tick_period=.5, blocked_goal_seconds=15.)


def load_parameters(files, robot_id='go2'):
    if robot_id not in ('go2', 'ranger'):raise ValueError('unknown approach robot')
    if not files:raise ValueError('arrival configuration files required')
    merged = {}
    for file in files:
        merged.update(yaml.safe_load(Path(file).read_text())['vlm_nav']['ros__parameters'])
    values = {}
    for key, baseline in BASELINE.items():
        value = merged.get(key)
        valid = (type(value) is str if isinstance(baseline, str) else
                 type(value) is int if type(baseline) is int else
                 type(value) in (int, float) and math.isfinite(value))
        if not valid or (robot_id == 'go2' and value != baseline):
            raise ValueError(f'{key}: configured {value!r}, frozen baseline {baseline!r}')
        if robot_id == 'ranger' and (not value if isinstance(value, str) else value < 0 if key == 'arrival_retry_limit' else value <= 0):
            raise ValueError(f'{key}: invalid Ranger configuration {value!r}')
        values[key] = value
    return SimpleNamespace(**values)


class SingleRobotApproach:
    def __init__(self, adapter, target, task_id, command_id, kind, parameters,
                 record, clock=time.monotonic):
        self.adapter, self.p, self.clock = adapter, parameters, clock
        self.task_id, self.command_id, self.goal_kind_wire = task_id, command_id, kind
        self.record = record
        self.state = APPROACHING
        self.reason = ''
        self.target_reference_position = (target.x, target.y)
        self.goal_id = ''
        self.goal_token = self.plan_token = 0
        self.goal_pending = self.plan_pending = False
        self.goal_handle = self.plan_handle = None
        self.goal_kind = self.goal_pose = self.active_motion_origin = None
        self.blocked_goals = {}
        self.arrival_retry_count = 0
        self.arrival_last_odom_received = 0.
        self.reset_approach_stop()
        self.path_planner = adapter.node.path_planner
        self.navigator = adapter.node.navigator
        self.make_pose_stamped = adapter.node.make_pose_stamped
        self.next_navigation = self.next_safety = self.clock()

    def emit(self, state, **fields):
        try:
            self.record(dict(state=state, task_id=self.task_id, command_id=self.command_id,
                             **dict(dict(goal_id=self.goal_id), **fields)))
        except Exception as error:
            self.get_logger().warning('Approach diagnostic write failed: '+str(error))

    def get_logger(self):
        return self.adapter.node.get_logger()

    def robot_pose(self):
        pose = self.adapter.get_health().pose
        return (pose.x, pose.y, pose.yaw)

    def publish_stop(self):
        self.adapter.node.publish_stop()

    def set_state(self, state):
        self.emit('APPROACH_STATE', approach_state=state,
                  distance=self.target_reference_distance(self.robot_pose()),
                  arrival_retry=self.arrival_retry_count,
                  stationary_samples=self.arrival_stationary_count)
        self.state = state

    def fail_safe(self, reason):
        self.reason = reason
        self.emit('APPROACH_FAILURE_DETAIL', reason=reason, arrival_retry=self.arrival_retry_count,
                  stationary_samples=self.arrival_stationary_count,
                  distance=self.target_reference_distance(self.robot_pose()))
        self.close()
        self.set_state(FAILED)
        self.emit('APPROACH_FAILED', reason=reason)

    def close(self):
        if getattr(self, '_closed', False):return
        self._closed = True
        self.cancel_motion(publish_stop=True)
        self.state = FAILED

    def tick(self):
        now = self.clock()
        if now >= self.next_safety:
            self.next_safety = now + .2
            self.safety_tick()
        if now >= self.next_navigation:
            self.next_navigation = now + max(.1, float(self.p.navigation_tick_period))
            self.navigation_tick()
        if self.state == APPROACH_STOPPING:
            self.emit('APPROACH_STOP_STATUS', action_terminal=self.arrival_action_terminal,
                      cancel_acknowledged=self.arrival_cancel_acknowledged,
                      stationary_samples=self.arrival_stationary_count,
                      odom_age=self.elapsed_since(self.arrival_last_odom_received, now),
                      distance=self.target_reference_distance(self.robot_pose()),
                      arrival_retry=self.arrival_retry_count)

    def navigation_tick(self):
        if self.state != APPROACHING:
            return
        pose = self.robot_pose()
        if pose is None or self.goal_handle is not None or self.goal_pending or self.plan_pending:
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
            self.plan_then_navigate([goal[:3]], "approach")
            return



    def safety_tick(self):
        if self.state not in (APPROACHING, APPROACH_STOPPING):
            return
        pose = self.robot_pose()
        if pose is None:
            return
        now = self.clock()
        if self.state == APPROACHING and self.target_reference_position is not None:
            if self.target_reference_distance(pose) <= self.approach_stop_radius():
                self.begin_approach_stop("distance_contract_reached")
                return
        if self.state == APPROACH_STOPPING:
            self.advance_approach_stop(now, pose)
            return
        if (
            (self.goal_handle is not None or self.goal_pending or self.plan_pending)
            and now - self.goal_started > float(self.p.goal_timeout)
        ):
            self.fail_safe("Navigation or path-validation goal timed out")

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
        self.arrival_cancel_requested_at = self.clock()
        self.emit('APPROACH_CANCEL_REQUESTED')
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
        self.emit('APPROACH_CANCEL_ACKNOWLEDGED')

    def begin_approach_stop(self, reason, action_terminal=False):
        if self.state == APPROACH_STOPPING:
            if action_terminal and not self.arrival_action_terminal:
                self.arrival_action_terminal = True
                self.arrival_action_terminal_at = self.clock()
            return
        self.emit('APPROACH_STOP_BEGIN', reason=reason)
        self.set_state(APPROACH_STOPPING)
        self.arrival_stop_started = self.clock()
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
            self.arrival_action_terminal_at = self.clock()
        self.get_logger().info(f"Approach stop confirmation started: {reason}")

    def advance_approach_stop(self, now=None, pose=None):
        if self.state != APPROACH_STOPPING:
            return
        now = self.clock() if now is None else float(now)
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
        self.emit('APPROACH_VERIFIED', distance=self.target_reference_distance(self.robot_pose()),
                  stationary_samples=self.arrival_stationary_count,
                  odom_age=self.elapsed_since(self.arrival_last_odom_received,self.clock()),
                  arrival_retry=self.arrival_retry_count)
        self.publish_stop()
        self.goal_handle = None
        self.goal_pending = False
        self.goal_kind = None
        self.goal_pose = None
        self.active_motion_origin = None
        self.reset_approach_stop(reset_retry=False)
        self.set_state(SUCCEEDED)

    def on_arrival_odom(self, message):
        twist = message.twist.twist
        self.arrival_last_odom_received = self.clock()
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

    @staticmethod
    def goal_key(pose):
        return round(float(pose[0]), 1), round(float(pose[1]), 1)

    @staticmethod
    def elapsed_since(timestamp, now):
        return math.inf if timestamp <= 0.0 else max(0.0, now - timestamp)

    def plan_then_navigate(self, candidates, kind):
        now = self.clock()
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
            # A replaced task still owns its pending planning request until acceptance.
            try:
                handle = future.result()
                if handle.accepted:handle.cancel_goal_async()
            except Exception as error:
                self.get_logger().warning('Late plan cancellation failed: '+str(error))
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

    def reject_current_plan(self, reason, token):
        if self.last_plan_rejection_reason == "none":
            self.last_plan_rejection_reason = str(reason)
        self.get_logger().warn(reason)
        if self.current_plan_pose is not None:
            self.blocked_goals[self.goal_key(self.current_plan_pose)] = (
                self.clock() + float(self.p.blocked_goal_seconds)
            )
        self.plan_handle = None
        self.request_next_plan(token)

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
                self.last_plan_commit_length = self.last_plan_path_length
                self.last_plan_radius_clipped = False
                self.rolling_goal_is_final = True
                self.active_motion_origin = self.plan_window_origin
                self.emit("APPROACH_PATH", requested_pose=requested_pose,
                          path_endpoint=points[-1], path=points, native_status=int(wrapped.status))
                self.plan_pending = False
                self.plan_handle = None
                self.plan_candidates = []
                self.plan_kind = None
                self.send_navigation_goal(*requested_pose, kind)
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

    def send_navigation_goal(self, x, y, yaw, kind):
        if not self.navigator.wait_for_server(timeout_sec=0.5):
            if kind == "frontier":
                self.reject_selected_frontier(
                    "NavigateToPose action server unavailable"
                )
            else:
                self.fail_safe("NavigateToPose action server unavailable")
            return
        self.goal_token += 1
        token = self.goal_token
        self.goal_pending = True
        self.goal_kind = kind
        self.goal_pose = (float(x), float(y), float(yaw))
        self.goal_started = self.clock()
        try:
            self.goal_id = self.adapter.start_approach_navigation(self, Pose2D(x, y, yaw), token)
        except Exception as error:
            self.fail_safe(f"{kind} goal request failed: {error}")
            return
        self.emit("APPROACH_NAVIGATE", goal_pose=(x, y, yaw), goal_id=self.goal_id)


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
                    self.arrival_action_terminal_at = self.clock()
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
                    self.clock() + float(self.p.blocked_goal_seconds)
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

