# Verbatim method slices from pppppst/VLM_Nav0@32eab6244ec4b483807f8ed68e8667f5495874bb
# Full source SHA256: 78fce28de80325c64c5324ae1fee4a2c8c1d30dc0a8c8f84044c01b60294f9c3
class VLMNavigator:
    @staticmethod
    def elapsed_since(timestamp, now):
        return math.inf if timestamp <= 0.0 else max(0.0, now - timestamp)


    def safety_tick(self):
        if (
            self.get_parameter("enabled").value
            and bool(getattr(self.p, "require_external_safety_gates", False))
            and not self.external_safety_gates_healthy()
        ):
            self.get_logger().error("external VLM safety gate became stale or false")
            self.set_parameters([Parameter("enabled", value=False)])
            return
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
            (self.goal_handle is not None or self.goal_pending or self.plan_pending)
            and now - self.goal_started > float(self.p.goal_timeout)
        ):
            self.fail_safe("Navigation or path-validation goal timed out")


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


