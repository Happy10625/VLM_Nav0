"""Ranger's 251f9fb local behavior, driven by the existing task owner.

The observer's timers remain disabled. Nav2Adapter owns motion identities and
execution-path monitoring; this class owns confirmation, recovery and arrival.
"""
import math
import time
from concurrent.futures import Future
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, Spin

from .models import WorkerResult
from .vlm_navigator import (
    VLMNavigator, DISARMED, API_ERROR, SUCCEEDED, FAILED, SENSOR_WAITING,
    SCANNING, SEARCHING, TARGET_CONFIRMING, TARGET_REOBSERVING,
    TARGET_ALIGNING, APPROACHING, APPROACH_STOPPING,
)


class RangerSingleRobotExecution(VLMNavigator):
    vlm_request_timeout = 8.0

    def easy_case_enabled(self):
        return False

    def get_parameter(self, name):
        if getattr(self, 'execution_active', False):
            if name == 'enabled':
                return SimpleNamespace(value=True)
            if name == 'target_description':
                return SimpleNamespace(value=self.execution_description)
        return super().get_parameter(name)

    def begin_execution(self, adapter, task_id, command_id, target_id, description):
        self.execution_adapter = adapter
        self.execution_active = False
        self.reset_task(SCANNING)
        self.execution_identity = (task_id, command_id)
        self.execution_target_id = target_id
        self.execution_description = description
        self.execution_active = True
        self.execution_emit = lambda *args, **kwargs: None
        self.execution_result = None
        self.execution_last_sequence = -1
        self.execution_recovering = False
        self.execution_descriptions = {target_id: description}
        self.execution_recovery_scan = None
        self.execution_scan_pose = None
        self.execution_on_observation = None
        self.execution_adapter.local_arrival_verified = False

    def end_execution(self):
        approach=getattr(self.execution_adapter,'approach',None)
        if approach is not None:approach.close()
        self.execution_active = False
        self.task_epoch += 1
        self.cancel_motion(publish_stop=True)
        self.set_state(DISARMED)

    def on_arrival_odom(self,message):
        approach=getattr(getattr(self,'execution_adapter',None),'approach',None)
        if approach is not None:
            approach.on_arrival_odom(message)
            return
        super().on_arrival_odom(message)

    def on_plan_response(self, future, token):
        if getattr(self.execution_adapter,'execution_paused',False):
            self.execution_adapter.paused_callbacks.append(lambda: self.on_plan_response(future,token))
            return
        super().on_plan_response(future,token)

    def on_plan_result(self, future, token):
        if getattr(self.execution_adapter,'execution_paused',False):
            self.execution_adapter.paused_callbacks.append(lambda: self.on_plan_result(future,token))
            return
        super().on_plan_result(future,token)

    def confirm_target(self, point):
        # Authorized exception: a confirmed reference is fixed until sensor recovery.
        if self.target_reference_position is None:
            super().confirm_target(point)

    def begin_scan(self, pose):
        super().begin_scan(pose)
        scan = self.execution_adapter.scan
        if scan is None and self.execution_recovering:
            self.execution_scan_pose = pose
            self.advance_recovery_scan()
            scan = self.execution_adapter.scan
        if scan is not None:
            scan.update(index=0, initial=pose[2], phase='SPIN', current_view_only=False,
                headings=[heading-pose[2] for heading in self.scan_headings])

    def advance_recovery_scan(self):
        pose = self.execution_scan_pose
        if pose is None or self.execution_adapter.motions or not self.execution_active:
            return
        self.execution_recovery_scan = self.execution_adapter.start_scan(
            self.execution_descriptions, self.execution_identity[1],
            stop_on_target=self.execution_target_id, task_id=self.execution_identity[0],
            command_id=self.execution_identity[1])
        self.execution_adapter.scan.update(index=0,initial=pose[2],phase='SPIN',current_view_only=False,
            headings=[heading-pose[2] for heading in self.scan_headings])
        self.execution_scan_pose = None

    def enter_sensor_wait(self, reason):
        if self.state in (DISARMED, API_ERROR, SUCCEEDED, FAILED):
            return
        first = self.state != SENSOR_WAITING
        super().enter_sensor_wait(reason)
        if first:
            self.execution_recovering = True
            adapter = self.execution_adapter
            adapter.vision.cancel()
            for goal_id in list(adapter.motions):
                adapter.cancel_navigation(goal_id)
            for handle in adapter.plans.values():
                if handle is not None:handle.cancel_goal_async()
            if adapter.scan is not None:adapter.scan['phase'] = 'LOCAL_RECOVERY'
            self.execution_emit('SENSOR_WAITING')

    def finish_approach_success(self):
        super().finish_approach_success()
        self.execution_adapter.local_arrival_verified = True

    def on_navigation_result(self, future, token, kind):
        if token != self.goal_token:return
        super().on_navigation_result(future, token, kind)
        if kind != 'approach':
            self.execution_emit('ARRIVED' if self.state != FAILED else 'NAV_FAILED')

    def prepare_stationary_observation(self):
        super().prepare_stationary_observation()
        scan = self.execution_adapter.scan
        if scan is not None:
            scan.update(phase='FRAME', after=self.scan_settle_until)

    def set_state(self, state):
        super().set_state(state)
        if state == APPROACH_STOPPING and getattr(self, 'execution_active', False):
            self.execution_emit('APPROACH_STOPPING')

    def send_navigation_goal(self, x, y, yaw, kind):
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose_stamped(x, y, yaw)
        self._send_execution_motion(self.navigator, goal, kind, (x, y, yaw))

    def send_spin(self, relative_yaw, kind='scan'):
        goal = Spin.Goal()
        goal.target_yaw = float(relative_yaw)
        allowance = max(0., float(self.p.spin_time_allowance))
        goal.time_allowance.sec = int(allowance)
        goal.time_allowance.nanosec = int((allowance-int(allowance))*1e9)
        self._send_execution_motion(self.spinner, goal, kind, None)

    def _send_execution_motion(self, client, goal, kind, pose):
        if not client.wait_for_server(timeout_sec=.5):
            self.fail_safe('NavigateToPose action server unavailable' if pose else 'Spin action server unavailable')
            return
        adapter = self.execution_adapter
        if adapter.motions:
            self.fail_safe('previous local motion is not terminal')
            return
        self.goal_token += 1
        token = self.goal_token
        self.goal_pending = True
        self.goal_kind, self.goal_pose = kind, pose
        self.goal_started = time.monotonic()
        task_id, command_id = self.execution_identity
        external_kind = ('DEPOSIT_NAV' if self.execution_target_id == 'C' else 'PICKUP_NAV') if kind == 'approach' else 'TARGET_PROBE' if pose else 'SCAN_SPIN'

        def accepted(handle, goal_id):
            if token != self.goal_token or not self.execution_active:
                adapter.cancel_navigation(goal_id)
                return
            self.goal_pending = False
            self.goal_handle = SimpleNamespace(cancel_goal_async=lambda: adapter.cancel_navigation(goal_id))
            if kind == 'approach' and self.state == APPROACH_STOPPING:
                self.request_approach_cancel()

        def completed(status, goal_id):
            result = Future()
            result.set_result(SimpleNamespace(status=status))
            self.on_navigation_result(result, token, kind)

        if pose:
            adapter._clear_path('new navigation')
            adapter.path_after_stamp=self.get_clock().now().nanoseconds*1e-9
        goal_id, _ = adapter._motion(client, goal, task_id, command_id, external_kind,
            list(pose) if pose else None, on_accept=accepted, on_result=completed)
        if pose:
            self.execution_emit('NAVIGATING', task_id=task_id, command_id=command_id,
                goal_id=goal_id, goal_kind=external_kind)

    def local_tick(self):
        if not getattr(self, 'execution_active', False):
            return
        if (getattr(self.execution_adapter,'approach',None) is not None
                or self.state in (APPROACHING,APPROACH_STOPPING)):
            return
        self.safety_tick()

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
        elif age > self.vlm_request_timeout:
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
            (self.goal_handle is not None or self.goal_pending or self.plan_pending)
            and now - self.goal_started > float(self.p.goal_timeout)
        ):
            self.fail_safe("Navigation or path-validation goal timed out")

