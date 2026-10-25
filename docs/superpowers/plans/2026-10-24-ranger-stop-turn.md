> Superseded: the later user request removes translation/rotation exclusivity. Current behavior allows mixed commands with abs(angular.z) < 0.3 rad/s whenever linear.x is nonzero (nominal moving target cap 0.29). Mode hysteresis and zero-axis transitions were removed. Collision checks, cancellation and watchdog stopping remain enabled; no forced 180-degree recovery that bypasses these protections was implemented. See README for current wiring. The following is the historical implementation record.

# Ranger stop / turn implementation plan

> **For agentic workers:** Use superpowers:executing-plans for the already-authorized implementation. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Execute mutually exclusive straight and in-place rotation commands, with hysteresis, braking transitions and collision scoring of the commands actually sent.

**Architecture:** Retain DWB and its scoring loop. A Ranger trajectory generator owns all acceleration limits and predicts each candidate from measured starting velocity; a thin DWB extension supplies the path heading and scores heading convergence during rotation. Controller and behavior outputs go to `/cmd_vel_nav`; a Ranger watchdog forwards valid exclusive commands to `/cmd_vel`, forwards zero immediately and publishes zero after 1 s without commands. Ordinary `velocity_smoother` is not launched on this Ranger profile.

**Tech Stack:** Installed Nav2 1.1.20 / ROS 2 Humble, DWB extension interfaces, existing ObstacleFootprintCritic, C++17 and existing Python ROS runtime.

**Spec:** The user's requirements in this conversation, especially the five acceptance scenarios. This plan records the concrete connection and does not require a further permission round.

## Global constraints

- Only the trajectory generator applies acceleration/deceleration. The watchdog never smooths or replays a nonzero command.
- Candidates are straight, rotation, or braking the old axis. There is a zero-command boundary and measured stop check before another axis starts.
- `traj.velocity` is the first generated control command; `traj.poses` predicts that command and subsequent acceleration / brake steps. No post-score projection of an arc.
- Mode survives ordinary `setPlan`; outer NavigateToPose goal identity / terminal status and controller lifecycle reset it.
- Keep PositionGoalChecker at 0.05 m and the existing task arrival contract unchanged.
- Reuse ObstacleFootprintCritic first, with the actual 0.72 x 0.50 m rectangle and 0.03 m padding. Sample translation and rotation at most 0.01 m / 0.01 rad, tightened to half a costmap cell at the outer corner when needed.
- Remove RotateToGoal / Oscillation from this Ranger profile: final yaw is not required by PositionGoalChecker; the heading latch handles direction mode and must not have necessary rotations rejected by stale sign locks.
- No robot motion, process restarts, external-workspace edits, or unsolicited commits.

## Review focus

- Measured velocity still on the old axis after a zero command: keep the new axis inhibited.
- Braking or angular sign reversal: never cross through zero into a different axis / direction in the same control command.
- Replanning while heading is between thresholds: preserve the mode.
- Obstacles touched only by a rotating corner at an intermediate prediction sample: reject the candidate.
- Missing / invalid commands, lifecycle shutdown and late results: zero stop, never replay saved nonzero velocity.

## Task 1: DWB candidate generator and controller

**Files:** `include/vlm_nav/ranger_dwb_controller.hpp`, `src/ranger_dwb_controller.cpp`, `ranger_controller_plugins.xml`, `test/test_ranger_controller.cpp`, `CMakeLists.txt`, `package.xml`.

**Interfaces:** `RangerTrajectoryGenerator` implements the installed DWB trajectory generator interface; `RangerDWBController` extends DWB without replacing its scoring loop. `setPath(pose, path)`, `resetMotion()`, and `commit(command)` manage per-goal motion state.

- [x] Write and run failing C++ tests for exclusive candidates, both brake transitions with delayed odometry, hysteresis through replan/reset, consistent rollout, legal rotations and rectangular footprint collision.
- [x] Implement the two extensions, register plugin exports, and compile against installed Humble libraries.
- [x] Run C++ tests against real DWB critics and synthetic costmaps, without hardware.

## Task 2: Explicit command routing and watchdog

**Files:** `vlm_nav/ranger_command_guard.py`, `scripts/ranger_command_guard`, `launch/navigation.launch.py`, `config/nav2_ranger_local.yaml`, `scripts/start_ranger_stack.sh`, Ranger observer stop topic, and their tests.

**Interfaces:** guard input `/cmd_vel_nav`, output `/cmd_vel`, zero immediately / command age > 1.0 s. Ranger Node list excludes `velocity_smoother` and includes `ranger_command_guard`; generic Nav2 launch remains unchanged.

- [x] Write and run failing routing / watchdog tests.
- [x] Implement pass-through validity / exclusivity checks, wall-clock timeout, shutdown zero, and controller lifecycle stop; route native behaviors and Ranger software stops through the same guard.
- [x] Evaluate launch actions and run isolated ROS topic tests that prove final output exclusivity and stopping.

## Task 3: Verification and operational instructions

- [x] Document the exact graph, thresholds (initial enter 0.35 rad / exit 0.12 rad, 0.4 m lookahead), preserved limits and unchanged arrival tolerance.
- [x] Build in the existing Ranger overlay, run the targeted C++ / Python tests and the complete existing test command. Report unrelated failures by name.
- [x] Review the final change against all five acceptance scenarios. Do not restart hardware; live low-speed calibration remains separate.

## Progress / rulings

- Work in the existing dirty checkout to preserve the user's current implementation. No Git reset, worktree copy of an older committed version, or commit is authorized/needed.
- The watchdog keeps the prior 1 s command timeout, using a monotonic clock. Emergency zero and timeout zero override normal acceleration limits.
- Review ruling: native Spin/BackUp keep their own existing behavior algorithms (they are not downstream smoothers of FollowPath). All behavior commands pass the same guard: an axis change first sends zero, and the other axis is inhibited until fresh chassis `/odom` confirms stop. The guard never ramps or generates nonzero commands.
- Review fixes: explicitly inactive controller state is latched until ACTIVE; SIGTERM shares the graceful zero/ack exit path; angular reversal checks both measured velocity and last committed command.
- Guard odometry age is at most 0.5 s; stopped thresholds are 0.01 m/s translation and 0.02 rad/s rotation. Native behaviors and path tracking share the guard, so a recovery cannot bypass these checks.
- First regression run: 7 C++ tests and 8 Python tests passed; full Python run had the same 26 pre-existing failures as the previous naming change (777 passed, 1 skipped). Review regressions were added before the fix pass; the delayed-odometry reversal reproduced a direct +0.015 to -0.015 command.

- Added native FollowPath action acceptance with synthetic TF and chassis odometry: reached the original 0.05 m position tolerance, canceled a rotation, rejected late velocity after deactivation, and stopped after the test controller process died. The final `/cmd_vel` history remained exclusive. Native behavior acceleration remains inside the native behavior, never applied a second time after a FollowPath candidate.
- Added startup rejection for a leftover ordinary `velocity_smoother`, including the direct Ranger navigation entry, so the supported startup cannot accidentally run two output chains.
- Final verification: 9 C++ and 13 Python/ROS acceptance tests passed; full Python suite: 783 passed, 26 failed, 1 skipped. The exact 26 failed IDs match the earlier isolated baseline. Build, shell syntax and whitespace checks passed. Logs and exact failure IDs: `build/ranger-control/verification.txt`, `acceptance-tests.log`, `full-pytest.log`.
- Final boundary regression: a nonfinite current robot pose reproduced motion-candidate generation; `setPath` now rejects it before heading or distance computation. No physical motion or live process restart was performed.
