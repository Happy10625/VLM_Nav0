# Ranger conditional turnaround recovery

> Execute inline with the executing-plans skill. Preserve the dirty checkout and existing changes; no commits or hardware restarts.

**Goal:** A valid navigation path followed by no legal local trajectory or failed movement progress attempts one footprint-checked 180-degree turn before retrying navigation; failed turn or failed retry ends navigation.

**Architecture:** The installed Humble FollowPath result has no error code. Extend the existing Ranger DWB controller and native SimpleProgressChecker to expose a typed failure reason through the controller's dotted `ranger_recovery.reason` parameter. A small BT service node reads it only after FollowPath fails. A Ranger-only PipelineSequence keeps 1 Hz replanning running while a contextual RecoveryNode executes a pi Spin and retries FollowPath once. Reuse native Spin and CostmapTopicCollisionChecker with a full remaining-angle sweep before each command.

**Scope:** This implements the user's latest conditional recovery request. Command silence, invalid/stale odometry, cancellation and health loss still stop immediately; no blind motion during a communications failure. During the handover from failed FollowPath to Spin, zero commands are expected; the overall NavigateToPose goal remains active rather than being declared failed immediately.

**Version evidence:** Nav2 1.1.20 `controller_server.cpp`, `dwb_local_planner.cpp`, `spin.cpp`, plus installed headers/action definitions. Native Spin's first collision loop uses accumulated angle and can skip the initial sweep, so increasing its horizon alone is insufficient.

## Files and checks

- [x] Add regression coverage for typed no-trajectory/progress classification, stale reason reset and nonrecoverable failures; implement the minimal controller/progress extensions and plugin registration.
- [x] Add the BT predicate and Ranger tree/config/launch wiring; exercise the native BT navigator against synthetic action servers for recovery success, rejection, retry failure, cancellation, and continuing planning during the turn.
- [x] Extend native Spin with full-sweep footprint checking; test a rectangular corner collision and a clear 180-degree turn using synthetic ROS inputs.
- [x] Build the existing overlay, run affected and full regression suites, perform independent review, and document restart steps/results.

## Operational limits

- One recovery per NavigateToPose execution; pi radians, 30 s turn allowance, existing 0.25 rad/s behavior speed and unchanged 0.05 m arrival tolerance.
- Arc commands still obey the previously implemented moving angular limit below 0.3 rad/s.
- A blocked full turn, unavailable failure diagnosis/TF/collision input, canceled goal or exhausted retry does not start additional recovery motion.
- Keep the existing coordinator/agent heartbeat, task deadline and costmap health checks. Do not turn path reception failures into permission to move.

## Verification

Built and installed into the existing Ranger overlay. C++ 10 passed; native navigator/Spin integration 10 passed; full Python suite 801 passed, 25 pre-existing failures, 1 skipped, with no new failing test IDs. Evidence: `build/ranger-control/recovery-verification.txt`. Independent review identified the inherited 0.3 s failure-tolerance stale-cause window; the Ranger overlay now sets `failure_tolerance: 0.0` and the merged-launch test asserts it. No hardware motion or process restart was performed.
