# Ranger distance retry recovery

User request: when Ranger remains outside the target distance after its movement retry, observe again and navigate toward the same task target.

Implement inline, preserving the existing dirty workspace. Reuse the existing RESUME_SEARCH branch; do not change the frozen per-attempt arrival controller, distance thresholds, or stop confirmation. Only its specific terminal, stationary, fresh-odometry distance failure is recoverable. Other cancellation/navigation/health failures remain fatal.

Approach: RangerAdapter.approach_steps recognizes that exact per-attempt failure, resets the observer to SCANNING (invalidating stale perception and old target coordinates but preserving execution task/command/target identity), emits recovery evidence and returns RESUME_SEARCH. The shared exploration generator schedules current-view observation before selecting a waypoint or retrying navigation. Existing task deadline and motion/heartbeat gates remain in force.

Files: adapters/ranger.py, go2_exploration.py, test/test_ranger_distance_reobserve.py. Reuse real controlled transport/perception fixtures. No config, protocol, Go2 behavior, deployment or hardware changes.

- [x] Add failing offline tests: actual retry exhaustion recovers and clears stale target; full observation/navigation loop preserves task and target for B/C, including initial-observation and scan-detection entry; unrelated failures stay fatal.
- [x] Add minimal recovery branch and current-view rescan flag.
- [x] Run affected offline suites (no runtime ROS tests); verify cancellation/health/deadline behavior and review saved baseline diff.
- [x] Document verification and leave modifications uncommitted.

Full-suite baseline from this session: 1083 passed, 28 failed, 1 skipped; unrelated missing Go2 assets/dependencies, ROS sandbox failures and existing assertions are recorded under build/heartbeat-investigation-20261029. Do not launch ROS-context tests to repeat these known environment failures.

Verification: initial tests 4 failed / 2 passed on the expected task-ending failure; final affected offline suite 265 passed (exit 0). Tests cover reobservation then fresh-coordinate navigation for B and C under unchanged task/command identities; cancellation, health loss, original deadline, and unrelated native action failure remain terminal. No ROS node or hardware startup. Existing frozen controller parity tests remain unchanged and pass.
