# Ranger SingleRobotApproach migration

Spec: user request in this session; source pppppst/VLM_Nav0@1acae1d56e4e43dac29514f2244d439b760520bf, receiver HEAD ab922276108c21f44612b7805e26fc0560300528 plus existing uncommitted work.

Use the source executor unchanged except parameter loading. Ranger reads its own robot.yaml; arrival odometry remains /fastlio/odom, outer chassis stop feedback remains /odom. Both entries already share explore_steps: replace only its Ranger approach branch. Preserve observation, confirmation, candidates, scanning, all hardware configuration and cooperative barriers. No commits, deployment or live motion.

- [x] Freeze actual reference methods and source differential cases; run baseline and red tests.
- [x] Copy SingleRobotApproach; adapt Ranger parameters, action callbacks, arrival odometry and cleanup. One action result consumer.
- [x] Wire shared exploration branch; retain health/resource checks with current-pose envelope only during approach; require approach evidence plus outer identity/health/stop/terminal before READY.
- [x] Add single entry shell wrapper; test cancellation, late callbacks, retries, independent/cooperative flows. Report source mappings, baseline/new failures and hardware gaps.

Existing work was copied to /tmp/ranger-migration-before before edits. Work remains on the requested branch. Read-only sources are cached in /tmp/ranger-source. Offline test commands source ROS and the existing overlay only; no ROS node/process launch is authorized.

Verification: focused differential/transport/legacy-observation/private tests pass; source class AST unchanged. Full baseline failure set retained. See delivery report for final counts. Final review found no introduced issues; independent callback health recheck additionally covered with a failing regression before the fix.
