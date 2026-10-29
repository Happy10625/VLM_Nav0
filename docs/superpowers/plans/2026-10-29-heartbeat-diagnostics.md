# Heartbeat diagnostics implementation plan

> Execute inline under executing-plans and TDD; preserve existing uncommitted changes.

Goal: distinguish missing callbacks, rejected peer messages, and expired local heartbeat timers during the next experiment. This records evidence; it does not fix the historical timeout.

Design authorized in this conversation: reuse agent.log, keep a bounded recent receive summary, emit a timeout snapshot, offline tests only. Keep timeout values, state transitions, QoS and executor unchanged. No ROS startup, deployment, rebuild or robot motion.

Approach: keep 32 recent receive entries in AgentMachine; record local monotonic entry time, publisher stamp, robot/task identity, accept/reject reason and peer timer update. Snapshot the actual timeout check's now/peer_received/peer_transport_received/sync_received plus ages and threshold before stopping. AgentNode emits the snapshot once as JSON through the existing logger. Normal reception performs no logging or disk I/O. Use existing Python collections/json; no dependency or configuration changes.

Alternatives: logging every message adds unnecessary I/O; an external probe cannot observe agent filtering. Defer probes to a future现场 run.

Files: agent.py (bounded history and timeout snapshot), agent_node.py (entry time and one-time log), tests/test_heartbeat_diagnostics.py (offline regression).

Review focus: wrong robot, wrong task, invalid identity, missing timers, exact timeout boundary, immutable snapshot, bounded memory, repeated paused ticks, unchanged healthy/motion behavior. No saved endpoint QoS or arrival times can be retroactively reconstructed.

- [x] Add failing offline tests for accepted/rejected messages, immutable bounded history, coordinator/peer timeout snapshot and one-time node warning.
- [x] Implement diagnostics only; preserve return values and stop calls.
- [x] Run focused tests plus existing agent/handshake tests, then the offline project suite via scripts/test_ranger_local.sh.
- [x] Review changes against saved current-workspace baseline, send Go2 evidence and activation limitations; leave changes uncommitted.

Ruling: use the existing working tree because this task extends its uncommitted runtime implementation and .git is read-only in this environment. Save pre-edit copies under build/heartbeat-investigation-20261029/baseline; do not alter existing changes or commit them.

Verification: initial new tests 4 failed as expected, focused 74 passed, final expanded related suite 125 passed (exit 0). Full-suite pytest printed 1083 passed / 28 failed / 1 skipped, then worker teardown lingered; interrupted the session (exit 130). Full failures retained in build/heartbeat-investigation-20261029/full-suite-failures.txt; do not claim full-suite success. Read-only reviewer approved; its suggested node wrong-robot regression was added.

Activation: installed agent.py and agent_node.py are symlinks to the edited sources. Existing processes are not restarted; the next normal agent process loads this version. No hardware test, network change, threshold change, rebuild or commit. Full-suite ROS-context tests could not validate transport in the restricted sandbox.
