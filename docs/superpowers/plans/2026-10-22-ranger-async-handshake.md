# Ranger asynchronous startup implementation plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan inline with test-driven development. Do not commit or switch branches.

**Goal:** Let either private-mode agent start first, wait without canceling the other, and execute only after both acknowledge the current task.

**Architecture:** Keep schema v1 and the existing five bridge topics. Separate peer message receipt from peer health; use WAITING_LOCAL / WAITING_PEER / IDLE before a task and SYNCING + reply=SYNCED for task acceptance. Preserve strict motion, READY/deposited and task identity gates.

**Tech Stack:** Python, dataclasses, existing ROS callbacks and AsyncVision futures; no new dependencies.

**Spec:** The user-approved design in this conversation: independent local readiness, nonblocking peer wait, bilateral task acknowledgment, fail-closed execution.

## Constraints

- Keep existing edits, branch, Domain0↔40 routing, private C mode and hardware parameters.
- No live bridge/agent startup, motion, Go2 connection or automatic commit.
- Empty-task startup waits recover automatically; active-task pauses never auto-resume.
- Old tasks/results cannot refresh or acknowledge the current task.
- Reply only on meaningful state transitions; periodic heartbeats must not cause reply storms.

## Review focus

- One agent starts 60 seconds late or restarts before any task.
- A healthy identity match with nav_ready=false remains live but unready.
- A stale task or stale acknowledgment cannot unlock EXECUTE.
- Loss of either agent or coordinator during execution still stops motion.
- Slow map preparation/rendering does not monopolize the ROS callback thread.

## Task 1: Waiting and task handshake

- [x] Add offline regressions for independent startup, health/liveness separation, fresh bilateral SYNCED, rejected early EXECUTE, timeout, restart and paused-task isolation; observe failures.
- [x] Update agent.py, coordinator.py, models.py, agent_node.py and the fake mission fixture. Handshake uses phase=SYNCING, task_id=T and reply=SYNCED; EXECUTE retains SEARCH_AB / SEARCH_C / DEPOSIT.
- [x] Run the handshake tests and existing private mission/strict acceptance regressions.

## Task 2: Callback responsiveness and operator contract

- [x] Add a pending-future regression for candidate preparation/rendering; observe failure.
- [x] Reuse AsyncVision for pure map preparation/rendering in cooperative exploration, retaining cancellation, freshness and identity validation.
- [x] Update startup instructions/scripts for waiting states, and write the exact Go2 interoperability contract and schema hashes.
- [x] Run relevant offline regressions and a scoped review; record any existing failures without broadening the change.

## Execution notes

- Baseline: branch ranger-reuse-go2-framework, HEAD 942c778d61c6300ab8562be360cb9c9efd1528cd, existing user edits preserved.
- Scope ruling: use existing workspace because the user explicitly prohibited branch switching and overwriting changes; no new worktree or commits.
- Interface ledger: Task 2 uses Task 1's waiting states and SYNCED behavior in operator text and callback tests.
- Ruling: a private new task must arrive with phase=SYNCING; ordinary old-task SYNC in PAUSED/SEARCH_AB cannot recruit a freshly restarted agent. WAITING_PEER is accepted by an explicit new start when both local states are fresh/healthy/stopped; bilateral task ACK proves the links before execution.
- Final review: scoped read-only reviewer found peer motion could revoke a completed handshake. Fixed with persisted ACK/commit state and two RED→GREEN delivery-order tests, retaining own stop and both sides' health/liveness checks.
- Additional RED→GREEN checks cover peer health loss immediately before phase commit, non-handshake old-task adoption, a single agent restart joining a new explicit task, and awaiting a previous canceled worker without blocking callbacks.
- Not checked by design: real DDS/hardware/Go2 deployment. Canceled background jobs are not forcibly terminated; their results are discarded and new pure preparation waits without blocking until the existing serial worker is available.
- Final validation: 238 passed, 1 skipped in 9.24s. The skip is the missing Go2-only SSH deployment script. Raw output `/tmp/ranger-async-handshake-20261022.log`; JUnit `/tmp/ranger-async-handshake-20261022.xml`. Strict arrival probes include 0.90m success, >1.00m rejection, failed navigation/stop/terminal rejection, missing executor and old-task/goal isolation.
- Offline `cooperative.sh ranger preflight` exited 0 with live_gates=[], motion_sent=false; all 33 installed Python package files match source. Shell syntax and git diff whitespace checks passed. No branch switch, commit, bridge/agent startup, Go2 connection or real motion was performed.

## Final contract implementation (2026-10-22)

User authorized implementing `docs/multi_agent/ans_async_startup.md` directly.
Scope: agent/coordinator state and freshness gates, model clean-state predicate,
existing worker digest checks and final motion validation, read-only preparation
sampler, startup scripts/docs, affected fixtures and offline regressions.
Preserve shared-coordinate behavior, hardware parameters and existing edits.

- [x] Canonical PREPARING/STANDBY/SYNCED, clean states and split receipt clocks.
- [x] Whole-task 600s budget including SYNCING; no 3s handshake cancellation.
- [x] Full digests yield to callbacks and final prerequisites are checked afterward.
- [x] Read-only, infinite three-frame preparation sampling and operator instructions.
- [x] Relevant offline tests, installed-source checks and scoped final review.

- Final contract regression RED→GREEN: canonical states and clean ACK rejection,
  delayed peer startup/health recovery, long handshake and total-budget expiry,
  current-task receipt isolation, recoverable identity mismatch before commit,
  execution mode changes and first-reason retention.
- Final scoped review found two important regressions. Fixed delayed old-task
  states clearing current ACK (both first-command delivery orders); fixed
  semantic_furniture motion-time asset detection via file signatures. Seven
  failing cases became green; full related offline suite follows.
- Read-only map monitoring reuses the existing serial worker for full digests;
  unchanged file metadata permits cached evidence while model work runs. Actual
  map_server filename and its <=3s evidence remain separate mandatory checks.
- Preparation sampler integration tests mock rclpy without creating DDS nodes;
  they cover >60s late peer, publisher duplication/source/QoS and Ctrl+C.

- Final final-contract validation: 371 passed, 3 native-ROS observer cases
  deselected (offline-only scope), 9.19s. `/tmp/ranger-final-contract.log`,
  `/tmp/ranger-final-contract.xml`, and exact argument list
  `/tmp/ranger-final-contract-command.txt` retain the evidence.
- Preflight exited0, 33/33 installed package Python files match source,
  live_gates=[], motion_sent=false, Domain0/lo ↔ Domain40/wlo1 unchanged.
  No real node startup, bridge startup, Go2 connection or real motion.
- The broad initial collection's environment/Go2-specific failures are recorded
  in final contract §9.1; no unrelated code/config was changed to silence them.
- Full map digests also revalidate periodically on the idle existing worker;
  metadata equality alone never exempts the next navigation from a full digest.

## Follow-up: independent full startup entry

User requested updating the full startup workflow. Removed Ranger's foreign-step
confirmations and foreground clean preparation sampler; keep only local RViz
pose confirmation, then launch local bridge/agent/displays. Go2's existing full
entry owns three-frame preparation sampling and the single task request. This
also avoids a Ranger sampler waiting forever once Go2 has auto-started.
Four old-flow failures observed before implementation, then startup/health/
sampler suite 89 passed. No live components, Go2 connection or motion started.
