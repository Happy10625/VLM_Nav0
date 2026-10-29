# Independent map validation worker

User authorized fixing the proven normal-stop cancellation of map validation. Keep the current dirty workspace and preserve earlier changes.

Root cause: AgentNode uses adapter.vision for map hashes; normal stop cancels the current vision reply. The next map poll interprets CancelledError as failed map evidence and pauses both robots.

Approach: reuse AsyncVision with a separate AgentNode-owned map_worker, same timeout as the former shared vision worker and one worker thread. Poll/submit map validation only through it; close it on node destruction. Keep the existing periodic check, map/file signature rules, service-path freshness and error handling unchanged. No new configuration, abstraction or dependency. Ignoring cancellation would hide missing evidence, so is not the chosen fix.

Scope: agent_node.py and offline tests; update three map-monitor fixtures to supply the independent worker. Fake/live constructor wiring test supplies vision timeout and checks distinct workers. No robot restart, deployment, network or message protocol changes.

- [x] Add failing regression: normal hold/stop/discard leaves map validation alive, busy vision does not starve map checks, actual file changes/mismatches/errors/timeouts still revoke health, destroy closes worker.
- [x] Implement independent worker lifecycle and update connected fixtures.
- [x] Run related offline suites and review task-only diff against saved baseline.
- [x] Report results and activation; leave work uncommitted.

Verification: seven new regression cases failed before implementation; focused suite 21 passed; expanded offline suite 271 passed in 13.60s (exit 0). Task-only review found no blockers. Logs, saved baseline and implementation.patch are in build/map-validation-fix-20261029/. Existing installed agent_node.py resolves to the edited source, so the next agent start loads the change. No running process was restarted or updated.

Remaining limit: AsyncVision timeout invalidates health but does not forcibly terminate an already-running file read; this behavior is unchanged. No hardware or complete repository suite verification in this fix.

Known unrelated full-suite failures from this session are preserved under build/heartbeat-investigation-20261029; do not repeat ROS-context tests or start hardware to bypass environment failures.
