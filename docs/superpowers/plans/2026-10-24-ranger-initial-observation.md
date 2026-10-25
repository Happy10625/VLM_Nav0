# Ranger initial observation implementation plan

> Superseded for the initial scan by `2026-10-24-ranger-current-view.md`: the starting pose now uses one current-view image without rotation.

> Execute inline using executing-plans; retain existing workspace changes, no commits or hardware restarts.

**Goal / spec:** For cooperative Ranger, treat the starting pose of a new task as its first observation point: use existing eight-view scanning before asking the VLM to choose and navigate to further observation points. A located target uses existing approach logic immediately.

**Architecture:** AgentRunner requests an initial scan on Ranger's first task command (sequence 1). explore_steps inserts a synthetic starting viewpoint, bypasses only its navigation, and reuses the existing scan/result/recording path. The starting point is recorded separately from generated candidates and cannot consume one. Later commands and cached target approaches retain their flow. No new states or configuration switches are required.

**Constraints:** Existing health, cancel, heartbeat, full-rotation-space checks and uncertain-depth handling remain. If the starting pose cannot safely rotate, stop with an explicit failure. Do not move elsewhere before that required observation. Reuse dated scan evidence and current VLM selection history. Scope is cooperative Ranger; Go2 and standalone checks retain their behavior.

- [x] Add failing integration regressions using AgentRunner and real generated-candidate preparation with synthetic hardware/VLM: scan before selection/navigation, located/uncertain target, unsafe start, cancellation, later command and new task.
- [x] Wire first-command flag; reuse scan loop for a starting viewpoint without a Nav2 transit; update README.
- [x] Run affected and full regressions, compare existing failure baseline, obtain one read-only final review and record results.

**Result:** 169 affected tests passed. Full suite: 810 passed, 25 baseline failures plus one DDS lifecycle response timeout; that single test passed its focused rerun. Independent review found no Critical/Important issues. Verification and named failures: `build/ranger-control/initial-observation-verification.txt`. Installed Python source links verified; no hardware or runtime restart.
