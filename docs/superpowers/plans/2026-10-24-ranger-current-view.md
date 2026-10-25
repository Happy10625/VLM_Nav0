# Ranger current-view initial observation

Execute inline; preserve unrelated changes, do not restart hardware or running agents.

User requirement: replace initial rotating scan with exactly one recognition of the current view. Reuse START bookkeeping, current task evidence and existing target/next-candidate flow. Later viewpoint scans retain their configured headings.

Implementation: initial-point start_scan requests current_view_only, enters FRAME directly, waits for a fresh image while stopped, infers once and terminates without Spin. Initial result completeness expects one view. Skip only the initial rotation/parking-space gate; movement and subsequent scanning retain their gates. No shared scan-angle configuration mutation.

- [x] Demonstrate failing regressions for no-spin single-view behavior, blocked rotation space and one-view completion.
- [x] Update adapter contract/implementation/fake, exploration caller/result handling, README and existing regression fixtures.
- [x] Verify affected/full suites, compare known baseline failures and perform one independent read-only review.

This supersedes the initial eight-view scan in 2026-10-24-ranger-initial-observation.md.

Verification: 169 affected tests passed, freshness tests 2 passed; full regression 810 passed, 25 baseline failures and one synthetic DDS lifecycle response timeout. Independent review found no Critical/Important issues. See `build/ranger-control/current-view-verification.txt` and focused DDS rerun log. No hardware restart or motion.
