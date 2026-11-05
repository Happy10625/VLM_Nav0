---
name: ranger_find_target
description: Use when the Ranger host supplies one fixed target for its existing search-and-approach task.
---

Use `start_find_target` once with the exact target string supplied by the host,
including whitespace. The host owns the target ID, configuration and task identity.
Do not translate, summarize, expand or substitute the target.

The existing executor performs exploration, ABC recognition, confirmation,
depth recovery, approach retries, stopping and acceptance. Internal retries and
RESUME_SEARCH belong to that executor; never restart the overall task on failure.

After start, use only `get_task_status(task_id)` or `cancel_task(task_id)`.
Cancellation acknowledgement is not confirmation that the robot stopped.
Do not use shell, code execution, files, delegation, another goal or another
session to control the robot. These instructions are usage guidance; the host
enforces the one-start rule and real-backend capability gate in code.

Report executor_success, stop_status and evidence_status separately, using the
host's fields. REPORTED_OK means the original flags and required records passed,
not an independent certification of all background evidence. UNKNOWN and
STOP_UNCONFIRMED must remain explicit. Never replace the host's result with a
self-reported success. If no task was started, report AGENT_NO_EXECUTION.
