#!/usr/bin/env bash
# Isolated software regression: no hardware startup, task bridge or provider calls.
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/scripts/common.sh"
if [[ -f "$REPO/build/ranger-local/install/setup.bash" ]]; then
  set +u
  source "$REPO/build/ranger-local/install/setup.bash"
  set -u
fi
cd "$REPO"
export PYTHONPATH="$REPO/cooperative_ws/src/multi_agent_nav:$REPO:${PYTHONPATH:-}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/ranger-local-test-logs
exec /usr/bin/python3 -m pytest "$@"
