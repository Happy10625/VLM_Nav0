#!/usr/bin/env bash
# Uses the Ranger environment recorded in 3.通讯验收.md; copy the shared Python file too.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for arg in "$@"; do
  if [[ "$arg" == --help || "$arg" == -h ]]; then
    exec /usr/bin/python3 "$script_dir/check_multi_agent_comm.py" --side ranger "$@"
  fi
done
export COMM_ENV_FILE="${COMM_ENV_FILE:-/home/isee601/ws/VLM_Nav/RangerVLM_env.bash}"
set +u
source "$COMM_ENV_FILE" local || exit $?
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${RANGER_CONFIG:-}/cyclonedds_ranger_local.xml}"
exec "${COOPERATIVE_RUNTIME_PYTHON:-/usr/bin/python3}" "$script_dir/check_multi_agent_comm.py" --side ranger "$@"
