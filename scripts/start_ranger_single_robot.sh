#!/usr/bin/env bash
# Task entry only; existing Ranger hardware/navigation must already be running.
set -eo pipefail
ranger_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help ]]; then
  echo '用法：bash scripts/start_ranger_single_robot.sh --allow-motion [--target-id B] [--target 描述] [--config-dir 目录] [--timeout 秒]'
  echo '复用 Ranger 原配置及已运行的导航；不启动硬件、bridge 或 coordinator。默认目标 B，未传 --allow-motion 不允许运动。'
  exit 0
fi
source "$ranger_repo/RangerVLM_env.bash" local
ranger_config="${RANGER_CONFIG:-$ranger_repo/cooperative_ws/src/multi_agent_nav/config}"
ranger_args=("$@")
for ((ranger_i=0; ranger_i<${#ranger_args[@]}; ranger_i++)); do
  case "${ranger_args[ranger_i]}" in
    --config-dir) ranger_config="${ranger_args[ranger_i+1]:-}" ;;
    --config-dir=*) ranger_config="${ranger_args[ranger_i]#*=}" ;;
  esac
done
export PYTHONPATH="$ranger_repo/cooperative_ws/src/multi_agent_nav:$ranger_repo${PYTHONPATH:+:$PYTHONPATH}"
ranger_timeout="$(python3 - "$ranger_config/scene.yaml" <<'PY'
import sys, yaml
from pathlib import Path
print(yaml.safe_load(Path(sys.argv[1]).read_text()).get('task_timeout', 600.))
PY
)"
exec python3 -m multi_agent_nav.ranger_check --config-dir "$ranger_config" \
  --action explore --approach-target --target-id B --timeout "$ranger_timeout" "$@"
