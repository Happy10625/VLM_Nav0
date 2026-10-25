#!/usr/bin/env bash
# Attach by default. --start-navigation explicitly starts the local map/TF/Nav2 layer.
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/scripts/common.sh"
OVERLAY="$REPO/build/ranger-local/install/setup.bash"
if [[ ! -f "$OVERLAY" ]]; then
  echo "缺少新 overlay: $OVERLAY；先按 ranger_local_runbook.md 构建。" >&2
  exit 1
fi
set +u
source "$OVERLAY"
set -u
# Offline checks must never create ROS nodes or contact a running ROS graph.
for arg in "$@"; do
  if [[ "$arg" == --offline ]]; then
    exec ros2 run multi_agent_nav multi_agent_ranger_check "$@"
  fi
done
nodes="$(ros2 node list --no-daemon)"
if [[ "$nodes" =~ /vlm_nav($|[[:space:]])|/multi_agent_coordinator|/multi_agent_.*agent|/multi_agent_.*check_motion|/multi_agent_.*rgbd ]]; then
  echo "检测到现有上层任务拥有者；请先由其入口取消并确认停稳：" >&2
  echo "$nodes" >&2
  exit 1
fi
# Report duplicate publishers' usual node names without killing or restarting anything.
for name in /ranger_base /ranger_mini_v3 /fastlio_odom_adapter /base_to_camera /odom_to_fastlio_world; do
  count="$(awk -v n="$name" '$0==n {c++} END {print c+0}' <<< "$nodes")"
  if (( count > 1 )); then
    echo "重复底盘/TF节点 $name ($count)，请核实发布者。" >&2
    exit 1
  fi
done
if [[ "${1:-}" == --start-navigation ]]; then
  shift
  if grep -Fxq /velocity_smoother <<< "$nodes"; then
    echo "旧velocity_smoother仍在运行；请正常结束原Nav2，避免再次平滑Ranger候选指令。" >&2
    exit 1
  fi
  if [[ "$nodes" =~ /amcl|/map_server|/controller_server|/planner_server|/fastlio_odom_adapter|/base_to_camera|/odom_to_fastlio_world ]]; then
    echo "已有定位/Nav2/TF节点，拒绝重复启动；可使用默认附着方式。" >&2
    exit 1
  fi
  exec ros2 launch vlm_nav ranger_local.launch.py map:="$REPO/shared_maps/lab_ranger_20261006/lab.yaml" "$@"
fi
exec ros2 run multi_agent_nav multi_agent_ranger_check "$@"
