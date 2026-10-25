#!/usr/bin/env bash
# Reuse the existing hardware/Nav2/RViz entries; never start a motion task.
set -euo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo '用法：start_ranger_stack.sh [RVIZ_CONFIG]'
  echo '依次启动硬件、Ranger Nav2、轻量RViz及只读诊断终端；已有完整实例时复用。'
  echo '使用当前协同环境和 COOP_CONFIG 中的 Ranger 地图；不启动协作任务。'
  exit 0
fi
(( $# <= 1 )) || { echo '[失败] 只接受一个可选的 RVIZ_CONFIG 路径。' >&2; exit 2; }
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$repo/Cooperative_env.bash" ranger "${COOP_CONFIG:-}"
navigation_files="$(python3 - "$COOP_CONFIG" <<'PY'
import sys
from multi_agent_nav.config import load_config, map_file_info
robot = load_config(sys.argv[1])['robot_ranger']
from vlm_nav.keepout import configured_mask
mask=configured_mask(robot)
print(map_file_info(robot['map_path'])['yaml_path'])
print(str(mask.path) if mask is not None else '')
print(mask.digest if mask is not None else '')
PY
)"
mapfile -t navigation_files <<< "$navigation_files"
map_path="${navigation_files[0]}"
keepout_args=()
if [[ -n "${navigation_files[1]:-}" ]]; then
  keepout_args=("keepout_mask_yaml:=${navigation_files[1]}" "keepout_mask_digest:=${navigation_files[2]}")
fi
echo "[配置] $COOP_CONFIG"
echo "[地图] $map_path"
rviz_config="${1:-/home/isee601/松灵小车2/固定地图轻量验证.rviz}"
[[ -r "$rviz_config" ]] || { echo "[失败] RViz 配置不可读：$rviz_config" >&2; exit 1; }
command -v gnome-terminal >/dev/null || { echo '[失败] 需要 gnome-terminal 打开各硬件终端。' >&2; exit 1; }
[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || { echo '[失败] 未检测到图形显示环境。' >&2; exit 1; }
mkdir -p "$RANGER_LOCAL_OUTPUT"
exec 9>"$RANGER_LOCAL_OUTPUT/stack-start.lock"
flock -n 9 || { echo '[失败] 另一个统一启动流程正在运行。' >&2; exit 1; }

nav_nodes=(/map_server /amcl /controller_server /planner_server /bt_navigator
  /behavior_server /smoother_server /waypoint_follower /ranger_command_guard
  /lifecycle_manager_navigation /lifecycle_manager_localization
  /fastlio_odom_adapter /odom_to_fastlio_world /base_to_camera /obstacle_cloud_filter)
if (( ${#keepout_args[@]} )); then
  nav_nodes+=(/keepout_mask_server /costmap_filter_info_server /lifecycle_manager_keepout)
fi
read_nodes() {
  nodes="$(timeout 8 ros2 node list --no-daemon --spin-time 2)" || {
    echo '[失败] ROS节点发现无响应；未继续启动。' >&2; return 1;
  }
}
nav_count() {
  local name count
  present=0
  for name in "${nav_nodes[@]}"; do
    count="$(awk -v name="$name" '$0==name {n++} END {print n+0}' <<< "$nodes")"
    (( count <= 1 )) || { echo "[失败] 重复节点：$name；请先处理原实例。" >&2; return 1; }
    present=$((present + count))
  done
}
open_terminal() {
  local title="$1"
  shift
  gnome-terminal --window --title="$title" -- bash -c '
    source "$1/Cooperative_env.bash" ranger "$2" || exit
    shift 2
    exec bash "$REPO/scripts/terminal_runner.sh" "$@"
  ' bash "$REPO" "$COOP_CONFIG" "$@" 9>&-
}

read_nodes
if grep -Fxq /velocity_smoother <<< "$nodes"; then
  echo '[失败] Ranger速度控制不能复用旧velocity_smoother；请从原R2终端正常结束旧Nav2后再启动。' >&2
  exit 1
fi
if [[ "$nodes" =~ /vlm_nav($|[[:space:]])|/slam_toolbox($|[[:space:]])|/multi_agent_coordinator|/multi_agent_.*agent|/multi_agent_.*check_motion|/multi_agent_.*rgbd ]]; then
  echo '[失败] 已有旧定位或上层任务；请从其原入口结束任务并确认停稳。' >&2
  exit 1
fi
nav_count
if (( present > 0 && present < ${#nav_nodes[@]} )); then
  echo '[失败] 已有部分定位/Nav2/TF节点；请检查原R2终端，不叠加第二套启动。' >&2
  exit 1
fi

echo '[启动 1/4] 相机 → Ranger → MID-360 → FAST-LIO'
bash "$REPO/scripts/start_hardware.sh" 9>&-
if (( present == ${#nav_nodes[@]} )); then
  echo '[复用] 已有完整定位/Nav2/TF节点；请用健康检查确认地图及状态。'
else
  echo '[启动 2/4] Ranger 固定地图定位与Nav2'
  open_terminal 'Ranger R2 Nav2' "$REPO/scripts/start_ranger_local.sh" --start-navigation "${keepout_args[@]}" "map:=$map_path"
  deadline=$((SECONDS + 60))
  while true; do
    read_nodes
    nav_count
    (( present == ${#nav_nodes[@]} )) && break
    if (( SECONDS >= deadline )); then
      echo '[失败] Nav2关键节点未在60秒内齐全；检查R2终端。' >&2
      exit 1
    fi
    sleep 1
  done
fi

# AMCL may need an initial pose before all Nav2 lifecycles can become active.
read_nodes
if grep -Fxq /rviz2 <<< "$nodes"; then
  echo '[复用] 已有RViz窗口，请确认使用轻量配置。'
else
  echo '[启动 3/4] 轻量RViz'
  open_terminal 'Ranger R3 轻量RViz' "$REPO/scripts/start_rviz.sh" "$rviz_config"
fi
if grep -Fxq /ranger_diagnostic <<< "$nodes"; then
  echo '[复用] 已有R5只读诊断终端。'
else
  echo '[启动 4/4] R5只读诊断终端（状态、动作、速度及结果）'
  open_terminal 'Ranger R5 诊断' "$REPO/scripts/monitor_ranger.sh"
fi
echo '[完成] 启动命令已提交；各组件终端请保持打开。'
echo '在RViz中用 2D Pose Estimate 设置真实初始位姿；已有正确定位时无需重设。'
echo '之后在当前终端运行中文只读检查：'
echo '  bash "$REPO/scripts/check_ranger.sh"'
echo '启动完成不等于健康检查通过；本入口没有发送导航目标或使能运动。'
