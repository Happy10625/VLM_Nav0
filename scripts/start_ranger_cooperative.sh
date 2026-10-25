#!/usr/bin/env bash
# Start local components independently; Go2 owns task preparation sampling and start.
set -euo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo '用法：bash scripts/start_ranger_cooperative.sh'
  echo 'Ranger：preflight → 硬件/定位确认 → 健康检查 → 任务桥/agent → state/peer。'
  echo '仅定位核对需输入步骤号4；其余本机步骤自动继续，不等待Go2。'
  echo '可与Go2完整入口先后启动；Ranger不请求start，Go2采样通过后请求一次。'
  echo '自定义配置先 source Cooperative_env.bash ranger /实际配置目录。'
  exit 0
fi
(( $# == 0 )) || { echo '不接受位置参数；配置通过 COOP_CONFIG 指定。' >&2; exit 2; }
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$repo/Cooperative_env.bash" ranger "${COOP_CONFIG:-}"
command -v gnome-terminal >/dev/null || { echo '[失败] 需要 gnome-terminal。' >&2; exit 1; }
[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || { echo '[失败] 请在本机图形桌面终端执行。' >&2; exit 1; }
mkdir -p "$RANGER_LOCAL_OUTPUT"
exec 8>"$RANGER_LOCAL_OUTPUT/cooperative-start.lock"
flock -n 8 || { echo '[失败] 另一个协同启动流程正在运行。' >&2; exit 1; }
log_root="${ACCEPTANCE_RUN:-$RANGER_LOCAL_OUTPUT}"
run_dir="$(mktemp -d "$log_root/cooperative-startup-$(date +%Y%m%d_%H%M%S)-XXXXXX")"
exec > >(tee "$run_dir/startup.log" 8>&-) 2>&1
echo "[配置] $COOP_CONFIG"
echo "[日志] $run_dir"
echo '本入口只启动Ranger本机组件，不等待Go2；任务由Go2准备采样通过后请求一次。'
trap 'echo "[中断] 不再启动后续组件；已打开的组件保持运行，任务暂停使用原入口。"; exit 130' INT

confirm_step() {
  local step="$1" answer
  echo
  echo "$2"
  while true; do
    if ! IFS= read -r -p "完成上述核对后输入步骤号 $step 继续（Ctrl+C 中止）：" answer; then
      echo '[停止] 未取得步骤确认，不再启动后续组件；已有组件保持运行。'
      exit 1
    fi
    if [[ "$answer" == "$step" ]]; then
      echo "[确认 $step] 操作者确认完成。"
      return
    fi
    echo "输入不符：须输入 $step，不接受空行或其他步骤号。"
  done
}

read_nodes() {
  nodes="$(timeout 8 ros2 node list --no-daemon --spin-time 2)" || {
    echo '[失败] ROS节点发现失败，不再继续。' >&2; return 1;
  }
}

count_node() {
  node_count="$(awk -v name="/$1" '$0==name || index($0,name "_")==1 {n++} END {print n+0}' <<< "$nodes")"
  if (( node_count > 1 )); then
    echo "[失败] 节点 $1 有 $node_count 个实例，请检查原终端。" >&2
    exit 1
  fi
}

open_terminal() {
  local title="$1" action="$2"
  gnome-terminal --window --title="$title" -- bash -c '
    source "$1/Cooperative_env.bash" ranger "$2" || exit
    log_file="$3"; action="$4"
    set -o pipefail
    bash "$REPO/scripts/terminal_runner.sh" "$REPO/scripts/cooperative.sh" ranger "$action" 2>&1 | tee -a "$log_file"
  ' bash "$REPO" "$COOP_CONFIG" "$run_dir/$action.log" "$action" 8>&-
}

ensure_node() {
  local name="$1" action="$2" title="$3" deadline
  read_nodes
  count_node "$name"
  if (( node_count == 1 )); then
    echo "[复用] $name；请确认原终端使用本轮配置。"
    return
  fi
  open_terminal "$title" "$action"
  deadline=$((SECONDS + 45))
  while true; do
    read_nodes
    count_node "$name"
    (( node_count == 1 )) && return
    if (( SECONDS >= deadline )); then
      echo "[失败] $name 未在45秒内出现，检查 $action 终端和日志。" >&2
      exit 1
    fi
    sleep 1
  done
}

echo '[步骤 3] Ranger 离线预检'
bash "$REPO/scripts/cooperative.sh" ranger preflight 8>&- | tee "$run_dir/preflight.json"
bridge_name="$(python3 - "$COOP_BRIDGE_YAML" <<'PY'
import sys,yaml
print(yaml.safe_load(open(sys.argv[1]))['name'])
PY
)"
read_nodes
if grep -Eq '^/(vlm_nav|multi_agent_coordinator|multi_agent_.*agent|multi_agent_.*rgbd|multi_agent_.*check_motion)$' <<< "$nodes"; then
  echo '[失败] 已有任务/观察器。先确认两机停稳，从原终端正常退出旧 agent/coordinator，再运行本入口。' >&2
  exit 1
fi
count_node "$bridge_name"
echo '本机预检通过；两端使用最终契约中已核对的地图身份、消息定义和场景，各自保留本机地图。'

echo '[步骤 4] 启动或复用 Ranger 硬件、Nav2、RViz及诊断'
bash "$REPO/scripts/start_ranger_stack.sh" 8>&-
confirm_step 4 '在 Ranger RViz 核对实际地图和位置，必要时用 2D Pose Estimate 初始化定位。若启用场地Keepout，同时核对本机中心位于掩膜白区。Go2 按其本机顺序独立准备，可晚于 Ranger；此处只确认 Ranger 本机，已有正确定位无需重设。'
bash "$REPO/scripts/check_ranger.sh" --config-dir "$COOP_CONFIG" 8>&- | tee "$run_dir/health.log"

echo '[步骤 6.2] 启动或复用本机任务桥，不等待Go2'
ensure_node "$bridge_name" bridge 'Ranger R5 任务桥'
echo '[步骤 6.3] 启动本机agent，独立准备并等待对端'
ensure_node multi_agent_ranger_agent agent 'Ranger R6 Agent'
open_terminal 'Ranger R0 本机状态' state
open_terminal 'Ranger R0 对端状态' peer

echo '[完成] Ranger本机组件已启动；保持各组件终端运行，agent持续准备并等待对端。'
echo 'PREPARING：本机尚未就绪；WAITING_PEER：本机就绪正在等对端；STANDBY：已看到合格对端。'
echo 'Go2完整入口负责双方各3条干净新鲜状态采样，并自动start一次；不重复start。'
echo '人工分组件路径：Go2启动唯一coordinator后，在Go2执行 cooperative.sh go2 wait，再显式start一次。'
echo '本机主脚本退出不会停止agent；不再前台等待准备采样，避免Go2已进入SYNCING后采样永远等待。'
echo 'start受理后先SYNCING，双方run_state=SYNCED且reply=SYNCED后才派工；握手与执行共用600秒预算。'
echo 'Ranger本机启动完成不等于双机静止通信或READY验收通过。'
echo '需要暂停任务时在 Go2 执行：bash "$REPO/scripts/cooperative.sh" go2 pause'
echo "[日志] $run_dir（现有桥的历史输出仍在其原终端）"
