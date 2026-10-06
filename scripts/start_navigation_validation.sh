#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

wait_for_topic_type() {
  local topic="$1"
  local expected_type="$2"
  local timeout_seconds="${3:-60}"
  local end=$((SECONDS + timeout_seconds))
  local actual_type=""

  printf '等待话题 %s ' "$topic"
  while (( SECONDS < end )); do
    actual_type="$(timeout 3 ros2 topic type "$topic" 2>/dev/null || true)"
    if [[ "$actual_type" == "$expected_type" ]]; then
      echo "就绪"
      return 0
    fi
    printf '.'
    sleep 1
  done

  echo
  echo "错误：${topic} 在 ${timeout_seconds}s 内未以 ${expected_type} 发布。" >&2
  return 1
}

run_component() {
  local component="$1"
  local target_description="${2:-chair}"
  local localization_mode="${3:-slam}"
  local map_yaml="${4:-}"
  local amcl_scan_topic="${5:-/scan}"

  source "${script_dir}/common.sh"
  unset ALL_PROXY all_proxy

  case "$component" in
    system)
      echo "[导航验证] 启动 ${localization_mode} 定位、Nav2 和 VLM_Nav（enabled=false）"
      exec ros2 launch vlm_nav "${VLM_NAV_SYSTEM_LAUNCH:-system.launch.py}" \
        enabled:=false \
        target_description:="$target_description" \
        localization_mode:="$localization_mode" \
        map:="$map_yaml" \
        amcl_scan_topic:="$amcl_scan_topic"
      ;;
    rviz)
      exec "${script_dir}/start_rviz.sh"
      ;;
    state)
      echo "[导航验证] 监控 /vlm_nav/state"
      wait_for_topic_type /vlm_nav/state std_msgs/msg/String
      exec ros2 topic echo /vlm_nav/state
      ;;
    diagnostics)
      echo "[导航验证] 监控 /vlm_nav/diagnostics"
      wait_for_topic_type \
        /vlm_nav/diagnostics \
        diagnostic_msgs/msg/DiagnosticArray
      exec ros2 topic echo /vlm_nav/diagnostics
      ;;
    *)
      echo "错误：未知组件 ${component}" >&2
      exit 2
      ;;
  esac
}

if [[ "${1:-}" == "--component" ]]; then
  if (( $# < 2 )); then
    echo "错误：--component 后缺少组件名称。" >&2
    exit 2
  fi
  run_component "$2" "${3:-chair}" "${4:-slam}" "${5:-}" "${6:-/scan}"
fi

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
用法：
  start_navigation_validation.sh [OPTIONS] [TARGET_DESCRIPTION]

选项：
  --localization-mode slam|amcl  定位模式，默认 slam
  --map MAP.yaml                 AMCL 模式必需的地图 YAML
  --amcl-scan-topic TOPIC        AMCL LaserScan，默认 /scan

分别打开四个终端：
  1. 建图、Nav2 和 VLM_Nav（始终以 enabled=false 启动）
  2. RViz
  3. /vlm_nav/state
  4. /vlm_nav/diagnostics

示例：
  ./VLM_Nav/scripts/start_navigation_validation.sh
  ./VLM_Nav/scripts/start_navigation_validation.sh "red chair"
  ./VLM_Nav/scripts/start_navigation_validation.sh \
    --localization-mode amcl --map /absolute/path/lab.yaml "red chair"
EOF
  exit 0
fi

localization_mode="slam"
map_yaml=""
amcl_scan_topic="/scan"
target_description="chair"
target_seen=false
while (( $# > 0 )); do
  case "$1" in
    --localization-mode)
      [[ $# -ge 2 ]] || { echo "错误：--localization-mode 缺少值。" >&2; exit 2; }
      localization_mode="$2"
      shift 2
      ;;
    --map)
      [[ $# -ge 2 ]] || { echo "错误：--map 缺少值。" >&2; exit 2; }
      map_yaml="$2"
      shift 2
      ;;
    --amcl-scan-topic)
      [[ $# -ge 2 ]] || { echo "错误：--amcl-scan-topic 缺少值。" >&2; exit 2; }
      amcl_scan_topic="$2"
      shift 2
      ;;
    --*)
      echo "错误：未知参数 $1" >&2
      exit 2
      ;;
    *)
      if [[ "$target_seen" == true ]]; then
        echo "错误：只接受一个可选的目标描述参数。" >&2
        exit 2
      fi
      target_description="$1"
      target_seen=true
      shift
      ;;
  esac
done

if [[ "$localization_mode" != "slam" && "$localization_mode" != "amcl" ]]; then
  echo "错误：--localization-mode 必须是 slam 或 amcl。" >&2
  exit 2
fi
if [[ "$localization_mode" == "amcl" ]]; then
  if [[ -z "$map_yaml" ]]; then
    echo "错误：AMCL 模式必须提供 --map MAP.yaml。" >&2
    exit 2
  fi
  if [[ ! -r "$map_yaml" ]]; then
    echo "错误：地图 YAML 不可读：${map_yaml}" >&2
    exit 2
  fi
  map_yaml="$(realpath -e "$map_yaml")"
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "错误：未找到 gnome-terminal，无法打开导航验证终端。" >&2
  exit 1
fi

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "错误：未检测到图形显示环境，无法启动 RViz。" >&2
  exit 1
fi

source "${script_dir}/common.sh"

vlm_nav_prefix="$(ros2 pkg prefix vlm_nav 2>/dev/null || true)"
if [[ -z "$vlm_nav_prefix" ]] \
  || ! ros2 pkg executables vlm_nav 2>/dev/null \
    | awk '$2 == "obstacle_cloud_filter" { found=1 } END { exit !found }'; then
  echo "错误：当前 ROS overlay 中缺少 vlm_nav/obstacle_cloud_filter。" >&2
  echo "当前 vlm_nav 前缀：${vlm_nav_prefix:-未找到}" >&2
  echo "请重新构建 VLM_Nav，并在重启前 source 对应 install/setup.bash。" >&2
  exit 1
fi
echo "使用 vlm_nav：${vlm_nav_prefix}"

if timeout 4 ros2 node list 2>/dev/null | grep -Fxq /vlm_nav; then
  echo "错误：/vlm_nav 已经运行。请先关闭旧的 system.launch.py。" >&2
  exit 1
fi

if [[ -z "${DASHSCOPE_API_KEY:-}" && -z "${OPENAI_API_KEY:-}" ]]; then
  echo "错误：DASHSCOPE_API_KEY 未设置；请先在当前终端导出百炼 API Key。" >&2
  exit 1
fi

if [[ -n "${DASHSCOPE_API_KEY:-}" ]] \
  && [[ -z "${DASHSCOPE_BASE_URL:-}" ]] \
  && [[ -z "${DASHSCOPE_WORKSPACE_ID:-}" ]]; then
  echo "错误：请设置 DASHSCOPE_WORKSPACE_ID 或 DASHSCOPE_BASE_URL。" >&2
  exit 1
fi

# HTTP_PROXY/HTTPS_PROXY cover the configured gateway. The inherited
# socks:// ALL_PROXY value is unsupported by the installed HTTP client.
unset ALL_PROXY all_proxy

open_component() {
  local title="$1"
  local component="$2"
  gnome-terminal --window --title="$title" -- \
    bash "${script_dir}/terminal_runner.sh" \
    "${script_dir}/start_navigation_validation.sh" \
    --component "$component" "$target_description" \
    "$localization_mode" "$map_yaml" "$amcl_scan_topic"
}

echo "${VLM_NAV_VALIDATION_LABEL:-启动导航验证}，目标：${target_description}，定位：${localization_mode}"
if [[ "$localization_mode" == "amcl" ]]; then
  echo "固定地图：${map_yaml}；AMCL scan：${amcl_scan_topic}"
fi
echo "VLM_Nav 保持 enabled=false，不会自动执行导航。"

open_component "VLM Navigation System" system
open_component "VLM RViz" rviz
open_component "VLM State" state
open_component "VLM Diagnostics" diagnostics

echo "已打开系统、RViz、状态和诊断终端。"
