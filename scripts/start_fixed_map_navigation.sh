#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/.." && pwd)"

map_yaml=""
amcl_scan_topic="/scan"
target_description="chair"
target_seen=false

usage() {
  cat <<EOF
用法：
  $(basename "$0") [OPTIONS] [TARGET_DESCRIPTION]

依次复用现有脚本启动 Ranger 硬件、固定地图 Nav2、VLM_Nav、RViz、
状态和诊断终端。VLM_Nav 始终以 enabled=false 启动，不会自动运动。

选项：
  --map MAP.yaml           地图 YAML
                           必需；路径会透传为 system.launch.py 的 map 参数
  --amcl-scan-topic TOPIC  AMCL LaserScan，默认 /scan
  -h, --help               显示帮助

示例：
  $(basename "$0") --map /absolute/path/lab.yaml "chair"
  $(basename "$0") --map /absolute/path/office.yaml "red chair"
EOF
}

while (( $# > 0 )); do
  case "$1" in
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
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      echo "错误：未知参数 $1" >&2
      usage >&2
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

if [[ -z "$amcl_scan_topic" ]]; then
  echo "错误：--amcl-scan-topic 不得为空。" >&2
  exit 2
fi
if [[ -z "$map_yaml" ]]; then
  echo "错误：必须使用 --map MAP.yaml 指定固定地图。" >&2
  exit 2
fi
if [[ ! -r "$map_yaml" ]]; then
  echo "错误：地图 YAML 不可读：${map_yaml}" >&2
  exit 2
fi
map_yaml="$(realpath -e "$map_yaml")"

# Validate YAML, its image path, and image decoding before starting hardware.
PYTHONPATH="${project_dir}:${PYTHONPATH:-}" python3 -c \
  'import sys; from vlm_nav.fixed_map import validate_map_yaml; validate_map_yaml(sys.argv[1])' \
  "$map_yaml"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "错误：未找到 gnome-terminal，无法启动硬件和导航终端。" >&2
  exit 1
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "错误：未检测到图形显示环境，无法启动 RViz。" >&2
  exit 1
fi
if [[ -z "${DASHSCOPE_API_KEY:-}" && -z "${OPENAI_API_KEY:-}" ]]; then
  echo "错误：请先设置 DASHSCOPE_API_KEY 或 OPENAI_API_KEY。" >&2
  exit 1
fi
if [[ -n "${DASHSCOPE_API_KEY:-}" ]] \
  && [[ -z "${DASHSCOPE_BASE_URL:-}" ]] \
  && [[ -z "${DASHSCOPE_WORKSPACE_ID:-}" ]]; then
  echo "错误：请设置 DASHSCOPE_WORKSPACE_ID 或 DASHSCOPE_BASE_URL。" >&2
  exit 1
fi

echo "[1/2] 启动并检查 Ranger、L515、MID-360 和 FAST-LIO"
"${script_dir}/start_hardware.sh"

echo
echo "[2/2] 启动固定地图 Nav2 和 VLM_Nav（enabled=false）"
echo "地图：${map_yaml}"
echo "目标：${target_description}"
exec "${script_dir}/start_navigation_validation.sh" \
  --localization-mode amcl \
  --map "$map_yaml" \
  --amcl-scan-topic "$amcl_scan_topic" \
  "$target_description"
