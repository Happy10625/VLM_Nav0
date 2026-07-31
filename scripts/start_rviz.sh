#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/common.sh"

usage() {
  cat <<'EOF'
用法：
  start_rviz.sh [RVIZ_CONFIG] [RVIZ参数...]

未指定 RVIZ_CONFIG 时，使用 VLM_Nav 配置并显示 VLM 目标、路径和文本。
示例：
  ./VLM_Nav/scripts/start_rviz.sh
  ./VLM_Nav/scripts/start_rviz.sh /path/to/custom.rviz
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if ! command -v rviz2 >/dev/null 2>&1; then
  echo "错误：未找到 rviz2，请安装 ros-humble-rviz2。" >&2
  exit 1
fi

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "错误：未检测到图形显示环境；本地运行或启用 SSH X11 转发后再试。" >&2
  exit 1
fi

if (( $# > 0 )); then
  config_file="$1"
  shift
else
  if ! vlm_share="$(ros2 pkg prefix --share vlm_nav 2>/dev/null)"; then
    echo "错误：未找到 vlm_nav，无法定位默认 RViz 配置。" >&2
    exit 1
  fi
  config_file="${vlm_share}/config/vlm_nav.rviz"
fi

if [[ ! -r "$config_file" ]]; then
  echo "错误：RViz 配置不存在或不可读：${config_file}" >&2
  exit 1
fi

echo "启动 RViz 验证（Fixed Frame: map）"
echo "配置：${config_file}"
echo "重点检查：相机 RGB、/map、/scan、TF、Nav2 路径、VLM 目标/路径和标注图像。"

exec ros2 run rviz2 rviz2 -d "$config_file" "$@"
