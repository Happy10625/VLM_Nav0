#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
用法：
  start_easy_case_validation.sh [TARGET_DESCRIPTION]

启动 8 方向扫描、三帧确认和空场低速直达测试。此模式的 Nav2
不加载任何静态、障碍或 voxel 代价地图层，不具备避障能力。

示例：
  ./VLM_Nav/scripts/start_easy_case_validation.sh "chair"
EOF
  exit 0
fi

cat <<'EOF'
警告：EASY CASE 模式不会避障。
只允许在完全清空、封闭且物理隔离楼梯/坑洞/平台边缘的场地使用，
并确保实体急停可用且全程有人监护。
EOF

export VLM_NAV_SYSTEM_LAUNCH="easy_case.launch.py"
export VLM_NAV_VALIDATION_LABEL="启动 VLM 椅子直达 Easy Case"
exec "${script_dir}/start_navigation_validation.sh" "$@"
