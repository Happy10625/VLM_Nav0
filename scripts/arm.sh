#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"${script_dir}/check_system.sh"
read -r -p "确认实体急停可用、场地封闭且有人监护。输入大写 ARM： " answer
if [[ "$answer" != "ARM" ]]; then
  echo "未确认，保持停车。"
  exit 1
fi
ros2 param set /vlm_nav enabled true
echo "VLM 导航已使能；紧急情况优先使用实体急停。"
