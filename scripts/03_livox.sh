#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

echo "[硬件终端 3] 启动 Livox MID-360"
exec ros2 launch livox_ros_driver2 msg_MID360_launch.py
