#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

echo "[硬件终端 4] 启动 FAST_LIO_ROS2"
exec ros2 launch fast_lio mapping.launch.py \
  config_file:=mid360.yaml \
  rviz:=false
