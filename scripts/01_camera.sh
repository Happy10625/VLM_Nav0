#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REALSENSE_CONFIG="${SCRIPT_DIR}/../config/realsense.yaml"

if [[ ! -f "${REALSENSE_CONFIG}" ]]; then
  echo "错误：未找到 RealSense 参数文件 ${REALSENSE_CONFIG}" >&2
  exit 1
fi

echo "[硬件终端 1] 启动 RealSense RGB-D 相机"
exec ros2 launch realsense2_camera rs_launch.py \
  config_file:="'${REALSENSE_CONFIG}'" \
  align_depth.enable:=true \
  enable_sync:=true \
  rgb_camera.profile:=1280x720x30 \
  depth_module.profile:=640x480x30
