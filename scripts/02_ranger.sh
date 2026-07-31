#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

echo "[硬件终端 2] 检查 can0"
can_output="$(ip -details link show can0 2>&1 || true)"
if ! grep -q '^[0-9].*can0:' <<<"$can_output"; then
  echo "错误：未找到 can0，请检查 USB-CAN 连接。" >&2
  exit 1
fi

if grep -q 'state DOWN' <<<"$can_output"; then
  echo "can0 当前为 DOWN，设置 500000 bit/s（需要 sudo 密码）"
  sudo ip link set can0 up type can bitrate 500000
elif grep -q 'state UP' <<<"$can_output"; then
  echo "can0 已经为 UP。"
else
  echo "错误：无法识别 can0 状态：" >&2
  echo "$can_output" >&2
  exit 1
fi

can_output="$(ip -details link show can0 2>&1 || true)"
if ! grep -q 'state UP' <<<"$can_output"; then
  echo "错误：can0 启动失败。" >&2
  exit 1
fi

echo "[硬件终端 2] 启动 Ranger；关闭底盘 odom TF，避免与 FAST_LIO 重复"
exec ros2 launch ranger_bringup ranger_mini_v3.launch.py \
  port_name:=can0 \
  publish_odom_tf:=false
