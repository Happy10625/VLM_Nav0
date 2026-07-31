#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/common.sh"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "错误：未找到 gnome-terminal，请分别运行 01_camera.sh～04_fastlio.sh。" >&2
  exit 1
fi

publisher_count() {
  local output
  output="$(timeout 4 ros2 topic info "$1" 2>/dev/null || true)"
  awk '/Publisher count:/ {print $3; found=1; exit} END {if (!found) print 0}' <<<"$output"
}

wait_for_publisher() {
  local topic="$1" timeout_seconds="$2" description="$3"
  local end=$((SECONDS + timeout_seconds))
  printf '等待 %s ' "$description"
  while (( SECONDS < end )); do
    if (( $(publisher_count "$topic") > 0 )); then
      echo "通过"
      return 0
    fi
    printf '.'
    sleep 1
  done
  echo
  echo "错误：${description} 在 ${timeout_seconds}s 内没有发布，请查看对应终端。" >&2
  return 1
}

require_single_publisher() {
  local topic="$1" description="$2" count
  count="$(publisher_count "$topic")"
  if [[ "$count" != "1" ]]; then
    echo "错误：${description} ${topic} 有 ${count} 个发布者，要求恰好为 1。" >&2
    exit 1
  fi
}

open_terminal() {
  local title="$1" script="$2"
  echo "打开：${title}"
  gnome-terminal --window --title="$title" -- \
    bash "${script_dir}/terminal_runner.sh" "$script"
}

start_if_missing() {
  local topic="$1" title="$2" script="$3"
  local count
  count="$(publisher_count "$topic")"
  if (( count > 1 )); then
    echo "错误：${topic} 已有 ${count} 个发布者，请先关闭重复节点。" >&2
    exit 1
  elif (( count == 1 )); then
    echo "跳过 ${title}：已经运行"
  else
    open_terminal "$title" "$script"
  fi
}

echo "VLM_Nav 硬件严格顺序启动器"
echo "本脚本只启动相机、底盘驱动、雷达和定位，不会使能小车运动。"

start_if_missing \
  /camera/color/image_raw \
  "VLM-01 RealSense" \
  "${script_dir}/01_camera.sh"
wait_for_publisher /camera/color/image_raw 40 "RealSense RGB"
wait_for_publisher /camera/aligned_depth_to_color/image_raw 40 "RealSense 对齐深度"
wait_for_publisher /camera/color/camera_info 20 "RealSense CameraInfo"
require_single_publisher /camera/color/image_raw "相机"
require_single_publisher /camera/aligned_depth_to_color/image_raw "相机"

odom_count="$(publisher_count /odom)"
if (( odom_count > 1 )); then
  echo "错误：/odom 有 ${odom_count} 个发布者，请关闭重复 Ranger 节点。" >&2
  exit 1
elif (( odom_count == 1 )); then
  echo "跳过 VLM-02 Ranger：已经运行"
else
  if ! ip -details link show can0 >/dev/null 2>&1; then
    echo "错误：未找到 can0，请检查 USB-CAN 连接。" >&2
    exit 1
  fi
  open_terminal "VLM-02 Ranger" "${script_dir}/02_ranger.sh"
fi
wait_for_publisher /odom 60 "Ranger /odom"
require_single_publisher /odom "Ranger"
ranger_tf="$(ros2 param get /ranger_base_node publish_odom_tf 2>/dev/null || true)"
if ! grep -q 'False' <<<"$ranger_tf"; then
  echo "错误：Ranger publish_odom_tf 必须为 false。" >&2
  exit 1
fi

start_if_missing \
  /livox/lidar \
  "VLM-03 Livox" \
  "${script_dir}/03_livox.sh"
wait_for_publisher /livox/lidar 30 "Livox 点云"
wait_for_publisher /livox/imu 20 "Livox IMU"
require_single_publisher /livox/lidar "Livox"
require_single_publisher /livox/imu "Livox"

start_if_missing \
  /Odometry \
  "VLM-04 FAST_LIO" \
  "${script_dir}/04_fastlio.sh"
wait_for_publisher /Odometry 70 "FAST_LIO /Odometry"
wait_for_publisher /cloud_registered_body 30 "FAST_LIO body 点云"
require_single_publisher /Odometry "FAST_LIO"
require_single_publisher /cloud_registered_body "FAST_LIO"

fastlio_position="$(
  timeout 5 ros2 topic echo /Odometry --once --field pose.pose.position 2>/dev/null || true
)"
if ! awk '
  /x:/ {x=$2; hx=1}
  /y:/ {y=$2; hy=1}
  /z:/ {z=$2; hz=1}
  END {exit !(hx && hy && hz && x>-10 && x<10 && y>-10 && y<10 && z>-10 && z<10)}
' <<<"$fastlio_position"; then
  echo "错误：FAST_LIO 初始位置缺失或超出 ±10 m，可能已经发散：" >&2
  echo "$fastlio_position" >&2
  exit 1
fi

echo
echo "相机、Ranger、Livox 和 FAST_LIO 已就绪。"
echo "下一步启动 VLM_Nav（仍保持 enabled=false）："
echo "  ros2 launch vlm_nav system.launch.py enabled:=false"
