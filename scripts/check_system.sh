#!/usr/bin/env bash
set -uo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/common.sh"

failures=0
pass() { printf '\033[32m[通过]\033[0m %s\n' "$1"; }
fail() { printf '\033[31m[失败]\033[0m %s\n' "$1"; failures=$((failures + 1)); }

check_tf() {
  local parent="$1" child="$2" output="" attempt
  # A newly started ROS graph can need several seconds for DDS discovery and
  # for SLAM Toolbox to publish its first map->odom transform.  Retry instead
  # of treating that startup delay as a broken TF tree.
  for attempt in 1 2; do
    output="$(timeout 8 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 || true)"
    if grep -q "Translation:" <<<"$output"; then
      pass "TF ${parent} → ${child}"
      return 0
    fi
    sleep 1
  done
  fail "TF ${parent} → ${child} 不连通（已重试 2 次）"
  grep -m1 -E "Waiting for transform|Invalid frame ID|Could not transform" \
    <<<"$output" >&2 || true
  return 1
}

check_vlm_latency() {
  local max_latency="${VLM_LATENCY_MAX_SECONDS:-4.0}"
  local request_timeout="${VLM_LATENCY_TIMEOUT_SECONDS:-8.0}"
  local samples="${VLM_LATENCY_SAMPLES:-1}"
  local probe="${script_dir}/qwen_latency_probe.py"

  echo "执行真实 VLM 请求延迟验证（${samples} 次，阈值 ${max_latency}s）"
  if python3 "$probe" \
    --topic /camera/color/image_raw \
    --target "${VLM_LATENCY_TARGET:-chair}" \
    --mode target \
    --samples "$samples" \
    --timeout "$request_timeout" \
    --max-latency "$max_latency"; then
    pass "VLM 输出返回延迟不超过 ${max_latency}s"
    return 0
  fi
  fail "VLM 请求失败、无有效输出或返回延迟超过 ${max_latency}s"
  return 1
}

for node in /ranger_base_node /livox_lidar_publisher /laser_mapping \
  /fastlio_odom_adapter /slam_toolbox /controller_server /planner_server \
  /bt_navigator /obstacle_cloud_filter /vlm_nav; do
  if ros2 node list 2>/dev/null | grep -Fxq "$node"; then
    pass "节点 ${node}"
  else
    fail "缺少节点 ${node}"
  fi
done

for topic in /camera/color/image_raw /camera/aligned_depth_to_color/image_raw \
  /camera/color/camera_info /cloud_registered_body \
  /vlm_nav/obstacle_cloud /scan /map /local_costmap/costmap_raw \
  /diagnostics /vlm_nav/state; do
  count="$(timeout 4 ros2 topic info "$topic" 2>/dev/null |
    awk '/Publisher count:/ {print $3; found=1; exit} END {if (!found) print 0}')"
  if [[ "$count" -ge 1 ]]; then
    pass "话题 ${topic} 有发布者"
  else
    fail "话题 ${topic} 无发布者"
  fi
done

for frames in "map base_link" "map camera_color_optical_frame" "base_link body"; do
  read -r parent child <<<"$frames"
  check_tf "$parent" "$child" || true
done

enabled="$(ros2 param get /vlm_nav enabled 2>/dev/null || true)"
if grep -q "False" <<<"$enabled"; then
  pass "VLM 导航保持未使能"
else
  fail "检查阶段 /vlm_nav enabled 必须为 False"
fi

easy_case="$(ros2 param get /vlm_nav easy_case_mode 2>/dev/null || true)"
if grep -q "True" <<<"$easy_case"; then
  pass "VLM Easy Case 模式已启用"
  for costmap in /global_costmap/global_costmap /local_costmap/local_costmap; do
    plugins="$(ros2 param get "$costmap" plugins 2>/dev/null || true)"
    if grep -q "\\[\\]" <<<"$plugins"; then
      pass "${costmap} plugins=[]（空场模式）"
    else
      fail "${costmap} 仍加载代价地图插件：${plugins}"
    fi
  done
  behavior_costmap="$(
    ros2 param get /behavior_server costmap_topic 2>/dev/null || true
  )"
  if grep -q "/local_costmap/costmap_raw" <<<"$behavior_costmap"; then
    pass "Spin 使用空白 local costmap"
  else
    fail "Easy Case 的 Spin 未使用空白 local costmap"
  fi
fi

check_vlm_latency || true

if (( failures == 0 )); then
  pass "系统与 VLM 延迟检查通过；可在封闭场地执行 scripts/arm.sh"
  exit 0
fi
fail "共有 ${failures} 项失败，禁止 ARM"
exit 1
