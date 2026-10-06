#!/usr/bin/env bash
set -uo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/common.sh"

failures=0
pass() { printf '\033[32m[通过]\033[0m %s\n' "$1"; }
fail() { printf '\033[31m[失败]\033[0m %s\n' "$1"; failures=$((failures + 1)); }

has_node() {
  ros2 node list 2>/dev/null | grep -Fxq "$1"
}

check_nodes() {
  local description="$1" node
  shift
  for node in "$@"; do
    if ! has_node "$node"; then
      fail "${description}缺少关键节点 ${node}"
      return 1
    fi
  done
  pass "${description}关键节点齐全"
}

publisher_count() {
  timeout 6 ros2 topic info "$1" 2>/dev/null |
    awk '/Publisher count:/ {print $3; found=1; exit} END {if (!found) print 0}'
}

check_topic() {
  local topic="$1" description="$2" count
  shift 2
  count="$(publisher_count "$topic")"
  if [[ "$count" -lt 1 ]]; then
    fail "${description} ${topic} 无发布者"
    return 1
  fi
  # DDS discovery can take several seconds on the loaded external controller.
  if timeout 12 ros2 topic echo "$topic" "$@" --once >/dev/null 2>&1; then
    pass "${description} ${topic}（发布者 ${count}，数据有效）"
    return 0
  fi
  fail "${description} ${topic} 有 ${count} 个发布者但未取到数据"
  return 1
}

check_lifecycle_pair() {
  local first="$1" second="$2" output_first output_second
  output_first="$(timeout 8 ros2 lifecycle get "$first" 2>/dev/null || true)"
  output_second="$(timeout 8 ros2 lifecycle get "$second" 2>/dev/null || true)"
  if grep -q "active \[3\]" <<<"$output_first" \
    && grep -q "active \[3\]" <<<"$output_second"; then
    pass "定位生命周期 ${first}、${second} 均为 active"
  else
    fail "定位生命周期异常：${first}=${output_first:-无响应}，${second}=${output_second:-无响应}"
  fi
}

check_tf() {
  local parent="$1" child="$2" output="" attempt
  for attempt in 1 2; do
    output="$(timeout 8 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 || true)"
    if grep -q "Translation:" <<<"$output"; then
      pass "TF ${parent} → ${child}"
      return 0
    fi
    sleep 1
  done
  fail "TF ${parent} → ${child} 不连通或无有效时间"
  return 1
}

check_fastlio_odom_health() {
  local topic="/fastlio/odom" count stamp sec nanosec age rate_output rate
  local max_age="${FASTLIO_ODOM_MAX_AGE_SECONDS:-0.5}"
  local min_rate="${FASTLIO_ODOM_MIN_RATE_HZ:-2.0}"

  count="$(publisher_count "$topic")"
  if [[ "$count" != "1" ]]; then
    fail "FAST-LIO 适配里程计 ${topic} 发布者为 ${count}，要求恰好 1 个"
    return 1
  fi

  stamp="$(
    timeout 12 ros2 topic echo "$topic" --once --field header.stamp 2>/dev/null \
      || true
  )"
  sec="$(awk '/sec:/ {print $2; exit}' <<<"$stamp")"
  nanosec="$(awk '/nanosec:/ {print $2; exit}' <<<"$stamp")"
  if [[ -z "$sec" || -z "$nanosec" ]]; then
    fail "FAST-LIO 适配里程计 ${topic} 未取到带时间戳的数据"
    return 1
  fi
  age="$(awk -v now="$(date +%s.%N)" -v s="$sec" -v ns="$nanosec" \
    'BEGIN {printf "%.3f", now - (s + ns / 1000000000.0)}')"

  rate_output="$(timeout 8 ros2 topic hz "$topic" 2>&1 || true)"
  rate="$(awk '/average rate:/ {value=$3} END {print value}' <<<"$rate_output")"
  if [[ -z "$rate" ]]; then
    fail "FAST-LIO 适配里程计 ${topic} 无法测得频率"
    return 1
  fi

  if awk -v age="$age" -v max_age="$max_age" -v rate="$rate" -v min_rate="$min_rate" \
    'BEGIN {exit !(age >= -0.1 && age <= max_age && rate >= min_rate)}'; then
    pass "FAST-LIO 适配里程计 ${topic}（${rate} Hz，时间龄 ${age}s）"
  else
    fail "FAST-LIO 适配里程计异常：${rate} Hz、时间龄 ${age}s；要求 ≥${min_rate} Hz 且 ≤${max_age}s"
  fi
}

parameter_matches() {
  local node="$1" parameter="$2" expected="$3" actual
  actual="$(timeout 6 ros2 param get "$node" "$parameter" 2>/dev/null || true)"
  grep -Fq "$expected" <<<"$actual"
}

check_amcl_contract() {
  local failed=false
  for spec in \
    "/controller_server odom_topic /fastlio/odom" \
    "/velocity_smoother odom_topic /fastlio/odom" \
    "/vlm_nav arrival_odom_topic /fastlio/odom" \
    "/vlm_nav scan_first True" \
    "/vlm_nav allow_frontier_after_scan False" \
    "/vlm_nav easy_case_mode False" \
    "/ranger_base_node publish_odom_tf False"; do
    read -r node parameter expected <<<"$spec"
    parameter_matches "$node" "$parameter" "$expected" || failed=true
  done
  if [[ "$failed" == false ]]; then
    pass "AMCL 导航参数、扫描行为及 TF owner 合约"
  else
    fail "AMCL 导航参数合约不一致；用 ros2 param get 检查 odom/frontier/Ranger TF"
  fi
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
    pass "VLM 图像输入与推理延迟不超过 ${max_latency}s"
  else
    fail "VLM 请求失败、无有效输出或延迟超过 ${max_latency}s"
  fi
}

localization_mode="${VLM_NAV_LOCALIZATION_MODE:-}"
if [[ -z "$localization_mode" ]]; then
  if has_node /amcl || has_node /map_server; then
    localization_mode="amcl"
  else
    localization_mode="slam"
  fi
fi
if [[ "$localization_mode" != "slam" && "$localization_mode" != "amcl" ]]; then
  fail "VLM_NAV_LOCALIZATION_MODE 必须是 slam 或 amcl"
  localization_mode="slam"
fi
pass "定位模式：${localization_mode}"

echo "--- 关键链路 ---"
check_nodes "硬件/里程计链：" /ranger_base_node /laser_mapping /fastlio_odom_adapter || true
check_topic /camera/color/image_raw "RGB-D 链：RGB" || true
check_topic /camera/aligned_depth_to_color/image_raw "RGB-D 链：对齐深度" || true
check_topic /scan "激光障碍链" || true
check_topic /Odometry "FAST-LIO 原始里程计" || true
check_fastlio_odom_health || true

if [[ "$localization_mode" == "amcl" ]]; then
  check_nodes "固定地图定位链：" /map_server /amcl || true
  check_lifecycle_pair /map_server /amcl
  if has_node /slam_toolbox; then
    fail "AMCL 模式仍有 /slam_toolbox，存在重复 map → odom owner"
  else
    pass "AMCL 与 SLAM Toolbox 互斥"
  fi
  check_topic /map "固定地图" \
    --qos-durability transient_local --qos-reliability reliable || true
  check_tf map odom || true
  check_tf odom base_link || true
  check_amcl_contract
else
  check_nodes "SLAM 定位链：" /slam_toolbox || true
  check_topic /map "SLAM 地图" || true
  check_tf map base_link || true
fi

check_nodes "Nav2/VLM 链：" /controller_server /bt_navigator /vlm_nav || true
check_topic /local_costmap/costmap_raw "Nav2 local costmap" || true

enabled="$(timeout 6 ros2 param get /vlm_nav enabled 2>/dev/null || true)"
if grep -q "False" <<<"$enabled"; then
  pass "VLM 导航保持未使能"
else
  fail "检查阶段 /vlm_nav enabled 必须为 False"
fi

check_vlm_latency

if (( failures == 0 )); then
  pass "关键链路检查通过；可在封闭场地执行 scripts/arm.sh"
  exit 0
fi
fail "共有 ${failures} 条关键链路失败，禁止 ARM"
exit 1
