#!/usr/bin/env bash
set -euo pipefail

wifi_interface="wlo1"
bridge_config="/home/isee601/ws/VLM_Nav/config/multi_agent_bridge_ranger.yaml"

if ! ip link show dev "$wifi_interface" >/dev/null 2>&1; then
  echo "ERROR: required Wi-Fi interface $wifi_interface does not exist." >&2
  exit 1
fi

link_status="$(ip -o link show dev "$wifi_interface")"
if ! grep -q 'state UP' <<<"$link_status"; then
  echo "ERROR: Wi-Fi interface $wifi_interface is not UP." >&2
  exit 1
fi

ipv4_status="$(ip -4 -o addr show dev "$wifi_interface" scope global)"
if ! grep -Eq ' inet 192\.168\.1\.[0-9]+/' <<<"$ipv4_status"; then
  echo "ERROR: $wifi_interface has no 192.168.1.* IPv4 address." >&2
  exit 1
fi

power_save_status="$(iw dev "$wifi_interface" get power_save 2>&1 || true)"
if grep -Eqi 'power save:[[:space:]]*on' <<<"$power_save_status"; then
  echo "WARNING: Wi-Fi power save may cause multi-agent latency/jitter." >&2
fi

# ROS setup scripts inspect optional variables that may be unset.
set +u
source /opt/ros/humble/setup.bash
source /home/isee601/ws/install/setup.bash
set -u

unset ROS_DOMAIN_ID
unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_DEFAULT_PROFILES_FILE

exec ros2 run domain_bridge domain_bridge "$bridge_config"
