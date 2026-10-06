#!/usr/bin/env bash
set -euo pipefail

# ROS setup scripts inspect optional variables that may be unset.
set +u
source /opt/ros/humble/setup.bash
source /home/isee-cdh/ws/install/setup.bash
set -u

export ROS_DOMAIN_ID=0

odom_type="$(ros2 topic type /odom 2>/dev/null || true)"
if [[ -z "$odom_type" ]]; then
  echo "WARNING: /odom has no visible publisher; ranger_agent_node will wait for it." >&2
elif [[ "$odom_type" != "nav_msgs/msg/Odometry" ]]; then
  echo "ERROR: /odom type is '$odom_type'; expected nav_msgs/msg/Odometry." >&2
  exit 1
fi

exec ros2 run multi_agent_comm ranger_agent_node
