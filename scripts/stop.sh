#!/usr/bin/env bash
set -euo pipefail

ros2 param set /vlm_nav enabled false 2>/dev/null || true
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null
echo "已禁用 VLM 导航并发送零速度。实体急停仍是首选。"
