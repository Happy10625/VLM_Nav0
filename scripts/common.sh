#!/usr/bin/env bash

# Shared ROS environment for the VLM_Nav hardware and navigation scripts.
# ROS setup files inspect optional variables, so temporarily disable nounset.
set +u

source /opt/ros/humble/setup.bash

for setup_file in \
  /home/isee601/ros2_ws/install/setup.bash \
  /home/isee601/rs515/ros2_ws/install/setup.bash \
  /home/isee601/agilex_ws/install/setup.bash \
  /home/isee601/ws/install_fastlio/setup.bash \
  /home/isee601/ws/VLM_Nav/install/setup.bash; do
  if [[ -f "$setup_file" ]]; then
    source "$setup_file"
  else
    echo "警告：未找到环境文件 ${setup_file}" >&2
  fi
done

export RCUTILS_COLORIZED_OUTPUT=1
set -u
