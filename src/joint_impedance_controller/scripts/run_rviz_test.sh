#!/usr/bin/env bash
set -euo pipefail

日志根目录="${1:-/tmp/joint_impedance_test_$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "${日志根目录}"
export ROS_LOG_DIR="${日志根目录}/ros"
mkdir -p "${ROS_LOG_DIR}"

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch joint_impedance_controller rviz_test.launch.py \
  ros_log_dir:="${ROS_LOG_DIR}" \
  record_bag:=true \
  bag_output:="${日志根目录}/bag" \
  2>&1 | tee "${日志根目录}/launch.log"
