#!/usr/bin/env bash
set -eo pipefail

log_root="${1:-/tmp/joint_impedance_test_$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "${log_root}"
export ROS_LOG_DIR="${log_root}/ros"
mkdir -p "${ROS_LOG_DIR}"

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
set -u

set +e
ros2 launch joint_impedance_controller rviz_test.launch.py \
  ros_log_dir:="${ROS_LOG_DIR}" \
  record_bag:=true \
  bag_output:="${log_root}/bag" \
  2>&1 | tee "${log_root}/launch.log"
launch_status=$?
set -e

# Ctrl-C 是交互测试的正常结束方式；ros2 launch 通常以 130 返回。
if [[ ${launch_status} -eq 130 ]]; then
  exit 0
fi
exit "${launch_status}"
