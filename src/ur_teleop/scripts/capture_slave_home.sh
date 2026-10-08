#!/usr/bin/env bash
# 加载 ROS 环境后，使用系统 Python 执行状态读取脚本。
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
exec /usr/bin/python3 /ros2_ws/src/ur_teleop/ur_teleop/capture_slave_home.py "$@"
