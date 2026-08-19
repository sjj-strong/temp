"""Stage 2: teleop core (+ data_recorder when mode=record).

RTDE servoJ 遥操作，无需 UR ROS2 driver。先运行 home.launch（阶段 1）。
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ur_teleop_rtde")
    config_file = os.path.join(pkg_share, "config", "ur_teleop_rtde.yaml")
    try:
        with open(config_file) as f:
            mode_default = yaml.safe_load(f).get("mode", "teleop")
    except Exception:
        mode_default = "teleop"

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config_file),
        DeclareLaunchArgument("mode", default_value=mode_default,
                              choices=["teleop", "record"]),
        DeclareLaunchArgument("force_home", default_value="false"),
        Node(
            package="ur_teleop_rtde", executable="teleop_node",
            parameters=[
                {"config_file": LaunchConfiguration("config_file"),
                 "mode": LaunchConfiguration("mode"),
                 "force_home": LaunchConfiguration("force_home")},
            ],
        ),
        Node(
            package="ur_teleop_rtde", executable="data_recorder",
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration("mode"), "' == 'record'"])),
        ),
    ])
