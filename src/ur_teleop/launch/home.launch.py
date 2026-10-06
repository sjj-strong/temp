"""Stage 1: start the UR cell (persistent) and move both arms to home."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ur_teleop.config import load_config


def _yaml_default(config_file: str, *path: str, fallback: str):
    try:
        with open(config_file) as f:
            data = yaml.safe_load(f)
        for p in path:
            data = data[p]
        # 同 cell.launch：yaml 布尔归一为小写，避免大小写失配。
        return str(data).lower() if isinstance(data, bool) else str(data)
    except Exception:
        return fallback


def _start_cell(context, pkg_share):
    """按配置选择 Alicia 原有单元或 Xbot 专用单元。"""
    config_path = LaunchConfiguration("config_file").perform(context)
    source = load_config(config_path)["teleop"].get("control_source", "alicia")
    if source == "xbot":
        launch_file = os.path.join(pkg_share, "launch", "xbot_cell.launch.py")
        arguments = {"config_file": config_path}
    else:
        launch_file = os.path.join(pkg_share, "launch", "cell.launch.py")
        arguments = {
            "config_file": LaunchConfiguration("config_file"),
            "sim": LaunchConfiguration("sim"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "gripper_port": LaunchConfiguration("gripper_port"),
            "ftdi_id": LaunchConfiguration("ftdi_id"),
            "enable_gripper": LaunchConfiguration("enable_gripper"),
            "enable_ft300": LaunchConfiguration("enable_ft300"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "alicia_port": LaunchConfiguration("alicia_port"),
            "launch_alicia": LaunchConfiguration("launch_alicia"),
            "controller": LaunchConfiguration("controller"),
        }
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=arguments.items(),
    )]


def generate_launch_description():
    pkg_share = get_package_share_directory("ur_teleop")
    config_file = os.path.join(pkg_share, "config", "ur_teleop.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config_file),
        DeclareLaunchArgument("sim",
                              default_value=_yaml_default(config_file, "sim", fallback="true")),
        DeclareLaunchArgument("robot_ip",
                              default_value=_yaml_default(config_file, "cell", "robot_ip",
                                                         fallback="0.0.0.0")),
        DeclareLaunchArgument("gripper_port",
                              default_value=_yaml_default(config_file, "cell", "gripper_port",
                                                         fallback="/dev/ttyUSB1")),
        DeclareLaunchArgument("ftdi_id",
                              default_value=_yaml_default(config_file, "cell", "ftdi_id",
                                                         fallback="")),
        DeclareLaunchArgument("enable_gripper",
                              default_value=_yaml_default(config_file, "gripper", "enabled",
                                                         fallback="true")),
        DeclareLaunchArgument("enable_ft300",
                              default_value=_yaml_default(config_file, "cell", "ft300_enabled",
                                                         fallback="true")),
        DeclareLaunchArgument("launch_rviz",
                              default_value=_yaml_default(config_file, "cell", "launch_rviz",
                                                         fallback="true")),
        DeclareLaunchArgument("alicia_port",
                              default_value=_yaml_default(config_file, "cell", "alicia_port",
                                                         fallback="")),
        DeclareLaunchArgument("launch_alicia",
                              default_value=_yaml_default(config_file, "cell", "launch_alicia",
                                                         fallback="true")),
        DeclareLaunchArgument("controller",
                              default_value=_yaml_default(config_file, "teleop", "controller",
                                                         fallback="forward_position"),
                              choices=["forward_position", "joint_impedance"]),
        OpaqueFunction(function=_start_cell, args=[pkg_share]),
        Node(
            package="ur_teleop", executable="home_node",
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
        ),
    ])
