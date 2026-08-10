"""Stage 1: start the UR cell (persistent) and move both arms to home."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _yaml_default(config_file: str, *path: str, fallback: str):
    try:
        with open(config_file) as f:
            data = yaml.safe_load(f)
        for p in path:
            data = data[p]
        return str(data)
    except Exception:
        return fallback


def _description_launchfile():
    """与 cell.launch.py 同一默认：ur10e_robotiq_ft 组合模型，未安装回退官方纯 UR。"""
    try:
        return os.path.join(get_package_share_directory("ur10e_robotiq_ft"),
                            "launch", "rsp.launch.py")
    except Exception:
        return os.path.join(get_package_share_directory("ur_robot_driver"),
                            "launch", "ur_rsp.launch.py")


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
        DeclareLaunchArgument("launch_rviz",
                              default_value=_yaml_default(config_file, "cell", "launch_rviz",
                                                         fallback="true")),
        DeclareLaunchArgument("description_launchfile",
                              default_value=_description_launchfile()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "cell.launch.py")
            ),
            launch_arguments={
                "config_file": LaunchConfiguration("config_file"),
                "sim": LaunchConfiguration("sim"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "gripper_port": LaunchConfiguration("gripper_port"),
                "ftdi_id": LaunchConfiguration("ftdi_id"),
                "launch_rviz": LaunchConfiguration("launch_rviz"),
                "description_launchfile": LaunchConfiguration("description_launchfile"),
            }.items(),
        ),
        Node(
            package="ur_teleop", executable="home_node",
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
        ),
    ])
