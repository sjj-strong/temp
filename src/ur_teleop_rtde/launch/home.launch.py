"""Stage 1: start the UR cell (persistent) and move both arms to home.

UR 侧用 RTDE moveJ（无需 UR ROS2 driver）；Alicia 侧持续发 /joint_commands。
cell（UR driver + FT300 + rviz）常驻运行，供阶段 2 teleop.launch 复用显示。
完成后打印 HOME REACHED 并退出；之后运行 teleop.launch（阶段 2）。
"""

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
        # bool 需规范成小写字符串（yaml `true` → "True" 会误判为真机模式）
        if isinstance(data, bool):
            return "true" if data else "false"
        return str(data)
    except Exception:
        return fallback


def generate_launch_description():
    pkg_share = get_package_share_directory("ur_teleop_rtde")
    config_file = os.path.join(pkg_share, "config", "ur_teleop_rtde.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config_file),
        DeclareLaunchArgument("sim",
                              default_value=_yaml_default(config_file, "cell", "sim",
                                                         fallback="true")),
        DeclareLaunchArgument("robot_ip",
                              default_value=_yaml_default(config_file, "robot", "robot_ip",
                                                         fallback="0.0.0.0")),
        DeclareLaunchArgument("ftdi_id",
                              default_value=_yaml_default(config_file, "cell", "ftdi_id",
                                                         fallback="")),
        DeclareLaunchArgument("launch_rviz",
                              default_value=_yaml_default(config_file, "cell", "launch_rviz",
                                                         fallback="true")),
        DeclareLaunchArgument("description_launchfile",
                              default_value=os.path.join(
                                  get_package_share_directory("ur10e_robotiq_ft_description"),
                                  "launch", "rsp.launch.py")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "cell.launch.py")
            ),
            launch_arguments={
                "config_file": LaunchConfiguration("config_file"),
                "sim": LaunchConfiguration("sim"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "ftdi_id": LaunchConfiguration("ftdi_id"),
                "launch_rviz": LaunchConfiguration("launch_rviz"),
                "description_launchfile": LaunchConfiguration("description_launchfile"),
            }.items(),
        ),
        Node(
            package="ur_teleop_rtde", executable="home_node",
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
        ),
    ])
