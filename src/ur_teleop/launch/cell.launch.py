"""UR cell: sim (mock + rviz) or real (hardware + gripper + FT300).

Persistent — started by home.launch and shared with teleop.launch (spec §4.1).
Launch args win over ur_teleop.yaml defaults.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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


def generate_launch_description():
    pkg_share = get_package_share_directory("ur_teleop")
    config_file = os.path.join(pkg_share, "config", "ur_teleop.yaml")

    sim_default = _yaml_default(config_file, "sim", fallback="true")
    ip_default = _yaml_default(config_file, "cell", "robot_ip", fallback="0.0.0.0")
    grip_default = _yaml_default(config_file, "cell", "gripper_port", fallback="/dev/ttyUSB1")
    ftdi_default = _yaml_default(config_file, "cell", "ftdi_id", fallback="")
    rviz_default = _yaml_default(config_file, "cell", "launch_rviz", fallback="true")
    ur_type_default = _yaml_default(config_file, "cell", "ur_type", fallback="ur10e")

    sim = LaunchConfiguration("sim")
    is_sim = PythonExpression(["'", sim, "' == 'true'"])          # "false" 字符串真值陷阱防护

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config_file),
        DeclareLaunchArgument("sim", default_value=sim_default),
        DeclareLaunchArgument("robot_ip", default_value=ip_default),
        DeclareLaunchArgument("gripper_port", default_value=grip_default),
        DeclareLaunchArgument("ftdi_id", default_value=ftdi_default),
        DeclareLaunchArgument("launch_rviz", default_value=rviz_default),
        DeclareLaunchArgument("ur_type", default_value=ur_type_default),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ur_robot_driver"),
                             "launch", "ur_control.launch.py")
            ),
            launch_arguments={
                "ur_type": LaunchConfiguration("ur_type"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "use_mock_hardware": sim,
                "mock_sensor_commands": "false",
                "launch_dashboard_client": "false",
                "launch_rviz": "false",
                "initial_joint_controller": "scaled_joint_trajectory_controller",
                "activate_joint_controller": "true",
            }.items(),
        ),
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", os.path.join(pkg_share, "config", "rviz", "ur_teleop.rviz")],
            condition=IfCondition(PythonExpression(
                ["'", sim, "' == 'true' and '", LaunchConfiguration("launch_rviz"), "' == 'true'"])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("robotiq_description"),
                             "launch", "robotiq_control.launch.py")
            ),
            condition=UnlessCondition(is_sim),
            launch_arguments={
                "com_port": LaunchConfiguration("gripper_port"),
                "launch_rviz": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("robotiq_ft_sensor_hardware"),
                             "launch", "ft_sensor_standalone.launch.py")
            ),
            condition=UnlessCondition(is_sim),
            launch_arguments={
                "ftdi_id": LaunchConfiguration("ftdi_id"),
                "frame_id": "robotiq_ft_frame_id",
            }.items(),
        ),
    ])
