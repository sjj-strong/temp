#!/usr/bin/env python3

"""UR10e、2F-85 与 FT300 的组合实机启动入口。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """复用官方 UR 启动文件，并启动独立的 Robotiq 夹爪控制栈。"""
    描述包目录 = get_package_share_directory("ur10e_robotiq_ft_description")
    UR驱动包目录 = get_package_share_directory("ur_robot_driver")
    夹爪描述包目录 = get_package_share_directory("robotiq_description")

    ur控制启动文件 = os.path.join(UR驱动包目录, "launch", "ur_control.launch.py")
    组合描述启动文件 = os.path.join(描述包目录, "launch", "rsp.launch.py")
    夹爪控制启动文件 = os.path.join(夹爪描述包目录, "launch", "robotiq_control.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur10e"),
        DeclareLaunchArgument("robot_ip", description="UR 控制柜 IP 地址。"),
        DeclareLaunchArgument(
            "ft_sensor_ftdi_id",
            description="FT300 设备名（不含 /dev/），例如 ttyUSB2。",
        ),
        DeclareLaunchArgument("ft_sensor_read_rate", default_value="10"),
        DeclareLaunchArgument("ft_sensor_max_retries", default_value="100"),
        DeclareLaunchArgument("gripper_com_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("launch_gripper", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("initial_joint_controller",
                              default_value="scaled_joint_trajectory_controller"),
        DeclareLaunchArgument("activate_joint_controller", default_value="true"),
        # FT300 由组合 URDF 中的 ros2_control SensorInterface 独占；不要再启动
        # robotiq_ft_sensor_standalone_node，否则两个驱动会竞争同一串口。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur控制启动文件),
            launch_arguments={
                "ur_type": LaunchConfiguration("ur_type"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "use_mock_hardware": "false",
                "ft_sensor_use_fake_mode": "false",
                "ft_sensor_ftdi_id": LaunchConfiguration("ft_sensor_ftdi_id"),
                "ft_sensor_read_rate": LaunchConfiguration("ft_sensor_read_rate"),
                "ft_sensor_max_retries": LaunchConfiguration("ft_sensor_max_retries"),
                "description_launchfile": 组合描述启动文件,
                "initial_joint_controller": LaunchConfiguration("initial_joint_controller"),
                "activate_joint_controller": LaunchConfiguration("activate_joint_controller"),
                "launch_rviz": LaunchConfiguration("launch_rviz"),
            }.items(),
        ),
        # 夹爪使用独立的 /robotiq_controller_manager，避免与 UR 的
        # /controller_manager 及其 /robot_description 相互冲突。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(夹爪控制启动文件),
            condition=IfCondition(LaunchConfiguration("launch_gripper")),
            launch_arguments={
                "com_port": LaunchConfiguration("gripper_com_port"),
                "launch_rviz": "false",
            }.items(),
        ),
    ])
