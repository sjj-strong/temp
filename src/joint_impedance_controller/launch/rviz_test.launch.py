#!/usr/bin/env python3

"""使用官方 UR mock 硬件进行关节阻抗控制器 RViz 测试。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    控制器目录 = get_package_share_directory("joint_impedance_controller")
    UR描述目录 = get_package_share_directory("ur_description")
    控制器配置 = os.path.join(控制器目录, "config", "ur10e_joint_impedance.yaml")
    模拟描述文件 = os.path.join(控制器目录, "urdf", "ur10e_joint_impedance_mock.urdf.xacro")
    RViz配置 = os.path.join(UR描述目录, "rviz", "view_robot.rviz")
    机器人描述 = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", 模拟描述文件]),
            value_type=str,
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="是否启动 RViz"
        ),
        DeclareLaunchArgument(
            "record_bag", default_value="false", description="是否录制测试 rosbag"
        ),
        DeclareLaunchArgument(
            "bag_output",
            default_value="/tmp/joint_impedance_test_bag",
            description="rosbag 输出目录",
        ),
        DeclareLaunchArgument(
            "ros_log_dir",
            default_value="/tmp/joint_impedance_test_logs",
            description="ROS 节点日志目录",
        ),
        SetEnvironmentVariable("ROS_LOG_DIR", LaunchConfiguration("ros_log_dir")),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[机器人描述],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[控制器配置, 机器人描述],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_impedance_controller", "-c", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", RViz配置],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
            output="log",
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record", "--output", LaunchConfiguration("bag_output"),
                "--topics", "/joint_states", "/joint_impedance_controller/target_joint_state",
            ],
            condition=IfCondition(LaunchConfiguration("record_bag")),
            output="screen",
        ),
    ])
