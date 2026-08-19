#!/usr/bin/env python3

"""Display UR + FT300 + Robotiq 2F-85 combined model."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # Launch arguments
    # ============================================================

    name = LaunchConfiguration("name")
    ur_type = LaunchConfiguration("ur_type")

    declared_arguments = [

        DeclareLaunchArgument(
            "name",
            default_value="ur10e_robotiq_ft",
            description="Robot name",
        ),

        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="UR robot model",
        ),
    ]

    # ============================================================
    # Xacro
    # ============================================================

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("ur10e_robotiq_ft_description"),
            "urdf",
            "ur10e_robotiq_ft.urdf.xacro",
        ]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " ",
            "name:=",
            name,
            " ",
            "ur_type:=",
            ur_type,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    # ============================================================
    # robot_state_publisher
    # ============================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ============================================================
    # joint_state_publisher_gui
    # ============================================================

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[robot_description],
    )

    # ============================================================
    # RViz
    # ============================================================

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("ur10e_robotiq_ft_description"),
            "rviz",
            "display.rviz",
        ]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz,
        ]
    )