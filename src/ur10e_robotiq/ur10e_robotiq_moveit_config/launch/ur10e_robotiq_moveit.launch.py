# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def declare_arguments():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'launch_rviz', default_value='true', description='Launch RViz?'
            ),
            DeclareLaunchArgument(
                'warehouse_sqlite_path',
                default_value=os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
                description='Path where the warehouse database should be stored',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation time',
            ),
            DeclareLaunchArgument(
                'publish_robot_description_semantic',
                default_value='true',
                description='MoveGroup publishes robot description semantic',
            ),
        ]
    )


def generate_launch_description():
    launch_rviz = LaunchConfiguration('launch_rviz')
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_robot_description_semantic = LaunchConfiguration(
        'publish_robot_description_semantic'
    )

    # The combined URDF is already published by mock_control.launch.py.  The
    # builder intentionally loads only this package's semantic/planning data.
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='ur10e_robotiq',
            package_name='ur10e_robotiq_moveit_config',
        )
        .robot_description_semantic(Path('srdf') / 'ur10e_robotiq.srdf.xacro')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': warehouse_sqlite_path,
    }

    wait_robot_description = Node(
        package='ur_robot_driver',
        executable='wait_for_robot_description',
        output='screen',
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                'use_sim_time': use_sim_time,
                'publish_robot_description_semantic': (
                    publish_robot_description_semantic
                ),
            },
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('ur10e_robotiq_moveit_config'), 'config', 'moveit.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(launch_rviz),
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    launch_description = LaunchDescription()
    launch_description.add_entity(declare_arguments())
    launch_description.add_action(wait_robot_description)
    launch_description.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[move_group_node, rviz_node],
            )
        )
    )
    return launch_description
