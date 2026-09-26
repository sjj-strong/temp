"""Xbot 模式单元：仿真 effort 闭环，实机使用 UR+FT300+夹爪组合描述。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ur_teleop.config import load_config


def _build_cell(context):
    cfg = load_config(LaunchConfiguration("config_file").perform(context))
    pkg_share = get_package_share_directory("ur_teleop")
    cart_share = get_package_share_directory("cartesian_impedance_controller")
    if cfg["teleop"].get("control_source") != "xbot":
        raise RuntimeError("xbot_cell 只接受 Xbot 配置文件")

    if cfg["sim"]:
        model = os.path.join(
            get_package_share_directory("ur10e_robotiq_ft_description"),
            "urdf", "ur10e_robotiq_ft.urdf.xacro",
        )
        description = {
            "robot_description": ParameterValue(
                Command([FindExecutable(name="xacro"), " ", model,
                         " name:=ur10e_robotiq_ft300",
                         " use_mock_hardware:=true ft_sensor_use_fake_mode:=true",
                         " xbot_effort_mock:=true"]),
                value_type=str,
            ),
        }
        actions = [
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 parameters=[description], output="screen"),
            Node(package="controller_manager", executable="ros2_control_node",
                 parameters=[os.path.join(pkg_share, "config", "xbot_mock_controllers.yaml"),
                             description], output="screen"),
        ]
        for name in ("joint_state_broadcaster", "scaled_joint_trajectory_controller",
                     "robotiq_gripper_controller"):
            actions.append(Node(package="controller_manager", executable="spawner",
                                arguments=[name, "-c", "/controller_manager"]))
        cart_config = os.path.join(pkg_share, "config", "xbot_cartesian_sim.yaml")
        if cfg.get("cell", {}).get("launch_rviz", False):
            actions.append(Node(package="rviz2", executable="rviz2",
                                arguments=["-d", os.path.join(pkg_share, "config", "rviz",
                                                              "ur_teleop.rviz")]))
    else:
        cell_cfg = cfg["cell"]
        bringup = os.path.join(
            get_package_share_directory("ur10e_robotiq_ft_description"),
            "launch", "real_bringup.launch.py",
        )
        actions = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={
                "robot_ip": str(cell_cfg["robot_ip"]),
                "ft_sensor_ftdi_id": str(cell_cfg["ftdi_id"]),
                "gripper_com_port": str(cell_cfg["gripper_port"]),
                "launch_rviz": str(bool(cell_cfg.get("launch_rviz", False))).lower(),
            }.items(),
        )]
        cart_config = os.path.join(
            cart_share, "config", "ur10e_ft300_cartesian_impedance.yaml",
        )

    actions.append(Node(
        package="controller_manager", executable="spawner",
        arguments=["cartesian_impedance_controller", "-c", "/controller_manager",
                   "--param-file", cart_config,
                   "--param-file", os.path.join(pkg_share, "config", "xbot_cartesian_type.yaml"),
                   "--inactive"],
    ))
    return actions


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("ur_teleop"), "config", "xbot_teleop.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        OpaqueFunction(function=_build_cell),
    ])
