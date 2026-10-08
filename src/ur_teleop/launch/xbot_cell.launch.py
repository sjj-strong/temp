"""Xbot 模式单元：仿真 effort 闭环，实机使用 UR+FT300+夹爪组合描述。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ur_teleop.config import load_config
from ur_teleop.impedance_config import resolve_damping


def _build_cell(context):
    cfg = load_config(LaunchConfiguration("config_file").perform(context))
    pkg_share = get_package_share_directory("ur_teleop")
    if cfg["teleop"].get("control_source") != "xbot":
        raise RuntimeError("xbot_cell 只接受 Xbot 配置文件")
    use_gripper = cfg.get("gripper", {}).get("enabled", True)
    use_ft300 = cfg.get("cell", {}).get("ft300_enabled", True)
    cart_config = resolve_damping(cfg["xbot"]["controller_config_file"])

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
        controllers = ["joint_state_broadcaster", "scaled_joint_trajectory_controller"]
        if use_gripper:
            controllers.append("robotiq_gripper_controller")
        for name in controllers:
            actions.append(Node(package="controller_manager", executable="spawner",
                                arguments=[name, "-c", "/controller_manager"]))
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
                "ft_sensor_use_fake_mode": str(not use_ft300).lower(),
                "gripper_com_port": str(cell_cfg["gripper_port"]),
                "launch_gripper": str(use_gripper).lower(),
                "launch_rviz": str(bool(cell_cfg.get("launch_rviz", False))).lower(),
                "controllers_file": os.path.join(pkg_share, "config", "xbot_ur_controllers.yaml"),
                # 先由轨迹控制器回 Home；此处禁止组合 bringup 自动激活阻抗。
                "use_cartesian_impedance": "false",
                "initial_joint_controller": "scaled_joint_trajectory_controller",
                "activate_joint_controller": "true",
            }.items(),
        )]
        if use_ft300:
            # FT300 已由组合 URDF 的 SensorInterface 打开；只加载 broadcaster。
            actions.append(Node(
                package="controller_manager", executable="spawner",
                arguments=["robotiq_force_torque_sensor_broadcaster", "-c", "/controller_manager",
                           "--param-file", os.path.join(pkg_share, "config", "xbot_ur_controllers.yaml")],
            ))

    actions.append(LogInfo(msg=f"笛卡尔阻抗控制器参数文件（--param-file）：{cart_config}"))
    actions.append(Node(
        package="controller_manager", executable="spawner",
        arguments=["cartesian_impedance_controller", "-c", "/controller_manager",
                   "--param-file", cart_config,
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
