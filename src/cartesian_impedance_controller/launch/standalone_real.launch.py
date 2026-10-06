"""独立启动 UR10e 笛卡尔阻抗控制；夹爪和 FT300 可分别关闭。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _as_bool(value, name):
    normalized = value.strip().lower()
    if normalized in ("true", "1", "yes"):
        return True
    if normalized in ("false", "0", "no"):
        return False
    raise ValueError(f"{name} 必须为 true 或 false，收到: {value}")


def _start_real_cell(context):
    """复用组合实机入口，并将设备开关传到硬件初始化层。"""
    use_gripper = _as_bool(LaunchConfiguration("use_gripper").perform(context), "use_gripper")
    use_ft300 = _as_bool(LaunchConfiguration("use_ft300").perform(context), "use_ft300")
    launch_rviz = _as_bool(LaunchConfiguration("launch_rviz").perform(context), "launch_rviz")
    bringup = os.path.join(
        get_package_share_directory("ur10e_robotiq_ft_description"),
        "launch", "real_bringup.launch.py",
    )
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip").perform(context),
            "ft_sensor_ftdi_id": LaunchConfiguration("ft_sensor_ftdi_id").perform(context),
            "ft_sensor_use_fake_mode": str(not use_ft300).lower(),
            "gripper_com_port": LaunchConfiguration("gripper_com_port").perform(context),
            "launch_gripper": str(use_gripper).lower(),
            "launch_rviz": str(launch_rviz).lower(),
            "use_cartesian_impedance": "true",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", description="UR 控制柜 IP 地址"),
        DeclareLaunchArgument("use_gripper", default_value="false",
                              description="是否启动 Robotiq 夹爪控制栈"),
        DeclareLaunchArgument("use_ft300", default_value="false",
                              description="是否连接真实 FT300 串口；关闭时使用虚拟接口"),
        DeclareLaunchArgument("gripper_com_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("ft_sensor_ftdi_id", default_value="ttyUSB2"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        OpaqueFunction(function=_start_real_cell),
    ])
