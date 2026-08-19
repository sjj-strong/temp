"""UR cell for ur_teleop_rtde: display/model stack (UR driver + FT300 + rviz).

遥操作控制本身不经过 cell（teleop_node 通过 RTDE 直连 UR）；cell 只为 rviz 提供
组合模型（UR+FT300+2F-85，来自 ur10e_robotiq_ft_description）的 /robot_description、
/joint_states 与 TF：

  - sim=true（默认）: ur_control.launch.py 以 use_mock_hardware 启动 → 离线 rviz
  - sim=false:       真实 UR driver + FT300 driver（ft_sensor_standalone.launch.py）

**不包含 robotiq_control.launch.py**：夹爪由 teleop_node 的 pyrobotiqgripper
独占串口（/dev/ttyUSB1），且 robotiq driver 自带的 robot_state_publisher 会以
world 为根重复发布组合模型已有的 robotiq_85_* 帧（TF 冲突）。需要 action server
调试时单独启动 robotiq_control.launch.py，但此时不能同时运行 teleop 的夹爪串口控制。

Persistent — started by home.launch and shared with teleop.launch。
Launch args win over ur_teleop_rtde.yaml defaults.
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
        # bool 需规范成小写字符串：yaml `true` → Python True → str() 会得 "True"，
        # 使 PythonExpression("'True' == 'true'") 误判为 false（真机模式）
        if isinstance(data, bool):
            return "true" if data else "false"
        return str(data)
    except Exception:
        return fallback


def _description_launchfile():
    """rviz 模型：优先 ur10e_robotiq_ft_description 组合模型（UR+FT300+2F-85），未安装回退官方纯 UR。"""
    try:
        return os.path.join(get_package_share_directory("ur10e_robotiq_ft_description"),
                            "launch", "rsp.launch.py")
    except Exception:
        return os.path.join(get_package_share_directory("ur_robot_driver"),
                            "launch", "ur_rsp.launch.py")


def _rviz_config():
    """rviz 配置跟随组合模型包；未安装则用 rviz 默认空配置。"""
    try:
        return os.path.join(get_package_share_directory("ur10e_robotiq_ft_description"),
                            "rviz", "display.rviz")
    except Exception:
        return ""


def generate_launch_description():
    pkg_share = get_package_share_directory("ur_teleop_rtde")
    config_file = os.path.join(pkg_share, "config", "ur_teleop_rtde.yaml")

    sim_default = _yaml_default(config_file, "cell", "sim", fallback="true")
    ip_default = _yaml_default(config_file, "robot", "robot_ip", fallback="0.0.0.0")
    ur_type_default = _yaml_default(config_file, "robot", "ur_type", fallback="ur10e")
    ftdi_default = _yaml_default(config_file, "cell", "ftdi_id", fallback="")
    rviz_default = _yaml_default(config_file, "cell", "launch_rviz", fallback="true")

    sim = LaunchConfiguration("sim")
    is_sim = PythonExpression(["'", sim, "' == 'true'"])          # "false" 字符串真值陷阱防护

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config_file),
        DeclareLaunchArgument("sim", default_value=sim_default),
        DeclareLaunchArgument("robot_ip", default_value=ip_default),
        DeclareLaunchArgument("ur_type", default_value=ur_type_default),
        DeclareLaunchArgument("ftdi_id", default_value=ftdi_default),
        DeclareLaunchArgument("launch_rviz", default_value=rviz_default),
        DeclareLaunchArgument("description_launchfile",
                              default_value=_description_launchfile()),
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
                "description_launchfile": LaunchConfiguration("description_launchfile"),
                "initial_joint_controller": "scaled_joint_trajectory_controller",
                "activate_joint_controller": "true",
            }.items(),
        ),
        # rviz 在 sim 与真实两种模式都启动（与 ur_teleop 不同——本 cell 的目的就是显示）
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", _rviz_config()] if _rviz_config() else [],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
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
