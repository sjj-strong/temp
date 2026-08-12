"""Stage 2: teleop core (+ data_recorder when mode=record) + ruckig smoother.

Connects to the cell already running from home.launch (spec §4.1).

ruckig_node 在 home 完成后随本 launch 启动 → 从当前（已 home）UR 状态初始化，
避免 home 阶段轨迹控制器移动导致 Ruckig 内部状态过期。teleop_node 把映射后的
UR 目标发到 /ruckig/target_joint_positions，ruckig_node 以 control_hz（默认
500 Hz，与 controller_manager 一致）做 jerk-limited 平滑后下发
/forward_position_controller/commands。
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ur_teleop")
    config_file = os.path.join(pkg_share, "config", "ur_teleop.yaml")
    try:
        with open(config_file) as f:
            cfg = yaml.safe_load(f)
        mode_default = cfg.get("mode", "teleop")
        ruckig_hz_default = str(cfg.get("ruckig", {}).get("control_hz", 500.0))
    except Exception:
        mode_default = "teleop"
        ruckig_hz_default = "500.0"

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config_file),
        DeclareLaunchArgument("mode", default_value=mode_default,
                              choices=["teleop", "record"]),
        DeclareLaunchArgument("force_home", default_value="false"),
        DeclareLaunchArgument("ruckig_control_hz", default_value=ruckig_hz_default),
        Node(
            package="ur_teleop", executable="teleop_node",
            parameters=[
                {"config_file": LaunchConfiguration("config_file"),
                 "mode": LaunchConfiguration("mode"),
                 "force_home": LaunchConfiguration("force_home")},
            ],
        ),
        # ruckig 平滑节点：消费 teleop_node 的映射目标，500 Hz 平滑后下发 forward controller。
        # 注意：不要同时用 cell.launch ruckig:=true —— 两者都会向
        # /forward_position_controller/commands 发布，会冲突。
        Node(
            package="ur_teleop", executable="ruckig_node",
            parameters=[{
                "config_file": LaunchConfiguration("config_file"),
                "control_hz": LaunchConfiguration("ruckig_control_hz"),
            }],
        ),
        Node(
            package="ur_teleop", executable="data_recorder",
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration("mode"), "' == 'record'"])),
        ),
    ])
