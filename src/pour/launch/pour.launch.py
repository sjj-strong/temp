"""启动 pour 包的两个节点:scale_serial_node + pour_control_node。

用法:
    ros2 launch pour pour.launch.py
    ros2 launch pour pour.launch.py port:=/dev/ttyUSB1 target_weight:=100.0 \
        control_rate:=9.0 kp:=0.0045 kd:=0.06 tolerance:=1.0

前置(需先启动 ur_control 并切换速度控制器):
    ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller \
        --activate forward_velocity_controller
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("pour")
    default_params = os.path.join(pkg_share, "config", "pour_params.yaml")

    declared_args = [
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="pour 参数文件路径"),
        DeclareLaunchArgument("port", default_value="",
                              description="覆盖串口端口(如 /dev/ttyUSB1)"),
        DeclareLaunchArgument("target_weight", default_value="",
                              description="覆盖目标重量(g)"),
        DeclareLaunchArgument("control_rate", default_value="",
                              description="覆盖控制频率(Hz)"),
        DeclareLaunchArgument("kp", default_value="", description="覆盖 kp"),
        DeclareLaunchArgument("kd", default_value="", description="覆盖 kd"),
        DeclareLaunchArgument("tolerance", default_value="",
                              description="覆盖误差允许范围(g)"),
    ]

    def _build_nodes(context):
        """把非空的 launch 覆盖值合进参数表;空串(未传)则回退到 yaml。"""
        params_file = LaunchConfiguration("params_file").perform(context)
        scale_params = [params_file]
        pour_params = [params_file]
        overrides = {
            "port": ("scale", "port"),
            "target_weight": ("pour", "target_weight"),
            "control_rate": ("pour", "control_rate"),
            "kp": ("pour", "kp"),
            "kd": ("pour", "kd"),
            "tolerance": ("pour", "tolerance"),
        }
        for launch_arg, (node_kind, param_key) in overrides.items():
            value = LaunchConfiguration(launch_arg).perform(context)
            if value:  # 空串 = 未提供, 回退 yaml
                (scale_params if node_kind == "scale" else pour_params).append(
                    {param_key: value})

        return [
            Node(package="pour", executable="scale_serial_node",
                 name="scale_node", output="screen",
                 parameters=scale_params),
            Node(package="pour", executable="pour_control_node",
                 name="pour_control_node", output="screen",
                 parameters=pour_params),
        ]

    return LaunchDescription(
        declared_args + [OpaqueFunction(function=_build_nodes)])
