"""Stage 2: teleop core (+ data_recorder when mode=record) + ruckig smoother.

Connects to the cell already running from home.launch (spec §4.1).

ruckig_node 在 home 完成后随本 launch 启动 → 从当前（已 home）UR 状态初始化，
避免 home 阶段轨迹控制器移动导致 Ruckig 内部状态过期。teleop_node 把映射后的
UR 目标发到 /ruckig/target_joint_positions，ruckig_node 以 control_hz（默认
500 Hz，与 controller_manager 一致）做 jerk-limited 平滑后下发
/forward_position_controller/commands。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ur_teleop.config import load_config


def _nodes(context):
    config = LaunchConfiguration('config_file').perform(context)
    cfg = load_config(config)
    mode = LaunchConfiguration('mode').perform(context) or cfg.get('mode', 'teleop')
    if mode not in ('teleop', 'record'):
        raise ValueError('mode 必须为 teleop 或 record')
    if cfg['teleop'].get('control_source', 'alicia') == 'xbot':
        nodes = [
            Node(package='joy', executable='joy_node', parameters=[{
                'autorepeat_rate': 50.0, 'deadzone': 0.0,
                'device_id': int(cfg.get('xbot', {}).get('device_id', 0)),
            }]),
            Node(package='ur_teleop', executable='xbot_teleop_node',
                 parameters=[{'config_file': config}], output='screen'),
        ]
    else:
        nodes = [Node(package='ur_teleop', executable='teleop_node', parameters=[{
            'config_file': config, 'mode': mode,
            'force_home': LaunchConfiguration('force_home'),
        }])]
        controller = cfg['teleop'].get('controller', 'forward_position')
        enabled = LaunchConfiguration('use_ruckig').perform(context)
        if (enabled == 'true' or (not enabled and
                (controller != 'joint_impedance' or cfg.get('ruckig', {}).get('enabled', True)))):
            hz = LaunchConfiguration('ruckig_control_hz').perform(context)
            nodes.append(Node(package='ur_teleop', executable='ruckig_node', parameters=[{
                'config_file': config,
                'control_hz': float(hz) if hz else float(cfg.get('ruckig', {}).get('control_hz', 500.)),
            }]))
    if mode == 'record':
        nodes.append(Node(package='ur_teleop', executable='data_recorder',
                          parameters=[{'config_file': config}], output='screen', emulate_tty=True))
    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory("ur_teleop")
    config_file = os.path.join(pkg_share, "config", "ur_teleop.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config_file),
        DeclareLaunchArgument("mode", default_value=""),
        DeclareLaunchArgument("force_home", default_value="false"),
        DeclareLaunchArgument("ruckig_control_hz", default_value=""),
        DeclareLaunchArgument("use_ruckig", default_value=""),
        OpaqueFunction(function=_nodes),
    ])
