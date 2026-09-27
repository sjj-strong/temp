"""只展开启动动作，验证分支和实机控制器顺序，不启动硬件。"""
import runpy
from pathlib import Path
import pytest
import yaml


def test_real_home_does_not_activate_impedance(tmp_path):
    pytest.importorskip('launch')
    from launch import LaunchContext
    path = Path(__file__).resolve().parents[1]
    config = tmp_path / 'config.yaml'
    config.write_text(yaml.safe_dump(dict(base_config=str(path / 'config/xbot_teleop.yaml'), sim=False)))
    context = LaunchContext()
    context.launch_configurations['config_file'] = str(config)
    module = runpy.run_path(str(path / 'launch/xbot_cell.launch.py'))
    actions = module['_build_cell'](context)
    arguments = dict(actions[0].launch_arguments)
    assert arguments['use_cartesian_impedance'] == 'false'
    assert arguments['activate_joint_controller'] == 'true'
    assert arguments['initial_joint_controller'] == 'scaled_joint_trajectory_controller'


def test_xbot_launch_skips_alicia_and_ruckig():
    pytest.importorskip('launch')
    from launch import LaunchContext
    path = Path(__file__).resolve().parents[1]
    context = LaunchContext()
    context.launch_configurations.update(config_file=str(path / 'config/xbot_teleop.yaml'), mode='')
    module = runpy.run_path(str(path / 'launch/teleop.launch.py'))
    nodes = module['_nodes'](context)
    executables = [n.node_executable for n in nodes]
    assert executables == ['joy_node', 'xbot_teleop_node', 'data_recorder']


def test_controller_and_recording_use_tool0():
    from ur_teleop.config import load_config
    path = Path(__file__).resolve().parents[1] / 'config'
    cfg = load_config(path / 'xbot_teleop.yaml')
    assert cfg['recorder']['ee_pose_parent_frame'] == 'base_link'
    assert cfg['recorder']['ee_pose_child_frame'] == 'tool0'
    for name in ('xbot_cartesian_sim.yaml', 'xbot_cartesian_type.yaml'):
        data = yaml.safe_load((path / name).read_text())
        params = next(value['ros__parameters'] for key, value in data.items()
                      if key.endswith('cartesian_impedance_controller'))
        assert params['base_frame'] == 'base_link'
        assert params['tip_frame'] == 'tool0'


def test_feedback_queries_tool0():
    pytest.importorskip('rclpy')
    from types import SimpleNamespace
    from rclpy.time import Time
    from rclpy.clock import ClockType
    from geometry_msgs.msg import TransformStamped
    from ur_teleop.xbot_teleop_node import XbotTeleopNode
    node = object.__new__(XbotTeleopNode)
    now = Time(seconds=10, clock_type=ClockType.ROS_TIME)
    transform = TransformStamped()
    transform.header.stamp = now.to_msg()
    transform.transform.rotation.w = 1.
    calls = []

    def lookup(parent, child, stamp):
        calls.append((parent, child))
        return transform

    node.buffer = SimpleNamespace(lookup_transform=lookup)
    node.get_clock = lambda: SimpleNamespace(now=lambda: now)
    node.x = dict(tcp_timeout_s=.25)
    assert node.actual_pose() is not None
    assert calls == [('base_link', 'tool0')]
