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
