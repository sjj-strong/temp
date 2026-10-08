"""只展开相机启动配置，不启动设备或预览窗口。"""
import runpy
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_camera_config_separated_from_teleop():
    camera = yaml.safe_load((ROOT / 'config/camera.yaml').read_text())
    assert camera['cameras']['opencv']['enabled'] is True
    for name in ('alicia_teleop.yaml', 'xbot_teleop.yaml'):
        config = yaml.safe_load((ROOT / 'config' / name).read_text())
        assert 'cameras' not in config
        assert config['recorder']['cameras'] == {}


def test_selected_config_and_cli_override(tmp_path):
    pytest.importorskip('launch')
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters

    module = runpy.run_path(str(ROOT / 'launch/camera.launch.py'))
    config = tmp_path / 'custom.yaml'
    config.write_text(yaml.safe_dump({'cameras': {
        'realsense': {'enabled': True, 'd435i_serial': 'custom_serial'},
        'opencv': {'enabled': False, 'config_file': 'devices.yaml'},
        'visualization': {'enabled': True, 'topics': ['/custom/image']},
    }}))
    context = LaunchContext()
    context.launch_configurations.update(
        config_file=str(config), launch_realsense='false')
    actions = module['_camera_actions'](context)
    for action in actions:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    values = context.launch_configurations
    assert values['launch_realsense'] == 'false'
    assert values['launch_opencv_cameras'] == 'false'
    assert values['d435i_serial'] == 'custom_serial'
    assert values['opencv_camera_config'] == str(tmp_path / 'devices.yaml')
    node = next(action for action in actions if isinstance(action, Node))
    assert list(evaluate_parameters(context, node._Node__parameters)[0]['topics']) == ['/custom/image']


def test_default_config_and_invalid_config(tmp_path):
    pytest.importorskip('launch')
    from launch import LaunchContext
    module = runpy.run_path(str(ROOT / 'launch/camera.launch.py'))
    context = LaunchContext()
    module['generate_launch_description']().entities[0].execute(context)
    assert context.launch_configurations['config_file'].endswith('/config/camera.yaml')
    config = tmp_path / 'invalid.yaml'
    config.write_text('recorder: {}\n')
    context.launch_configurations['config_file'] = str(config)
    with pytest.raises(ValueError, match='相机配置'):
        module['_camera_actions'](context)
