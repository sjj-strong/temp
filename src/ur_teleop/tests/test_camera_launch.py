"""只展开相机启动配置，不启动设备或预览窗口。"""
import runpy
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_camera_config_separated_from_teleop():
    camera = yaml.safe_load((ROOT / 'config/camera.yaml').read_text())
    assert camera['cameras']['opencv']['enabled'] is True
    assert 'config_file' not in camera['cameras']['opencv']
    assert [entry['name'] for entry in camera['opencv_cameras']] == [
        'usb_front', 'usb_left', 'usb_right']
    assert not (ROOT / 'config/opencv_cameras.yaml').exists()
    for name in ('alicia_teleop.yaml', 'xbot_teleop.yaml'):
        config = yaml.safe_load((ROOT / 'config' / name).read_text())
        assert 'cameras' not in config
        assert config['recorder']['cameras'] == {}


def test_selected_config_and_cli_override(tmp_path):
    pytest.importorskip('launch')
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters

    module = runpy.run_path(str(ROOT / 'launch/camera.launch.py'))
    config = tmp_path / 'custom.yaml'
    config.write_text(yaml.safe_dump({'cameras': {
        'realsense': {'enabled': True, 'd435i_serial': 'custom_serial'},
        'opencv': {'enabled': False},
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
    assert 'opencv_camera_config' not in values
    usb = [action for action in actions if isinstance(action, IncludeLaunchDescription)][1]
    assert dict(usb.launch_arguments)['camera_config'] == str(config)
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


def test_unified_config_accepted_by_usb_driver(tmp_path):
    """驱动直接读取同一文件的 USB 参数，并保留逐台启用开关。"""
    camera_config = pytest.importorskip('data_collection.camera_config')
    specs = camera_config.load_camera_specs(ROOT / 'config/camera.yaml')
    assert len(specs) == 3
    assert specs[0].width == 640 and specs[0].height == 480
    assert specs[0].fps == 30 and specs[0].fourcc == 'MJPG'
    config = yaml.safe_load((ROOT / 'config/camera.yaml').read_text())
    config['opencv_cameras'][1]['enabled'] = False
    config['opencv_cameras'][0].update(width=320, height=240, fps=15)
    custom = tmp_path / 'camera.yaml'
    custom.write_text(yaml.safe_dump(config))
    specs = camera_config.load_camera_specs(custom)
    assert [spec.name for spec in specs] == ['usb_front', 'usb_right']
    assert (specs[0].width, specs[0].height, specs[0].fps) == (320, 240, 15)
