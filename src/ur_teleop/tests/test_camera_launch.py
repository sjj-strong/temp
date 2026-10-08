"""只展开独立相机启动配置，不访问设备或启动预览窗口。"""
import runpy
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def camera_actions(tmp_path, config):
    pytest.importorskip('launch')
    from launch import LaunchContext
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    path = tmp_path / 'camera.yaml'
    path.write_text(yaml.safe_dump(config, sort_keys=False))
    context = LaunchContext()
    context.launch_configurations['config_file'] = str(path)
    module = runpy.run_path(str(ROOT / 'launch/camera.launch.py'))
    actions = module['_camera_actions'](context)
    parameters = [evaluate_parameters(context, action._Node__parameters)[0]
                  for action in actions if isinstance(action, Node)]
    return actions, parameters


def test_default_camera_config(tmp_path):
    config = yaml.safe_load((ROOT / 'config/camera.yaml').read_text())
    assert 'cameras' not in config and 'opencv_cameras' not in config
    assert list(config) == ['usb_front', 'usb_left', 'usb_right', 'd435i', 'd455']
    _, params = camera_actions(tmp_path, config)
    assert [p['camera_name'] for p in params if 'camera_name' in p] == [
        'usb_front', 'usb_left', 'd455']
    assert list(params[-1]['topics']) == [f'/camera/{name}/color/image_raw'
                                    for name in ('usb_front', 'usb_left', 'd455')]
    for name in ('alicia_teleop.yaml', 'xbot_teleop.yaml'):
        teleop = yaml.safe_load((ROOT / 'config' / name).read_text())
        assert 'cameras' not in teleop
        assert teleop['recorder']['cameras'] == {}


def test_custom_names_and_independent_preview(tmp_path):
    _, params = camera_actions(tmp_path, dict(
        front=dict(type='usb', device='/dev/video2', visualize=True, width=320, height=240, fps=15),
        wrist=dict(type='realsense', serial_no='001234567890', visualize=False),
        unused=dict(type='usb', enabled=False, visualize=True)))
    assert params[0]['camera_name'] == 'front'
    assert params[0]['image_topic'] == '/camera/front/color/image_raw'
    assert params[0]['frame_id'] == 'front_color_optical_frame'
    assert (params[0]['width'], params[0]['height'], params[0]['fps']) == (320, 240, 15.)
    assert params[1]['camera_name'] == 'wrist'
    assert params[1]['serial_no'] == '001234567890'
    assert params[1]['rgb_camera.color_profile'] == '640,480,30'
    assert list(params[-1]['topics']) == ['/camera/front/color/image_raw']


def test_multiple_realsense_cameras_are_independent(tmp_path):
    _, params = camera_actions(tmp_path, dict(
        left=dict(type='realsense', serial_no='123', visualize=True, width=1280, height=720, fps=15),
        right=dict(type='realsense', serial_no='456', visualize=True, enable_depth=True)))
    assert [p['serial_no'] for p in params[:2]] == ['123', '456']
    assert params[0]['rgb_camera.color_profile'] == '1280,720,15'
    assert params[1]['enable_depth'] is True
    assert list(params[-1]['topics']) == ['/camera/left/color/image_raw', '/camera/right/color/image_raw']


def test_custom_topic_applies_to_driver_and_preview(tmp_path):
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node
    actions, params = camera_actions(tmp_path, dict(
        wrist=dict(type='realsense', serial_no='123', topic='/custom/image', visualize=True)))
    node = next(a for a in actions if isinstance(a, Node))
    source, target = node._Node__remappings[0]
    context = LaunchContext()
    assert perform_substitutions(context, source) == '/camera/wrist/color/image_raw'
    assert perform_substitutions(context, target) == '/custom/image'
    assert list(params[-1]['topics']) == ['/custom/image']


def test_no_preview_or_enabled_cameras(tmp_path):
    actions, params = camera_actions(tmp_path, {'front': dict(type='usb', device=0, visualize=False)})
    assert len(actions) == len(params) == 1
    assert params[0]['device'] == '0'
    actions, _ = camera_actions(tmp_path, {'front': dict(type='usb', enabled=False, visualize=True)})
    assert actions == []
    assert camera_actions(tmp_path, {})[0] == []


@pytest.mark.parametrize('config, message', [
    ({'cameras': {}}, 'type'),
    ({'bad/name': dict(type='usb')}, '名称'),
    ({'front': dict(type='other')}, 'type'),
    ({'front': dict(type='usb', device='/dev/video0', visualize='false')}, 'visualize'),
    ({'front': dict(type='usb')}, 'device'),
    ({'front': dict(type='realsense', serial_no=123)}, 'serial_no'),
    ({'front': dict(type='usb', device='/dev/video0', width=0)}, 'width'),
])
def test_invalid_camera_config(tmp_path, config, message):
    with pytest.raises(ValueError, match=message):
        camera_actions(tmp_path, config)


def test_default_config_argument():
    pytest.importorskip('launch')
    from launch import LaunchContext
    module = runpy.run_path(str(ROOT / 'launch/camera.launch.py'))
    context = LaunchContext()
    module['generate_launch_description']().entities[0].execute(context)
    assert context.launch_configurations['config_file'].endswith('/config/camera.yaml')
