"""相机检查逻辑测试，不访问实际相机或机器人。"""
import pytest
import yaml
from ur_teleop.camera_inspector import parse_controls, snippet


def test_controls_preserve_ranges_and_menu():
    controls = parse_controls('''
 brightness 0x00980900 (int) : min=-64 max=64 step=1 default=0 value=-3
 exposure_auto 0x009a0901 (menu) : min=0 max=3 default=3 value=3
    1: Manual Mode
    3: Aperture Priority Mode
 exposure_absolute 0x009a0902 (int) : min=1 max=10000 step=1 value=100 flags=inactive
 readout 0x009a0903 (int) : min=0 max=50 value=1 flags=read-only
''')
    assert [c['name'] for c in controls] == ['brightness', 'exposure_auto', 'exposure_absolute']
    assert controls[0]['min'] == -64
    assert controls[0]['value'] == -3
    assert controls[1]['menu'] == {1: 'Manual Mode', 3: 'Aperture Priority Mode'}


def test_readonly_menu_does_not_pollute_previous_control():
    controls = parse_controls('''
 exposure_auto 0x009a0901 (menu) : min=0 max=3 value=3
    3: Auto
 ignored 0x009a0902 (menu) : min=0 max=5 value=0 flags=read-only
    5: Ignored
''')
    assert controls[0]['menu'] == {3: 'Auto'}


def test_usb_yaml_uses_stable_path():
    config = yaml.safe_load(snippet(dict(kind='usb', path='/dev/v4l/by-path/usb-port'),
                                    1280, 720, 30, 'MJPG', 'usb_left'))
    entry = config['opencv_cameras'][0]
    assert entry['device'] == '/dev/v4l/by-path/usb-port'
    assert entry['width'] == 1280
    assert entry['topic'] == '/camera/usb_left/color/image_raw'
    assert entry['fourcc'] == 'MJPG'
    assert 'exposure' not in entry


@pytest.mark.parametrize('model,key,enabled', [('Intel RealSense D455', 'd455_serial', False),
                                             ('Intel RealSense D435I', 'd435i_serial', True)])
def test_realsense_yaml_preserves_serial_string(model, key, enabled):
    config = yaml.safe_load(snippet(dict(kind='realsense', model=model, serial='001234567890'),
                                    640, 480, 30, 'MJPG', 'unused'))['cameras']['realsense']
    assert config[key] == '001234567890'
    assert config['enable_d435i'] is enabled
    assert config['color_profile'] == '640,480,30'


def test_unsupported_model_and_invalid_name():
    with pytest.raises(ValueError):
        snippet(dict(kind='realsense', model='L515', serial='123'), 640, 480, 30, 'MJPG', 'unused')
    with pytest.raises(ValueError):
        snippet(dict(kind='usb', path='/dev/video0'), 640, 480, 30, 'MJPG', 'bad/name')
