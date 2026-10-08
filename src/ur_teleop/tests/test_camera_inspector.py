"""Camera inspection logic tests without accessing physical cameras or robots."""
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


def test_discovery_filters_metadata_and_uses_sdk_serial(tmp_path, monkeypatch):
    """USB descriptors and the SDK can report different RealSense serials."""
    import sys
    from pathlib import Path
    from types import SimpleNamespace
    from ur_teleop import camera_inspector as inspector

    nodes = tmp_path / 'video4linux'
    nodes.mkdir()
    for prefix, model, serial, indexes in [('usb', 'USB Camera', 'same', [0, 1]),
                                          ('rs', 'Intel RealSense Depth Camera 455', 'descriptor', [0, 1])]:
        parent = tmp_path / prefix
        parent.mkdir()
        for key, value in [('product', model), ('serial', serial), ('idVendor', '8086')]:
            (parent / key).write_text(value)
        interface = parent / 'interface'
        interface.mkdir()
        for index in indexes:
            node = nodes / f'video{index + (4 if prefix == "rs" else 0)}'
            node.mkdir()
            (node / 'device').symlink_to(interface, target_is_directory=True)
            (node / 'index').write_text(str(index))
    real_path = Path
    monkeypatch.setattr(inspector, 'Path', lambda path: nodes if path == '/sys/class/video4linux'
                        else real_path(path))
    info = SimpleNamespace(name='name', serial_number='serial', physical_port='port')
    values = dict(name='RealSense D455', serial='sdk_serial', port=str(tmp_path / 'rs' / 'interface'))
    device = SimpleNamespace(get_info=lambda field: values[field])
    sdk = SimpleNamespace(camera_info=info,
                          context=lambda: SimpleNamespace(query_devices=lambda: [device]))
    monkeypatch.setitem(sys.modules, 'pyrealsense2', sdk)
    devices, _ = inspector.discover()
    assert len(devices) == 2
    assert devices[0]['device'] == '/dev/video0'
    assert devices[1]['serial'] == 'sdk_serial'
    assert devices[1]['nodes'] == ['/dev/video4', '/dev/video5']
