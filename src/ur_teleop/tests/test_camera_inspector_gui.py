"""Verify the Qt preview and controls using simulated cameras."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import numpy as np
import pytest

pytest.importorskip('PySide6')
pytest.importorskip('pyqtgraph')
import pyqtgraph as pg
from ur_teleop import camera_inspector as inspector


@pytest.fixture
def window(monkeypatch):
    app = pg.mkQApp('Camera Setup Tests')
    devices = [dict(kind='usb', model='USB Camera', device=f'/dev/video{i}',
                    path=f'/dev/video{i}') for i in (0, 2)]

    class FakeStream:
        def __init__(self, device, *settings):
            self.actual = settings
            self.frame = np.array([[[10, 20, 30], [40, 50, 60]],
                                   [[70, 80, 90], [100, 110, 120]]], dtype=np.uint8)
            self.error = ''
            self.closed = False
            self.applied = []
            self.controls = [dict(name='brightness', min=-64, max=64, step=1,
                                  value=0, kind='int', menu={}),
                             dict(name='exposure_auto', min=1, max=3, step=1,
                                  value=3, kind='menu', menu={1: 'Manual', 3: 'Auto'})]

        def close(self):
            self.closed = True

        def set_control(self, device, control, value):
            self.applied.append((device, control['name'], value))
            return f"{control['name']}: {value}"

    monkeypatch.setattr(inspector, 'Stream', FakeStream)
    widget = inspector.create_window(devices, [])
    widget.show()
    app.processEvents()
    yield widget
    widget.close()
    widget.deleteLater()
    app.processEvents()


def test_preview_preserves_rgb_and_fixed_display_levels(window):
    window.start()
    window.tick()
    np.testing.assert_array_equal(window.image.image, window.stream.frame[:, :, ::-1])
    np.testing.assert_array_equal(window.image.getLevels(), [0, 255])
    assert window.image.axisOrder == 'row-major'


def test_numeric_and_menu_controls_apply_immediately(window):
    window.start()
    stream = window.stream
    window.value.setValue(12)
    window.value.editingFinished.emit()
    assert stream.applied[-1][1:] == ('brightness', 12)
    assert window.status.text() == 'Device readback: brightness: 12.0'
    window.controls.setCurrentRow(1)
    window.menu.setCurrentIndex(window.menu.findData(1))
    window.menu.activated.emit(window.menu.currentIndex())
    assert stream.applied[-1][1:] == ('exposure_auto', 1)


def test_switching_device_stops_previous_stream(window):
    window.start()
    previous = window.stream
    window.choice.setCurrentIndex(1)
    assert previous.closed
    assert window.stream is None
    assert window.image.image is None
    window.start()
    current = window.stream
    window.close()
    assert current.closed
    assert not window.timer.isActive()


def test_yaml_uses_active_profile_and_errors_are_visible(window):
    window.start()
    window.stream.actual = (1280, 720, 30, 'YUYV')
    window.generate()
    assert 'width: 1280' in window.output.toPlainText()
    assert 'fourcc: YUYV' in window.output.toPlainText()
    window.stop()
    window.fourcc.setCurrentText('invalid')
    window.safe(window.start)
    assert window.stream is None
    assert window.status.text() == 'USB format must contain four characters.'


def test_realsense_name_is_editable_and_exported(window):
    """RealSense 与 USB 均通过 Name 字段自定义配置名称。"""
    import yaml
    window.devices.append(dict(kind='realsense', model='RealSense D455', serial='00123'))
    window.populate()
    window.choice.setCurrentIndex(2)
    assert window.name.isEnabled()
    window.name.setText('wrist')
    window.generate()
    config = yaml.safe_load(window.output.toPlainText())
    assert list(config) == ['wrist']
    assert config['wrist']['type'] == 'realsense'
    assert config['wrist']['serial_no'] == '00123'
