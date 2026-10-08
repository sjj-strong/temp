"""Camera inspection, live preview and native controls before dataset collection."""
import argparse
import json
import re
import shutil
import subprocess
import threading
from pathlib import Path


def v4l(device, *args):
    """Keep driver errors visible when a control cannot be applied."""
    if not shutil.which('v4l2-ctl'):
        raise RuntimeError('Install v4l-utils to query formats and adjust USB camera controls.')
    result = subprocess.run(['v4l2-ctl', '-d', device, *args],
                            capture_output=True, text=True, timeout=5)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip())
    return result.stdout


def parse_controls(output):
    """Parse writable numeric controls and menus from the device."""
    controls = []
    active = None
    for line in output.splitlines():
        match = re.match(r'\s*(\w+)\s+0x[\da-f]+\s+\((\w+)\)\s*:\s*(.*)', line)
        if not match:
            menu = re.match(r'\s+(\d+):\s*(.*)', line)
            if menu and active and active['kind'] in ('menu', 'intmenu'):
                active['menu'][int(menu[1])] = menu[2]
            continue
        active = None
        name, kind, values = match.groups()
        if kind not in ('int', 'bool', 'menu', 'intmenu') or 'read-only' in values:
            continue
        fields = {k: int(v) for k, v in re.findall(r'(min|max|step|value)=(-?\d+)', values)}
        active = dict(name=name, kind=kind, min=fields.get('min', 0),
                      max=fields.get('max', 1), step=fields.get('step', 1),
                      value=fields.get('value', 0), menu={})
        controls.append(active)
    return controls


def discover():
    """Discover video nodes and group RealSense nodes by serial number."""
    devices, warnings, realsense = [], [], {}
    for node in sorted(Path('/sys/class/video4linux').glob('video*')):
        device = '/dev/' + node.name
        port = (node / 'device').resolve()
        parent = next((p for p in port.parents if (p / 'idVendor').exists()), None)
        def attr(key):
            return (parent / key).read_text().strip() if parent and (parent / key).exists() else ''
        model = attr('product') or (node / 'name').read_text().strip()
        serial = attr('serial')
        if 'realsense' in model.lower() and serial:
            entry = realsense.setdefault(serial, dict(kind='realsense', model=model,
                serial=serial, port=str(parent), nodes=[], accessible=False))
            entry['nodes'].append(device)
            entry['accessible'] |= Path(device).exists()
            continue
        # Metadata nodes are not separate cameras.
        if (node / 'index').exists() and (node / 'index').read_text().strip() != '0':
            continue
        paths = [str(p) for p in Path('/dev/v4l/by-path').glob('*') if str(p.resolve()) == device]
        ids = [str(p) for p in Path('/dev/v4l/by-id').glob('*') if str(p.resolve()) == device]
        devices.append(dict(kind='usb', model=model, serial=serial, device=device,
                            path=paths[0] if paths else device, by_id=ids,
                            port=str(port), accessible=Path(device).exists()))
    try:
        import pyrealsense2 as rs
        for dev in rs.context().query_devices():
            serial = dev.get_info(rs.camera_info.serial_number)
            sdk_port = dev.get_info(rs.camera_info.physical_port)
            fallback = next((key for key, entry in realsense.items()
                             if key == serial or sdk_port.startswith(entry['port'] + '/')), None)
            nodes = realsense.pop(fallback)['nodes'] if fallback else []
            # The SDK serial is the identifier accepted by RealSense pipelines.
            realsense[serial] = dict(kind='realsense', model=dev.get_info(rs.camera_info.name),
                                    serial=serial, port=sdk_port, nodes=nodes, accessible=True)
    except ImportError:
        warnings.append('pyrealsense2 is missing. Serial information is available; RealSense preview is unavailable.')
    except RuntimeError as exc:
        warnings.append('RealSense SDK discovery failed: ' + str(exc))
    devices.extend(realsense.values())
    if not devices:
        warnings.append('No cameras found. Check USB connections, device mapping and permissions.')
    elif not any(d['accessible'] for d in devices):
        warnings.append('Cameras are detected but video nodes are unavailable inside the container. Preview is unavailable.')
    if any(d['kind'] == 'usb' and d['path'] == d['device'] for d in devices):
        warnings.append('Stable /dev/v4l/by-path links are missing. Video node numbers may change.')
    return devices, warnings


def snippet(device, width, height, fps, fourcc, name):
    """Generate only fields supported by the existing YAML configurations."""
    import yaml
    if device['kind'] == 'usb':
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
            raise ValueError('Name must start with a letter and contain only letters, numbers and underscores.')
        return yaml.safe_dump({'opencv_cameras': [dict(
            name=name, enabled=True, device=device['path'], topic=f'/camera/{name}/color/image_raw',
            frame_id=f'{name}_color_optical_frame', width=width, height=height, fps=fps,
            fourcc=fourcc, publish_compressed=True, compressed_quality=90)]},
            allow_unicode=True, sort_keys=False)
    model = device['model'].lower()
    if 'd435' in model or 'camera 435' in model:
        key = 'd435i_serial'
    elif 'd455' in model or 'camera 455' in model:
        key = 'd455_serial'
    else:
        raise ValueError('The existing launch configuration supports only D435-series and D455 cameras.')
    return yaml.safe_dump({'cameras': {'realsense': dict(
        enabled=True, **{key: device['serial']}, enable_d435i=key == 'd435i_serial',
        enable_color=True, enable_depth=False, color_profile=f'{width},{height},{fps}',
        camera_namespace='camera')}}, allow_unicode=True, sort_keys=False)


class Stream:
    """Capture in a worker thread and release the device when capture ends."""
    def __init__(self, device, width, height, fps, fourcc):
        import cv2
        self.cv2 = cv2
        self.frame = None
        self.error = ''
        self.stop = threading.Event()
        self.controls = []
        self.actual = (width, height, fps, fourcc)
        self.cap = self.pipeline = None
        try:
            if device['kind'] == 'usb':
                if not Path(device['path']).exists():
                    raise RuntimeError('Video node is missing. Map the device into the container before previewing.')
                self.cap = cv2.VideoCapture(device['path'], cv2.CAP_V4L2)
                if not self.cap.isOpened():
                    raise RuntimeError('Cannot open video node. It may be a metadata node, inaccessible or busy.')
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
                for prop, value in [(cv2.CAP_PROP_FRAME_WIDTH, width),
                                    (cv2.CAP_PROP_FRAME_HEIGHT, height), (cv2.CAP_PROP_FPS, fps)]:
                    self.cap.set(prop, value)
                code = int(self.cap.get(cv2.CAP_PROP_FOURCC))
                self.actual = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                               int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                               self.cap.get(cv2.CAP_PROP_FPS),
                               ''.join(chr((code >> (8*i)) & 255) for i in range(4)))
                try:
                    self.controls = parse_controls(v4l(device['device'], '--list-ctrls-menus'))
                except RuntimeError as exc:
                    self.error = str(exc)
            else:
                import pyrealsense2 as rs
                self.pipeline = rs.pipeline()
                config = rs.config()
                config.enable_device(device['serial'])
                config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
                profile = self.pipeline.start(config)
                self.actual = (width, height, fps, 'BGR8')
                self.sensor = profile.get_device().first_color_sensor()
                for option in self.sensor.get_supported_options():
                    if self.sensor.is_option_read_only(option):
                        continue
                    limits = self.sensor.get_option_range(option)
                    self.controls.append(dict(name=str(option), option=option, min=limits.min,
                                              max=limits.max, step=limits.step,
                                              value=self.sensor.get_option(option), menu={}, kind='float'))
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()
        except Exception:
            if self.cap is not None:
                self.cap.release()
            if self.pipeline is not None:
                try:
                    self.pipeline.stop()
                except RuntimeError:
                    pass
            raise

    def run(self):
        try:
            while not self.stop.is_set():
                if self.cap is not None:
                    ok, frame = self.cap.read()
                    if not ok:
                        raise RuntimeError('Frame capture failed. Check connection, device access and supported formats.')
                else:
                    import numpy as np
                    color = self.pipeline.wait_for_frames(1500).get_color_frame()
                    if not color:
                        continue
                    frame = np.asanyarray(color.get_data())
                self.frame = frame
        except Exception as exc:
            self.error = str(exc)
        finally:
            if self.cap is not None:
                self.cap.release()
            if self.pipeline is not None:
                self.pipeline.stop()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError('Capture has not stopped yet. Wait for the device to respond and retry.')

    def set_control(self, device, control, value):
        if self.cap is not None:
            v4l(device['device'], f"--set-ctrl={control['name']}={int(value)}")
            return v4l(device['device'], f"--get-ctrl={control['name']}").strip()
        self.sensor.set_option(control['option'], float(value))
        return f"{control['name']}: {self.sensor.get_option(control['option'])}"


def create_window(devices, warnings):
    """Build a minimal desktop window; keep Qt optional for --list."""
    from PySide6 import QtCore, QtWidgets
    import pyqtgraph as pg
    import numpy as np

    class CameraWindow(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.devices = devices
            self.stream = None
            self.current = None
            self.last_frame = None
            self.setWindowTitle('Camera Setup')
            self.resize(1100, 780)
            layout = QtWidgets.QVBoxLayout(self)
            row = QtWidgets.QHBoxLayout()
            self.choice = QtWidgets.QComboBox()
            row.addWidget(self.choice, 1)
            refresh = QtWidgets.QPushButton('Refresh Devices')
            refresh.clicked.connect(lambda: self.safe(self.refresh))
            row.addWidget(refresh)
            layout.addLayout(row)
            self.info = QtWidgets.QLabel()
            self.info.setWordWrap(True)
            self.info.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
            layout.addWidget(self.info)
            row = QtWidgets.QHBoxLayout()
            self.name = QtWidgets.QLineEdit('usb_front')
            self.name.setMaximumWidth(120)
            row.addWidget(QtWidgets.QLabel('Name'))
            row.addWidget(self.name)
            self.profile = []
            for label, default in [('Width', 640), ('Height', 480), ('FPS', 30)]:
                field = QtWidgets.QSpinBox()
                field.setRange(1, 16384 if label != 'FPS' else 240)
                field.setValue(default)
                row.addWidget(QtWidgets.QLabel(label))
                row.addWidget(field)
                self.profile.append(field)
            self.fourcc = QtWidgets.QComboBox()
            self.fourcc.setEditable(True)
            self.fourcc.addItems(['MJPG', 'YUYV'])
            row.addWidget(QtWidgets.QLabel('USB Format'))
            row.addWidget(self.fourcc)
            layout.addLayout(row)
            row = QtWidgets.QHBoxLayout()
            for label, action in [('Start / Restart', self.start), ('Stop', self.stop),
                                  ('Supported Formats', self.formats), ('YAML Parameters', self.generate)]:
                button = QtWidgets.QPushButton(label)
                button.clicked.connect(lambda checked=False, action=action: self.safe(action))
                row.addWidget(button)
            layout.addLayout(row)
            row = QtWidgets.QHBoxLayout()
            self.canvas = pg.GraphicsLayoutWidget()
            self.view = self.canvas.addViewBox(lockAspect=True, enableMenu=False)
            self.view.invertY(True)
            self.view.setMouseEnabled(x=False, y=False)
            self.image = pg.ImageItem(axisOrder='row-major')
            self.view.addItem(self.image)
            row.addWidget(self.canvas, 3)
            panel = QtWidgets.QVBoxLayout()
            panel.addWidget(QtWidgets.QLabel('Camera Controls'))
            self.controls = QtWidgets.QListWidget()
            self.controls.currentRowChanged.connect(self.select_control)
            panel.addWidget(self.controls)
            self.value = QtWidgets.QDoubleSpinBox()
            self.value.setDecimals(4)
            self.value.setKeyboardTracking(False)
            self.value.editingFinished.connect(lambda: self.safe(self.apply))
            self.menu = QtWidgets.QComboBox()
            self.menu.activated.connect(lambda _: self.safe(self.apply))
            panel.addWidget(self.value)
            panel.addWidget(self.menu)
            hint = QtWidgets.QLabel('Change a value and press Enter.\nDisable auto exposure / white balance\nbefore changing manual values.')
            hint.setWordWrap(True)
            panel.addWidget(hint)
            row.addLayout(panel, 1)
            layout.addLayout(row, 1)
            self.output = QtWidgets.QPlainTextEdit()
            self.output.setReadOnly(True)
            self.output.setMaximumHeight(170)
            layout.addWidget(self.output)
            self.status = QtWidgets.QLabel('; '.join(warnings))
            self.status.setWordWrap(True)
            layout.addWidget(self.status)
            self.choice.currentIndexChanged.connect(lambda _: self.safe(self.change_device))
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.tick)
            self.timer.start(33)
            self.populate()

        def safe(self, action):
            try:
                action()
            except Exception as exc:
                self.status.setText(str(exc))

        def selected(self):
            index = self.choice.currentIndex()
            if index < 0:
                raise ValueError('Select a camera first.')
            return self.devices[index]

        def populate(self):
            self.choice.blockSignals(True)
            self.choice.clear()
            for device in self.devices:
                self.choice.addItem(f"{device['model']} | {device.get('device', device.get('serial'))}")
            self.choice.blockSignals(False)
            self.value.setEnabled(False)
            self.menu.hide()
            if self.devices:
                self.change_device()
            else:
                self.info.setText('No cameras found.')

        def change_device(self):
            self.stop()
            device = self.selected()
            self.info.setText(json.dumps(device, ensure_ascii=True))
            self.name.setEnabled(device['kind'] == 'usb')
            self.fourcc.setEnabled(device['kind'] == 'usb')

        def refresh(self):
            self.stop()
            self.devices, notes = discover()
            self.populate()
            self.status.setText('; '.join(notes) or 'Device list refreshed.')

        def settings(self):
            fourcc = self.fourcc.currentText()
            if len(fourcc) != 4:
                raise ValueError('USB format must contain four characters.')
            return (*[field.value() for field in self.profile], fourcc)

        def start(self):
            self.stop()
            device = self.selected()
            self.stream = Stream(device, *self.settings())
            self.current = device
            self.controls.addItems([control['name'] for control in self.stream.controls])
            self.status.setText(f'Actual width, height, FPS, format: {self.stream.actual}')
            if self.stream.controls:
                self.controls.setCurrentRow(0)

        def stop(self):
            if self.stream:
                self.stream.close()
            self.stream = self.current = self.last_frame = None
            self.controls.clear()
            self.value.setEnabled(False)
            self.image.clear()

        def select_control(self, index):
            if not self.stream or index < 0:
                return
            control = self.stream.controls[index]
            self.value.blockSignals(True)
            self.value.setDecimals(4 if control['kind'] == 'float' else 0)
            self.value.setRange(control['min'], control['max'])
            self.value.setSingleStep(control['step'] or 1)
            self.value.setValue(control['value'])
            self.value.blockSignals(False)
            self.value.setEnabled(True)
            self.value.setVisible(not bool(control['menu']))
            self.menu.clear()
            for value, label in control['menu'].items():
                self.menu.addItem(f'{value}: {label}', value)
            self.menu.setCurrentIndex(self.menu.findData(int(control['value'])))
            self.menu.setVisible(bool(control['menu']))

        def apply(self):
            index = self.controls.currentRow()
            if not self.stream or index < 0:
                return
            control = self.stream.controls[index]
            value = self.menu.currentData() if control['menu'] else self.value.value()
            if value is None:
                raise ValueError('Select a supported menu value.')
            self.status.setText('Device readback: ' + self.stream.set_control(self.current, control, value))
            control['value'] = value

        def tick(self):
            if not self.stream:
                return
            frame = self.stream.frame
            if frame is not None and frame is not self.last_frame:
                self.last_frame = frame
                # Fixed display levels preserve visible changes in camera brightness.
                self.image.setImage(np.ascontiguousarray(frame[:, :, ::-1]),
                                    autoLevels=False, levels=(0, 255))
                self.view.setRange(xRange=(0, frame.shape[1]), yRange=(0, frame.shape[0]), padding=0)
            if self.stream.error:
                self.status.setText(self.stream.error)

        def generate(self):
            device = self.selected()
            width, height, fps, fourcc = self.settings()
            if self.stream and self.current == device:
                width, height, fps, fourcc = self.stream.actual
                fps = int(round(fps))
                if min(width, height, fps) <= 0:
                    raise ValueError('The driver did not return a valid profile.')
            self.output.setPlainText(snippet(device, width, height, fps, fourcc, self.name.text()))
            self.status.setText('Copy the YAML fields into the matching configuration. Camera controls are session settings.')

        def formats(self):
            device = self.selected()
            if device['kind'] == 'usb':
                text = v4l(device['device'], '--list-formats-ext')
            else:
                import pyrealsense2 as rs
                dev = next(d for d in rs.context().query_devices()
                           if d.get_info(rs.camera_info.serial_number) == device['serial'])
                text = '\n'.join(str(p) for sensor in dev.query_sensors() for p in sensor.get_stream_profiles())
            self.output.setPlainText(text)

        def closeEvent(self, event):
            try:
                self.stop()
            except Exception as exc:
                self.status.setText(str(exc))
                event.ignore()
                return
            self.timer.stop()
            event.accept()

    return CameraWindow()


def gui(devices, warnings):
    from PySide6 import QtWidgets
    import pyqtgraph as pg
    app = pg.mkQApp('Camera Setup')
    window = create_window(devices, warnings)
    window.show()
    return app.exec()


def main():
    parser = argparse.ArgumentParser(description='Camera model and port inspection, live preview, controls and YAML fields.')
    parser.add_argument('--list', action='store_true', help='Print device information without opening the interface.')
    args = parser.parse_args()
    devices, warnings = discover()
    if args.list:
        print(json.dumps(dict(devices=devices, warnings=warnings), ensure_ascii=False, indent=2))
    else:
        gui(devices, warnings)


if __name__ == '__main__':
    main()
