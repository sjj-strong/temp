"""采集前相机检查、实时预览和原生调参；不启动机器人或 ROS。"""
import argparse
import base64
import json
import re
import shutil
import subprocess
import threading
from pathlib import Path


def v4l(device, *args):
    """保留驱动错误，禁止把设置失败报告为成功。"""
    if not shutil.which('v4l2-ctl'):
        raise RuntimeError('需要安装 v4l-utils 才能查询格式和调节 USB 参数')
    result = subprocess.run(['v4l2-ctl', '-d', device, *args],
                            capture_output=True, text=True, timeout=5)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip())
    return result.stdout


def parse_controls(output):
    """解析可写数值控制和菜单，取值范围来自实际设备。"""
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
    """使用系统设备信息和可选 SDK，避免把 RealSense 子节点当作多台相机。"""
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
        paths = [str(p) for p in Path('/dev/v4l/by-path').glob('*') if str(p.resolve()) == device]
        ids = [str(p) for p in Path('/dev/v4l/by-id').glob('*') if str(p.resolve()) == device]
        devices.append(dict(kind='usb', model=model, serial=serial, device=device,
                            path=paths[0] if paths else device, by_id=ids,
                            port=str(port), accessible=Path(device).exists()))
    try:
        import pyrealsense2 as rs
        for dev in rs.context().query_devices():
            serial = dev.get_info(rs.camera_info.serial_number)
            realsense[serial] = dict(kind='realsense', model=dev.get_info(rs.camera_info.name),
                                    serial=serial, port=dev.get_info(rs.camera_info.physical_port),
                                    nodes=realsense.get(serial, {}).get('nodes', []), accessible=True)
    except ImportError:
        warnings.append('未安装 pyrealsense2：可从系统信息读取序列号，SDK 预览不可用。')
    except RuntimeError as exc:
        warnings.append('RealSense SDK 枚举失败：' + str(exc))
    devices.extend(realsense.values())
    if not devices:
        warnings.append('未发现相机，请检查 USB 连接、容器设备映射和权限。')
    elif not any(d['accessible'] for d in devices):
        warnings.append('系统能识别相机，但容器未暴露视频节点；当前只能查询，不能预览。')
    if any(d['kind'] == 'usb' and d['path'] == d['device'] for d in devices):
        warnings.append('没有 /dev/v4l/by-path 稳定路径，视频节点可能随插拔改变。')
    return devices, warnings


def snippet(device, width, height, fps, fourcc, name):
    """只生成现有 YAML 支持的字段，不擅自添加曝光等无效字段。"""
    import yaml
    if device['kind'] == 'usb':
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
            raise ValueError('配置名称应以英文字母开头，只使用字母、数字和下划线')
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
        raise ValueError('现有启动配置仅支持 D435 系列与 D455，此型号不能直接映射')
    return yaml.safe_dump({'cameras': {'realsense': dict(
        enabled=True, **{key: device['serial']}, enable_d435i=key == 'd435i_serial',
        enable_color=True, enable_depth=False, color_profile=f'{width},{height},{fps}',
        camera_namespace='camera')}}, allow_unicode=True, sort_keys=False)


class Stream:
    """后台取帧，主线程只绘图；取帧结束后释放设备。"""
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
                    raise RuntimeError('容器内缺少视频节点，请先映射设备后再预览')
                self.cap = cv2.VideoCapture(device['path'], cv2.CAP_V4L2)
                if not self.cap.isOpened():
                    raise RuntimeError('无法打开节点：可能是元数据节点、权限不足或设备已占用')
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
                        raise RuntimeError('取帧失败：检查设备断开、占用或格式支持情况')
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
            raise RuntimeError('设备读取仍未退出，请等待设备响应后重试')

    def set_control(self, device, control, value):
        if self.cap is not None:
            v4l(device['device'], f"--set-ctrl={control['name']}={int(value)}")
            return v4l(device['device'], f"--get-ctrl={control['name']}").strip()
        self.sensor.set_option(control['option'], float(value))
        return f"{control['name']}: {self.sensor.get_option(control['option'])}"


def gui(devices, warnings):
    import tkinter as tk
    from tkinter import ttk, messagebox
    root = tk.Tk()
    root.title('采集前相机检查与实时调参')
    root.geometry('1200x850')
    stream = current = image = None
    choice = ttk.Combobox(root, state='readonly', width=100)
    choice.pack(fill='x', padx=8, pady=8)
    info = tk.Text(root, height=6)
    info.pack(fill='x')
    status = tk.StringVar(value='；'.join(warnings))
    ttk.Label(root, textvariable=status, wraplength=1150).pack(fill='x')
    inputs = ttk.Frame(root)
    inputs.pack(fill='x')
    fields = {}
    for key, label, default in [('name', '配置名称', 'usb_front'), ('width', '宽', '640'),
                                ('height', '高', '480'), ('fps', '帧率', '30'), ('fourcc', 'USB 格式', 'MJPG')]:
        ttk.Label(inputs, text=label).pack(side='left')
        fields[key] = tk.StringVar(value=default)
        ttk.Entry(inputs, textvariable=fields[key], width=12).pack(side='left')
    buttons = ttk.Frame(root)
    buttons.pack(fill='x')
    middle = ttk.Frame(root)
    middle.pack(fill='both', expand=True)
    preview = ttk.Label(middle, text='实时预览（可逐台切换检查）')
    preview.pack(side='left', fill='both', expand=True)
    controls = tk.Listbox(middle, width=38, exportselection=False)
    controls.pack(side='right', fill='y')
    control_bar = ttk.Frame(root)
    control_bar.pack(fill='x')
    control_value = tk.DoubleVar()
    slider = tk.Scale(control_bar, orient='horizontal', variable=control_value, length=650)
    slider.pack(side='left')
    menu_text = tk.StringVar()
    ttk.Label(root, textvariable=menu_text, wraplength=1150).pack(fill='x')
    output = tk.Text(root, height=10)
    output.pack(fill='x')

    def selected():
        if choice.current() < 0:
            raise ValueError('请先选择相机')
        return devices[choice.current()]

    def values():
        width, height, fps = [int(fields[k].get()) for k in ('width', 'height', 'fps')]
        fourcc = fields['fourcc'].get()
        if min(width, height, fps) <= 0 or len(fourcc) != 4:
            raise ValueError('宽、高、帧率必须为正整数，USB 格式必须为四个字符')
        return width, height, fps, fourcc

    def guarded(action):
        def callback(*args):
            try:
                action(*args)
            except Exception as exc:
                messagebox.showerror('相机操作失败', str(exc))
        return callback

    def stop():
        nonlocal stream, current
        if stream:
            stream.close()
        stream = current = None
        controls.delete(0, 'end')
        preview.configure(image='', text='预览已关闭')

    def show_info(*_):
        info.delete('1.0', 'end')
        info.insert('end', json.dumps(selected(), ensure_ascii=False, indent=2))

    def refresh():
        nonlocal devices
        stop()
        devices, notes = discover()
        choice['values'] = [f"{d['model']} | {d.get('device', d.get('serial'))}" for d in devices]
        if devices:
            choice.current(0)
            show_info()
        else:
            choice.set('')
            info.delete('1.0', 'end')
        status.set('；'.join(notes) or '已刷新设备列表')

    def start():
        nonlocal stream, current
        stop()
        device = selected()
        stream = Stream(device, *values())
        current = device
        for ctrl in stream.controls:
            controls.insert('end', ctrl['name'])
        status.set(f'设备实际宽、高、帧率、格式：{stream.actual}；{stream.error}')

    def select_control(*_):
        if not stream or not controls.curselection():
            return
        ctrl = stream.controls[controls.curselection()[0]]
        slider.configure(from_=ctrl['min'], to=ctrl['max'], resolution=ctrl['step'] or 1)
        control_value.set(ctrl['value'])
        menu_text.set('菜单值：' + str(ctrl['menu']) if ctrl['menu'] else
                      '先关闭自动曝光/自动白平衡，再调手动值；单位由驱动定义。')

    def apply():
        if not stream or not controls.curselection():
            raise ValueError('请打开预览并选择要调节的参数')
        ctrl = stream.controls[controls.curselection()[0]]
        value = control_value.get()
        if ctrl['menu'] and int(value) not in ctrl['menu']:
            raise ValueError('请选择菜单中存在的数值')
        status.set('设备回读：' + stream.set_control(current, ctrl, value))
        ctrl['value'] = value

    def generate():
        device = selected()
        width, height, fps, fourcc = values()
        if stream and current == device:
            width, height, fps, fourcc = stream.actual
            fps = int(round(fps))
            if min(width, height, fps) <= 0:
                raise ValueError('驱动未返回有效配置，请关闭预览后按已验证的参数生成')
        text = snippet(device, width, height, fps, fourcc, fields['name'].get())
        output.delete('1.0', 'end')
        output.insert('end', text)
        status.set('已生成配置片段；替换对应条目。曝光等调参值不属于这两个 YAML 的现有字段。')

    def formats():
        device = selected()
        if device['kind'] == 'usb':
            text = v4l(device['device'], '--list-formats-ext')
        else:
            import pyrealsense2 as rs
            dev = next(d for d in rs.context().query_devices()
                       if d.get_info(rs.camera_info.serial_number) == device['serial'])
            text = '\n'.join(str(p) for s in dev.query_sensors() for p in s.get_stream_profiles())
        output.delete('1.0', 'end')
        output.insert('end', text)

    def copy():
        root.clipboard_clear()
        root.clipboard_append(output.get('1.0', 'end-1c'))
        status.set('已复制下方文本')

    def tick():
        nonlocal image
        if stream:
            if stream.frame is not None:
                frame = stream.frame
                h, w = frame.shape[:2]
                scale = min(760/w, 380/h)
                frame = stream.cv2.resize(frame, (int(w*scale), int(h*scale)))
                ok, png = stream.cv2.imencode('.png', frame)
                if ok:
                    image = tk.PhotoImage(data=base64.b64encode(png).decode())
                    preview.configure(image=image, text='')
            if stream.error:
                status.set(stream.error)
        root.after(50, tick)

    for label, action in [('刷新设备', refresh), ('打开/重启预览', start), ('关闭预览', stop),
                          ('支持的格式', formats), ('生成 YAML 参数', generate), ('复制文本', copy)]:
        ttk.Button(buttons, text=label, command=guarded(action)).pack(side='left')
    ttk.Button(control_bar, text='应用参数（实时生效）', command=guarded(apply)).pack(side='left')
    choice.bind('<<ComboboxSelected>>', guarded(show_info))
    controls.bind('<<ListboxSelect>>', guarded(select_control))

    def close():
        stop()
        root.destroy()
    root.protocol('WM_DELETE_WINDOW', guarded(close))
    refresh()
    tick()
    root.mainloop()


def main():
    parser = argparse.ArgumentParser(description='相机型号/端口检查、实时预览调参与配置片段生成')
    parser.add_argument('--list', action='store_true', help='只打印设备信息，不打开界面')
    args = parser.parse_args()
    devices, warnings = discover()
    if args.list:
        print(json.dumps(dict(devices=devices, warnings=warnings), ensure_ascii=False, indent=2))
    else:
        gui(devices, warnings)


if __name__ == '__main__':
    main()
