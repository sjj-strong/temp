"""独立手柄测试：自动启动输入驱动，只打印日志，不发送机器人指令。"""

import argparse
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy

# 支持直接用源码路径运行，无需先构建工作区。
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ur_teleop.xbot_core import JoyMapping, dominant_axis


TOPIC = '/xbot_joy_test/joy'
FUNCTIONS = {
    'rb': '运动使能', 'lb': '精细模式', 'a': '切换夹爪',
    'x': '切换参考系', 'menu': '开始录制', 'y': '保存片段',
    'b': '丢弃片段', 'view': '长按结束录制',
}


def parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml',
                        help='读取其中的手柄设备、标定、死区和超时配置')
    parser.add_argument('--device-id', type=int, help='覆盖配置中的手柄编号')
    parser.add_argument('--rate', type=float, default=5.0, help='状态日志频率，默认 5 Hz')
    parser.add_argument('--no-start-joy', action='store_true',
                        help='只监听专用测试话题，由外部进程提供 Joy 输入')
    options = parser.parse_args(args)
    if not math.isfinite(options.rate) or options.rate <= 0:
        parser.error('--rate 必须是有限正数')
    if options.device_id is not None and options.device_id < 0:
        parser.error('--device-id 必须非负')
    return options


class JoyDiagnostics:
    """按有效标定项独立解析输入，只预览动作，不依赖机器人反馈。"""

    def __init__(self, mapping=None, timeout=0.25, config=None, gripper_enabled=True):
        self.mapping = mapping
        self.timeout = timeout
        self.config = config or {}
        self.gripper_enabled = gripper_enabled
        self.received_at = None
        self.previous = {}
        self.layout = None
        self.unavailable = None
        self.frame = 'base'
        self.view_since = None
        self.view_fired = False
        self.moving = False
        self.axes = {}
        self.buttons = {}

    def decode(self, msg):
        """总数量不同不影响有效项；越界或非法输入只使对应动作不可用。"""
        axes, buttons, unavailable = {}, {}, []
        if self.mapping is None:
            return axes, buttons, ['未加载有效标定，无法识别动作']
        cfg = self.mapping.cfg
        for name, index in cfg['buttons'].items():
            if 0 <= index < len(msg.buttons):
                buttons[name] = bool(msg.buttons[index])
            else:
                action = FUNCTIONS.get(name, name.upper())
                unavailable.append(f'{action}（{name.upper()}）不可用：映射超出当前设备范围')
        for name, spec in cfg['axes'].items():
            if spec.get('kind') == 'buttons':
                if not all(0 <= spec[key] < len(msg.buttons) for key in ('positive', 'negative')):
                    unavailable.append(f'{name.upper()} 运动输入不可用：映射超出当前设备范围')
                    continue
                value = (float(bool(msg.buttons[spec['positive']]))
                         - float(bool(msg.buttons[spec['negative']])))
            else:
                index = spec['index']
                if not 0 <= index < len(msg.axes):
                    unavailable.append(f'{name.upper()} 运动输入不可用：映射超出当前设备范围')
                    continue
                raw = msg.axes[index]
                if not math.isfinite(raw) or abs(raw) > 1.01:
                    unavailable.append(f'{name.upper()} 运动输入不可用：输入值非法')
                    continue
                value = (raw - spec['rest']) / (spec['positive'] - spec['rest'])
            value = min(1.0, max(0.0 if name in ('lt', 'rt') else -1.0, value))
            deadzone = self.mapping.deadzone
            axes[name] = math.copysign(max(0.0, abs(value) - deadzone) / (1.0 - deadzone), value)
        return axes, buttons, unavailable

    def reset_input(self):
        """断连后清除按键和长按计时，保留测试参考系选择。"""
        self.previous = {}
        self.view_since = None
        self.view_fired = False
        self.moving = False
        self.axes = {}
        self.buttons = {}

    def movement(self):
        lx, ly = self.axes.get('lx', 0.0), self.axes.get('ly', 0.0)
        if not self.config.get('left_stick_xy_free', False):
            lx, ly = dominant_axis(lx, ly)
        rx, ry = dominant_axis(self.axes.get('rx', 0.0), self.axes.get('ry', 0.0))
        translation = (-ly, -lx, self.axes.get('lt', 0.0) - self.axes.get('rt', 0.0))
        rotation = (ry, rx, self.axes.get('yaw', 0.0))
        scale = self.config.get('precision_scale', 0.25) if self.buttons.get('lb') else 1.0
        # 与遥操作一致：先限制向量模长，再应用精细模式比例。
        vectors = []
        for vector in (translation, rotation):
            norm = max(1.0, math.sqrt(sum(value * value for value in vector)))
            vectors.append(tuple(value / norm * scale for value in vector))
        return tuple(vectors)

    def hold_event(self, now):
        if (self.view_since is not None and not self.view_fired
                and now - self.view_since >= self.config.get('view_hold_s', 0.5)):
            self.view_fired = True
            return '动作预览：结束录制（View 长按达到阈值，action=finalize）'
        return None

    def receive(self, msg, now):
        events = []
        layout = (len(msg.axes), len(msg.buttons))
        reconnect = self.received_at is not None and now - self.received_at > self.timeout
        if self.received_at is None or reconnect or layout != self.layout:
            self.reset_input()
            events.append('手柄输入已恢复' if reconnect else '手柄输入已连接')
        self.axes, self.buttons, unavailable = self.decode(msg)
        if tuple(unavailable) != self.unavailable:
            events.extend(unavailable)
            self.unavailable = tuple(unavailable)
        for name, pressed in self.buttons.items():
            if pressed == self.previous.get(name, False):
                continue
            action = FUNCTIONS.get(name, '自定义功能')
            detail = ''
            if name == 'x' and pressed:
                self.frame = 'tcp' if self.frame == 'base' else 'base'
                detail = f'，当前参考系={self.frame.upper()}'
            elif name == 'a' and pressed:
                reasons = []
                if not self.gripper_enabled:
                    reasons.append('配置已关闭夹爪')
                if not self.buttons.get('rb'):
                    reasons.append('RB 未使能')
                detail = '，不可执行：' + '、'.join(reasons) if reasons else '，请求切换夹爪'
            elif name == 'view':
                self.view_since = now if pressed else None
                self.view_fired = False
                if pressed:
                    detail = f'，等待持续按住 {self.config.get("view_hold_s", 0.5):g} 秒'
            event_id = {'menu': 'start', 'y': 'save', 'b': 'discard'}.get(name)
            if pressed and event_id:
                detail += f'，action={event_id}'
            events.append(f'动作预览：{action}（{name.upper()}）{"按下" if pressed else "松开"}{detail}')
        moving = any(value != 0.0 for vector in self.movement() for value in vector)
        if self.moving and not moving:
            events.append('动作预览：运动输入归零')
        self.moving = moving
        self.previous = dict(self.buttons)
        self.layout = layout
        self.received_at = now
        hold = self.hold_event(now)
        if hold:
            events.append(hold)
        return events

    def snapshot(self, now, timeout):
        if self.received_at is None:
            return '等待手柄输入：检查连接、设备编号和手柄驱动日志'
        if now - self.received_at > timeout:
            self.reset_input()
            return '输入超时：运动预览已停止，等待重新连接'
        if self.mapping is None:
            return '无法预览动作：请检查指定标定文件'
        status = ('运动已使能' if self.buttons.get('rb') else '运动未使能（RB 未按住）')
        mode = '精细模式' if self.buttons.get('lb') else '普通模式'
        movements = []
        translation, rotation = self.movement()
        for kind, vector in (('平移', translation), ('旋转', rotation)):
            for axis, value in zip('XYZ', vector):
                if value != 0.0:
                    direction = f'{"+" if value > 0 else "-"}{axis}'
                    prefix = '绕 ' if kind == '旋转' else ''
                    movements.append(f'{prefix}{direction} {kind} 强度={abs(value):.3f}')
        pressed = '、'.join(FUNCTIONS.get(name, name.upper())
                           for name, value in self.buttons.items() if value)
        result = (f'动作预览：{status}；{mode}；参考系={self.frame.upper()}；'
                  + ('，'.join(movements) if movements else '运动输入归零')
                  + f'；当前按住={pressed or "无"}')
        hold = self.hold_event(now)
        return result + ('；' + hold if hold else '')


class JoyTestNode(Node):
    def __init__(self, diagnostics, rate, timeout):
        super().__init__('xbot_joy_test_monitor')
        self.diagnostics = diagnostics
        self.timeout = timeout
        self.create_subscription(Joy, TOPIC, self.receive, qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self.report)

    def receive(self, msg):
        for event in self.diagnostics.receive(msg, time.monotonic()):
            self.get_logger().info(event)

    def report(self):
        self.get_logger().info(self.diagnostics.snapshot(time.monotonic(), self.timeout))


def main(args=None):
    options = parse_args(args)
    config_path = Path(options.config).expanduser().resolve()
    with config_path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
        cfg = config['xbot']
    device_id = options.device_id if options.device_id is not None else int(cfg.get('device_id', 0))
    timeout = float(cfg.get('joy_timeout_s', 0.25))
    if device_id < 0 or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('设备编号必须非负，joy_timeout_s 必须为有限正数')
    mapping = None
    try:
        calibration = Path(cfg['calibration_file']).expanduser()
        if not calibration.is_absolute():
            calibration = config_path.parent / calibration
        with calibration.open(encoding='utf-8') as stream:
            mapping = JoyMapping(yaml.safe_load(stream), float(cfg.get('deadzone', 0.08)))
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        print(f'标定加载失败：{exc}；无法识别对应动作，请检查指定标定文件。', flush=True)

    driver = None
    node = None
    rclpy.init(args=[])
    try:
        diagnostics = JoyDiagnostics(
            mapping, timeout, cfg, config.get('gripper', {}).get('enabled', True))
        node = JoyTestNode(diagnostics, options.rate, timeout)
        node.get_logger().info(
            f'独立手柄测试：设备={device_id}，话题={TOPIC}，日志={options.rate:g}Hz；'
            '只显示动作预览，不执行机器人或录制动作。Ctrl+C 退出。')
        if not options.no_start_joy:
            executable = Path(get_package_prefix('joy')) / 'lib/joy/joy_node'
            driver = subprocess.Popen([
                str(executable), '--ros-args', '-r', '__node:=xbot_joy_test_driver',
                '-r', f'joy:={TOPIC}', '-p', f'device_id:={device_id}',
                '-p', 'deadzone:=0.0', '-p', 'autorepeat_rate:=50.0',
            ], start_new_session=True)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if driver is not None and driver.poll() is not None:
                raise RuntimeError(f'手柄驱动已退出，返回码={driver.returncode}，请检查驱动日志')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if driver is not None and driver.poll() is None:
            driver.send_signal(signal.SIGINT)
            try:
                driver.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                driver.terminate()
                try:
                    driver.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    driver.kill()
                    driver.wait()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        print('手柄测试已停止。', flush=True)


if __name__ == '__main__':
    main()
