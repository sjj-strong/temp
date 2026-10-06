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
from ur_teleop.xbot_core import JoyMapping


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
    """记录全部原始按键边沿，即使日志定时器尚未触发也不会漏掉短按。"""

    def __init__(self, mapping=None, timeout=0.25):
        self.mapping = mapping
        self.timeout = timeout
        self.latest = None
        self.received_at = None
        self.count = 0
        self.previous = None

    def button_label(self, index):
        if self.mapping is not None:
            for name, number in self.mapping.cfg['buttons'].items():
                if number == index:
                    return f'{name.upper()}（{FUNCTIONS.get(name, "自定义功能")}）'
        return '未映射按键'

    def receive(self, msg, now):
        # 断连或布局变化后重新建立基线，不把旧状态当作当前状态。
        previous = self.previous
        if self.received_at is not None and now - self.received_at > self.timeout:
            previous = None
        events = []
        if previous is None or len(previous) != len(msg.buttons):
            previous = [0] * len(msg.buttons)
            events.append(f'收到输入：轴数={len(msg.axes)}，按键数={len(msg.buttons)}')
        for index, (before, after) in enumerate(zip(previous, msg.buttons)):
            if bool(before) != bool(after):
                events.append(f'按键[{index}] {self.button_label(index)} '
                              f'{"按下" if after else "松开"}')
        self.previous = list(msg.buttons)
        self.latest = msg
        self.received_at = now
        self.count += 1
        return events

    def snapshot(self, now, timeout):
        if self.received_at is None:
            return '等待手柄输入：检查连接、设备编号和手柄驱动日志'
        age = now - self.received_at
        if age > timeout:
            self.previous = None
            return f'输入超时：{age:.2f} 秒未收到 Joy，等待重新连接'
        msg = self.latest
        raw_axes = ','.join(f'{i}:{value:+.3f}' for i, value in enumerate(msg.axes))
        raw_buttons = ','.join(f'{i}:{value}' for i, value in enumerate(msg.buttons))
        mapped = '未加载标定，仅显示原始输入'
        if self.mapping is not None:
            try:
                axes, buttons = self.mapping.decode(msg.axes, msg.buttons)
                mapped = '归一化轴[' + ','.join(
                    f'{name.upper()}:{value:+.3f}' for name, value in axes.items()) + '] '
                mapped += '功能键[' + ','.join(
                    f'{name.upper()}:{int(value)}' for name, value in buttons.items()) + ']'
            except ValueError as exc:
                mapped = f'标定不匹配：{exc}；原始输入仍可测试'
        return (f'输入正常 消息数={self.count} 数据龄={age:.3f}s '
                f'原始轴[{raw_axes}] 原始按键[{raw_buttons}] {mapped}')


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
        cfg = yaml.safe_load(stream)['xbot']
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
        print(f'标定加载失败：{exc}；继续测试全部原始输入。', flush=True)

    driver = None
    node = None
    rclpy.init(args=[])
    try:
        node = JoyTestNode(JoyDiagnostics(mapping, timeout), options.rate, timeout)
        node.get_logger().info(
            f'独立手柄测试：设备={device_id}，话题={TOPIC}，日志={options.rate:g}Hz；'
            '功能名仅作提示，按键不会执行机器人或录制动作。Ctrl+C 退出。')
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
