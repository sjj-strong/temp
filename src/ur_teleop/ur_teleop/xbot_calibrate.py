"""只读 /joy 的交互校准，不创建任何机器人命令发布器。"""

import argparse
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from ur_teleop.xbot_core import BUTTONS, JoyMapping


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='~/.config/ur_teleop/xbot_joy.yaml')
    args = parser.parse_args()
    output = Path(args.output).expanduser()
    if output.exists():
        raise FileExistsError(f'不覆盖已有校准，请指定新的 --output: {output}')
    rclpy.init()
    node = Node('xbot_calibrate')
    latest = [None, 0.]

    def receive(msg):
        latest[:] = [msg, time.monotonic()]

    node.create_subscription(Joy, '/joy', receive, qos_profile_sensor_data)

    def sample():
        rclpy.spin_once(node, timeout_sec=.1)
        msg, stamp = latest
        if msg is None or time.monotonic() - stamp > .3:
            return None
        return msg

    def wait_capture(prompt, baseline, kind):
        print(prompt, flush=True)
        deadline = time.monotonic() + 30.
        while time.monotonic() < deadline:
            msg = sample()
            if msg is None:
                continue
            if len(msg.axes) != len(baseline.axes) or len(msg.buttons) != len(baseline.buttons):
                raise ValueError('校准期间 Joy 布局发生变化')
            if kind in ('button', 'either'):
                pressed = [i for i, (a, b) in enumerate(zip(msg.buttons, baseline.buttons)) if a and not b]
                if len(pressed) == 1:
                    return {'kind': 'button', 'index': pressed[0]}
            if kind in ('axis', 'either'):
                changes = [abs(a-b) for a, b in zip(msg.axes, baseline.axes)]
                if changes and max(changes) > .75:
                    index = changes.index(max(changes))
                    # 首次跨阈值不是端点，继续采样以记录推到极限后的最大行程。
                    endpoint = float(msg.axes[index])
                    end = time.monotonic() + 1.
                    while time.monotonic() < end:
                        peak = sample()
                        if peak is not None and len(peak.axes) == len(baseline.axes):
                            value = float(peak.axes[index])
                            if abs(value - baseline.axes[index]) > abs(endpoint - baseline.axes[index]):
                                endpoint = value
                    return {'kind': 'axis', 'index': index,
                            'rest': float(baseline.axes[index]), 'positive': endpoint}
        raise TimeoutError('未收到清晰输入；请检查 joy_node 后重试')

    def neutral():
        input('松开所有按键/扳机、摇杆回中，按 Enter 后保持一秒：')
        deadline = time.monotonic() + 1.
        msg = None
        while time.monotonic() < deadline:
            msg = sample()
        if msg is None or any(msg.buttons):
            raise ValueError('无新鲜 Joy 或仍有按键按下')
        return msg

    try:
        print('请先单独运行 ros2 run joy joy_node --ros-args -p autorepeat_rate:=50.0')
        base = neutral()
        cfg = dict(version=1, axis_count=len(base.axes), button_count=len(base.buttons), buttons={}, axes={})
        for name in BUTTONS:
            cfg['buttons'][name] = wait_capture(f'按 {name.upper()}', neutral(), 'button')['index']
        for name, hint in [('ly', '左摇杆向上（+X）'), ('lx', '左摇杆向左（+Y）'),
                           ('ry', '右摇杆向上（绕 +X）'), ('rx', '右摇杆向左（绕 +Y）'),
                           ('lt', 'LT 扳机压到底（-Z）'), ('rt', 'RT 扳机压到底（+Z）')]:
            cfg['axes'][name] = wait_capture(hint + '并保持到捕获', neutral(), 'axis')
        positive = wait_capture('十字键向左（绕 +Z）并保持', neutral(), 'either')
        if positive['kind'] == 'button':
            negative = wait_capture('十字键向右（绕 -Z）并保持', neutral(), 'button')
            cfg['axes']['yaw'] = dict(kind='buttons', positive=positive['index'], negative=negative['index'])
        else:
            cfg['axes']['yaw'] = positive
        JoyMapping(cfg)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x', encoding='utf-8') as stream:
            yaml.safe_dump(cfg, stream, allow_unicode=True, sort_keys=False)
        print(f'校准已写入 {output}；真机运行前仍需低速逐轴验证方向。')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
