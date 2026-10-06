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


def parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/ros2_ws/src/ur_teleop/config/xbot_joy.yaml')
    return parser.parse_args(args)


def wait_neutral(sample, reference=None, clock=time.monotonic, timeout=30.):
    """自动等待 Joy 回中并连续稳定一秒。"""
    print('松开所有按键/扳机、摇杆回中；稳定一秒后自动继续。', flush=True)
    deadline = clock() + timeout
    stable_since = None
    candidate = None
    while clock() < deadline:
        msg = sample()
        if msg is None:
            stable_since = None
            continue
        if reference is not None and (len(msg.axes) != len(reference.axes) or
                                      len(msg.buttons) != len(reference.buttons)):
            raise ValueError('校准期间 Joy 布局发生变化')
        rest = reference if reference is not None else candidate
        if rest is not None and (len(msg.axes) != len(rest.axes) or
                                 len(msg.buttons) != len(rest.buttons)):
            raise ValueError('校准期间 Joy 布局发生变化')
        if any(msg.buttons) or (rest is not None and
                                any(abs(a - b) > .1 for a, b in zip(msg.axes, rest.axes))):
            stable_since = None
            candidate = None if reference is None else reference
            continue
        if stable_since is None:
            stable_since = clock()
            candidate = msg
        if clock() - stable_since >= 1.:
            return msg
    raise TimeoutError('Joy 未连续回中一秒；请松开输入并检查 joy_node')


def wait_capture(sample, prompt, baseline, kind, clock=time.monotonic, timeout=30.):
    """按键和十字键立即记录；模拟轴稳定保持一秒后记录端点。"""
    print(prompt + ('，保持一秒' if kind == 'axis' else '，按下即可'), flush=True)
    deadline = clock() + timeout
    active = None
    active_since = None
    endpoint = None
    while clock() < deadline:
        msg = sample()
        if msg is None:
            active = active_since = endpoint = None
            continue
        if len(msg.axes) != len(baseline.axes) or len(msg.buttons) != len(baseline.buttons):
            raise ValueError('校准期间 Joy 布局发生变化')
        detected = None
        value = None
        if kind in ('button', 'either'):
            pressed = [i for i, (a, b) in enumerate(zip(msg.buttons, baseline.buttons)) if a and not b]
            if len(pressed) == 1:
                print(f'已记录 {prompt}：按键 {pressed[0]}', flush=True)
                return {'kind': 'button', 'index': pressed[0]}
        if detected is None and kind in ('axis', 'either'):
            changes = [abs(a - b) for a, b in zip(msg.axes, baseline.axes)]
            if changes and max(changes) > .75:
                index = changes.index(max(changes))
                detected = ('axis', index)
                value = float(msg.axes[index])
                if kind == 'either':
                    print(f'已记录 {prompt}：轴 {index}，端点 {value:.3f}', flush=True)
                    return {'kind': 'axis', 'index': index,
                            'rest': float(baseline.axes[index]), 'positive': value}
        if detected != active:
            active = detected
            active_since = clock() if detected is not None else None
            endpoint = value
        elif detected is not None:
            if value is not None and abs(value - baseline.axes[detected[1]]) > abs(endpoint - baseline.axes[detected[1]]):
                endpoint = value
            if clock() - active_since >= 1.:
                print(f'已记录 {prompt}：轴 {detected[1]}，端点 {endpoint:.3f}', flush=True)
                return {'kind': 'axis', 'index': detected[1],
                        'rest': float(baseline.axes[detected[1]]), 'positive': endpoint}
    raise TimeoutError('未收到清晰输入；请检查 joy_node 后重试')


def main():
    args = parse_args()
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

    try:
        print('请先单独运行 ros2 run joy joy_node --ros-args -p autorepeat_rate:=50.0')
        base = wait_neutral(sample)
        cfg = dict(version=1, axis_count=len(base.axes), button_count=len(base.buttons), buttons={}, axes={})
        for name in BUTTONS:
            cfg['buttons'][name] = wait_capture(sample, f'按 {name.upper()}',
                                                wait_neutral(sample, base), 'button')['index']
        for name, hint in [('ly', '左摇杆向上（+X）'), ('lx', '左摇杆向左（+Y）'),
                           ('ry', '右摇杆向上（绕 +X）'), ('rx', '右摇杆向左（绕 +Y）'),
                           ('lt', 'LT 扳机压到底（-Z）'), ('rt', 'RT 扳机压到底（+Z）')]:
            cfg['axes'][name] = wait_capture(sample, hint, wait_neutral(sample, base), 'axis')
        positive = wait_capture(sample, '十字键向左（绕 +Z）', wait_neutral(sample, base), 'either')
        if positive['kind'] == 'button':
            negative = wait_capture(sample, '十字键向右（绕 -Z）', wait_neutral(sample, base), 'button')
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
