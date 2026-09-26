"""手柄映射、按键边沿与笛卡尔积分；不依赖 ROS。"""

import math
import numpy as np


BUTTONS = ('rb', 'lb', 'a', 'x', 'menu', 'y', 'b', 'view')
AXES = ('lx', 'ly', 'rx', 'ry', 'lt', 'rt', 'yaw')


def multiply(a, b):
    """xyzw 四元数乘法。"""
    av, bv = np.asarray(a[:3]), np.asarray(b[:3])
    return np.r_[a[3] * bv + b[3] * av + np.cross(av, bv),
                 a[3] * b[3] - np.dot(av, bv)]


def rotate(q, v):
    return multiply(multiply(q, np.r_[v, 0.]), np.r_[-q[:3], q[3]])[:3]


def delta_quaternion(w):
    angle = np.linalg.norm(w)
    if angle < 1e-12:
        return np.array([0., 0., 0., 1.])
    return np.r_[w / angle * math.sin(angle / 2), math.cos(angle / 2)]


def orientation_distance(a, b):
    return 2 * math.acos(float(np.clip(abs(np.dot(a, b)), 0., 1.)))


class JoyMapping:
    def __init__(self, cfg, deadzone=0.08):
        self.cfg = cfg
        self.deadzone = deadzone
        if cfg.get('version') != 1 or not 0 <= deadzone < 1:
            raise ValueError('校准版本或死区无效')
        for name in BUTTONS:
            index = cfg['buttons'][name]
            if not isinstance(index, int) or not 0 <= index < cfg['button_count']:
                raise ValueError(f'按键映射无效: {name}')
        if len(set(cfg['buttons'][n] for n in BUTTONS)) != len(BUTTONS):
            raise ValueError('功能按键不允许复用')
        for name in AXES:
            axis = cfg['axes'][name]
            if axis.get('kind', 'axis') == 'buttons' and name == 'yaw':
                if any(not 0 <= axis[n] < cfg['button_count'] for n in ('positive', 'negative')):
                    raise ValueError('十字键映射越界')
            elif (not 0 <= axis['index'] < cfg['axis_count'] or
                  not math.isfinite(axis['rest']) or not math.isfinite(axis['positive']) or
                  abs(axis['positive'] - axis['rest']) < 0.3):
                raise ValueError(f'轴映射无效: {name}')

    def decode(self, axes, buttons):
        if len(axes) != self.cfg['axis_count'] or len(buttons) != self.cfg['button_count']:
            raise ValueError('Joy 布局与校准不一致，请重新校准')
        if not all(math.isfinite(x) and abs(x) <= 1.01 for x in axes):
            raise ValueError('Joy 含非法轴值')
        values = {}
        for name in AXES:
            spec = self.cfg['axes'][name]
            if spec.get('kind') == 'buttons':
                v = float(bool(buttons[spec['positive']])) - float(bool(buttons[spec['negative']]))
            else:
                v = (axes[spec['index']] - spec['rest']) / (spec['positive'] - spec['rest'])
            v = float(np.clip(v, 0 if name in ('lt', 'rt') else -1, 1))
            values[name] = math.copysign(max(0., abs(v) - self.deadzone) / (1 - self.deadzone), v)
        return values, {n: bool(buttons[self.cfg['buttons'][n]]) for n in BUTTONS}


class ButtonEvents:
    def __init__(self, hold_s=2.):
        self.previous = {n: True for n in BUTTONS}
        self.view_since = None
        self.view_fired = False
        self.hold_s = hold_s

    def reset(self):
        self.previous = {n: True for n in BUTTONS}
        self.view_since = None
        self.view_fired = False

    def update(self, buttons, now):
        edges = {n for n in BUTTONS if buttons[n] and not self.previous[n]}
        if not buttons['view']:
            self.view_since = None
            self.view_fired = False
        elif 'view' in edges:
            self.view_since = now
        if self.view_since is not None and not self.view_fired and now - self.view_since >= self.hold_s:
            edges.add('finalize')
            self.view_fired = True
        self.previous = dict(buttons)
        return edges


class PoseIntegrator:
    def __init__(self, cfg):
        self.cfg = cfg
        self.frame = 'base'
        self.target = None
        self.enabled = False
        self.released = False

    def stop(self, actual=None):
        if actual is not None:
            self.target = np.array(actual, dtype=float)
        self.enabled = False
        self.released = False

    def step(self, actual, axes, buttons, dt, safe, toggle=False):
        action = np.zeros(6)
        if not safe or actual is None or not 0 < dt <= 0.1:
            self.stop(actual)
            return action
        if toggle:
            self.frame = 'tcp' if self.frame == 'base' else 'base'
            return action  # 切换当帧目标严格不变。
        if not buttons['rb']:
            if self.enabled or self.target is None:
                self.target = np.array(actual, dtype=float)
            self.enabled = False
            self.released = True
            return action
        if not self.enabled:
            if not self.released:
                return action
            self.target = np.array(actual, dtype=float)
            self.enabled = True
            self.released = False
        v = np.array([axes['ly'], axes['lx'], axes['rt'] - axes['lt']])
        w = np.array([axes['ry'], axes['rx'], axes['yaw']])
        scale = self.cfg['precision_scale'] if buttons['lb'] else 1.
        v *= self.cfg['max_linear_speed_m_s'] * scale / max(1., np.linalg.norm(v))
        w *= self.cfg['max_angular_speed_rad_s'] * scale / max(1., np.linalg.norm(w))
        if self.frame == 'tcp':
            v, w = rotate(np.asarray(actual[3:]), v), rotate(np.asarray(actual[3:]), w)
        candidate = self.target.copy()
        candidate[:3] += v * dt
        candidate[3:] = multiply(delta_quaternion(w * dt), candidate[3:])
        candidate[3:] /= np.linalg.norm(candidate[3:])
        if (np.linalg.norm(candidate[:3] - actual[:3]) > self.cfg['target_lead_m'] or
                orientation_distance(candidate[3:], actual[3:]) > self.cfg['target_lead_rad']):
            self.stop(actual)
            return action
        self.target = candidate
        return np.r_[v, w]
