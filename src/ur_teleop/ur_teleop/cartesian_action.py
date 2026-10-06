"""笛卡尔阻抗目标的录制格式；abs 为目标位姿，rel 为相对实测位姿增量。"""
import math

import numpy as np

from ur_teleop.xbot_core import multiply


def action_names(mode):
    if mode == 'abs':
        return ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'cmd_gripper']
    if mode == 'rel':
        return ['dx', 'dy', 'dz', 'drx', 'dry', 'drz', 'cmd_gripper']
    raise ValueError('recorder.action_mode 必须为 abs 或 rel')


def action_label(mode, reference_link='base_link', tcp_link='tool0'):
    action_names(mode)
    return f'cartesian_pose:{mode}:{reference_link}:{tcp_link}'


def encode_action(target, actual, gripper, mode):
    """使用控制周期内的目标与反馈，避免录制线程再次查询 TF 造成时间错位。"""
    action_names(mode)
    target = np.array(target, dtype=float)
    actual = np.array(actual, dtype=float)
    if target.shape != (7,) or actual.shape != (7,) or not np.isfinite(np.r_[target, actual, gripper]).all():
        raise ValueError('目标、实测位姿和夹爪必须有限且维度正确')
    for pose in (target, actual):
        norm = np.linalg.norm(pose[3:])
        if norm < 1e-8 or not np.isfinite(norm):
            raise ValueError('姿态四元数无效')
        pose[3:] /= norm
    if mode == 'abs':
        return np.r_[target, gripper]
    # 基座旋转增量满足 q_target = dq_base * q_actual；取最短旋转向量。
    dq = multiply(target[3:], np.r_[-actual[3:6], actual[6]])
    if dq[3] < 0:
        dq = -dq
    sine = np.linalg.norm(dq[:3])
    rotation = 2. * dq[:3] if sine < 1e-12 else dq[:3] * (2. * math.atan2(sine, dq[3]) / sine)
    return np.r_[target[:3] - actual[:3], rotation, gripper]
