"""读取 Xbot 控制器坐标系，并转换完整目标位姿。"""
from pathlib import Path

import numpy as np
import yaml

from ur_teleop.xbot_core import multiply, rotate


def controller_base_frame(config_file):
    """从实际传给控制器的 YAML 读取基座 link，避免另选一份参数文件。"""
    path = Path(config_file)
    with path.open() as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f'笛卡尔阻抗控制器参数文件格式无效: {path}')
    matches = [value.get('ros__parameters') for key, value in data.items()
               if key.endswith('cartesian_impedance_controller') and isinstance(value, dict)]
    if len(matches) != 1 or not isinstance(matches[0], dict):
        raise ValueError(f'参数文件缺少唯一的 cartesian_impedance_controller 配置: {path}')
    params = matches[0]
    prefix = params.get('tf_prefix', '')
    if (not isinstance(prefix, str) or not isinstance(params.get('base_frame'), str) or
            not params['base_frame'] or not isinstance(params.get('tip_frame'), str)):
        raise ValueError(f'控制器基座或末端 link 配置无效: {path}')
    if prefix + params['tip_frame'] != 'tool0':
        raise ValueError('Xbot 控制器末端必须为 tool0')
    return prefix + params['base_frame']


def transform_pose(pose, transform):
    """transform 表示目标坐标系到原坐标系的 TF，数组均为 xyz+xyzw。"""
    pose, transform = np.asarray(pose, dtype=float), np.asarray(transform, dtype=float)
    if pose.shape != (7,) or transform.shape != (7,) or not np.isfinite(np.r_[pose, transform]).all():
        raise ValueError('位姿与坐标变换必须为有限的七维数组')
    p_norm, t_norm = np.linalg.norm(pose[3:]), np.linalg.norm(transform[3:])
    if min(p_norm, t_norm) < 1e-8:
        raise ValueError('坐标变换四元数无效')
    rotation = transform[3:] / t_norm
    return np.r_[rotate(rotation, pose[:3]) + transform[:3],
                 multiply(rotation, pose[3:] / p_norm)]


def inverse_transform_pose(pose, transform):
    """将目标坐标系中的位姿逆变换回原坐标系。"""
    transform = np.asarray(transform, dtype=float)
    q = transform[3:] / np.linalg.norm(transform[3:])
    inverse_q = np.r_[-q[:3], q[3]]
    inverse = np.r_[-rotate(inverse_q, transform[:3]), inverse_q]
    return transform_pose(pose, inverse)


def clip_workspace_target(pose, origin, half_extent):
    """在参考坐标系中按初始位姿的 XYZ 矩形范围裁剪目标。"""
    result = np.asarray(pose, dtype=float).copy()
    result[:3] = np.clip(result[:3], np.asarray(origin) - np.asarray(half_extent),
                         np.asarray(origin) + np.asarray(half_extent))
    return result
