"""读取 Xbot 控制器坐标系，并转换完整目标位姿。"""
from pathlib import Path

import numpy as np
import yaml

from ur_teleop.xbot_core import multiply, rotate


def controller_base_frame(sim):
    from ament_index_python.packages import get_package_share_directory
    name = ('ur10e_xbot_sim_cartesian_impedance.yaml' if sim
            else 'ur10e_ft300_cartesian_impedance.yaml')
    path = Path(get_package_share_directory('cartesian_impedance_controller')) / 'config' / name
    with path.open() as stream:
        data = yaml.safe_load(stream)
    params = next(value['ros__parameters'] for key, value in data.items()
                  if key.endswith('cartesian_impedance_controller'))
    prefix = params.get('tf_prefix', '')
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
