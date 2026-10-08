"""启动前将空阻尼配置换算为归一化临界阻尼。"""
import math
import tempfile
from pathlib import Path

import yaml


def resolve_damping(config_file):
    """显式阻尼优先；null 按 D_i = 2√K_i 写入临时 ROS 参数文件。"""
    data = yaml.safe_load(Path(config_file).read_text())
    changed = False
    for block in data.values():
        params = block.get('ros__parameters', {})
        if 'stiffness' in params and params.get('damping', []) is None:
            params['damping'] = [2. * math.sqrt(k) for k in params['stiffness']]
            changed = True
    if not changed:
        return str(config_file)
    with tempfile.NamedTemporaryFile(mode='w', prefix='ur_teleop_impedance_',
                                     suffix='.yaml', delete=False) as output:
        yaml.safe_dump(data, output)
        return output.name
