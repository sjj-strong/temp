"""阻尼解析的离线测试。"""
from pathlib import Path

import pytest
import yaml

from ur_teleop.impedance_config import resolve_damping


@pytest.mark.parametrize('controller', ['joint_impedance_controller', 'cartesian_impedance_controller'])
@pytest.mark.parametrize('damping', [None, [3., 4., 5., 6., 7., 8.]])
def test_null_computes_damping_and_explicit_values_take_priority(tmp_path, controller, damping):
    config = tmp_path / 'controller.yaml'
    data = {f'/**/{controller}': {'ros__parameters': {
        'stiffness': [0., 1., 4., 9., 16., 25.], 'damping': damping}}}
    config.write_text(yaml.safe_dump(data))
    resolved = Path(resolve_damping(config))
    try:
        params = yaml.safe_load(resolved.read_text())[f'/**/{controller}']['ros__parameters']
        assert params['damping'] == (damping if damping is not None else [0., 2., 4., 6., 8., 10.])
        assert yaml.safe_load(config.read_text()) == data
        assert (resolved == config) == (damping is not None)
    finally:
        if resolved != config:
            resolved.unlink()
