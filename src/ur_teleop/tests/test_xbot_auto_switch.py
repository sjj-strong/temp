"""自动切换的离线测试，不连接控制器服务。"""
from types import SimpleNamespace

import numpy as np
import pytest

from ur_teleop.xbot_teleop_node import XbotTeleopNode
from ur_teleop.xbot_core import PoseIntegrator, AXES, BUTTONS


def fixture_node():
    node = object.__new__(XbotTeleopNode)
    node.estop = node.finished = node.controller_active = False
    node.switch_future = None
    node.controllers_at = 10.
    node.controllers = {'cartesian_impedance_controller': 'inactive',
                        'scaled_joint_trajectory_controller': 'active'}
    node.joints = np.zeros(6)
    node.cfg = {'home': {'slave': [0.]*6}}
    node.core = PoseIntegrator(dict(max_translation_delta_m=.0004, max_rotation_delta_rad=.002,
                                   precision_scale=.25, target_lead_m=.03, target_lead_rad=.15))
    calls = []
    def switch(*args):
        calls.append(args)
        return object()
    node.switcher = SimpleNamespace(switch=switch)
    node.get_logger = lambda: SimpleNamespace(info=lambda *args: None)
    return node, calls


def test_startup_switch_without_joy_or_rb():
    node, calls = fixture_node()
    node.ensure_impedance(np.array([0, 0, 0, 0, 0, 0, 1.]), True, 10.)
    assert calls == [(['cartesian_impedance_controller'], ['scaled_joint_trajectory_controller'])]
    node.ensure_impedance(None, True, 10.1)
    assert len(calls) == 1


@pytest.mark.parametrize('block', ['feedback', 'home', 'estop', 'finished', 'stale'])
def test_unsafe_startup_does_not_switch(block):
    node, calls = fixture_node()
    if block == 'home': node.joints[:] = 1.
    if block in ('estop', 'finished'): setattr(node, block, True)
    node.ensure_impedance(None, block != 'feedback', 13. if block == 'stale' else 10.)
    assert not calls


def test_active_controller_adoption_still_requires_rb_release():
    node, calls = fixture_node()
    node.controllers['cartesian_impedance_controller'] = 'active'
    node.joints[:] = 1.  # 已激活接管不需要再次回 Home。
    pose = np.array([.1, .2, .3, 0, 0, 0, 1.])
    node.ensure_impedance(pose, True, 10.)
    assert node.controller_active and not calls
    np.testing.assert_allclose(node.core.target, pose)
    axes, buttons = dict.fromkeys(AXES, 0.), dict.fromkeys(BUTTONS, False)
    axes['ly'], buttons['rb'] = 1., True
    node.core.step(pose, axes, buttons, .02, True)
    assert not node.core.enabled
    buttons['rb'] = False
    node.core.step(pose, axes, buttons, .02, True)
    np.testing.assert_allclose(node.core.target, pose)
    buttons['rb'] = True
    node.core.step(pose, axes, buttons, .02, True)
    assert node.core.enabled
