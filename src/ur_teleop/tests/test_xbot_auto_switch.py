"""自动切换的离线测试，不连接控制器服务。"""
from types import SimpleNamespace

import numpy as np
import pytest

from ur_teleop.xbot_teleop_node import XbotTeleopNode
from ur_teleop.xbot_core import PoseIntegrator, AXES, BUTTONS


def fixture_node():
    node = object.__new__(XbotTeleopNode)
    node.estop = node.finished = node.controller_active = False
    node.awaiting_controller_confirmation = False
    node.switch_future = None
    node.list_future = None
    node.list_at = 10.
    node.controllers_at = 10.
    node.controllers = {'cartesian_impedance_controller': 'inactive',
                        'scaled_joint_trajectory_controller': 'active'}
    node.joints = np.zeros(6)
    node.cfg = {'home': {'slave': [0.]*6}}
    node.core = PoseIntegrator(dict(max_linear_speed_m_s=.02, max_angular_speed_rad_s=.1,
                                   precision_scale=.25))
    calls = []
    def switch(*args):
        calls.append(args)
        return object()
    node.switcher = SimpleNamespace(switch=switch, list_controllers=lambda: None,
                                    list_result=lambda future: future.result())
    node.get_logger = lambda: SimpleNamespace(info=lambda *args: None,
                                               warn=lambda *args, **kwargs: None,
                                               error=lambda *args, **kwargs: None)
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


def test_query_delay_keeps_controller_active_but_explicit_inactive_latches_fault():
    node, _ = fixture_node()
    node.controller_active = True
    node.controllers['cartesian_impedance_controller'] = 'active'
    node.controllers_at = 10.
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    node.monitor_controllers(12.1, pose)
    assert node.controller_active and not node.finished
    node.list_future = SimpleNamespace(done=lambda: True, result=lambda: None)
    node.monitor_controllers(12.15, pose)
    assert node.controller_active and not node.finished and node.controllers_at == 10.
    node.list_future = SimpleNamespace(done=lambda: True,
                                       result=lambda: {'cartesian_impedance_controller': 'inactive'})
    node.monitor_controllers(12.2, pose)
    assert node.finished and not node.controller_active


def test_controller_state_unconfirmed_for_ten_seconds_latches_fault():
    node, _ = fixture_node()
    node.controller_active = True
    node.controllers_at = 10.
    node.list_future = SimpleNamespace(done=lambda: False)
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    node.monitor_controllers(19.9, pose)
    assert node.controller_active and not node.finished
    node.monitor_controllers(20., pose)
    assert node.finished and not node.controller_active


def test_diagnostic_shows_comparable_poses_and_is_rate_limited():
    node = object.__new__(XbotTeleopNode)
    messages = []
    node.get_logger = lambda: SimpleNamespace(info=messages.append)
    node.diagnostic_hz = 5.
    node.last_diagnostic_at = -float('inf')
    node.x = {'joy_timeout_s': .25, 'tcp_timeout_s': .25}
    node.core = SimpleNamespace(target=np.array([.12, .2, .3, 0., 0., 0., 1.]),
                                frame='base', enabled=True)
    node.finished = node.estop = node.awaiting_controller_confirmation = False
    node.controller_active = True
    node.controllers = {'cartesian_impedance_controller': 'active'}
    node.axes = dict.fromkeys(AXES, 0.)
    node.axes['ly'] = 1.
    node.buttons = dict.fromkeys(BUTTONS, False)
    node.buttons['rb'] = True
    node.joy_at = node.joints_at = node.controllers_at = 10.
    node.tcp_age_s = .01
    actual = np.array([.1, .2, .3, 0., 0., 0., 1.])
    identity = np.array([0., 0., 0., 0., 0., 0., 1.])
    node.log_diagnostic(10., actual, True, identity)
    assert len(messages) == 1
    assert '当前=xyz=(0.1000,0.2000,0.3000)' in messages[0]
    assert '目标=xyz=(0.1200,0.2000,0.3000)' in messages[0]
    assert '超前=20.0mm/0.000rad' in messages[0]
    assert '目标已发布=1' in messages[0]
    node.log_diagnostic(10.1, actual, True, identity)
    assert len(messages) == 1
    node.log_diagnostic(10.21, actual, True, identity)
    assert len(messages) == 2
