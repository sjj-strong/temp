"""手柄参考系与恢复保护测试。"""
import numpy as np
import pytest
from ur_teleop.xbot_core import AXES, BUTTONS, ButtonEvents, JoyMapping, PoseIntegrator


CFG = dict(precision_scale=.25, max_linear_speed_m_s=.02,
           max_angular_speed_rad_s=.1, target_lead_m=.03, target_lead_rad=.15)


def inputs():
    return dict.fromkeys(AXES, 0.), dict.fromkeys(BUTTONS, False)


def test_tcp_rotation_and_translation():
    for frame, expected in [('base', [1, 0, 0]), ('tcp', [0, 1, 0])]:
        core = PoseIntegrator(CFG)
        core.frame = frame
        pose = np.array([0., 0., 0., 0., 0., np.sqrt(.5), np.sqrt(.5)])
        a, b = inputs()
        core.step(pose, a, b, .02, True)
        b['rb'] = True
        a['ly'] = a['ry'] = 1.
        action = core.step(pose, a, b, .02, True)
        np.testing.assert_allclose(action[:3], np.array(expected) * .02, atol=1e-10)
        np.testing.assert_allclose(action[3:], np.array(expected) * .1, atol=1e-10)


def test_toggle_continuity_and_fault_requires_release():
    c = PoseIntegrator(CFG)
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    a, b = inputs()
    c.step(pose, a, b, .02, True)
    b['rb'], a['ly'] = True, 1.
    c.step(pose, a, b, .02, True)
    old = c.target.copy()
    assert not c.step(pose, a, b, .02, True, toggle=True).any()
    np.testing.assert_array_equal(c.target, old)
    c.step(pose, a, b, .02, False)
    assert not c.step(pose, a, b, .02, True).any()
    b['rb'] = False
    c.step(pose, a, b, .02, True)
    b['rb'] = True
    assert c.step(pose, a, b, .02, True).any()


def test_lead_limit_and_timer_stall():
    c = PoseIntegrator(CFG)
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    a, b = inputs()
    c.step(pose, a, b, .02, True)
    b['rb'], a['ly'] = True, 1.
    for _ in range(100):
        c.step(pose, a, b, .02, True)
    assert not c.enabled
    np.testing.assert_allclose(c.target, pose)
    assert not c.step(pose, a, b, .5, True).any()


def test_fault_latches_hold_pose_instead_of_following_feedback():
    c = PoseIntegrator(CFG)
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    a, b = inputs()
    c.step(pose, a, b, .02, True)
    c.step(pose, a, b, .02, False)
    moved = pose.copy()
    moved[0] = .01
    c.step(moved, a, b, .02, False)
    np.testing.assert_array_equal(c.target, pose)


def test_precision_and_vector_speed_limit():
    c = PoseIntegrator(CFG)
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    a, b = inputs()
    c.step(pose, a, b, .02, True)
    a['ly'] = a['lx'] = a['rt'] = 1.
    b['rb'] = b['lb'] = True
    action = c.step(pose, a, b, .02, True)
    assert np.linalg.norm(action[:3]) == pytest.approx(.005)


def test_button_edges_and_view_hold():
    e = ButtonEvents()
    _, b = inputs()
    e.update(b, 0.)
    b['x'] = b['view'] = True
    assert e.update(b, .1) == {'x', 'view'}
    assert e.update(b, 1.) == set()
    assert e.update(b, 2.2) == {'finalize'}
    assert e.update(b, 3.) == set()
    e.reset()
    assert e.update(b, 10.) == set()


def test_trigger_rest_and_layout_validation():
    cfg = dict(version=1, axis_count=7, button_count=8,
               buttons=dict(zip(BUTTONS, range(8))),
               axes={n: dict(index=i, rest=1. if n in ('lt', 'rt') else 0.,
                             positive=-1.) for i, n in enumerate(AXES)})
    mapping = JoyMapping(cfg)
    axes = [0., 0., 0., 0., 1., -1., 0.]
    decoded, _ = mapping.decode(axes, [0]*8)
    assert decoded['lt'] == 0. and decoded['rt'] == 1.
    with pytest.raises(ValueError):
        mapping.decode(axes[:-1], [0]*8)
    axes[0] = float('nan')
    with pytest.raises(ValueError):
        mapping.decode(axes, [0]*8)
