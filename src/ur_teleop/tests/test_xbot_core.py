"""手柄参考系与恢复保护测试。"""
import numpy as np
import pytest
from ur_teleop.xbot_core import (
    AXES, BUTTONS, ButtonEvents, JoyMapping, PoseIntegrator, delta_quaternion, multiply,
)


CFG = dict(precision_scale=.25, max_translation_delta_m=.0004,
           max_rotation_delta_rad=.002, max_target_position_error_m=.02,
           max_target_orientation_error_rad=.1)


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
        np.testing.assert_allclose(action[:3], np.array(expected) * .0004, atol=1e-10)
        np.testing.assert_allclose(action[3:], np.array(expected) * .002, atol=1e-10)


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


def test_timer_stall_still_stops():
    c = PoseIntegrator(CFG)
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    a, b = inputs()
    c.step(pose, a, b, .02, True)
    b['rb'], a['ly'] = True, 1.
    assert c.step(pose, a, b, .02, True).any()
    assert not c.step(pose, a, b, .5, True).any()
    assert not c.enabled
    np.testing.assert_allclose(c.target, pose)


def test_large_configured_delta_respects_target_lead_limit():
    c = PoseIntegrator(dict(CFG, max_translation_delta_m=.04, max_rotation_delta_rad=.2))
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    a, b = inputs()
    c.step(pose, a, b, .02, True)
    b['rb'], a['ly'], a['ry'] = True, 1., 1.
    delta = c.step(pose, a, b, .02, True)
    np.testing.assert_allclose(delta, [.04, 0, 0, .2, 0, 0])
    np.testing.assert_allclose(c.target[:3], [.02, 0, 0])
    np.testing.assert_allclose(c.target[3:], [np.sin(.05), 0, 0, np.cos(.05)])
    assert c.enabled


def test_fixed_feedback_does_not_accumulate_target_while_input_held():
    c = PoseIntegrator(CFG)
    actual = np.array([.2, .1, .3, 0., 0., 0., 1.])
    a, b = inputs()
    c.step(actual, a, b, .02, True)
    b['rb'] = True
    a['ly'] = a['ry'] = 1.
    for _ in range(100):
        c.step(actual, a, b, .02, True)
        assert c.enabled
    np.testing.assert_allclose(c.target[:3], [.2004, .1, .3])
    np.testing.assert_allclose(c.target[3:], [np.sin(.001), 0., 0., np.cos(.001)])


def test_target_follows_current_feedback_with_one_increment():
    c = PoseIntegrator(CFG)
    actual = np.array([0., 0., 0., 0., 0., 0., 1.])
    a, b = inputs()
    c.step(actual, a, b, .02, True)
    b['rb'], a['ly'] = True, 1.
    for _ in range(100):
        c.step(actual, a, b, .02, True)
    np.testing.assert_allclose(c.target[:3], [.0004, 0., 0.])
    actual[0] = .01
    c.step(actual, a, b, .02, True)
    np.testing.assert_allclose(c.target[:3], [.0104, 0., 0.])


def test_release_and_center_still_limit_target_after_feedback_moves():
    c = PoseIntegrator(dict(CFG, max_translation_delta_m=.02, max_rotation_delta_rad=.1))
    actual = np.array([0., 0., 0., 0., 0., 0., 1.])
    a, b = inputs()
    c.step(actual, a, b, .02, True)
    b['rb'], a['ly'], a['ry'] = True, 1., 1.
    for _ in range(100):
        c.step(actual, a, b, .02, True)
    target = c.target.copy()
    a['ly'] = a['ry'] = 0.
    c.step(actual, a, b, .02, True)
    np.testing.assert_array_equal(c.target, target)
    b['rb'] = False
    actual[:3] = [-.01, 0., 0.]
    actual[3:] = delta_quaternion(np.array([-.05, 0., 0.]))
    c.step(actual, a, b, .02, True)
    np.testing.assert_allclose(c.target[:3], [.01, 0., 0.])
    np.testing.assert_allclose(c.target[3:], [np.sin(.025), 0., 0., np.cos(.025)], atol=1e-12)
    assert not c.enabled


def test_lead_limit_does_not_copy_uncommanded_axis_drift_into_target():
    c = PoseIntegrator(dict(CFG, max_translation_delta_m=.02))
    actual = np.array([0., 0., 0., 0., 0., 0., 1.])
    a, b = inputs()
    c.step(actual, a, b, .02, True)
    a['ly'], b['rb'] = 1., True
    for _ in range(100):
        c.step(actual, a, b, .02, True)
    np.testing.assert_allclose(c.target[:3], [.02, 0., 0.])
    a['ly'] = 0.
    actual[2] = .01
    c.step(actual, a, b, .02, True)
    np.testing.assert_allclose(c.target[:3], [np.sqrt(.02 ** 2 - .01 ** 2), 0., 0.])
    assert np.linalg.norm(c.target[:3] - actual[:3]) == pytest.approx(.02)


@pytest.mark.parametrize('frame', ['base', 'tcp'])
def test_each_step_uses_latest_actual_pose(frame):
    c = PoseIntegrator(dict(CFG, max_target_position_error_m=.5,
                            max_target_orientation_error_rad=np.pi))
    c.frame = frame
    a, b = inputs()
    c.step(np.array([0., 0., 0., 0., 0., 0., 1.]), a, b, .02, True)
    b['rb'] = True
    a['ly'] = a['ry'] = 1.
    c.step(np.array([0., 0., 0., 0., 0., 0., 1.]), a, b, .02, True)
    # 新反馈与旧目标不同：本次目标必须以新的实测位姿为基准。
    h = np.sqrt(.5)
    actual = np.array([.1, .2, .3, 0., 0., h, h])
    c.step(actual, a, b, .04, True)
    expected_position = [.1004, .2, .3] if frame == 'base' else [.1, .2004, .3]
    if frame == 'base':
        expected_orientation = multiply(
            delta_quaternion(np.array([.002, 0., 0.])), actual[3:])
    else:
        expected_orientation = multiply(
            delta_quaternion(np.array([0., .002, 0.])), actual[3:])
    np.testing.assert_allclose(c.target[:3], expected_position)
    np.testing.assert_allclose(c.target[3:], expected_orientation, atol=1e-12)
    assert c.enabled
    np.testing.assert_array_equal(actual, [.1, .2, .3, 0., 0., h, h])


def test_zero_increment_keeps_last_target():
    c = PoseIntegrator(CFG)
    actual = np.array([0., 0., 0., 0., 0., 0., 1.])
    a, b = inputs()
    c.step(actual, a, b, .02, True)
    b['rb'], a['ly'] = True, 1.
    c.step(actual, a, b, .02, True)
    target = c.target.copy()
    a['ly'] = 0.
    actual[1] = .01
    assert not c.step(actual, a, b, .02, True).any()
    np.testing.assert_array_equal(c.target, target)


def test_release_and_repress_without_input_keep_target():
    c = PoseIntegrator(CFG)
    actual = np.array([0., 0., 0., 0., 0., 0., 1.])
    a, b = inputs()
    c.step(actual, a, b, .02, True)
    b['rb'], a['ly'] = True, 1.
    c.step(actual, a, b, .02, True)
    target = c.target.copy()
    actual[1] = .01
    b['rb'] = False
    c.step(actual, a, b, .02, True)
    assert not c.enabled
    np.testing.assert_array_equal(c.target, target)
    b['rb'], a['ly'] = True, 0.
    c.step(actual, a, b, .02, True)
    np.testing.assert_array_equal(c.target, target)
    a['ly'] = 1.
    c.step(actual, a, b, .02, True)
    np.testing.assert_allclose(c.target[:3], [.0004, .01, 0.])
    c.step(actual, a, b, .02, False)
    np.testing.assert_array_equal(c.target, actual)


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


def test_precision_and_vector_delta_limit():
    c = PoseIntegrator(CFG)
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    a, b = inputs()
    c.step(pose, a, b, .02, True)
    a['ly'] = a['lx'] = a['rt'] = 1.
    b['rb'] = b['lb'] = True
    action = c.step(pose, a, b, .02, True)
    assert np.linalg.norm(action[:3]) == pytest.approx(.0001)


@pytest.mark.parametrize('dt', [.005, .01, .02, .04])
@pytest.mark.parametrize('frame', ['base', 'tcp'])
def test_delta_does_not_depend_on_control_period(dt, frame):
    c = PoseIntegrator(CFG)
    c.frame = frame
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    a, b = inputs()
    c.step(pose, a, b, dt, True)
    a['ly'], a['ry'], b['rb'] = 1., 1., True
    delta = c.step(pose, a, b, dt, True)
    np.testing.assert_allclose(delta, [.0004, 0, 0, .002, 0, 0])
    np.testing.assert_allclose(c.target[:3], [.0004, 0, 0])


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
