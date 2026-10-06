"""Xbot 归一化输入、固定周期增量和保持行为。"""
import numpy as np
import pytest

from ur_teleop.xbot_core import AXES, BUTTONS, ButtonEvents, JoyMapping, PoseIntegrator
from ur_teleop.controller_frame import (clip_workspace_target, transform_pose,
                                         inverse_transform_pose)
from ur_teleop.cartesian_action import encode_action

CFG = dict(precision_scale=.25, max_linear_speed_m_s=1., max_angular_speed_rad_s=5.)


def inputs():
    return dict.fromkeys(AXES, 0.), dict.fromkeys(BUTTONS, False)


def armed(frame='base'):
    core = PoseIntegrator(CFG)
    core.frame = frame
    actual = np.array([0., 0., 0., 0., 0., 0., 1.])
    axes, buttons = inputs()
    core.step(actual, axes, buttons, .02, True)
    buttons['rb'] = True
    return core, actual, axes, buttons


@pytest.mark.parametrize('frame, expected', [('base', [1., 0., 0.]), ('tcp', [0., 1., 0.])])
def test_base_tcp_direction(frame, expected):
    core, actual, axes, buttons = armed(frame)
    actual[5:] = np.sqrt(.5)
    axes['ly'] = axes['ry'] = 1.
    delta = core.step(actual, axes, buttons, .02, True)
    np.testing.assert_allclose(delta[:3], np.array(expected) * .02, atol=1e-12)
    np.testing.assert_allclose(delta[3:], np.array(expected) * .1, atol=1e-12)


def test_input_scale_and_fixed_dt():
    core, actual, axes, buttons = armed()
    axes.update(ly=1., lx=.2, rt=1.)
    buttons['lb'] = True
    delta = core.step(actual, axes, buttons, .02, True)
    assert np.linalg.norm(delta[:3]) == pytest.approx(.005)
    assert delta[1] == 0.
    assert delta[0] == pytest.approx(.005 / np.sqrt(2))
    buttons['lb'] = False
    assert np.linalg.norm(core.step(actual, axes, buttons, .02, True)[:3]) == pytest.approx(.02)


def test_uses_current_actual_and_holds_when_centered():
    core, actual, axes, buttons = armed()
    axes['ly'] = 1.
    core.step(actual, axes, buttons, .02, True)
    for _ in range(20):
        core.step(actual, axes, buttons, .02, True)
    assert core.target[0] == pytest.approx(.02)
    actual[0] = .1
    core.step(actual, axes, buttons, .02, True)
    assert core.target[0] == pytest.approx(.12)
    axes['ly'] = 0.
    actual[0] = .15
    core.step(actual, axes, buttons, .02, True)
    assert core.target[0] == pytest.approx(.12)
    buttons['rb'] = False
    core.step(actual, axes, buttons, .02, True)
    assert core.target[0] == pytest.approx(.12)


def test_moving_y_keeps_uncommanded_z_and_orientation_target():
    core, actual, axes, buttons = armed()
    axes['lx'] = 1.
    core.step(actual, axes, buttons, .02, True)
    assert core.target[1] == pytest.approx(.02)
    actual[:3] = [.03, .1, -.04]
    actual[3:] = [0., 0., np.sqrt(.5), np.sqrt(.5)]
    core.step(actual, axes, buttons, .02, True)
    np.testing.assert_allclose(core.target[:3], [0., .12, 0.])
    np.testing.assert_allclose(core.target[3:], [0., 0., 0., 1.])


def test_moving_z_keeps_uncommanded_y_target():
    core, actual, axes, buttons = armed()
    axes['lx'] = 1.
    core.step(actual, axes, buttons, .02, True)
    axes['lx'] = 0.
    axes['rt'] = 1.
    core.step(actual, axes, buttons, .02, True)
    actual[:3] = [.03, .08, .1]
    core.step(actual, axes, buttons, .02, True)
    np.testing.assert_allclose(core.target[:3], [0., .02, .12])


def test_rotation_only_keeps_position_target():
    core, actual, axes, buttons = armed()
    axes['ry'] = 1.
    actual[:3] = [.03, .08, -.04]
    core.step(actual, axes, buttons, .02, True)
    np.testing.assert_allclose(core.target[:3], [0., 0., 0.])
    assert core.target[3] != pytest.approx(0.)


def test_fault_requires_rb_release():
    core, actual, axes, buttons = armed()
    axes['ly'] = 1.
    core.step(actual, axes, buttons, .02, True)
    core.step(actual, axes, buttons, .02, False)
    assert not core.step(actual, axes, buttons, .02, True).any()
    buttons['rb'] = False
    core.step(actual, axes, buttons, .02, True)
    buttons['rb'] = True
    assert core.step(actual, axes, buttons, .02, True).any()


def test_workspace_clip_and_recorded_delta_match_final_target():
    transform = np.array([.3, 0., 0., 0., 0., 0., 1.])
    actual = np.array([.19, 0., 0., 0., 0., 0., 1.])
    proposed = np.array([.23, 0., 0., 0., 0., 0., 1.])
    final = clip_workspace_target(transform_pose(proposed, transform),
                                  [.3, 0., 0.], [.2, .2, .2])
    assert final[0] == pytest.approx(.5)
    np.testing.assert_allclose(inverse_transform_pose(final, transform)[:3], [.2, 0., 0.])
    absolute = encode_action(final, transform_pose(actual, transform), 0., 'abs')
    relative = encode_action(final, transform_pose(actual, transform), 0., 'rel')
    np.testing.assert_allclose(absolute[:7], final)
    np.testing.assert_allclose(relative[:3], [.01, 0., 0.])


def test_button_edges_and_view_hold():
    events = ButtonEvents()
    _, buttons = inputs()
    events.update(buttons, 0.)
    buttons['x'] = buttons['view'] = True
    assert events.update(buttons, .1) == {'x', 'view'}
    assert events.update(buttons, 2.2) == {'finalize'}
    assert events.update(buttons, 3.) == set()


def test_trigger_rest_and_layout_validation():
    cfg = dict(version=1, axis_count=7, button_count=8,
               buttons=dict(zip(BUTTONS, range(8))),
               axes={name: dict(index=i, rest=1. if name in ('lt', 'rt') else 0.,
                                positive=-1.) for i, name in enumerate(AXES)})
    mapping = JoyMapping(cfg)
    values, _ = mapping.decode([0., 0., 0., 0., 1., -1., 0.], [0] * 8)
    assert values['lt'] == 0. and values['rt'] == 1.
    with pytest.raises(ValueError):
        mapping.decode([0.] * 6, [0] * 8)
