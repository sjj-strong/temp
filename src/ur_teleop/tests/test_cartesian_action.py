"""绝对位姿、基座相对增量及 action 格式校验。"""
import numpy as np
import pytest

from ur_teleop.cartesian_action import encode_action, action_label
from ur_teleop.xbot_core import multiply, delta_quaternion
from ur_teleop.config import load_config, ConfigError


def test_abs_matches_published_target():
    actual = [.1, .2, .3, 0, 0, 0, 1]
    target = [.11, .22, .33, 0, 0, np.sin(.2), np.cos(.2)]
    np.testing.assert_allclose(encode_action(target, actual, 1., 'abs'), target + [1.])
    assert action_label('abs', 'base') == 'cartesian_pose:abs:base:tool0'


def test_rel_reconstructs_target_in_base():
    actual = np.array([.1, .2, .3, 0, 0, np.sqrt(.5), np.sqrt(.5)])
    target = actual.copy()
    target[:3] += [.001, -.002, .003]
    target[3:] = multiply(delta_quaternion(np.array([.01, -.02, .03])), actual[3:])
    action = encode_action(target, actual, 0., 'rel')
    np.testing.assert_allclose(action, [.001, -.002, .003, .01, -.02, .03, 0.], atol=1e-12)
    np.testing.assert_allclose(multiply(delta_quaternion(action[3:6]), actual[3:]), target[3:])


def test_rel_identical_orientation_with_opposite_quaternion_sign():
    np.testing.assert_allclose(encode_action([0, 0, 0, 0, 0, 0, -1],
                                            [0, 0, 0, 0, 0, 0, 1], 1., 'rel'), [0]*6 + [1.])


@pytest.mark.parametrize('field, value', [('action_mode', 'velocity'), ('action_space', 'cartesian_velocity')])
def test_invalid_recording_mode_rejected(tmp_path, field, value):
    file = tmp_path / 'invalid.yaml'
    file.write_text('base_config: /ros2_ws/src/ur_teleop/config/xbot_teleop.yaml\n'
                    f'recorder:\n  {field}: {value}\n')
    with pytest.raises(ConfigError):
        load_config(file)


def test_recorder_rejects_mismatched_action_mode():
    pytest.importorskip('rclpy')
    import threading
    from std_msgs.msg import Float64MultiArray, MultiArrayDimension
    from ur_teleop.data_recorder import DataRecorderNode
    node = object.__new__(DataRecorderNode)
    node._lock = threading.Lock()
    node._xbot = True
    node._action_size = 8
    node._action_label = action_label('abs')
    msg = Float64MultiArray(data=[0., 0., 0., 0., 0., 0., 1., 1.])
    msg.layout.dim = [MultiArrayDimension(label=action_label('abs'), size=8, stride=8)]
    node._cmd_cb(msg)
    assert node._teleop_cmd == list(msg.data)
    msg.layout.dim[0].label = action_label('rel')
    node._cmd_cb(msg)
    assert node._teleop_cmd is None
