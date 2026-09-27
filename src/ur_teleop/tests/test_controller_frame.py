"""离线验证控制器基座转换，不连接或控制机械臂。"""
import numpy as np
import pytest

from ur_teleop.controller_frame import controller_base_frame, transform_pose


def test_installed_controller_frames():
    assert controller_base_frame(True) == 'base_link'
    assert controller_base_frame(False) == 'base'


def test_full_pose_conversion_and_roundtrip():
    pose = np.array([.1, .2, .3, 0., 0., 0., 1.])
    transform = np.array([1., 2., 3., 0., 0., 1., 0.])
    result = transform_pose(pose, transform)
    np.testing.assert_allclose(result, [.9, 1.8, 3.3, 0, 0, 1, 0])
    inverse = [1., 2., -3., 0., 0., -1., 0.]
    np.testing.assert_allclose(transform_pose(result, inverse), pose, atol=1e-12)
    np.testing.assert_allclose(pose, [.1, .2, .3, 0, 0, 0, 1])


def test_mock_identity_conversion():
    pose = [.1, .2, .3, 0., 0., 0., 1.]
    np.testing.assert_allclose(transform_pose(pose, [0, 0, 0, 0, 0, 0, 1]), pose)


@pytest.mark.parametrize('transform', [[0]*7, [float('nan'), 0, 0, 0, 0, 0, 1]])
def test_invalid_transform_rejected(transform):
    with pytest.raises(ValueError):
        transform_pose([0, 0, 0, 0, 0, 0, 1], transform)


def test_missing_tf_blocks_target():
    from types import SimpleNamespace
    from tf2_ros import TransformException
    from ur_teleop.xbot_teleop_node import XbotTeleopNode
    node = object.__new__(XbotTeleopNode)
    node.controller_frame = 'base'
    def missing(*args):
        raise TransformException('缺少 TF')
    node.buffer = SimpleNamespace(lookup_transform=missing)
    node.get_logger = lambda: SimpleNamespace(error=lambda *args, **kwargs: None)
    assert node.controller_transform() is None
