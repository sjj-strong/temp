"""使用内存数据集验证手柄录制事件，不写用户数据集。"""
import queue
import time
from types import SimpleNamespace

import numpy as np
import pytest

from ur_teleop.frame_builder import FrameBuilder


def test_cartesian_features_preserve_alicia():
    cart = FrameBuilder(dict(action_space='cartesian_velocity'), {})
    joint = FrameBuilder({}, {})
    assert cart.features()[2] == ['vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'cmd_gripper']
    assert joint.features()[2][0] == 'cmd_shoulder_pan_joint'
    action = [.01, .02, .03, .04, .05, .06, 1.]
    frame = cart.build([0]*6, [0, 0, 0, 0, 0, 0, 1], 0., action)
    np.testing.assert_allclose(frame['action'], action)


def test_episode_events_and_stale_gate(monkeypatch):
    pytest.importorskip('rclpy')
    from ur_teleop.data_recorder import DataRecorderNode
    from std_msgs.msg import String
    import threading

    # 绕过 ROS 构造，仅测试同一录制线程中的事件与数据写入逻辑。
    node = object.__new__(DataRecorderNode)
    node._xbot = True
    node._rec = {}
    node._events = queue.Queue()
    node._ready = True
    node._data_timeout = .5
    node._ready_at = node._cmd_at = node._joint_at = time.monotonic()
    node._gripper_at = time.monotonic()
    node._camera_at, node._cameras, node._camera_frames = {}, {}, {}
    node._teleop_cmd = [.01, 0, 0, 0, 0, 0, 1.]
    node._ur_joints = [0.]*6
    node._ur_gripper_rad = 0.
    node._recording = False
    node._frame_count = node._episode_count = 0
    node._min_frames = 2
    node._missing_cam_warned = set()
    node._lock = threading.Lock()
    node._builder = FrameBuilder(dict(action_space='cartesian_velocity'), {})
    node._ee_source, node._ee_warned = 'tf', False
    node._enable_sent = False
    saved, frames, finished = [], [], []
    node._dataset = SimpleNamespace(add_frame=frames.append,
        save_episode=lambda: (saved.append(list(frames)), frames.clear()),
        clear_episode_buffer=frames.clear, finalize=lambda: finished.append(True))
    node._finished_pub = SimpleNamespace(publish=lambda m: None)
    node._enable_pub = SimpleNamespace(publish=lambda m: pytest.fail('Xbot 不应等待或发布 Alicia enable'))
    log = SimpleNamespace(info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None)
    monkeypatch.setattr(DataRecorderNode, 'get_logger', lambda self: log)
    monkeypatch.setattr(DataRecorderNode, '_get_ee_pose', lambda self: [0, 0, 0, 0, 0, 0, 1])

    def event(name):
        node._event_cb(String(data=name))
        return node.poll_event()

    assert not node._recording  # 预摆位阶段不录制。
    event('start')
    assert node._recording
    node._record_frame()
    node._record_frame()
    event('save')
    assert len(saved) == 1 and not node._recording
    event('start')
    node._record_frame()
    event('discard')
    assert not frames and node._episode_count == 1
    node._cmd_at -= 2.
    event('start')
    assert not node._recording
    node._cmd_at = time.monotonic()
    event('start')
    node._record_frame()
    node._record_frame()
    assert event('finalize')
    node.finalize()
    assert finished == [True] and len(saved) == 2


def test_independent_gripper_joint_state():
    pytest.importorskip('rclpy')
    from ur_teleop.data_recorder import DataRecorderNode
    from ur_teleop.config import UR_GRIPPER_JOINT
    from sensor_msgs.msg import JointState
    import threading
    node = object.__new__(DataRecorderNode)
    node._xbot = True
    node._lock = threading.Lock()
    node._ur_joints = [1.]*6
    node._ur_gripper_rad = 0.
    node._joint_cb(JointState(name=[UR_GRIPPER_JOINT], position=[.79]))
    assert node._ur_gripper_rad == .79
    assert node._ur_joints == [1.]*6
