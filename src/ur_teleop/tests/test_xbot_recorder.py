"""使用内存数据集验证手柄录制事件，不写用户数据集。"""
import queue
import time
from types import SimpleNamespace

import numpy as np
import pytest

from ur_teleop.frame_builder import FrameBuilder


@pytest.mark.parametrize('mode, names, action', [
    ('abs', ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'cmd_gripper'], [.1, .2, .3, 0, 0, 0, 1, 1]),
    ('rel', ['dx', 'dy', 'dz', 'drx', 'dry', 'drz', 'cmd_gripper'], [.001, .002, 0, 0, 0, .01, 1]),
])
def test_cartesian_features_preserve_alicia(mode, names, action):
    cart = FrameBuilder(dict(action_space='cartesian_pose', action_mode=mode), {})
    joint = FrameBuilder({}, {})
    assert cart.features()[2] == names
    assert cart.features()[0]['action']['shape'] == (len(action),)
    assert joint.features()[2][0] == 'cmd_shoulder_pan_joint'
    frame = cart.build([0]*6, [0, 0, 0, 0, 0, 0, 1], action,
                       reference_link='base_link', tcp_link='tool0')
    np.testing.assert_allclose(frame['action'], action)
    assert cart.build([0]*6, [0, 0, 0, 0, 0, 0, 1], action[:-1],
                      reference_link='base_link', tcp_link='tool0') is None


@pytest.mark.parametrize('mode', ['abs', 'rel'])
def test_episode_events_and_stale_gate(monkeypatch, mode):
    pytest.importorskip('rclpy')
    from ur_teleop.data_recorder import DataRecorderNode
    from std_msgs.msg import String
    import threading

    # 绕过 ROS 构造，仅测试同一录制线程中的事件与数据写入逻辑。
    node = object.__new__(DataRecorderNode)
    node._xbot = True
    node._rec = dict(action_space='cartesian_pose', action_mode=mode)
    node._events = queue.Queue()
    node._ready = True
    node._data_timeout = .5
    node._ready_at = node._cmd_at = node._joint_at = time.monotonic()
    node._camera_at, node._cameras, node._camera_frames = {}, {}, {}
    node._teleop_cmd = [.1, .2, .3, 0, 0, 0, 1., 1.] if mode == 'abs' else [.01, 0, 0, 0, 0, 0, 1.]
    node._action_size = len(node._teleop_cmd)
    node._ur_joints = [0.]*6
    node._joint_velocity = node._joint_effort = node._wrench = None
    node._wrench_reference_link = None
    node._reference_link = 'base_link'
    node._tcp_link = 'tool0'
    node._joint_velocity_at = node._joint_effort_at = node._wrench_at = -float('inf')
    node._recording = False
    node._frame_count = node._episode_count = 0
    node._min_frames = 2
    node._missing_cam_warned = set()
    node._lock = threading.Lock()
    node._builder = FrameBuilder(node._rec, {})
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


def test_gripper_feedback_does_not_change_arm_observation():
    pytest.importorskip('rclpy')
    from ur_teleop.data_recorder import DataRecorderNode
    from ur_teleop.config import UR_GRIPPER_JOINT
    from sensor_msgs.msg import JointState
    import threading
    node = object.__new__(DataRecorderNode)
    node._xbot = True
    node._lock = threading.Lock()
    node._ur_joints = [1.]*6
    node._joint_cb(JointState(name=[UR_GRIPPER_JOINT], position=[.79]))
    assert node._ur_joints == [1.]*6


def test_xbot_optional_observation_fields_and_reference_links():
    rec = dict(action_space='cartesian_pose', action_mode='rel', record_joint_position=False,
               record_joint_velocity=True, record_joint_effort=True, record_tcp_pose=False,
               record_wrench=True, record_action_gripper=False,
               cameras={'off': {'enabled': False, 'topic': '/unused'}})
    builder = FrameBuilder(rec, {})
    features, names, action_names = builder.features()
    assert 'observation.images.off' not in features
    assert len(names) == 18 and len(action_names) == 6
    assert 'observation.tcp_link' not in features
    frame = builder.build(None, None, [0.] * 7,
                          joint_velocity=[1.] * 6, joint_effort=[2.] * 6,
                          wrench=[3.] * 6, wrench_reference_link='ft300_sensor',
                          reference_link='base')
    assert frame['observation.state'].shape == (18,)
    assert frame['action'].shape == (6,)
    assert frame['action.reference_link'] == 'base'
    assert frame['observation.wrench_reference_link'] == 'ft300_sensor'
    assert builder.build(None, None, [0.] * 7, joint_velocity=None,
                         joint_effort=[2.] * 6, wrench=[3.] * 6,
                         wrench_reference_link='ft300_sensor', reference_link='base') is None


def test_only_enabled_observations_gate_recording():
    pytest.importorskip('rclpy')
    from ur_teleop.data_recorder import DataRecorderNode
    now = time.monotonic()
    node = object.__new__(DataRecorderNode)
    node._rec = dict(action_space='cartesian_pose', record_joint_position=False,
                     record_joint_velocity=False, record_joint_effort=False,
                     record_tcp_pose=False, record_wrench=False)
    node._builder = FrameBuilder(node._rec, {})
    node._ready = True
    node._ready_at = node._cmd_at = now
    node._joint_at = node._joint_velocity_at = node._joint_effort_at = -float('inf')
    node._wrench_at = -float('inf')
    node._data_timeout = .5
    node._teleop_cmd = [0.] * 7 + [1.]
    node._action_size = 8
    node._cameras = node._camera_at = {}
    assert node._xbot_data_ready()
    node._rec['record_joint_velocity'] = True
    assert not node._xbot_data_ready()
    node._joint_velocity_at = now
    assert node._xbot_data_ready()
