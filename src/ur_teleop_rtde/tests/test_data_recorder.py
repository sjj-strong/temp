"""Data recorder tests: FrameBuilder integration + LeRobot lifecycle with mock dataset."""

import sys
import threading
import types

import numpy as np
import pytest

from ur_teleop_rtde.frame_builder import FrameBuilder


class StubLogger:
    def info(self, *a, **k): pass
    def warn(self, *a, **k): pass
    def error(self, *a, **k): pass


class StubPublisher:
    def publish(self, msg):
        pass


def _make_recorder_node(cfg, min_frames=2):
    """DataRecorderNode instance bypassing __init__ (no rclpy needed)."""
    from ur_teleop_rtde.data_recorder import DataRecorderNode
    node = object.__new__(DataRecorderNode)
    node.get_logger = lambda: StubLogger()
    node._rec = cfg.get("recorder", {})
    node._fps = 50
    node._min_frames = min_frames
    node._cameras = {}
    node._builder = FrameBuilder(node._rec, cfg.get("gripper", {}))
    node._features, _, _ = node._builder.features()
    node._lock = threading.Lock()
    node._ur_joints = None
    node._ur_ee_pose = None
    node._gripper_open = False
    node._teleop_cmd = None
    node._camera_frames = {}
    node._enable_sent = False
    node._missing_cam_warned = set()
    node._dataset = None
    node._episode_count = 0
    node._recording = False
    node._frame_count = 0
    node._kb = None
    node._enable_pub = StubPublisher()
    return node


class MockLeRobotDataset:
    """Fake LeRobotDataset capturing create/add_frame/save_episode calls."""

    created = []
    instances = []

    def __init__(self):
        self.frames = []
        self.saved = 0
        self.discarded = 0
        self.finalized = False
        MockLeRobotDataset.instances.append(self)

    @classmethod
    def create(cls, **kwargs):
        ds = cls()
        cls.created.append(kwargs)
        return ds

    def add_frame(self, frame):
        self.frames.append(frame)

    def save_episode(self):
        self.saved += 1

    def clear_episode_buffer(self):
        self.discarded += 1

    def finalize(self):
        self.finalized = True


@pytest.fixture(autouse=True)
def mock_lerobot(monkeypatch):
    mod = types.ModuleType("lerobot")
    datasets = types.ModuleType("lerobot.datasets")
    datasets.LeRobotDataset = MockLeRobotDataset
    sys.modules["lerobot"] = mod
    sys.modules["lerobot.datasets"] = datasets
    MockLeRobotDataset.created = []
    MockLeRobotDataset.instances = []
    return datasets


# ----------------------------------------------------------------------
# FrameBuilder integration
# ----------------------------------------------------------------------

def test_frame_builder_full_state(cfg):
    rec = cfg.get("recorder", {})
    builder = FrameBuilder(rec, cfg.get("gripper", {}))
    features, state_names, action_names = builder.features()
    assert features["observation.state"]["shape"] == (14,)
    assert features["action"]["shape"] == (7,)

    ur = [0.1, -1.5, 0.2, -1.5, 0.0, 0.1]
    ee = [0.3, 0.4, 0.5, 0.0, 0.0, 0.0, 1.0]
    cmd = [0.1, -1.5, 0.2, -1.5, 0.0, 0.1, 0.0]
    frame = builder.build(ur, ee, gripper_state_rad=1.0, teleop_cmd=cmd)
    assert frame["observation.state"].shape == (14,)
    assert np.allclose(frame["observation.state"][:6], ur)
    assert np.allclose(frame["observation.state"][6:13], ee)
    assert frame["observation.state"][13] == 1.0  # open
    assert np.allclose(frame["action"][:6], cmd[:6])
    assert frame["task"] == rec.get("task", "teleoperation")


def test_frame_builder_gripper_closed(cfg):
    rec = cfg.get("recorder", {})
    builder = FrameBuilder(rec, cfg.get("gripper", {}))
    ur = [0.1, -1.5, 0.2, -1.5, 0.0, 0.1]
    cmd = [0.1, -1.5, 0.2, -1.5, 0.0, 0.1, 1.0]
    frame = builder.build(ur, None, gripper_state_rad=0.0, teleop_cmd=cmd)
    assert frame["observation.state"][13] == 0.0  # closed
    assert np.isnan(frame["observation.state"][6]).all()  # EE None → NaN


def test_frame_builder_returns_none_without_essential(cfg):
    rec = cfg.get("recorder", {})
    builder = FrameBuilder(rec, cfg.get("gripper", {}))
    assert builder.build(None, None, 1.0, [0.0] * 7) is None
    assert builder.build([0.0] * 6, None, 1.0, None) is None


# ----------------------------------------------------------------------
# DataRecorderNode episode lifecycle (no ROS — direct method calls)
# ----------------------------------------------------------------------

def test_episode_lifecycle(cfg, mock_lerobot):
    """start → record frames → save → finalize with the mock dataset."""
    node = _make_recorder_node(cfg)

    # start episode
    node._start_episode()
    assert node._recording
    assert node._dataset is not None

    # record a few frames
    node._ur_joints = [0.1, -1.5, 0.2, -1.5, 0.0, 0.1]
    node._ur_ee_pose = [0.3, 0.4, 0.5, 0.0, 0.0, 0.0, 1.0]
    node._gripper_open = True
    node._teleop_cmd = [0.1, -1.5, 0.2, -1.5, 0.0, 0.1, 0.0]
    for _ in range(3):
        node._record_frame()
    assert node._frame_count == 3
    assert len(node._dataset.frames) == 3

    # save
    node._save_episode()
    assert node._dataset.saved == 1
    assert node._episode_count == 1
    assert not node._recording

    # finalize
    node.finalize()
    assert node._dataset.finalized


def test_short_episode_discarded(cfg, mock_lerobot):
    node = _make_recorder_node(cfg, min_frames=10)  # more than we record

    node._start_episode()
    node._ur_joints = [0.1, -1.5, 0.2, -1.5, 0.0, 0.1]
    node._teleop_cmd = [0.1, -1.5, 0.2, -1.5, 0.0, 0.1, 0.0]
    node._record_frame()  # 1 frame < 10 min
    node._save_episode()
    assert node._dataset.discarded == 1
    assert node._dataset.saved == 0
