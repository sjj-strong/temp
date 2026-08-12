import math

import pytest

from ur_teleop.frame_builder import FrameBuilder

REC = {
    "fps": 50,
    "task": "teleoperation",
    "cameras": {},
    "state_threshold_rad": 0.4,
}
UR = [0.1, -1.5, 0.2, -1.5, 0.0, 0.1]
EE = [0.5, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]
CMD = [0.11, -1.51, 0.21, -1.51, 0.01, 0.11, 1.0]  # 7 维，夹爪指令 1.0


def test_features_14_and_7():
    fb = FrameBuilder(REC, {})
    features, state_names, action_names = fb.features()
    assert features["observation.state"]["shape"] == (14,)
    assert features["action"]["shape"] == (7,)
    assert state_names[-1] == "gripper_state"
    assert action_names[-1] == "cmd_gripper"


def test_build_full_frame():
    fb = FrameBuilder(REC, {})
    frame = fb.build(UR, EE, 0.79, CMD)
    assert frame is not None
    assert frame["observation.state"].shape == (14,)
    assert frame["observation.state"][-1] == pytest.approx(1.0)   # 0.79 rad > 0.4 → closed
    assert frame["action"].shape == (7,)
    assert frame["action"][-1] == pytest.approx(1.0)
    assert frame["task"] == "teleoperation"


def test_gripper_state_threshold_open():
    fb = FrameBuilder(REC, {})
    frame = fb.build(UR, EE, 0.0, CMD)
    assert frame["observation.state"][-1] == pytest.approx(0.0)


def test_missing_joints_returns_none():
    fb = FrameBuilder(REC, {})
    assert fb.build(None, EE, 0.0, CMD) is None
    assert fb.build(UR, EE, 0.0, None) is None


def test_missing_ee_pose_uses_nan():
    fb = FrameBuilder(REC, {})
    frame = fb.build(UR, None, 0.0, CMD)
    assert math.isnan(frame["observation.state"][6])  # ee 段起始（6 关节之后）


def test_camera_feature_included():
    rec = dict(REC, cameras={"wrist": {"topic": "/x", "image_key": "wrist",
                                       "height": 480, "width": 640}})
    features, _, _ = FrameBuilder(rec, {}).features()
    assert "observation.images.wrist" in features
    assert features["observation.images.wrist"]["shape"] == (480, 640, 3)
