import math

import pytest

from ur_teleop.frame_builder import FrameBuilder

REC = {
    "fps": 50,
    "task": "teleoperation",
    "cameras": {},
}
UR = [0.1, -1.5, 0.2, -1.5, 0.0, 0.1]
EE = [0.5, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]
CMD = [0.11, -1.51, 0.21, -1.51, 0.01, 0.11, 1.0]  # 7 维，夹爪指令 1.0


def test_features_13_and_7():
    fb = FrameBuilder(REC, {})
    features, state_names, action_names = fb.features()
    assert features["observation.state"]["shape"] == (13,)
    assert features["action"]["shape"] == (7,)
    assert "gripper_state" not in state_names
    assert action_names[-1] == "cmd_gripper"


def test_build_full_frame():
    fb = FrameBuilder(REC, {})
    frame = fb.build(UR, EE, CMD)
    assert frame is not None
    assert frame["observation.state"].shape == (13,)
    assert frame["action"].shape == (7,)
    assert frame["action"][-1] == pytest.approx(1.0)
    assert frame["task"] == "teleoperation"


@pytest.mark.parametrize("cartesian", [False, True])
@pytest.mark.parametrize("enabled", [False, True])
@pytest.mark.parametrize("command", [0., 1.])
def test_single_gripper_recording_switch(cartesian, enabled, command):
    rec = dict(REC, record_action_gripper=enabled)
    if cartesian:
        rec.update(action_space='cartesian_pose', action_mode='rel')
    fb = FrameBuilder(rec, {})
    features, state_names, action_names = fb.features()
    frame = fb.build(UR, EE, CMD[:6] + [command],
                     reference_link='base', tcp_link='tool0')
    assert 'gripper_state' not in state_names
    assert ('cmd_gripper' in action_names) == enabled
    assert frame['action'].shape == features['action']['shape']
    assert len(frame['action']) == (7 if enabled else 6)
    if enabled:
        assert frame['action'][-1] == command


def test_missing_joints_returns_none():
    fb = FrameBuilder(REC, {})
    assert fb.build(None, EE, CMD) is None
    assert fb.build(UR, EE, None) is None


def test_missing_ee_pose_uses_nan():
    fb = FrameBuilder(REC, {})
    frame = fb.build(UR, None, CMD)
    assert math.isnan(frame["observation.state"][6])  # ee 段起始（6 关节之后）


def test_camera_feature_included():
    rec = dict(REC, cameras={"wrist": {"topic": "/x", "image_key": "wrist",
                                       "height": 480, "width": 640}})
    features, _, _ = FrameBuilder(rec, {}).features()
    assert "observation.images.wrist" in features
    assert features["observation.images.wrist"]["shape"] == (480, 640, 3)
