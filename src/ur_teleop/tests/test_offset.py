import pytest

from ur_teleop.offset import SessionOffset
from ur_teleop.joint_mapper import JointMapper

UR = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def test_capture_stores_copies():
    off = SessionOffset()
    m, s = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], [-1.0, -2.0, 0.0, 0.0, 0.0, 0.0]
    off.capture(m, s)
    assert off.captured
    assert off.master_home == m
    assert off.slave_home == s
    m[0] = 99.0  # 外部修改不影响内部
    assert off.master_home[0] == 0.1


def test_not_captured_by_default():
    assert not SessionOffset().captured


def test_mapper_with_captured_offset_identity():
    """捕获 offset 后，主臂在捕获位时从臂指令 == 捕获位（sign=1 scale=1）。"""
    off = SessionOffset()
    master = [0.0, -1.2, 0.5, 0.0, 0.0, 0.0]
    slave = [0.01, -1.58, 0.02, -1.55, -0.01, 0.02]
    off.capture(master, slave)
    mapping = {
        "alicia_joint_order": ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"],
        "ur_joint_order": UR,
        "sign": [1, 1, 1, 1, 1, 1],
        "scale": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        "safety": {"clamp_margin_rad": 0.1, "limits": {j: [-6.283, 6.283] for j in UR}},
    }
    m = JointMapper(mapping, off.master_home, off.slave_home)
    assert m.master_to_slave(master) == pytest.approx(slave)
