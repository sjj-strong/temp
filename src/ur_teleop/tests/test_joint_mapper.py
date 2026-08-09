import pytest

from ur_teleop.joint_mapper import JointMapper

UR = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
MAPPING = {
    "alicia_joint_order": ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"],
    "ur_joint_order": UR,
    "sign": [1, 1, 1, 1, 1, 1],
    "scale": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
}
SAFETY = {
    "clamp_margin_rad": 0.1,
    "limits": {j: [-6.283, 6.283] for j in UR},
}
MASTER_HOME = [0.0, -1.2, 0.5, 0.0, 0.0, 0.0]
SLAVE_HOME = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]


def _mapper(**overrides):
    cfg = dict(MAPPING, safety=SAFETY, **overrides)
    return JointMapper(cfg, MASTER_HOME, SLAVE_HOME)


def test_at_home_maps_to_slave_home():
    assert _mapper().master_to_slave(MASTER_HOME) == pytest.approx(SLAVE_HOME)


def test_offset_mapping():
    master = [x + 0.1 for x in MASTER_HOME]
    assert _mapper().master_to_slave(master) == pytest.approx([s + 0.1 for s in SLAVE_HOME])


def test_scale_is_applied():
    m = _mapper(scale=[2.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    master = [MASTER_HOME[0] + 0.1] + MASTER_HOME[1:]
    out = m.master_to_slave(master)
    assert out[0] == pytest.approx(SLAVE_HOME[0] + 0.2)   # scale=2 生效
    assert out[1] == pytest.approx(SLAVE_HOME[1])         # 其他关节不变


def test_negative_sign_flips_direction():
    m = _mapper(sign=[-1, 1, 1, 1, 1, 1])
    master = [MASTER_HOME[0] + 0.1] + MASTER_HOME[1:]
    assert m.master_to_slave(master)[0] == pytest.approx(SLAVE_HOME[0] - 0.1)


def test_clamp_to_safety_limits():
    m = _mapper()
    m._limits = {"shoulder_pan_joint": [-1.0, 1.0], **{j: [-6.283, 6.283] for j in UR[1:]}}
    master = [MASTER_HOME[0] + 5.0] + MASTER_HOME[1:]
    out = m.master_to_slave(master)
    assert out[0] == pytest.approx(1.0 - 0.1)   # 上限 clamp（含 margin）


def test_wrong_master_length_raises():
    with pytest.raises(ValueError):
        _mapper().master_to_slave([0.0] * 5)


def test_missing_joint_raises():
    cfg = dict(MAPPING, safety=SAFETY,
               ur_joint_order=UR[:5] + ["bogus_joint"])
    with pytest.raises(ValueError, match="bogus_joint"):
        JointMapper(cfg, MASTER_HOME, SLAVE_HOME)
