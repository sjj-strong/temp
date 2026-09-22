import pytest

from ur_teleop.config import (
    ConfigError,
    UR_JOINT_NAMES,
    load_config,
    gripper_position_to_value,
    gripper_value_to_position,
)

LIMITS = "\n".join(f"    {j}: [-6.283, 6.283]" for j in UR_JOINT_NAMES)
BASE = f"""\
mode: teleop
sim: true
home:
  master: [0.0, -1.2, 0.5, 0, 0, 0]
  slave: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
mapping:
  alicia_joint_order: [Joint1, Joint2, Joint3, Joint4, Joint5, Joint6]
  ur_joint_order: {UR_JOINT_NAMES}
  sign: [1, 1, 1, 1, 1, 1]
  scale: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
safety:
  clamp_margin_rad: 0.1
  limits:
{LIMITS}
teleop:
  command_rate_hz: 50
"""


def _write(tmp_path, text: str):
    p = tmp_path / "ur_teleop.yaml"
    p.write_text(text)
    return str(p)


def test_load_minimal_valid_config(tmp_path):
    cfg = load_config(_write(tmp_path, BASE))
    assert cfg["mode"] == "teleop"
    assert cfg["teleop"]["command_rate_hz"] == 50
    assert len(cfg["home"]["master"]) == 6
    assert cfg["safety"]["limits"]["shoulder_pan_joint"] == [-6.283, 6.283]


def test_joint_impedance_controller_config_is_accepted(tmp_path):
    cfg = load_config(_write(tmp_path, BASE + """
  controller: joint_impedance
"""))
    assert cfg["teleop"]["controller"] == "joint_impedance"


def test_unknown_teleop_controller_is_rejected(tmp_path):
    with pytest.raises(ConfigError, match="teleop.controller"):
        load_config(_write(tmp_path, BASE + """
  controller: unsupported
"""))


def test_ruckig_enabled_boolean_is_accepted(tmp_path):
    cfg = load_config(_write(tmp_path, BASE + """
ruckig:
  enabled: false
"""))
    assert cfg["ruckig"]["enabled"] is False


def test_ruckig_enabled_rejects_non_boolean(tmp_path):
    with pytest.raises(ConfigError, match="ruckig.enabled"):
        load_config(_write(tmp_path, BASE + """
ruckig:
  enabled: "false"
"""))


def test_missing_required_key_raises(tmp_path):
    with pytest.raises(ConfigError, match="mode"):
        load_config(_write(tmp_path, "sim: true\n"))


def test_missing_file_raises(tmp_path):
    with pytest.raises(ConfigError, match="not found"):
        load_config(str(tmp_path / "nope.yaml"))


def test_invalid_mode_raises(tmp_path):
    with pytest.raises(ConfigError, match="mode must be"):
        load_config(_write(tmp_path, BASE.replace("mode: teleop", "mode: bogus")))


def test_home_length_mismatch_raises(tmp_path):
    with pytest.raises(ConfigError, match="home"):
        load_config(_write(tmp_path, BASE.replace(
            "  master: [0.0, -1.2, 0.5, 0, 0, 0]", "  master: [0.0]")))


def test_mapping_missing_joint_order_raises(tmp_path):
    with pytest.raises(ConfigError, match="mapping.alicia_joint_order"):
        load_config(_write(tmp_path, BASE.replace("  alicia_joint_order: [Joint1, Joint2, Joint3, Joint4, Joint5, Joint6]\n", "")))


def test_limits_missing_joint_raises(tmp_path):
    with pytest.raises(ConfigError, match="shoulder_pan_joint"):
        load_config(_write(tmp_path, BASE.replace(
            "    shoulder_pan_joint: [-6.283, 6.283]\n", "")))


def test_gripper_conversion_50mm():
    assert gripper_position_to_value(0.0) == 1000.0      # open
    assert gripper_position_to_value(0.025) == 0.0       # closed
    assert gripper_position_to_value(0.0125) == pytest.approx(500.0)
    assert gripper_value_to_position(1000.0) == 0.0
    assert gripper_value_to_position(0.0) == pytest.approx(0.025)
    assert gripper_value_to_position(500.0) == pytest.approx(0.0125)


def test_gripper_conversion_100mm():
    assert gripper_position_to_value(0.05, "100mm") == 0.0
    assert gripper_value_to_position(0.0, "100mm") == pytest.approx(0.05)
