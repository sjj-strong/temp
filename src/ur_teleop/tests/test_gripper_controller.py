import pytest

from ur_teleop.gripper_controller import GripperController, GripperTarget

CFG = {
    "enabled": True,
    "action_server": "/robotiq_gripper_controller/gripper_cmd",
    "open_pos_rad": 0.0,
    "close_pos_rad": 0.79,
    "close_threshold_m": 0.0125,
    "open_threshold_m": 0.005,
    "max_effort": 50.0,
}


def test_disabled_by_config():
    g = GripperController(dict(CFG, enabled=False))
    assert not g.enabled
    assert g.update(0.03) == GripperTarget.UNKNOWN


def test_open_to_closed_transition():
    g = GripperController(CFG)
    assert g.update(0.0) == GripperTarget.OPEN
    assert g.update(0.03) == GripperTarget.CLOSED
    assert g.get_knuckle_command(GripperTarget.CLOSED) == pytest.approx(0.79)
    assert g.get_knuckle_command(GripperTarget.OPEN) == pytest.approx(0.0)
    assert g.get_gripper_command_signal(GripperTarget.CLOSED) == 1.0
    assert g.get_gripper_command_signal(GripperTarget.OPEN) == 0.0


def test_hysteresis_deadband_keeps_previous_target():
    g = GripperController(CFG)
    g.update(0.03)               # CLOSED
    assert g.update(0.010) == GripperTarget.UNKNOWN   # 0.005 < 0.010 < 0.0125 死区
    assert g.update(0.004) == GripperTarget.OPEN      # 越过 open 阈值
    assert g.update(0.010) == GripperTarget.UNKNOWN   # 死区保持 OPEN
    assert g.update(0.02) == GripperTarget.CLOSED


def test_current_target_follows_updates():
    g = GripperController(CFG)
    g.update(0.03)
    assert g.current_target == GripperTarget.CLOSED


def test_knuckle_default_safe_open():
    g = GripperController(CFG)
    assert g.get_knuckle_command(GripperTarget.UNKNOWN) == pytest.approx(0.0)
