"""Robotiq 2F-85 gripper control — hysteresis-based binary FSM.

Pure logic — no rclpy imports. Maps Alicia gripper position (meters, 0=open)
to a binary target; Robotiq knuckle command is rad (0=open, 0.79=closed).
"""

from enum import Enum


class GripperTarget(Enum):
    OPEN = 0
    CLOSED = 1
    UNKNOWN = 2


class GripperController:
    """Binary FSM with hysteresis; update() returns UNKNOWN while target is unchanged."""

    def __init__(self, gripper_config: dict):
        self.enabled = bool(gripper_config.get("enabled", False))
        self.action_server = str(
            gripper_config.get("action_server", "/robotiq_gripper_controller/gripper_cmd")
        )
        self._open_pos = float(gripper_config.get("open_pos_rad", 0.0))
        self._close_pos = float(gripper_config.get("close_pos_rad", 0.79))
        self._close_threshold = float(gripper_config.get("close_threshold_m", 0.0125))
        self._open_threshold = float(gripper_config.get("open_threshold_m", 0.005))
        self._max_effort = float(gripper_config.get("max_effort", 50.0))
        self._current = GripperTarget.UNKNOWN

    @property
    def open_position(self) -> float:
        return self._open_pos

    @property
    def close_position(self) -> float:
        return self._close_pos

    @property
    def max_effort(self) -> float:
        return self._max_effort

    @property
    def current_target(self) -> GripperTarget:
        return self._current

    def update(self, alicia_gripper_m: float) -> GripperTarget:
        """Return OPEN/CLOSED on target change, UNKNOWN while in deadband/unchanged."""
        if not self.enabled:
            return GripperTarget.UNKNOWN
        if alicia_gripper_m > self._close_threshold:
            new_target = GripperTarget.CLOSED
        elif alicia_gripper_m < self._open_threshold:
            new_target = GripperTarget.OPEN
        else:
            new_target = self._current
        changed = new_target != self._current
        self._current = new_target
        return new_target if changed else GripperTarget.UNKNOWN

    def get_knuckle_command(self, target: GripperTarget) -> float:
        return self._close_pos if target == GripperTarget.CLOSED else self._open_pos

    def get_gripper_command_signal(self, target: GripperTarget) -> float:
        return 1.0 if target == GripperTarget.CLOSED else 0.0
