"""Configuration loading/validation and Alicia gripper unit conversion.

Pure logic — no rclpy imports. All other modules obtain config via load_config().
"""

import os
from pathlib import Path
from typing import Any

import yaml

UR_JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
ALICIA_JOINT_NAMES = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
GRIPPER_JOINT = "Gripper"
UR_GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"

_REQUIRED_TOP = ["mode", "sim", "home", "mapping", "safety", "teleop"]
_REQUIRED_MAPPING = ["alicia_joint_order", "ur_joint_order", "sign", "scale"]
_REQUIRED_HOME = ["master", "slave"]


class ConfigError(ValueError):
    """Raised when the config file is missing/invalid."""


def load_config(path: str | Path) -> dict[str, Any]:
    """Load and validate ur_teleop.yaml. Raises ConfigError on any problem."""
    p = Path(path)
    if not p.exists():
        raise ConfigError(f"Config file not found: {p}")
    data = yaml.safe_load(p.read_text()) or {}

    for key in _REQUIRED_TOP:
        if key not in data:
            raise ConfigError(f"Missing required top-level key '{key}' in {p}")

    if data["mode"] not in ("teleop", "record"):
        raise ConfigError(f"mode must be 'teleop' or 'record', got '{data['mode']}'")

    for key in _REQUIRED_MAPPING:
        if key not in data["mapping"]:
            raise ConfigError(f"Missing required key 'mapping.{key}' in {p}")
    if len(data["mapping"]["ur_joint_order"]) != 6:
        raise ConfigError("mapping.ur_joint_order must have 6 joints")

    for key in _REQUIRED_HOME:
        if len(data["home"].get(key, [])) != 6:
            raise ConfigError(f"home.{key} must have 6 values in {p}")

    limits = data["safety"].get("limits", {})
    for joint in UR_JOINT_NAMES:
        if joint not in limits:
            raise ConfigError(f"safety.limits missing joint '{joint}' in {p}")

    return data


def default_config_path() -> str:
    """Package share path to ur_teleop.yaml; '' if the package is not installed."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("ur_teleop"),
                            "config", "ur_teleop.yaml")
    except Exception:
        return ""


def gripper_position_to_value(position_m: float, gripper_type: str = "50mm") -> float:
    """Alicia gripper position (m, 0=open) → command value (0-1000, 0=closed).

    Mirrors alicia_d_driver: value = 1000 - clamp(pos,0,stroke)/stroke * 1000.
    """
    stroke = 0.05 if gripper_type == "100mm" else 0.025
    m = max(0.0, min(stroke, position_m))
    return max(0.0, min(1000.0, 1000.0 - (m / stroke) * 1000.0))


def gripper_value_to_position(value: float, gripper_type: str = "50mm") -> float:
    """Alicia gripper command value (0-1000, 0=closed) → position (m, 0=open)."""
    stroke = 0.05 if gripper_type == "100mm" else 0.025
    v = max(0.0, min(1000.0, value))
    return (stroke * (1000.0 - v)) / 1000.0
