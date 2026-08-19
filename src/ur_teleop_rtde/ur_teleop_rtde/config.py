"""Configuration loading/validation for ur_teleop_rtde.

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

_REQUIRED_TOP = ["mode", "robot", "home", "mapping", "rtde", "teleop", "gripper"]
_REQUIRED_MAPPING = ["alicia_joint_order", "ur_joint_order", "sign", "scale"]
_REQUIRED_HOME = ["master", "slave"]


class ConfigError(ValueError):
    """Raised when the config file is missing/invalid."""


def load_config(path: str | Path) -> dict[str, Any]:
    """Load and validate ur_teleop_rtde.yaml. Raises ConfigError on any problem."""
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
    if len(data["mapping"]["alicia_joint_order"]) != 6:
        raise ConfigError("mapping.alicia_joint_order must have 6 joints")
    if len(data["mapping"]["ur_joint_order"]) != 6:
        raise ConfigError("mapping.ur_joint_order must have 6 joints")
    if len(data["mapping"]["sign"]) != 6:
        raise ConfigError("mapping.sign must have 6 values")
    if len(data["mapping"]["scale"]) != 6:
        raise ConfigError("mapping.scale must have 6 values")

    for key in _REQUIRED_HOME:
        if len(data["home"].get(key, [])) != 6:
            raise ConfigError(f"home.{key} must have 6 values in {p}")

    safety = data["mapping"].get("safety", {})
    limits = safety.get("limits", {})
    for joint in UR_JOINT_NAMES:
        if joint not in limits:
            raise ConfigError(f"mapping.safety.limits missing joint '{joint}' in {p}")

    rtde = data["rtde"]
    if float(rtde.get("servoj_dt", 0.002)) <= 0:
        raise ConfigError("rtde.servoj_dt must be positive")
    if not (100 <= int(rtde.get("servoj_gain", 100)) <= 2000):
        raise ConfigError("rtde.servoj_gain must be in [100, 2000]")
    if not (0.03 <= float(rtde.get("servoj_lookahead", 0.2)) <= 0.2):
        raise ConfigError("rtde.servoj_lookahead must be in [0.03, 0.2]")

    ip = data["robot"].get("robot_ip", "")
    if not isinstance(ip, str) or not ip:
        raise ConfigError("robot.robot_ip must be a non-empty string")

    return data


def default_config_path() -> str:
    """Package share path to ur_teleop_rtde.yaml; '' if the package is not installed."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("ur_teleop_rtde"),
                            "config", "ur_teleop_rtde.yaml")
    except Exception:
        return ""
