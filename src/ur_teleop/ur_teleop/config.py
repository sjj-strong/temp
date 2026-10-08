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

_REQUIRED_TOP = ["mode", "sim", "home", "teleop"]
_REQUIRED_MAPPING = ["alicia_joint_order", "ur_joint_order", "sign", "scale"]
_REQUIRED_HOME = ["master", "slave"]
_SUPPORTED_TELEOP_CONTROLLERS = ("forward_position", "joint_impedance")


class ConfigError(ValueError):
    """Raised when the config file is missing/invalid."""


def load_config(path: str | Path) -> dict[str, Any]:
    """Load and validate ur_teleop.yaml. Raises ConfigError on any problem."""
    data = _read_config(Path(path), set())
    p = Path(path)

    for key in _REQUIRED_TOP:
        if key not in data:
            raise ConfigError(f"Missing required top-level key '{key}' in {p}")

    if data["mode"] not in ("teleop", "record"):
        raise ConfigError(f"mode must be 'teleop' or 'record', got '{data['mode']}'")

    if not isinstance(data.get("debug", False), bool):
        raise ConfigError("debug 必须为布尔值")
    data.setdefault("debug", False)

    from ur_teleop.recorder_config import validate_recorder
    try:
        validate_recorder(data.get("recorder", {}))
    except ValueError as exc:
        raise ConfigError(str(exc)) from exc

    source = data["teleop"].get("control_source", "alicia")
    if source not in ("alicia", "xbot"):
        raise ConfigError("teleop.control_source must be 'alicia' or 'xbot'")
    if not isinstance(data.get('cell', {}).get('ft300_enabled', True), bool):
        raise ConfigError('cell.ft300_enabled 必须为布尔值')
    if not isinstance(data.get('gripper', {}).get('enabled', True), bool):
        raise ConfigError('gripper.enabled 必须为布尔值')
    if source == "alicia":
        controller = data["teleop"].get("controller", "forward_position")
        if controller not in _SUPPORTED_TELEOP_CONTROLLERS:
            raise ConfigError(
                "teleop.controller must be one of "
                f"{', '.join(_SUPPORTED_TELEOP_CONTROLLERS)}, got '{controller}'"
            )
    else:
        if data['teleop'].get('controller', 'cartesian_impedance') != 'cartesian_impedance':
            raise ConfigError('Xbot 的 teleop.controller 必须为 cartesian_impedance')
        controller_file = data.get('xbot', {}).get('controller_config_file')
        if not isinstance(controller_file, str) or not controller_file.strip():
            raise ConfigError('xbot.controller_config_file 必须是非空文件路径')
        controller_path = Path(controller_file).expanduser()
        if not controller_path.is_absolute():
            controller_path = p.parent / controller_path
        controller_path = controller_path.resolve()
        if not controller_path.is_file():
            raise ConfigError(f'笛卡尔阻抗控制器参数文件不存在: {controller_path}')
        from ur_teleop.controller_frame import controller_base_frame
        try:
            controller_base_frame(controller_path)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            raise ConfigError(f'笛卡尔阻抗控制器参数文件无效: {exc}') from exc
        data['xbot']['controller_config_file'] = str(controller_path)
        rec = data.get('recorder', {})
        if rec.get('action_mode', 'abs') not in ('abs', 'rel'):
            raise ConfigError('recorder.action_mode 必须为 abs 或 rel')
        if rec.get('action_space', 'cartesian_pose') != 'cartesian_pose':
            raise ConfigError('Xbot 的 recorder.action_space 必须为 cartesian_pose，不再记录速度')
        for key in ('record_action_gripper', 'record_joint_position', 'record_joint_velocity',
                    'record_joint_effort', 'record_tcp_pose', 'record_wrench'):
            if key in rec and not isinstance(rec[key], bool):
                raise ConfigError(f'recorder.{key} 必须为布尔值')
        for name, camera in rec.get('cameras', {}).items():
            if not isinstance(camera, dict) or not isinstance(camera.get('enabled', True), bool):
                raise ConfigError(f'recorder.cameras.{name}.enabled 必须为布尔值')

    ruckig = data.get("ruckig", {})
    if not isinstance(ruckig, dict):
        raise ConfigError("ruckig must be a mapping")
    if "enabled" in ruckig and not isinstance(ruckig["enabled"], bool):
        raise ConfigError("ruckig.enabled must be a boolean")

    if source == "alicia":
        for key in ("mapping", "safety"):
            if key not in data:
                raise ConfigError(f"Missing required top-level key '{key}' in {p}")
        for key in _REQUIRED_MAPPING:
            if key not in data["mapping"]:
                raise ConfigError(f"Missing required key 'mapping.{key}' in {p}")
        if len(data["mapping"]["ur_joint_order"]) != 6:
            raise ConfigError("mapping.ur_joint_order must have 6 joints")

    home_keys = _REQUIRED_HOME if source == "alicia" else ["slave"]
    for key in home_keys:
        if len(data["home"].get(key, [])) != 6:
            raise ConfigError(f"home.{key} must have 6 values in {p}")

    if source == "alicia":
        limits = data["safety"].get("limits", {})
        for joint in UR_JOINT_NAMES:
            if joint not in limits:
                raise ConfigError(f"safety.limits missing joint '{joint}' in {p}")

    return data


def _read_config(path: Path, visited: set[Path]) -> dict[str, Any]:
    """读取配置，可用相对路径 base_config 继承现有单元设置。"""
    p = path.resolve()
    if p in visited:
        raise ConfigError(f"配置继承出现循环: {p}")
    if not p.exists():
        raise ConfigError(f"Config file not found: {p}")
    data = yaml.safe_load(p.read_text()) or {}
    if not isinstance(data, dict):
        raise ConfigError(f"配置必须是映射: {p}")
    parent_name = data.pop("base_config", None)
    if parent_name is None:
        return data
    if not isinstance(parent_name, str) or not parent_name:
        raise ConfigError("base_config 必须是非空路径")
    parent = _read_config(p.parent / parent_name, visited | {p})
    return _merge_config(parent, data)


def _merge_config(base: dict, override: dict) -> dict:
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge_config(result[key], value)
        else:
            result[key] = value
    return result


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
