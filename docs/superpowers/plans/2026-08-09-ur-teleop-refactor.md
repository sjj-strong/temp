# ur_teleop 功能包重构实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 重构 `/ros2_ws/src/ur_teleop`：两阶段（home → teleop）启动流程、单一配置文件、修复 spec 问题 1–16、补充单元与集成测试，保留"夹爪跟随 / 只保留 real 主臂 / 独立 recorder 进程"的已批准设计。

**Architecture:** 纯 Python ament 包。3 个 launch（cell / home / teleop）+ 3 个可执行节点（home_node 一次性、teleop_node 常驻、data_recorder 常驻 record 模式）。纯逻辑类（config / joint_mapper / offset / gripper_controller / frame_builder / keyboard）无 ROS 依赖可单测；节点仅做编排。

**Tech Stack:** ROS2 Jazzy (rclpy)、lerobot 0.5.1（editable 于 /opt/lerobot_venv）、ur_robot_driver（mock 硬件）、numpy、pytest。

**Spec:** `docs/superpowers/specs/2026-08-09-ur-teleop-refactor-design.md`

## Global Constraints

- 依赖包清单（不得越界）：Alicia-D-ROS2、Universal_Robots_ROS2_Driver、Universal_Robots_ROS2_Description、ur10e_robotiq_ft、ros2_robotiq_gripper、rq_fts_ros2_driver、lerobot。需要清单外功能包必须先向用户询问。
- 配置单一入口：`config/ur_teleop.yaml`，缺失键解析时报错（ConfigError），不用静默默认值。
- 纯逻辑类（joint_mapper / gripper_controller / offset / config / frame_builder / keyboard）不得 import rclpy。
- 节点回调内不得阻塞（禁止 while+time.sleep 于回调、禁止嵌套 spin_until_future_complete 与 spin_once 循环）。
- 节点参数一律 `declare_parameter`（config_file / mode / force_home）；**launch 参数优先，yaml 兜底**。
- 禁止硬编码 `/ros2_ws/src/...` 绝对路径；配置文件路径默认走 `get_package_share_directory("ur_teleop")`。
- 所有 topic/服务/action 名与数值按接口表，不得自行改名。
- 单测跑 `colcon test`；集成测试标 `@pytest.mark.integration`，pytest.ini `addopts = -m "not integration"` 默认排除，`--pytest-args "-m integration"` 启用。
- 每次任务结束必须 git commit，commit message 中文，遵循 `feat:`/`test:`/`refactor:`/`chore:` 前缀。

### 接口表（来自 spec 与探索，任务内直接引用）

| 项 | 值 |
|---|---|
| Alicia 状态 | `/joint_states` `Joint1..Joint6`（rad）+ `Gripper`（m，0=开） |
| Alicia 命令 | `/joint_commands` `JointState`：name=Joint1..6（rad）+ `Gripper`（0–1000 反向，0=闭 1000=开）；velocity 可省 |
| Alicia 力矩 | `/demonstration` Bool：true=零力矩拖拽，false=恢复 |
| UR 前向 pos | `/forward_position_controller/commands` `Float64MultiArray` 6 维，顺序=ur_joint_order |
| UR home 轨迹 | action `/scaled_joint_trajectory_controller/follow_joint_trajectory`（`control_msgs/action/FollowJointTrajectory`） |
| 控制器服务 | `/controller_manager/{switch,list,load}_controller`（`controller_manager_msgs/srv/*`） |
| 夹爪 action | `/robotiq_gripper_controller/gripper_cmd`（`control_msgs/action/ParallelGripperCommand`，goal.command 是 `sensor_msgs/JointState`：name=[robotiq_85_left_knuckle_joint], position=[rad], effort=[N]） |
| 内部协议 | `/teleop/commands` Float64MultiArray 7 维（6 关节+夹爪指令）；`/teleop/status` Bool（ACTIVE=true）；`/teleop/enable` Bool 边沿（recorder→teleop）；`/teleop/e_stop` Bool 订阅 |
| cell sim | include `ur10e.launch.py`：robot_ip、use_mock_hardware=true、launch_dashboard_client=false、initial_joint_controller=scaled_joint_trajectory_controller、activate_joint_controller=true |
| cell real 夹爪 | include `robotiq_description/launch/robotiq_control.launch.py`：com_port / model / launch_rviz=false |
| cell real FT | include `robotiq_ft_sensor_hardware/launch/ft_sensor_standalone.launch.py`：namespace / max_retries / read_rate / **ftdi_id** / frame_id（已核实无 port 参数 → 配置键用 `ftdi_id`，见自检记录） |
| 夹爪换算 | Alicia 状态 m→0/1 信号阈值：close>0.0125、open<0.005；Alicia 命令 m↔0-1000：`1000 - m/stroke*1000`，stroke=0.025(50mm)/0.05(100mm)；夹爪指令 rad：open 0.0 / close 0.79 |
| UR 关节 | `shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint` |
| 关节极限 | 沿用现状 joint_mapping.yaml 的 6 组值（shoulder_pan/lift ±6.283、elbow ±3.142、wrist_1/2/3 ±6.283），clamp_margin 0.1 |

---

## 文件结构

**新建：**
- `config/ur_teleop.yaml` — 单一配置（任务 1）
- `ur_teleop/config.py` — 配置解析/校验 + 夹爪单位换算 + default_config_path（任务 1）
- `ur_teleop/offset.py` — 会话 offset 捕获（任务 3）
- `ur_teleop/keyboard.py` — 非阻塞键盘（任务 5）
- `ur_teleop/home_node.py` — 阶段 1（任务 7）
- `ur_teleop/frame_builder.py` — 录制帧组装纯逻辑（任务 9）
- `launch/cell.launch.py`、`launch/home.launch.py`、`launch/teleop.launch.py`（任务 10）
- `config/rviz/ur_teleop.rviz` — 最小 rviz 配置（任务 10）
- `tests/test_config.py`、`test_joint_mapper.py`、`test_offset.py`、`test_gripper_controller.py`、`test_keyboard.py`、`test_frame_builder.py`、`test_integration.py`、`fake_master.py`、`conftest.py`（各任务）；`pytest.ini`（包根，任务 11）
- `README.md`（任务 12）

**重写（覆盖现有同名文件）：**
- `ur_teleop/joint_mapper.py`（任务 2）、`ur_teleop/gripper_controller.py`（任务 4）、`ur_teleop/controller_switcher.py`（任务 6）、`ur_teleop/teleop_node.py`（任务 8）、`ur_teleop/data_recorder.py`（任务 9）

**删除（任务 12）：**
- `ur_teleop/fake_alicia.py`、`ur_teleop/calibration.py`、`ur_teleop/utils.py`
- `launch/teleop_only.launch.py`、`record.launch.py`、`calibrate.launch.py`、`view.launch.py`、`alicia_display.launch.py`
- `config/joint_mapping.yaml`、`calibration_offset.yaml`、`teleop_params.yaml`、`recorder_params.yaml`、`robotiq_gripper.yaml`
- `scripts/run_teleop.sh`、`docs/`（旧文档，README.md 替代）

**修改：** `package.xml`（任务 1 依赖、任务 12 最终确认）、`setup.py`（任务 7）

---

### Task 1: 配置层（config.py + ur_teleop.yaml）

**Files:**
- Create: `config/ur_teleop.yaml`
- Create: `ur_teleop/config.py`
- Test: `tests/test_config.py`
- Modify: `package.xml`（补 depend：`python3-yaml`、`python3-numpy`、`trajectory_msgs`、`control_msgs`、`controller_manager_msgs`、`sensor_msgs`、`std_msgs`、`geometry_msgs`、`tf2_ros`、`tf2_geometry_msgs`、`cv_bridge`；test_depend 已有 ament_pytest）

**Interfaces:**
- Consumes: 无（第一批）。
- Produces: `load_config(path) -> dict`（校验后返回配置，缺失键抛 `ConfigError`）；`default_config_path() -> str`（package share 路径，异常返回 ""）；`gripper_position_to_value(pos_m, gripper_type="50mm") -> float`；`gripper_value_to_position(value, gripper_type="50mm") -> float`；常量 `UR_JOINT_NAMES`、`ALICIA_JOINT_NAMES`、`GRIPPER_JOINT`、`UR_GRIPPER_JOINT`。后续所有任务经 `load_config` 取配置。

- [ ] **Step 1: 写失败测试** `tests/test_config.py`

```python
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


def test_missing_required_key_raises(tmp_path):
    with pytest.raises(ConfigError, match="mode"):
        load_config(_write(tmp_path, "sim: true\n"))


def test_missing_file_raises(tmp_path):
    with pytest.raises(ConfigError, match="not found"):
        load_config(str(tmp_path / "nope.yaml"))


def test_invalid_mode_raises(tmp_path):
    with pytest.raises(ConfigError, match="mode"):
        load_config(_write(tmp_path, "mode: bogus\nsim: true\nhome: {}\nmapping: {}\n"))


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
```

- [ ] **Step 2: 运行确认失败**

Run: `colcon test --packages-select ur_teleop --event-handlers console_direct+`
Expected: FAIL — `ModuleNotFoundError: ur_teleop.config`

- [ ] **Step 3: 写实现** `ur_teleop/config.py`

```python
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
```

- [ ] **Step 4: 运行确认通过**

Run: `colcon test --packages-select ur_teleop --event-handlers console_direct+`
Expected: PASS（9 tests）

- [ ] **Step 5: 创建 `config/ur_teleop.yaml`**（spec §8 完整配置；`limits` 用现状 joint_mapping.yaml 的 6 组值；`home.move_duration_s`/`verify_duration_s` 与 `recorder.state_threshold_rad` 为 home_node/FrameBuilder 消费的补充键，见自检记录）

```yaml
mode: teleop            # teleop | record —— 配置文件配置模式
sim: true               # cell 端：sim(mock+rviz) / real(真机+夹爪+FT)
cell:
  ur_type: ur10e
  robot_ip: 192.168.1.1
  gripper_port: /dev/ttyUSB1     # real 模式，include robotiq_control 用
  ftdi_id: ""                    # real 模式，include rq_fts 驱动用（launch 参数即 ftdi_id）
  launch_rviz: true
home:
  master: [0.0, -1.2, 0.5, 0, 0, 0]     # 目标 home（用户提前设置，本包不处理含义）
  slave:  [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
  master_gripper_value: 1000
  at_home_tolerance_rad: 0.05
  settle_time_s: 2.0
  settle_motion_threshold_rad: 0.01
  move_timeout_s: 30.0
  move_duration_s: 8.0
  verify_duration_s: 2.0
mapping:
  alicia_joint_order: [Joint1, Joint2, Joint3, Joint4, Joint5, Joint6]
  ur_joint_order: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
  sign: [1, 1, 1, 1, 1, 1]
  scale: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
safety:
  clamp_margin_rad: 0.1
  limits:
    shoulder_pan_joint: [-6.283, 6.283]
    shoulder_lift_joint: [-6.283, 6.283]
    elbow_joint: [-3.142, 3.142]
    wrist_1_joint: [-6.283, 6.283]
    wrist_2_joint: [-6.283, 6.283]
    wrist_3_joint: [-6.283, 6.283]
teleop:
  command_rate_hz: 50
  watchdog_timeout_s: 0.5
  restore_controller_on_exit: true
gripper:
  enabled: false                 # sim 默认 false，real 设 true
  action_server: /robotiq_gripper_controller/gripper_cmd
  close_threshold_m: 0.0125
  open_threshold_m: 0.005
  open_pos_rad: 0.0
  close_pos_rad: 0.79
  max_effort: 50.0
recorder:
  repo_id: my_user/ur_teleop
  root: ""                       # 空 = HF_LEROBOT_HOME
  fps: 50
  robot_type: ur10e_alicia_teleop
  use_videos: true
  ee_pose_source: tf             # tf | topic(/tcp_pose) | none
  ee_pose_topic: /tcp_pose
  ee_pose_parent_frame: base_link
  ee_pose_child_frame: gripper_tcp
  cameras: {}                    # {"wrist": {topic, image_key, height, width}}
  task: teleoperation
  min_frames_per_episode: 2
  state_threshold_rad: 0.4       # robotiq 夹爪 rad → 0/1 state 阈值
```

- [ ] **Step 6: 运行全部单测 + 提交**

Run: `colcon test --packages-select ur_teleop`
Expected: PASS
```bash
git add config/ur_teleop.yaml ur_teleop/config.py tests/test_config.py package.xml
git commit -m "feat: 配置层（load_config 校验 + 夹爪单位换算）与单一 ur_teleop.yaml"
```

---

### Task 2: JointMapper 重写（scale 生效）

**Files:**
- Rewrite: `ur_teleop/joint_mapper.py`
- Test: `tests/test_joint_mapper.py`

**Interfaces:**
- Consumes: `cfg["mapping"]` + `cfg["safety"]`（由任务 8 的 `build_mapping_config(cfg)` 合并为一个 dict 传入）、`cfg["home"]["master"]`。
- Produces: `JointMapper(mapping_config: dict, master_home: list[float], slave_home: list[float])`；方法 `master_to_slave(master_q: list[float]) -> list[float]`、属性 `ur_joint_order`、`alicia_joint_order`、`num_joints`、`get_master_home()`、`get_slave_home()`。构造时校验长度 6；UR 关节不在 `safety.limits` 抛 `ValueError`。公式（位置对齐，alicia_order[i] ↔ ur_order[i]）：`ur_cmd[i] = slave_home[i] + sign[i]*scale[i]*(master_q[i] - master_home[i])`，再 clamp 到 `[limit_min+margin, limit_max-margin]`。

- [ ] **Step 1: 写失败测试** `tests/test_joint_mapper.py`

```python
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
```

- [ ] **Step 2: 运行确认失败**

Run: `colcon test --packages-select ur_teleop`
Expected: FAIL（旧实现签名/键结构不匹配）

- [ ] **Step 3: 重写实现** `ur_teleop/joint_mapper.py`

```python
"""Joint mapping: Alicia-D → UR10e affine transform with safety clamping.

Pure logic — no rclpy imports.
Per joint (positional: alicia_joint_order[i] maps to ur_joint_order[i]):
    ur_cmd[i] = slave_home[i] + sign[i] * scale[i] * (master_q[i] - master_home[i])
"""


class JointMapper:
    """Per-joint sign/scale/offset mapping with clamping to safety limits."""

    def __init__(self, mapping_config: dict, master_home: list[float], slave_home: list[float]):
        self._alicia_order = list(mapping_config["alicia_joint_order"])
        self._ur_order = list(mapping_config["ur_joint_order"])
        self._signs = [float(s) for s in mapping_config["sign"]]
        self._scales = [float(s) for s in mapping_config["scale"]]
        safety = mapping_config.get("safety", {})
        self._limits = dict(safety.get("limits", {}))
        self._margin = float(safety.get("clamp_margin_rad", 0.1))

        if len(self._alicia_order) != 6 or len(self._ur_order) != 6:
            raise ValueError("alicia_joint_order and ur_joint_order must each have 6 joints")
        if len(self._signs) != 6 or len(self._scales) != 6:
            raise ValueError("sign and scale must each have 6 values")
        if len(master_home) != 6 or len(slave_home) != 6:
            raise ValueError("master_home and slave_home must have 6 values")

        self._master_home = [float(v) for v in master_home]
        self._slave_home = [float(v) for v in slave_home]

        for i, ur_joint in enumerate(self._ur_order):
            if ur_joint not in self._limits:
                raise ValueError(f"UR joint '{ur_joint}' missing from safety.limits")

    @property
    def ur_joint_order(self) -> list[str]:
        return list(self._ur_order)

    @property
    def alicia_joint_order(self) -> list[str]:
        return list(self._alicia_order)

    @property
    def num_joints(self) -> int:
        return len(self._ur_order)

    def master_to_slave(self, master_q: list[float]) -> list[float]:
        """Alicia positions (rad, alicia_joint_order) → clamped UR commands (rad, ur_joint_order)."""
        if len(master_q) != len(self._alicia_order):
            raise ValueError(f"Expected {len(self._alicia_order)} master joints, got {len(master_q)}")
        ur_cmd = []
        for i in range(self.num_joints):
            raw = (
                self._slave_home[i]
                + self._signs[i] * self._scales[i] * (master_q[i] - self._master_home[i])
            )
            ur_cmd.append(self._clamp(self._ur_order[i], raw))
        return ur_cmd

    def _clamp(self, joint_name: str, value: float) -> float:
        limits = self._limits.get(joint_name)
        if limits is None or len(limits) < 2:
            return value
        lo = float(limits[0]) + self._margin
        hi = float(limits[1]) - self._margin
        return max(lo, min(hi, value))

    def get_master_home(self) -> list[float]:
        return list(self._master_home)

    def get_slave_home(self) -> list[float]:
        return list(self._slave_home)
```

- [ ] **Step 4: 运行确认通过**

Run: `colcon test --packages-select ur_teleop`
Expected: PASS

- [ ] **Step 5: 提交**

```bash
git add ur_teleop/joint_mapper.py tests/test_joint_mapper.py
git commit -m "feat: 重写 JointMapper（scale 生效、safety.limits 校验、纯逻辑）"
```

---

### Task 3: 会话 offset（offset.py）

**Files:**
- Create: `ur_teleop/offset.py`
- Test: `tests/test_offset.py`

**Interfaces:**
- Consumes: 无。
- Produces: `SessionOffset`：字段 `master_home: list[float] | None`、`slave_home: list[float] | None`；方法 `capture(master_q, slave_q)`（深拷贝存入）；属性 `captured -> bool`。任务 8 在 CAPTURE_OFFSET 状态调用 `capture`，随后用其值构造 JointMapper。

- [ ] **Step 1: 写失败测试** `tests/test_offset.py`

```python
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
```

- [ ] **Step 2: 运行确认失败**

Run: `colcon test --packages-select ur_teleop`
Expected: FAIL — `ModuleNotFoundError: ur_teleop.offset`

- [ ] **Step 3: 写实现** `ur_teleop/offset.py`

```python
"""Session-local offset captured at home.

Pure logic — no rclpy imports. Values are captured once per session after both
arms settle at home, and never persisted (spec §3: offset is session-scoped).
"""

from typing import Optional


class SessionOffset:
    """Holds the actual master/slave positions at home for the current session."""

    def __init__(self):
        self.master_home: Optional[list[float]] = None
        self.slave_home: Optional[list[float]] = None

    @property
    def captured(self) -> bool:
        return self.master_home is not None and self.slave_home is not None

    def capture(self, master_q: list[float], slave_q: list[float]) -> None:
        self.master_home = list(master_q)
        self.slave_home = list(slave_q)
```

- [ ] **Step 4: 运行确认通过**

Run: `colcon test --packages-select ur_teleop`
Expected: PASS

- [ ] **Step 5: 提交**

```bash
git add ur_teleop/offset.py tests/test_offset.py
git commit -m "feat: 会话级 offset 捕获（SessionOffset）"
```

---

### Task 4: GripperController 重写（去死代码）

**Files:**
- Rewrite: `ur_teleop/gripper_controller.py`
- Test: `tests/test_gripper_controller.py`

**Interfaces:**
- Consumes: `cfg["gripper"]`（键：enabled / action_server / open_pos_rad / close_pos_rad / close_threshold_m / open_threshold_m / max_effort）。
- Produces: `GripperController(gripper_config: dict)`；属性 `enabled`、`action_server`、`max_effort`、`open_position`、`close_position`、`current_target -> GripperTarget`；`update(alicia_gripper_m: float) -> GripperTarget`（OPEN/CLOSED 仅在目标变化时返回，死区内返回 UNKNOWN）；`get_knuckle_command(target) -> float`；`get_gripper_command_signal(target) -> float`（0/1）。删除 `mark_sent()`/`_last_sent_target`（死代码）。

- [ ] **Step 1: 写失败测试** `tests/test_gripper_controller.py`

```python
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
```

- [ ] **Step 2: 运行确认失败**

Run: `colcon test --packages-select ur_teleop`
Expected: FAIL — 旧构造键结构与新测试不匹配

- [ ] **Step 3: 重写实现** `ur_teleop/gripper_controller.py`

```python
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
```

- [ ] **Step 4: 运行确认通过**

Run: `colcon test --packages-select ur_teleop`
Expected: PASS

- [ ] **Step 5: 提交**

```bash
git add ur_teleop/gripper_controller.py tests/test_gripper_controller.py
git commit -m "feat: 重写 GripperController（新配置键、删死代码 mark_sent）"
```

---

### Task 5: 非阻塞键盘（keyboard.py）

**Files:**
- Create: `ur_teleop/keyboard.py`
- Test: `tests/test_keyboard.py`

**Interfaces:**
- Consumes: 无。
- Produces: `KeyboardReader(stream=None)`（默认 `sys.stdin`）；`read_key(timeout: float = 0.0) -> str | None`：timeout 内无输入返回 None；`\n`/`\r` 返回 `"enter"`；其他键小写返回；EOF 返回 None。teleop_node 与 data_recorder 共用（任务 8、9）。

- [ ] **Step 1: 写失败测试** `tests/test_keyboard.py`

```python
import os

from ur_teleop.keyboard import KeyboardReader


def test_reads_available_char():
    r, w = os.pipe()
    os.write(w, b"S")
    reader = KeyboardReader(os.fdopen(r, "r"))
    assert reader.read_key(0.2) == "s"      # 小写化
    os.close(w)


def test_enter_key_normalized():
    r, w = os.pipe()
    os.write(w, b"\n")
    reader = KeyboardReader(os.fdopen(r, "r"))
    assert reader.read_key(0.2) == "enter"
    os.close(w)


def test_no_input_returns_none():
    r, w = os.pipe()
    reader = KeyboardReader(os.fdopen(r, "r"))
    assert reader.read_key(0.05) is None
    os.close(w)


def test_eof_returns_none():
    r, w = os.pipe()
    os.close(w)                             # 立即 EOF
    reader = KeyboardReader(os.fdopen(r, "r"))
    assert reader.read_key(0.05) is None
```

- [ ] **Step 2: 运行确认失败**

Run: `colcon test --packages-select ur_teleop`
Expected: FAIL — `ModuleNotFoundError: ur_teleop.keyboard`

- [ ] **Step 3: 写实现** `ur_teleop/keyboard.py`

```python
"""Non-blocking single-key stdin reader.

Pure logic (stdlib only). Nodes pass their stdin; timeout 0 makes read_key
cheap enough to call from a 50 Hz timer callback.
"""

import select
import sys


class KeyboardReader:
    def __init__(self, stream=None):
        self._stream = stream if stream is not None else sys.stdin

    def read_key(self, timeout: float = 0.0) -> str | None:
        """One key within `timeout` seconds, lowercased; 'enter' for newline; None otherwise."""
        try:
            ready, _, _ = select.select([self._stream], [], [], timeout)
        except (ValueError, OSError):
            return None
        if not ready:
            return None
        ch = self._stream.read(1)
        if not ch:
            return None
        if ch in ("\n", "\r"):
            return "enter"
        return ch.lower()
```

- [ ] **Step 4: 运行确认通过**

Run: `colcon test --packages-select ur_teleop`
Expected: PASS

- [ ] **Step 5: 提交**

```bash
git add ur_teleop/keyboard.py tests/test_keyboard.py
git commit -m "feat: 非阻塞键盘读取（KeyboardReader）"
```

---

### Task 6: ControllerSwitcher 异步化（无嵌套 spin）

**Files:**
- Rewrite: `ur_teleop/controller_switcher.py`

**Interfaces:**
- Consumes: `/controller_manager` services（接口表）。
- Produces: `ControllerSwitcher(node, timeout_s=10.0)`；`services_ready() -> bool`（非阻塞，三 client 的 service_is_ready 与）；`wait_for_services(timeout_s=None) -> bool`（阻塞版，供 home_node 一次性主循环用）；`list_controllers() -> Future | None`；`list_result(future) -> dict[str,str]`（static，{name: state}）；`load_controller(name) -> Future | None`；`switch(activate: list[str], deactivate: list[str]) -> Future | None`（STRICT）；`switch_ok(future) -> bool`（static）。**所有方法不调用 spin_until_future_complete / spin_once** —— 调用方（teleop_node 状态机）轮询 future.done()。

- [ ] **Step 1: 重写实现**（无单元测试 —— 依赖真实 controller_manager，由任务 11 集成测试覆盖；本任务验证语法与构建）

`ur_teleop/controller_switcher.py`:

```python
"""Async controller_manager client — never spins the executor (spec §5, 问题 10)."""

import logging

from controller_manager_msgs.srv import (
    SwitchController,
    ListControllers,
    LoadController,
)

logger = logging.getLogger(__name__)


class ControllerSwitcher:
    """Client for /controller_manager services.

    All calls are async (call_async + returned Future); the caller polls
    future.done() from its own state machine — the switcher never blocks.
    """

    def __init__(self, node, timeout_s: float = 10.0):
        self._node = node
        self._timeout_s = timeout_s
        self._switch_cli = node.create_client(SwitchController, "/controller_manager/switch_controller")
        self._list_cli = node.create_client(ListControllers, "/controller_manager/list_controllers")
        self._load_cli = node.create_client(LoadController, "/controller_manager/load_controller")

    def services_ready(self) -> bool:
        return (self._switch_cli.service_is_ready()
                and self._list_cli.service_is_ready()
                and self._load_cli.service_is_ready())

    def wait_for_services(self, timeout_s: float | None = None) -> bool:
        """Blocking wait (main thread / one-shot processes only)."""
        timeout = timeout_s if timeout_s is not None else self._timeout_s
        deadline = self._node.get_clock().now().nanoseconds / 1e9 + timeout
        for cli, name in [
            (self._load_cli, "load_controller"),
            (self._switch_cli, "switch_controller"),
            (self._list_cli, "list_controllers"),
        ]:
            while self._node.get_clock().now().nanoseconds / 1e9 < deadline and self._node.context.ok():
                if cli.wait_for_service(timeout_sec=0.5):
                    break
            else:
                logger.error(f"Timed out waiting for /controller_manager/{name}")
                return False
        return True

    def list_controllers(self):
        """Future resolving to the ListControllers response; None if client not ready."""
        if not self._list_cli.service_is_ready():
            return None
        return self._list_cli.call_async(ListControllers.Request())

    @staticmethod
    def list_result(future) -> dict:
        if future is None or not future.done() or future.result() is None:
            return {}
        return {c.name: c.state for c in future.result().controller}

    def load_controller(self, controller_name: str):
        """Future resolving to a LoadController response (result.ok on success)."""
        if not self._load_cli.service_is_ready():
            return None
        req = LoadController.Request()
        req.name = controller_name
        return self._load_cli.call_async(req)

    def switch(self, activate: list[str], deactivate: list[str]):
        """Strict switch request; returns Future, or None if client not ready."""
        if not self._switch_cli.service_is_ready():
            return None
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.STRICT
        return self._switch_cli.call_async(req)

    @staticmethod
    def switch_ok(future) -> bool:
        return (
            future is not None
            and future.done()
            and future.result() is not None
            and future.result().ok
        )
```

- [ ] **Step 2: 验证语法与构建**

Run: `python3 -c "import ast; ast.parse(open('/ros2_ws/src/ur_teleop/ur_teleop/controller_switcher.py').read())"` 与 `colcon build --packages-select ur_teleop --symlink-install`
Expected: 无输出 / 构建成功（旧调用方 teleop_node 仍引用旧 API，任务 8 一起替换）

- [ ] **Step 3: 提交**

```bash
git add ur_teleop/controller_switcher.py
git commit -m "feat: ControllerSwitcher 异步化（无嵌套 spin，调用方轮询 future）"
```

---

### Task 7: home_node（阶段 1）

**Files:**
- Create: `ur_teleop/home_node.py`
- Modify: `setup.py`（console_scripts：`home_node`、`teleop_node`、`data_recorder`；删 `calibrate`、`fake_alicia`；data_files 指向新 config/launch/rviz）

**Interfaces:**
- Consumes: `load_config`、`default_config_path`、`UR_JOINT_NAMES`/`ALICIA_JOINT_NAMES`（config.py）、Alicia `/joint_commands`、UR trajectory action、`/joint_states`。
- Produces: 一次性流程（spec §7）：等 cell 就绪（`/joint_states` + action server）→ UR home 轨迹（失败打印原因 + 目标/当前对比，非零退出）→ 循环发布 Alicia home 到 `/joint_commands` 直到双臂验证到位（容差 + `verify_duration_s` 持续）→ 打印 READY → 0；超时报告"未到位"非零退出。节点参数：`config_file`（declare_parameter，默认 share 路径）。

- [ ] **Step 1: 写实现** `ur_teleop/home_node.py`

```python
"""Stage 1: move both arms to their configured home joints (one-shot).

Waits for the UR cell (started by home.launch), sends a UR home trajectory
via scaled_joint_trajectory_controller, commands the Alicia to home via
/joint_commands, verifies arrival, prints READY, exits 0/1.

Single-threaded: all waiting is done by polling an executor.spin_once in the
main loop — no background spin thread, no nested spin_until_future_complete.
"""

import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ur_teleop.config import (
    ALICIA_JOINT_NAMES,
    GRIPPER_JOINT,
    UR_JOINT_NAMES,
    default_config_path,
    load_config,
)


class HomeNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("home_node")
        self.declare_parameter("config_file", default_config_path())
        cfg = load_config(self.get_parameter("config_file").value)
        home = cfg["home"]
        self._master_home = list(home["master"])
        self._slave_home = list(home["slave"])
        self._gripper_value = float(home.get("master_gripper_value", 1000.0))
        self._tolerance = float(home.get("at_home_tolerance_rad", 0.05))
        self._move_timeout = float(home.get("move_timeout_s", 30.0))
        self._move_duration = float(home.get("move_duration_s", 8.0))
        self._verify_duration = float(home.get("verify_duration_s", 2.0))

        self._joint_states: JointState | None = None
        self._joint_sub = self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self._cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self._traj_client = ActionClient(
            self, FollowJointTrajectory, "/scaled_joint_trajectory_controller/follow_joint_trajectory"
        )

    def _joint_cb(self, msg: JointState):
        self._joint_states = msg

    def _q(self, names: list[str]) -> list[float] | None:
        js = self._joint_states
        if js is None or not all(n in js.name for n in names):
            return None
        return [js.position[js.name.index(n)] for n in names]

    def slave_q(self) -> list[float] | None:
        return self._q(UR_JOINT_NAMES)

    def master_q(self) -> list[float] | None:
        return self._q(ALICIA_JOINT_NAMES)

    def cell_ready(self) -> bool:
        return self.slave_q() is not None and self._traj_client.server_is_available()

    def publish_alicia_home(self):
        msg = JointState()
        msg.name = ALICIA_JOINT_NAMES + [GRIPPER_JOINT]
        msg.position = self._master_home + [self._gripper_value]   # 0-1000，1000=开
        self._cmd_pub.publish(msg)

    def send_ur_home_trajectory(self, executor) -> bool:
        """Send home trajectory; poll async action via executor; log contrast on failure."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = UR_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = self._slave_home
        pt.time_from_start = Duration(seconds=self._move_duration)
        goal.trajectory.points = [pt]

        self.get_logger().info(f"UR home trajectory -> {self._slave_home}")
        future = self._traj_client.send_goal_async(goal)
        deadline = time.time() + 10.0
        while rclpy.ok() and not future.done() and time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)
        if not future.done() or future.result() is None or not future.result().accepted:
            self._log_home_failure("UR home trajectory rejected/timed out")
            return False
        result_future = future.result().get_result_async()
        deadline = time.time() + self._move_timeout
        while rclpy.ok() and not result_future.done() and time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)
        if result_future.done() and result_future.result() is not None:
            code = result_future.result().result.error_code
            if code == FollowJointTrajectory.Result.SUCCESSFUL:
                return True
            self._log_home_failure(f"UR home trajectory failed, error_code={code}")
            return False
        self._log_home_failure("UR home trajectory timed out")
        return False

    def _log_home_failure(self, reason: str):
        current = self.slave_q()
        self.get_logger().error(
            f"{reason}. 目标={self._slave_home}, 当前={current if current is not None else '无 /joint_states'}. "
            f"请检查机器人与 cell；cell 保持运行。"
        )

    def at_home(self) -> bool:
        m, s = self.master_q(), self.slave_q()
        if m is None or s is None:
            return False
        m_err = max(abs(a - b) for a, b in zip(m, self._master_home))
        s_err = max(abs(a - b) for a, b in zip(s, self._slave_home))
        return m_err <= self._tolerance and s_err <= self._tolerance


def main():
    rclpy.init()
    node = HomeNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    rc = 0
    try:
        node.get_logger().info("等待 UR cell 就绪（/joint_states + trajectory action server）...")
        deadline = time.time() + 30.0
        while rclpy.ok() and not node.cell_ready() and time.time() < deadline:
            executor.spin_once(timeout_sec=0.5)
        if not node.cell_ready():
            node.get_logger().error("cell 未就绪。请先运行 home.launch（含 cell）。")
            rc = 1
        elif not node.send_ur_home_trajectory(executor):
            rc = 1
        else:
            node.get_logger().info(f"Alicia home -> {node._master_home}（夹爪开）")
            deadline = time.time() + node._move_timeout
            ok_verified = False
            while rclpy.ok() and time.time() < deadline:
                executor.spin_once(timeout_sec=0.05)
                node.publish_alicia_home()          # 持续命令，直到到位
                if node.at_home():
                    ok_verified = True
                    break
            if not ok_verified:
                node.get_logger().error(
                    "到位超时（未在 move_timeout 内验证双臂位于 home）。"
                    "可用 teleop.launch force_home:=true 跳过验证。"
                )
                rc = 1
            else:
                node.get_logger().info("=" * 50)
                node.get_logger().info("HOME REACHED — 双臂已到位。现在运行 teleop.launch（阶段 2）。")
                node.get_logger().info("=" * 50)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: 更新 setup.py**

console_scripts 段改为：
```python
"console_scripts": [
    "home_node = ur_teleop.home_node:main",
    "teleop_node = ur_teleop.teleop_node:main",
    "data_recorder = ur_teleop.data_recorder:main",
],
```
data_files：`config/ur_teleop.yaml`、`config/rviz/ur_teleop.rviz`、`launch/*.py`、`resource/ur_teleop`、`package.xml`（删除指向旧 yaml 的条目）。

- [ ] **Step 3: 构建验证**

Run: `colcon build --packages-select ur_teleop --symlink-install`
Expected: 成功（teleop_node/data_recorder 尚为旧实现，入口存在即可）

- [ ] **Step 4: 提交**

```bash
git add ur_teleop/home_node.py setup.py
git commit -m "feat: home_node（UR 轨迹 + Alicia home + 到位验证，executor 轮询无嵌套 spin）"
```

---

### Task 8: teleop_node 状态机重写

**Files:**
- Rewrite: `ur_teleop/teleop_node.py`

**Interfaces:**
- Consumes: `load_config`/`default_config_path`、`JointMapper`、`SessionOffset`、`GripperController`/`GripperTarget`、`ControllerSwitcher`、`KeyboardReader`；topic：`/joint_states`（按关节名分主/从）、`/teleop/enable`、`/teleop/e_stop`；发布：`/forward_position_controller/commands`、`/teleop/commands`（7 维）、`/teleop/status`、`/demonstration`；夹爪 action。
- Produces: 状态机（spec §5）：WAITING_CELL（30 s 超时报错退出）→ VERIFY_HOME（容差，`force_home` 跳过）→ SETTLING（2 s 静止）→ CAPTURE_OFFSET → ARMED（teleop 模式读键盘；record 模式等 enable）→ SWITCHING（list→load→switch 异步链，5 次重试）→ ACTIVE（50 Hz 映射发布 + status=true + demonstration=true + 10 Hz 夹爪 FSM）↕ INACTIVE（watchdog，发当前位置，自动恢复）。e_stop 冻结一切。节点参数：`config_file`、`mode`（""=用 yaml）、`force_home`（false）。

- [ ] **Step 1: 写实现** `ur_teleop/teleop_node.py`

```python
"""Teleop core: state machine home-verify → settle → offset → Enter → mirror.

Single-threaded executor; timers do the work; nothing blocks; the controller
switch is an async chain polled by the state machine (spec §5, 问题 7/10).
"""

import sys
import threading
import time
from enum import Enum, auto

import rclpy
from control_msgs.action import ParallelGripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray

from ur_teleop.config import (
    ALICIA_JOINT_NAMES,
    GRIPPER_JOINT,
    UR_GRIPPER_JOINT,
    UR_JOINT_NAMES,
    default_config_path,
    load_config,
)
from ur_teleop.controller_switcher import ControllerSwitcher
from ur_teleop.gripper_controller import GripperController, GripperTarget
from ur_teleop.joint_mapper import JointMapper
from ur_teleop.keyboard import KeyboardReader
from ur_teleop.offset import SessionOffset


class State(Enum):
    WAITING_CELL = auto()
    VERIFY_HOME = auto()
    SETTLING = auto()
    CAPTURE_OFFSET = auto()
    ARMED = auto()
    SWITCHING = auto()
    ACTIVE = auto()
    INACTIVE = auto()


def build_mapping_config(cfg: dict) -> dict:
    """mapping + safety merged into the shape JointMapper expects."""
    return dict(
        cfg["mapping"],
        safety={
            "clamp_margin_rad": cfg["safety"]["clamp_margin_rad"],
            "limits": cfg["safety"]["limits"],
        },
    )


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")
        self.declare_parameter("config_file", default_config_path())
        self.declare_parameter("mode", "")
        self.declare_parameter("force_home", False)
        cfg = load_config(self.get_parameter("config_file").value)
        if self.get_parameter("force_home").value:
            cfg = dict(cfg, home=dict(cfg["home"], at_home_tolerance_rad=float("inf")))
        self._cfg = cfg
        self._mode = self.get_parameter("mode").value or cfg["mode"]   # launch 参数优先 yaml 兜底
        self._command_rate = float(cfg["teleop"].get("command_rate_hz", 50))
        self._watchdog_timeout = float(cfg["teleop"].get("watchdog_timeout_s", 0.5))
        self._restore_on_exit = bool(cfg["teleop"].get("restore_controller_on_exit", True))
        self._traj_ctrl = "scaled_joint_trajectory_controller"
        self._fwd_ctrl = "forward_position_controller"

        self._state = State.WAITING_CELL
        self._start_time = time.time()
        self._fatal_error = False
        self._e_stop = False
        self._master_engaged = False
        self._last_master_stamp = 0.0
        self._master_q: list[float] | None = None
        self._master_gripper_m = 0.0
        self._slave_q: list[float] | None = None
        self._slave_gripper_rad = 0.0
        self._lock = threading.Lock()

        self._mapper: JointMapper | None = None
        self._offset = SessionOffset()
        self._gripper = GripperController(cfg.get("gripper", {}))
        self._gripper_probed = False
        self._gripper_future = None
        self._switcher = ControllerSwitcher(self)
        self._switch_future = None
        self._switch_phase = ""
        self._switch_attempt = 0
        self._kb = KeyboardReader()

        self._joint_sub = self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self._enable_sub = self.create_subscription(Bool, "/teleop/enable", self._enable_cb, 10)
        self._estop_sub = self.create_subscription(Bool, "/teleop/e_stop", self._estop_cb, 10)
        self._fwd_pub = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
        self._cmd_pub = self.create_publisher(Float64MultiArray, "/teleop/commands", 10)
        self._status_pub = self.create_publisher(Bool, "/teleop/status", 10)
        self._demo_pub = self.create_publisher(Bool, "/demonstration", 10)
        self._gripper_action = (
            ActionClient(self, ParallelGripperCommand, self._gripper.action_server)
            if self._gripper.enabled else None
        )

        self._timer = self.create_timer(1.0 / self._command_rate, self._tick)
        if self._gripper.enabled:
            self._gripper_timer = self.create_timer(
                1.0 / float(cfg.get("gripper", {}).get("fsm_rate_hz", 10.0)), self._gripper_tick
            )
        self._settle_start = 0.0
        self._last_pose: list[float] | None = None

    @property
    def fatal_error(self) -> bool:
        return self._fatal_error

    # ---------- callbacks ----------

    def _joint_cb(self, msg: JointState):
        names = set(msg.name)
        if all(n in names for n in ALICIA_JOINT_NAMES):
            with self._lock:
                self._master_q = [msg.position[msg.name.index(n)] for n in ALICIA_JOINT_NAMES]
                if GRIPPER_JOINT in names:
                    self._master_gripper_m = msg.position[msg.name.index(GRIPPER_JOINT)]
                self._master_engaged = True
                self._last_master_stamp = time.time()
        if all(n in names for n in UR_JOINT_NAMES):
            with self._lock:
                self._slave_q = [msg.position[msg.name.index(n)] for n in UR_JOINT_NAMES]
                if UR_GRIPPER_JOINT in names:
                    self._slave_gripper_rad = msg.position[msg.name.index(UR_GRIPPER_JOINT)]

    def _enable_cb(self, msg: Bool):
        if msg.data and self._state == State.ARMED:
            self.get_logger().info("[teleop] enable 收到 — 开始控制")
            self._begin_switch()
        elif msg.data:
            self.get_logger().info(f"[teleop] enable 收到但状态为 {self._state.name}，忽略")

    def _estop_cb(self, msg: Bool):
        self._e_stop = msg.data
        self.get_logger().warn(f"[teleop] e_stop={'ON' if msg.data else 'OFF'}")

    # ---------- state machine ----------

    def _tick(self):
        if self._e_stop:
            return                                            # 冻结：关节指令与夹爪 FSM 均暂停
        if self._state == State.WAITING_CELL:
            if time.time() - self._start_time > 30.0:
                self._fatal_error = True
                self.get_logger().error(
                    "30 s 内未检测到 cell（/joint_states + controller_manager）。"
                    "请先运行 home.launch。"
                )
                rclpy.try_shutdown()
                return
            with self._lock:
                cell_ok = self._slave_q is not None and self._master_q is not None
            if cell_ok and self._switcher.services_ready():
                self._log_state(State.VERIFY_HOME)
        elif self._state == State.VERIFY_HOME:
            self._verify_home()
        elif self._state == State.SETTLING:
            self._settling()
        elif self._state == State.CAPTURE_OFFSET:
            self._capture_offset()
        elif self._state == State.ARMED:
            self._armed()
        elif self._state == State.SWITCHING:
            self._switching()
        elif self._state == State.ACTIVE:
            self._active()
        elif self._state == State.INACTIVE:
            self._inactive()

    def _log_state(self, new: State):
        self._state = new
        self.get_logger().info(f"[teleop] 状态: {new.name}")

    def _verify_home(self):
        home = self._cfg["home"]
        tol = float(home.get("at_home_tolerance_rad", 0.05))
        with self._lock:
            m_err = max(abs(a - b) for a, b in zip(self._master_q, home["master"]))
            s_err = max(abs(a - b) for a, b in zip(self._slave_q, home["slave"]))
        if m_err <= tol and s_err <= tol:
            self._settle_start = time.time()
            self._last_pose = None
            self._log_state(State.SETTLING)
        else:
            self.get_logger().warn(
                f"[teleop] 双臂不在 home（master err={m_err:.3f} rad, slave err={s_err:.3f} rad），"
                f"请先运行 home.launch；确认已到位可用 force_home:=true 跳过"
            )

    def _settling(self):
        settle = float(self._cfg["home"].get("settle_time_s", 2.0))
        thresh = float(self._cfg["home"].get("settle_motion_threshold_rad", 0.01))
        with self._lock:
            pose = list(self._master_q) + list(self._slave_q)
        if self._last_pose is None:
            self._last_pose = pose
            return
        motion = max(abs(a - b) for a, b in zip(pose, self._last_pose))
        self._last_pose = pose
        if motion > thresh:
            self._settle_start = time.time()
        elif time.time() - self._settle_start >= settle:
            self._log_state(State.CAPTURE_OFFSET)

    def _capture_offset(self):
        with self._lock:
            self._offset.capture(list(self._master_q), list(self._slave_q))
        self._mapper = JointMapper(
            build_mapping_config(self._cfg), self._offset.master_home, self._offset.slave_home
        )
        self.get_logger().info(
            f"[teleop] offset 捕获完成 master={self._offset.master_home} slave={self._offset.slave_home}"
        )
        if self._mode == "record":
            self.get_logger().info("[teleop] 等待 recorder 的 Enter（/teleop/enable）...")
        else:
            self.get_logger().info("[teleop] 按 Enter 开始控制")
        self._log_state(State.ARMED)

    def _armed(self):
        if self._mode == "teleop" and self._kb.read_key(0.0) == "enter":
            self.get_logger().info("[teleop] Enter 按下 — 开始")
            self._begin_switch()

    def _begin_switch(self):
        self._switch_attempt = 0
        self._switch_phase = "list"
        self._switch_future = self._switcher.list_controllers()
        self._log_state(State.SWITCHING)

    def _switching(self):
        fut = self._switch_future
        if fut is None:
            self.get_logger().error("controller_manager 服务未就绪，无法切换")
            self._log_state(State.ARMED)
            return
        if not fut.done():
            return
        if self._switch_phase == "list":
            controllers = ControllerSwitcher.list_result(fut)
            if self._fwd_ctrl not in controllers:
                self._switch_phase = "load"
                self._switch_future = self._switcher.load_controller(self._fwd_ctrl)
            else:
                self._switch_phase = "switch"
                self._switch_future = self._switcher.switch([self._fwd_ctrl], [self._traj_ctrl])
        elif self._switch_phase == "load":
            if fut.result() is not None and fut.result().ok:
                self._switch_phase = "switch"
                self._switch_future = self._switcher.switch([self._fwd_ctrl], [self._traj_ctrl])
            else:
                self.get_logger().error("加载 forward_position_controller 失败")
                self._log_state(State.ARMED)
        elif self._switch_phase == "switch":
            if ControllerSwitcher.switch_ok(fut):
                self.get_logger().info("[teleop] 控制器切换完成 → ACTIVE")
                self._publish_demo(True)
                self._publish_status(True)
                self._log_state(State.ACTIVE)
            else:
                self._switch_attempt += 1
                if self._switch_attempt >= 5:
                    self.get_logger().error("控制器切换 5 次失败，回到 ARMED；检查 controller_manager")
                    self._log_state(State.ARMED)
                else:
                    self.get_logger().warn(f"[teleop] 切换失败（第 {self._switch_attempt} 次），重试...")
                    self._switch_future = self._switcher.switch([self._fwd_ctrl], [self._traj_ctrl])

    def _active(self):
        if time.time() - self._last_master_stamp > self._watchdog_timeout:
            self.get_logger().warn("[teleop] 主臂数据超时 → INACTIVE（改发当前位置）")
            self._publish_status(False)
            self._log_state(State.INACTIVE)
            return
        with self._lock:
            cmd = self._mapper.master_to_slave(self._master_q)
        self._publish_commands(cmd)

    def _inactive(self):
        with self._lock:
            if time.time() - self._last_master_stamp <= self._watchdog_timeout:
                self.get_logger().info("[teleop] 主臂恢复 → ACTIVE")
                self._publish_status(True)
                self._log_state(State.ACTIVE)
                return
            hold = list(self._slave_q) if self._slave_q else [0.0] * 6
        self._publish_commands(hold)                        # 发当前位置避免跳变

    # ---------- gripper FSM ----------

    def _gripper_tick(self):
        if self._e_stop or not self._gripper.enabled:
            return
        if not self._gripper_probed:
            self._gripper_probed = True
            if self._gripper_action is None or not self._gripper_action.server_is_available():
                self.get_logger().warn("[teleop] 夹爪 action server 不存在，禁用夹爪 FSM")
                self._gripper.enabled = False
                return
        if self._gripper_future is not None and not self._gripper_future.done():
            return                                          # 上一个 goal 未完成，跳过本 tick
        with self._lock:
            target = self._gripper.update(self._master_gripper_m)
        if target != GripperTarget.UNKNOWN:
            self._send_gripper_goal(target)

    def _send_gripper_goal(self, target: GripperTarget):
        goal = ParallelGripperCommand.Goal()
        goal.command.name = [UR_GRIPPER_JOINT]
        goal.command.position = [self._gripper.get_knuckle_command(target)]
        goal.command.effort = [self._gripper.max_effort]
        self._gripper_future = self._gripper_action.send_goal_async(goal)

    # ---------- helpers ----------

    def _publish_commands(self, cmd: list[float]):
        fwd = Float64MultiArray()
        fwd.data = list(cmd)
        self._fwd_pub.publish(fwd)
        tcmd = Float64MultiArray()
        tcmd.data = list(cmd) + [self._gripper.get_gripper_command_signal(self._gripper.current_target)]
        self._cmd_pub.publish(tcmd)

    def _publish_status(self, active: bool):
        self._status_pub.publish(Bool(data=active))

    def _publish_demo(self, on: bool):
        self._demo_pub.publish(Bool(data=on))

    def shutdown(self):
        """Ctrl-C 退出流程：恢复力矩 → 切回 trajectory controller（spec §5）。"""
        self._publish_demo(False)
        if self._restore_on_exit and self._state in (State.ACTIVE, State.INACTIVE, State.SWITCHING):
            fut = self._switcher.switch([self._traj_ctrl], [self._fwd_ctrl])
            if fut is not None:
                deadline = time.time() + 5.0
                while not ControllerSwitcher.switch_ok(fut) and time.time() < deadline:
                    time.sleep(0.1)


def main():
    rclpy.init()
    node = TeleopNode()
    node.get_logger().info("=" * 60)
    node.get_logger().info(f"ur_teleop 就绪 — mode={node._mode}, sim={node._cfg['sim']}")
    node.get_logger().info("  等待双臂到位 → 静止 → offset → Enter 开始控制")
    node.get_logger().info("=" * 60)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if node.fatal_error else 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: 构建验证**

Run: `colcon build --packages-select ur_teleop --symlink-install && colcon test --packages-select ur_teleop`
Expected: 构建成功、既有单测全绿

- [ ] **Step 3: 提交**

```bash
git add ur_teleop/teleop_node.py
git commit -m "feat: teleop_node 状态机重写（无阻塞、异步切换带重试、offset 会话化、e_stop 冻结）"
```

---

### Task 9: data_recorder 重写（FrameBuilder + enable 协议）

**Files:**
- Create: `ur_teleop/frame_builder.py`
- Rewrite: `ur_teleop/data_recorder.py`
- Test: `tests/test_frame_builder.py`

**Interfaces:**
- Consumes: `cfg["recorder"]`、`cfg["gripper"]`；订阅 `/joint_states`（UR 端过滤）、`/teleop/commands`、相机、`/tcp_pose`（topic 源）；TF `base_link → gripper_tcp`；发布 `/teleop/enable`。
- Produces: `FrameBuilder(recorder_config: dict, gripper_config: dict)`（纯逻辑）：`features() -> (features, state_names, action_names)`、`build(ur_joints, ee_pose, gripper_state_rad, teleop_cmd) -> dict | None`（缺关节/命令返回 None；缺 ee 填 NaN；夹爪 state 由 rad 按 `state_threshold_rad` 变 0/1）。`DataRecorderNode`：键盘 Enter（首次发 enable + 开始 episode）/S/D/Q；50 Hz 主循环 `_record_frame`；节点参数 `config_file`。

- [ ] **Step 1: 写失败测试** `tests/test_frame_builder.py`

```python
import math

import numpy as np
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
```

- [ ] **Step 2: 运行确认失败**

Run: `colcon test --packages-select ur_teleop`
Expected: FAIL — `ModuleNotFoundError: ur_teleop.frame_builder`

- [ ] **Step 3: 写实现** `ur_teleop/frame_builder.py`

```python
"""LeRobot frame assembly — pure logic, no rclpy imports (spec §7).

State = 6 UR joints + 7 EE pose + 1 gripper_state (0/1). Missing EE pose →
NaN segment (never a zero placeholder). Action = 6 joint commands + 1 gripper
command signal, taken from /teleop/commands (7 dims).
"""

import numpy as np

from ur_teleop.config import UR_JOINT_NAMES


class FrameBuilder:
    """Builds observation.state + action numpy arrays for LeRobotDataset.add_frame."""

    EE_NAMES = ["ee_x", "ee_y", "ee_z", "ee_qx", "ee_qy", "ee_qz", "ee_qw"]

    def __init__(self, recorder_config: dict, gripper_config: dict):
        self._rec = recorder_config
        self._gripper = gripper_config
        self._state_threshold = float(recorder_config.get("state_threshold_rad", 0.4))
        self._task = recorder_config.get("task", "teleoperation")
        self._use_videos = bool(recorder_config.get("use_videos", True))

    def features(self):
        features = {}
        state_names, action_names = [], []
        if self._rec.get("record_ur_joints", True):
            state_names.extend(UR_JOINT_NAMES)
        if self._rec.get("record_ur_ee_pose", True):
            state_names.extend(self.EE_NAMES)
        if self._rec.get("record_ur_gripper", True):
            state_names.append("gripper_state")
        if state_names:
            features["observation.state"] = {
                "dtype": "float32", "shape": (len(state_names),), "names": state_names,
            }
        if self._rec.get("record_action_joints", True):
            action_names.extend([f"cmd_{n}" for n in UR_JOINT_NAMES])
        if self._rec.get("record_action_gripper", True):
            action_names.append("cmd_gripper")
        if action_names:
            features["action"] = {
                "dtype": "float32", "shape": (len(action_names),), "names": action_names,
            }
        for cam_name, cam_cfg in self._rec.get("cameras", {}).items():
            key = f"observation.images.{cam_cfg.get('image_key', cam_name)}"
            features[key] = {
                "dtype": "video" if self._use_videos else "image",
                "shape": (int(cam_cfg.get("height", 480)), int(cam_cfg.get("width", 640)), 3),
                "names": ["height", "width", "channels"],
            }
        return features, state_names, action_names

    def build(self, ur_joints, ee_pose, gripper_state_rad, teleop_cmd):
        """None when essential data missing; ee_pose None → NaN segment."""
        if ur_joints is None or teleop_cmd is None:
            return None
        state_parts = []
        if self._rec.get("record_ur_joints", True):
            state_parts.extend(ur_joints[:6])
        if self._rec.get("record_ur_ee_pose", True):
            state_parts.extend(ee_pose if ee_pose is not None else [np.nan] * 7)
        if self._rec.get("record_ur_gripper", True):
            state_parts.append(1.0 if gripper_state_rad > self._state_threshold else 0.0)
        action_parts = []
        if self._rec.get("record_action_joints", True):
            action_parts.extend(teleop_cmd[:6])
        if self._rec.get("record_action_gripper", True):
            action_parts.append(float(teleop_cmd[6]) if len(teleop_cmd) > 6 else 0.0)
        return {
            "observation.state": np.array(state_parts, dtype=np.float32),
            "action": np.array(action_parts, dtype=np.float32),
            "task": self._task,
        }
```

- [ ] **Step 4: 运行确认通过**

Run: `colcon test --packages-select ur_teleop`
Expected: PASS

- [ ] **Step 5: 重写 `ur_teleop/data_recorder.py`**（键盘归属 + enable 协议 + FrameBuilder；主循环沿用现有 spin 线程 + 1/fps 节奏）

```python
"""LeRobot data recorder (record mode). Owns the keyboard; the first Enter
starts episode 1 AND sends /teleop/enable so teleop_node begins control
(spec §6). Keys: Enter=开始 S=保存并结束 D=丢弃并重置 Q=退出并 finalize.
"""

import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image as RosImage
from std_msgs.msg import Bool, Float64MultiArray

try:
    from lerobot.datasets import LeRobotDataset
except ImportError:
    LeRobotDataset = None

from ur_teleop.config import UR_GRIPPER_JOINT, UR_JOINT_NAMES, default_config_path, load_config
from ur_teleop.frame_builder import FrameBuilder
from ur_teleop.keyboard import KeyboardReader


class DataRecorderNode(Node):
    def __init__(self):
        super().__init__("data_recorder")
        if LeRobotDataset is None:
            raise RuntimeError("LeRobot 未安装：source /opt/lerobot_venv/bin/activate")
        self.declare_parameter("config_file", default_config_path())
        cfg = load_config(self.get_parameter("config_file").value)
        self._rec = cfg.get("recorder", {})
        self._fps = int(self._rec.get("fps", 50))
        self._min_frames = int(self._rec.get("min_frames_per_episode", 2))
        self._cameras = self._rec.get("cameras", {})
        self._builder = FrameBuilder(self._rec, cfg.get("gripper", {}))

        self._lock = threading.Lock()
        self._ur_joints = None
        self._ur_gripper_rad = 0.0
        self._teleop_cmd = None
        self._ur_ee_pose = None
        self._camera_frames = {}
        self._enable_sent = False
        self._ee_warned = False

        self._features, _, _ = self._builder.features()
        self._joint_sub = self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self._cmd_sub = self.create_subscription(Float64MultiArray, "/teleop/commands", self._cmd_cb, 10)
        self._enable_pub = self.create_publisher(Bool, "/teleop/enable", 10)

        self._ee_source = self._rec.get("ee_pose_source", "tf")
        if self._ee_source == "tf":
            from tf2_ros.buffer import Buffer
            from tf2_ros.transform_listener import TransformListener
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        elif self._ee_source == "topic":
            from geometry_msgs.msg import PoseStamped
            self.create_subscription(PoseStamped, self._rec.get("ee_pose_topic", "/tcp_pose"),
                                     self._tcp_cb, 10)

        for cam_name, cam_cfg in self._cameras.items():
            topic = cam_cfg.get("topic", "")
            if topic:
                self.create_subscription(RosImage, topic,
                                         lambda msg, cn=cam_name: self._camera_cb(cn, msg), 10)

        self._dataset = None
        self._episode_count = 0
        self._recording = False
        self._frame_count = 0
        self._kb = KeyboardReader()

    # ---------- callbacks ----------

    def _joint_cb(self, msg: JointState):
        names = set(msg.name)
        if not all(n in names for n in UR_JOINT_NAMES):
            return
        with self._lock:
            self._ur_joints = [msg.position[msg.name.index(n)] for n in UR_JOINT_NAMES]
            if UR_GRIPPER_JOINT in names:
                self._ur_gripper_rad = msg.position[msg.name.index(UR_GRIPPER_JOINT)]

    def _cmd_cb(self, msg: Float64MultiArray):
        with self._lock:
            self._teleop_cmd = list(msg.data) if msg.data else None

    def _tcp_cb(self, msg):
        p = msg.pose
        with self._lock:
            self._ur_ee_pose = [p.position.x, p.position.y, p.position.z,
                                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]

    def _camera_cb(self, cam_name: str, msg: RosImage):
        try:
            from cv_bridge import CvBridge
            img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="rgb8")
            with self._lock:
                self._camera_frames[cam_name] = img
        except Exception:
            pass

    def _get_ee_pose(self):
        """7 维 [x,y,z,qx,qy,qz,qw] 或 None（source=none 恒 None → NaN 段）。"""
        if self._ee_source == "topic":
            with self._lock:
                return list(self._ur_ee_pose) if self._ur_ee_pose else None
        if self._ee_source == "none":
            return None
        try:
            import rclpy.time
            t = self._tf_buffer.lookup_transform(
                self._rec.get("ee_pose_parent_frame", "base_link"),
                self._rec.get("ee_pose_child_frame", "gripper_tcp"),
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            tr, rot = t.transform.translation, t.transform.rotation
            return [tr.x, tr.y, tr.z, rot.x, rot.y, rot.z, rot.w]
        except Exception:
            return None

    # ---------- dataset ----------

    def _init_dataset(self):
        if self._dataset is not None:
            return
        repo_id = self._rec.get("repo_id", "my_user/ur_teleop")
        root = Path(self._rec["root"]) if self._rec.get("root") else None
        kwargs = dict(
            repo_id=repo_id, fps=self._fps, features=self._features, root=root,
            robot_type=self._rec.get("robot_type", "ur10e_alicia_teleop"),
            use_videos=self._rec.get("use_videos", True),
            image_writer_processes=self._rec.get("image_writer_processes", 0),
            image_writer_threads=self._rec.get("image_writer_threads", 2),
        )
        try:
            self._dataset = LeRobotDataset.create(**kwargs)
            self.get_logger().info(f"创建数据集: {repo_id}")
        except FileExistsError:
            import datetime
            new_id = f"{repo_id}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
            kwargs["repo_id"] = new_id
            self._dataset = LeRobotDataset.create(**kwargs)
            self.get_logger().warn(f"数据集已存在，新建带时间戳: {new_id}")

    # ---------- episode control ----------

    def _start_episode(self):
        if self._recording:
            return
        self._init_dataset()
        if not self._enable_sent:
            self._enable_pub.publish(Bool(data=True))
            self._enable_sent = True
            self.get_logger().info("已发送 /teleop/enable → teleop_node 开始控制")
        self._recording = True
        self._frame_count = 0
        self.get_logger().info(f"Episode {self._episode_count + 1} 开始（S=保存 D=丢弃 Q=退出）")

    def _save_episode(self):
        if not self._recording:
            return
        if self._frame_count < self._min_frames:
            self.get_logger().warn(f"少于 {self._min_frames} 帧，自动丢弃")
            self._dataset.clear_episode_buffer()
        else:
            self._dataset.save_episode()
            self._episode_count += 1
            self.get_logger().info(f"Episode {self._episode_count} 已保存（{self._frame_count} 帧）")
        self._recording = False

    def _discard_episode(self):
        if not self._recording:
            return
        self._dataset.clear_episode_buffer()
        self.get_logger().info(f"Episode 已丢弃（{self._frame_count} 帧）")
        self._recording = False

    def _record_frame(self):
        with self._lock:
            ur = list(self._ur_joints) if self._ur_joints else None
            cmd = list(self._teleop_cmd) if self._teleop_cmd else None
            gripper_rad = self._ur_gripper_rad
            cameras = dict(self._camera_frames)
        ee = self._get_ee_pose()
        if ee is None and not self._ee_warned and self._ee_source != "none":
            self._ee_warned = True
            self.get_logger().warn("EE 位姿查询失败，该段以 NaN 记录（仅警告一次）")
        frame = self._builder.build(ur, ee, gripper_rad, cmd)
        if frame is None:
            return
        for cam_name, cam_cfg in self._cameras.items():
            img = cameras.get(cam_name)
            if img is not None:
                frame[f"observation.images.{cam_cfg.get('image_key', cam_name)}"] = img
        try:
            self._dataset.add_frame(frame)
            self._frame_count += 1
        except Exception as e:
            self.get_logger().error(f"add_frame 失败: {e}")

    def finalize(self):
        if self._recording:
            self._save_episode()
        if self._dataset is not None:
            self._dataset.finalize()
            self.get_logger().info(f"数据集 finalize 完成（{self._episode_count} episodes）")


def main():
    rclpy.init()
    node = DataRecorderNode()
    node.get_logger().info("=" * 60)
    node.get_logger().info("Data Recorder 就绪 — Enter=开始 S=保存 D=丢弃 Q=退出")
    node.get_logger().info("=" * 60)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    period = 1.0 / node._fps
    try:
        while rclpy.ok():
            t0 = time.time()
            if node._recording:
                node._record_frame()
            key = node._kb.read_key(0.0)
            if key == "enter":
                node._start_episode()
            elif key == "s":
                node._save_episode()
            elif key == "d":
                node._discard_episode()
            elif key == "q":
                node.get_logger().info("Q 按下，退出")
                break
            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 6: 构建 + 单测 + 提交**

Run: `colcon build --packages-select ur_teleop --symlink-install && colcon test --packages-select ur_teleop`
Expected: 全绿
```bash
git add ur_teleop/frame_builder.py ur_teleop/data_recorder.py tests/test_frame_builder.py
git commit -m "feat: data_recorder 重写（FrameBuilder 纯逻辑、enable 协议、夹爪真实状态、EE 缺失 NaN）"
```

---

### Task 10: launch 文件（cell / home / teleop）+ rviz 配置

**Files:**
- Create: `launch/cell.launch.py`、`launch/home.launch.py`、`launch/teleop.launch.py`
- Create: `config/rviz/ur_teleop.rviz`

**Interfaces:**
- Consumes: 接口表的 launch 入口（ur10e.launch.py、robotiq_control.launch.py、ft_sensor_standalone.launch.py）。
- Produces: 三 launch。**launch 参数优先，yaml 兜底**（spec §8）：`generate_launch_description()` 内先读 `config_file` 的 yaml 得默认值，`DeclareLaunchArgument` 显式参数覆盖。条件求值一律用 `PythonExpression` 显式比较（避免 "false" 字符串真值陷阱）。

- [ ] **Step 1: 写 `launch/cell.launch.py`**

```python
"""UR cell: sim (mock + rviz) or real (hardware + gripper + FT300).

Persistent — started by home.launch and shared with teleop.launch (spec §4.1).
Launch args win over ur_teleop.yaml defaults.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _yaml_default(config_file: str, *path: str, fallback: str):
    try:
        with open(config_file) as f:
            data = yaml.safe_load(f)
        for p in path:
            data = data[p]
        return str(data)
    except Exception:
        return fallback


def generate_launch_description():
    pkg_share = get_package_share_directory("ur_teleop")
    config_file = os.path.join(pkg_share, "config", "ur_teleop.yaml")

    sim_default = _yaml_default(config_file, "sim", fallback="true")
    ip_default = _yaml_default(config_file, "cell", "robot_ip", fallback="0.0.0.0")
    grip_default = _yaml_default(config_file, "cell", "gripper_port", fallback="/dev/ttyUSB1")
    ftdi_default = _yaml_default(config_file, "cell", "ftdi_id", fallback="")
    rviz_default = _yaml_default(config_file, "cell", "launch_rviz", fallback="true")

    sim = LaunchConfiguration("sim")
    is_sim = PythonExpression(["'", sim, "' == 'true'"])          # "false" 字符串真值陷阱防护

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config_file),
        DeclareLaunchArgument("sim", default_value=sim_default),
        DeclareLaunchArgument("robot_ip", default_value=ip_default),
        DeclareLaunchArgument("gripper_port", default_value=grip_default),
        DeclareLaunchArgument("ftdi_id", default_value=ftdi_default),
        DeclareLaunchArgument("launch_rviz", default_value=rviz_default),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ur_robot_driver"),
                             "launch", "ur10e.launch.py")
            ),
            launch_arguments={
                "robot_ip": LaunchConfiguration("robot_ip"),
                "use_mock_hardware": sim,
                "mock_sensor_commands": "false",
                "launch_dashboard_client": "false",
                "initial_joint_controller": "scaled_joint_trajectory_controller",
                "activate_joint_controller": "true",
            }.items(),
        ),
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", os.path.join(pkg_share, "config", "rviz", "ur_teleop.rviz")],
            condition=IfCondition(PythonExpression(
                ["'", sim, "' == 'true' and '", LaunchConfiguration("launch_rviz"), "' == 'true'"])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("robotiq_description"),
                             "launch", "robotiq_control.launch.py")
            ),
            condition=UnlessCondition(is_sim),
            launch_arguments={
                "com_port": LaunchConfiguration("gripper_port"),
                "launch_rviz": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("robotiq_ft_sensor_hardware"),
                             "launch", "ft_sensor_standalone.launch.py")
            ),
            condition=UnlessCondition(is_sim),
            launch_arguments={
                "ftdi_id": LaunchConfiguration("ftdi_id"),
                "frame_id": "robotiq_ft_frame_id",
            }.items(),
        ),
    ])
```

- [ ] **Step 2: 写 `launch/home.launch.py`**

```python
"""Stage 1: start the UR cell (persistent) and move both arms to home."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _yaml_default(config_file: str, *path: str, fallback: str):
    try:
        with open(config_file) as f:
            data = yaml.safe_load(f)
        for p in path:
            data = data[p]
        return str(data)
    except Exception:
        return fallback


def generate_launch_description():
    pkg_share = get_package_share_directory("ur_teleop")
    config_file = os.path.join(pkg_share, "config", "ur_teleop.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config_file),
        DeclareLaunchArgument("sim",
                              default_value=_yaml_default(config_file, "sim", fallback="true")),
        DeclareLaunchArgument("robot_ip",
                              default_value=_yaml_default(config_file, "cell", "robot_ip",
                                                         fallback="0.0.0.0")),
        DeclareLaunchArgument("gripper_port",
                              default_value=_yaml_default(config_file, "cell", "gripper_port",
                                                         fallback="/dev/ttyUSB1")),
        DeclareLaunchArgument("ftdi_id",
                              default_value=_yaml_default(config_file, "cell", "ftdi_id",
                                                         fallback="")),
        DeclareLaunchArgument("launch_rviz",
                              default_value=_yaml_default(config_file, "cell", "launch_rviz",
                                                         fallback="true")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "cell.launch.py")
            ),
            launch_arguments={
                "config_file": LaunchConfiguration("config_file"),
                "sim": LaunchConfiguration("sim"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "gripper_port": LaunchConfiguration("gripper_port"),
                "ftdi_id": LaunchConfiguration("ftdi_id"),
                "launch_rviz": LaunchConfiguration("launch_rviz"),
            }.items(),
        ),
        Node(
            package="ur_teleop", executable="home_node",
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
        ),
    ])
```

- [ ] **Step 3: 写 `launch/teleop.launch.py`**（mode 通过参数 dict 传给 teleop_node，避免解析期字符串拼接）

```python
"""Stage 2: teleop core (+ data_recorder when mode=record).

Connects to the cell already running from home.launch (spec §4.1).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ur_teleop")
    config_file = os.path.join(pkg_share, "config", "ur_teleop.yaml")
    try:
        with open(config_file) as f:
            mode_default = yaml.safe_load(f).get("mode", "teleop")
    except Exception:
        mode_default = "teleop"

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config_file),
        DeclareLaunchArgument("mode", default_value=mode_default,
                              choices=["teleop", "record"]),
        DeclareLaunchArgument("force_home", default_value="false"),
        Node(
            package="ur_teleop", executable="teleop_node",
            parameters=[
                {"config_file": LaunchConfiguration("config_file"),
                 "mode": LaunchConfiguration("mode"),
                 "force_home": LaunchConfiguration("force_home")},
            ],
        ),
        Node(
            package="ur_teleop", executable="data_recorder",
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration("mode"), "' == 'record'"])),
        ),
    ])
```

- [ ] **Step 4: 写 `config/rviz/ur_teleop.rviz`**（最小配置：Grid + TF + RobotModel，固定坐标系 base_link）

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Plane Cell Count: 10
    - Class: rviz_default_plugins/TF
      Name: TF
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Description Topic:
        Value: /robot_description
  Fixed Frame:
    Value: base_link
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.5
      Pitch: 0.6
      Yaw: 0.8
```

- [ ] **Step 5: 构建验证**

Run: `colcon build --packages-select ur_teleop --symlink-install`
Expected: 成功；`ros2 launch ur_teleop cell.launch.py sim:=true launch_rviz:=false` 可启动（无显示环境则 rviz 报错属预期，本命令不带 rviz）

- [ ] **Step 6: 提交**

```bash
git add launch/cell.launch.py launch/home.launch.py launch/teleop.launch.py config/rviz/ur_teleop.rviz
git commit -m "feat: cell/home/teleop 三 launch 与 rviz 配置（launch 参数优先 yaml 兜底）"
```

---

### Task 11: 集成测试（fake_master + 冒烟断言）

**Files:**
- Create: `tests/fake_master.py`、`tests/conftest.py`、`pytest.ini`（包根）、`tests/test_integration.py`

**Interfaces:**
- Consumes: 任务 1–10 全部产物；`ros2 launch`/`ros2 run`/`ros2 topic` 子进程。
- Produces: `@pytest.mark.integration` 测试（pytest.ini `addopts = -m "not integration"` 排除，`--pytest-args "-m integration"` 启用）；`fake_master`：前 12 s 发布 home（zeros），之后正弦波；`--off-home` 变体恒发 [0.5]*6。

- [ ] **Step 1: 写 `tests/fake_master.py`**

```python
"""Test-only fake Alicia master (spec §10) — NOT installed, lives in tests/.

Publishes /joint_states: home positions (zeros) for 12 s, then sine waves;
--off-home publishes [0.5]*6 forever (VERIFY_HOME rejection tests).
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
AMPL = [0.3, 0.2, 0.4, 0.3, 0.5, 0.2]
FREQ = [0.5, 0.4, 0.6, 0.7, 0.8, 0.9]


class FakeMaster(Node):
    def __init__(self, off_home: bool):
        super().__init__("fake_master")
        self._off_home = off_home
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self._t0 = time.time()
        self.create_timer(0.02, self._tick)  # 50 Hz

    def _tick(self):
        t = time.time() - self._t0
        msg = JointState()
        msg.name = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6", "Gripper"]
        if self._off_home or t < 12.0:
            q = HOME if not self._off_home else [0.5] * 6
        else:
            q = [HOME[i] + AMPL[i] * math.sin(2 * math.pi * FREQ[i] * (t - 12.0))
                 for i in range(6)]
        gripper_m = 0.025 if int(t / 5) % 2 == 1 else 0.0
        msg.position = list(q) + [gripper_m]
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = FakeMaster(off_home="--off-home" in sys.argv)
    rclpy.spin(node)
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 写 `tests/conftest.py` 与 `pytest.ini`（包根）**

```python
# conftest.py — 无全局 fixture；仅确保 tests/ 在 sys.path（fake_master 作为脚本运行，不需要 import）
```
```ini
# pytest.ini（位于 /ros2_ws/src/ur_teleop/ 包根）
[pytest]
markers =
    integration: full-stack smoke tests (launch subprocesses, slow)
addopts = -m "not integration"
```

- [ ] **Step 3: 写 `tests/test_integration.py`**

```python
"""Full-stack integration tests: cell(mock) + teleop_node + fake_master.

Excluded by default (pytest.ini addopts). Run with:
  colcon test --packages-select ur_teleop --pytest-args "-m integration"
Requires a sourced ROS environment (and the lerobot venv for the record test).
"""

import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif("AMENT_PREFIX_PATH" not in os.environ,
                       reason="需要已 source 的 ROS 环境"),
]

SELF = Path(__file__).resolve().parent
REPO = SELF.parent
FAKE_MASTER = SELF / "fake_master.py"

CFG_BODY = """\
mode: teleop
sim: true
cell:
  launch_rviz: false
home:
  master: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  slave: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  master_gripper_value: 1000
  at_home_tolerance_rad: 0.05
  settle_time_s: 2.0
  settle_motion_threshold_rad: 0.01
  move_timeout_s: 30.0
mapping:
  alicia_joint_order: [Joint1, Joint2, Joint3, Joint4, Joint5, Joint6]
  ur_joint_order: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
  sign: [1, 1, 1, 1, 1, 1]
  scale: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
safety:
  clamp_margin_rad: 0.1
  limits:
    shoulder_pan_joint: [-6.283, 6.283]
    shoulder_lift_joint: [-6.283, 6.283]
    elbow_joint: [-3.142, 3.142]
    wrist_1_joint: [-6.283, 6.283]
    wrist_2_joint: [-6.283, 6.283]
    wrist_3_joint: [-6.283, 6.283]
teleop:
  command_rate_hz: 50
  watchdog_timeout_s: 0.5
  restore_controller_on_exit: true
gripper:
  enabled: false
recorder:
  repo_id: test/ur_teleop_it
  root: ""
  fps: 50
  robot_type: ur10e_alicia_teleop
  use_videos: false
  ee_pose_source: none
  cameras: {}
  min_frames_per_episode: 2
"""


def _start(*argv, cwd=REPO, stdin=None):
    return subprocess.Popen(list(argv), cwd=str(cwd), stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, stdin=stdin,
                            start_new_session=True)


def _kill(procs):
    for p in procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
    time.sleep(1.0)
    for p in procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass


def _topic_once(topic, field=None, timeout=6.0):
    """ros2 topic echo --once; returns (ok, payload). ok=False on timeout/no message."""
    cmd = ["ros2", "topic", "echo", topic, "--once"]
    if field:
        cmd += ["--field", field]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return (out.returncode == 0 and bool(out.stdout.strip())), out.stdout.strip()
    except subprocess.TimeoutExpired:
        return False, ""


def _pub_bool(topic, value):
    subprocess.run(["ros2", "topic", "pub", "--once", topic, "std_msgs/msg/Bool",
                    f"{{data: {str(value).lower()}}}"], capture_output=True, timeout=10.0)


def _wait_status_true(timeout=25.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        ok, val = _topic_once("/teleop/status", "data", timeout=min(5.0, deadline - time.time()))
        if ok and "true" in val.lower():
            return True
    return False


def _launch_stack(tmp_path, master_off_home=False):
    cfg_path = tmp_path / "ur_teleop.yaml"
    cfg_path.write_text(CFG_BODY)
    procs = [
        _start("ros2", "launch", "ur_teleop", "cell.launch.py",
               f"config_file:={cfg_path}", "sim:=true", "launch_rviz:=false"),
        _start("ros2", "run", "ur_teleop", "teleop_node",
               "--ros-args", "-p", f"config_file:={cfg_path}"),
        _start(sys.executable, str(FAKE_MASTER),
               "--off-home" if master_off_home else ""),
    ]
    return procs


def test_enter_gate_then_mirror(tmp_path):
    """无 enable 不 ACTIVE；enable 后 50 Hz 映射发布、clamp 生效、跟随正弦。"""
    procs = _launch_stack(tmp_path)
    try:
        time.sleep(20.0)                       # cell 启动 + settle 2 s + offset 捕获
        ok, val = _topic_once("/teleop/status", "data", timeout=5.0)
        assert not ok, f"Enter/enable 前不应 ACTIVE，却收到 status={val}"

        _pub_bool("/teleop/enable", True)      # record 模式键盘 Enter 的协议等价物
        assert _wait_status_true(), "enable 后 25 s 内未进入 ACTIVE"

        samples = []
        for _ in range(4):
            ok, val = _topic_once("/forward_position_controller/commands", "data", timeout=5.0)
            assert ok, "未收到 /forward_position_controller/commands"
            samples.append([float(x) for x in re.findall(r"-?\d+\.?\d*", val)])
            time.sleep(1.0)
        assert all(len(s) == 6 for s in samples), f"命令维数错误: {samples}"
        lo, hi = -6.283 + 0.1, 6.283 - 0.1
        assert all(lo <= v <= hi for s in samples for v in s), f"指令越出 safety 范围: {samples}"
        spread = [max(x) - min(x) for x in zip(*samples)]
        assert max(spread) > 0.01, f"指令未跟随主臂运动: {samples}"
    finally:
        _kill(procs)


def test_verify_home_rejects_off_home_master(tmp_path):
    """主臂不在 home 时停在 VERIFY_HOME，不进入 ACTIVE（spec §9）。"""
    procs = _launch_stack(tmp_path, master_off_home=True)
    try:
        time.sleep(18.0)
        ok, val = _topic_once("/teleop/status", "data", timeout=5.0)
        assert not ok, f"主臂不在 home 时不应 ACTIVE，却收到 status={val}"
    finally:
        _kill(procs)


def test_record_one_episode(tmp_path):
    """record 冒烟：Enter → ACTIVE → 录帧 → D 丢弃 → Q finalize（spec §11.3）。"""
    try:
        import lerobot  # noqa: F401
    except ImportError:
        pytest.skip("lerobot 未安装（需要 source /opt/lerobot_venv/bin/activate）")

    root = tmp_path / "data"
    cfg_path = tmp_path / "ur_teleop.yaml"
    cfg_path.write_text(CFG_BODY.replace("mode: teleop", "mode: record")
                                .replace('root: ""', f"root: {root}"))
    procs = [
        _start("ros2", "launch", "ur_teleop", "cell.launch.py",
               f"config_file:={cfg_path}", "sim:=true", "launch_rviz:=false"),
        _start("ros2", "run", "ur_teleop", "teleop_node",
               "--ros-args", "-p", f"config_file:={cfg_path}"),
        _start(sys.executable, str(FAKE_MASTER)),
        _start("ros2", "run", "ur_teleop", "data_recorder",
               "--ros-args", "-p", f"config_file:={cfg_path}", stdin=subprocess.PIPE),
    ]
    recorder = procs[-1]
    try:
        time.sleep(20.0)
        recorder.stdin.write("enter\n")
        recorder.stdin.flush()                 # 开始 episode 1 + 发 /teleop/enable
        assert _wait_status_true(), "Enter 后 25 s 内未进入 ACTIVE"
        time.sleep(3.0)                        # 录 ~150 帧
        recorder.stdin.write("d\n")
        recorder.stdin.flush()                 # 丢弃
        time.sleep(1.0)
        recorder.stdin.write("q\n")
        recorder.stdin.flush()                 # 退出 + finalize
        recorder.wait(timeout=15.0)
        assert (root / "test" / "ur_teleop_it").exists(), "数据集目录未创建"
        log = recorder.stdout.read()
        assert "finalize" in log.lower(), f"recorder 日志未见 finalize:\n{log[-2000:]}"
    finally:
        _kill(procs)
```

- [ ] **Step 4: 运行集成测试**

Run: `source /opt/ros/jazzy/setup.bash && source /opt/lerobot_venv/bin/activate && cd /ros2_ws && colcon build --packages-select ur_teleop --symlink-install && colcon test --packages-select ur_teleop --pytest-args "-m integration" --event-handlers console_direct+`
Expected: 3 条集成断言通过（门控+跟随+clamp / VERIFY_HOME 拒绝 / record 冒烟）；失败时按子进程日志定位状态机问题（各测试 finally 前可临时 `print(procs[i].stdout.read())` 查日志）

- [ ] **Step 5: 提交**

```bash
git add tests/fake_master.py tests/conftest.py tests/test_integration.py pytest.ini
git commit -m "test: 全栈集成测试（fake_master + 门控/enable/clamp/VERIFY_HOME/record 冒烟）"
```

---

### Task 12: 清理与文档收尾

**Files:**
- Delete: `ur_teleop/fake_alicia.py`、`ur_teleop/calibration.py`、`ur_teleop/utils.py`、`launch/teleop_only.launch.py`、`launch/record.launch.py`、`launch/calibrate.launch.py`、`launch/view.launch.py`、`launch/alicia_display.launch.py`、`config/joint_mapping.yaml`、`config/calibration_offset.yaml`、`config/teleop_params.yaml`、`config/recorder_params.yaml`、`config/robotiq_gripper.yaml`、`scripts/run_teleop.sh`、`docs/`（旧中文文档整目录）
- Create: `README.md`
- Modify: `package.xml`（最终依赖确认：depend = rclpy、sensor_msgs、std_msgs、control_msgs、controller_manager_msgs、geometry_msgs、trajectory_msgs、tf2_ros、tf2_geometry_msgs、python3-yaml、python3-numpy、cv_bridge；lerobot 以注释说明可选（/opt/lerobot_venv））

- [ ] **Step 1: 删除旧文件**

```bash
cd /ros2_ws/src/ur_teleop
git rm ur_teleop/fake_alicia.py ur_teleop/calibration.py ur_teleop/utils.py \
       launch/teleop_only.launch.py launch/record.launch.py launch/calibrate.launch.py \
       launch/view.launch.py launch/alicia_display.launch.py \
       config/joint_mapping.yaml config/calibration_offset.yaml config/teleop_params.yaml \
       config/recorder_params.yaml config/robotiq_gripper.yaml scripts/run_teleop.sh
git rm -r docs
```

- [ ] **Step 2: 全量验证**

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur_teleop --symlink-install
colcon test --packages-select ur_teleop --event-handlers console_direct+
grep -rn "utils\|fake_alicia\|calibration\|joint_mapping\|teleop_params" src/ur_teleop/ur_teleop/ || echo "无残留引用"
```
Expected: 构建成功、单测全绿、无旧模块引用残留

- [ ] **Step 3: 写 `README.md`**（覆盖：安装与依赖、`ur_teleop.yaml` 关键配置说明（mode/sim/home/mapping/safety/gripper/recorder）、两阶段使用流程（sim 与 real 各一条命令序列）、键盘键位表（teleop 模式 Enter；record 模式 Enter/S/D/Q）、record 数据格式（state 14 维 + action 7 维，特征名列表）、测试命令（单测 + 集成）、spec §2 问题 1–16 对照修复表）

- [ ] **Step 4: 提交**

```bash
git add -A
git commit -m "chore: 清理旧模块/launch/配置/文档，补 README，最终依赖确认"
```

---

## 自检记录

**1. Spec 覆盖核对（spec §1–§11 → 任务）：**

| Spec 项 | 任务 |
|---|---|
| §3 决策：两阶段 home / 夹爪跟随 / 只留 real / 独立 recorder / 单一 yaml / offset 不写盘 | 全部（T7 两阶段、T4+T8 夹爪、T12 删 fake_alicia、T9 独立进程、T1 单 yaml、T3 offset） |
| §4.1 三 launch 架构 | T10（cell 持久，home/teleop 复用） |
| §4.2 数据流（commands/status/enable/e_stop、demonstration、joint_commands） | T8（协议实现）、T9（enable 发布） |
| §5 状态机 8 态 + 30 s 超时 + force_home + settle 阈值 + offset 捕获 + 切换重试 5 次自动 load + watchdog 收敛 + e_stop 冻结 + 退出流程 | T8 |
| §6 Enter 门控与键盘单一归属 | T8（teleop 模式读键盘）、T9（record 模式发 enable） |
| §7 组件职责与关键实现决策（scale 生效、单位转换集中 config、夹爪探测一次、EE 缺失 NaN、夹爪 state 用真实状态） | T2（scale）、T1（换算）、T8（探测一次）、T9+FrameBuilder（NaN 与真实 state） |
| §8 配置结构与 launch 参数优先 yaml 兜底 | T1、T10 |
| §9 错误处理表全部 12 行 | T8（cell 未启动/不在 home/watchdog/恢复/e_stop）、T7（轨迹失败对比/到位超时）、T9（数据集已存在/EE NaN/夹爪不存在） |
| §10 测试策略（4 单测文件 + 集成 3 项） | T1–T4、T11 |
| §11 成功标准 | T11（冒烟断言）+ T12（README 流程） |

**2. 与 spec 的有意偏差（均已在正文标注）：**
- `cell.ft_port` → `cell.ftdi_id`：`ft_sensor_standalone.launch.py` 实际参数只有 `ftdi_id`（无 port 参数，已核实源码），用 spec 键会产生配置死键（spec 问题 13）。
- `home.move_duration_s`/`verify_duration_s`/`recorder.state_threshold_rad`/`gripper.fsm_rate_hz` 为 home_node/FrameBuilder 消费的补充键（有默认值，非死键）。
- 集成测试的 Enter 注入：recorder 通过 `stdin=PIPE` 写字符（真实按键路径）；teleop 门控测试用 `ros2 topic pub /teleop/enable`（record 协议等价物）。
- `safety.limits` 用关节名键值 dict（spec 示例为 `[[min,max]]` 占位），JointMapper 按关节名查表。
- pytest.ini 位于包根（colcon 的 rootdir 向上搜索能找到），非 tests/ 内。

**3. 类型一致性抽查：**
- `load_config(path) -> dict`、`default_config_path() -> str`：T1 定义，T7/T8/T9 消费 ✓
- `JointMapper(mapping_config, master_home, slave_home)` + `build_mapping_config(cfg)`：T2 定义，T3 测试构造同形状（含 safety），T8 消费 ✓
- `SessionOffset.capture(master_q, slave_q)` / `captured`：T3 → T8 ✓
- `GripperController.update/get_knuckle_command/get_gripper_command_signal/current_target`：T4 定义（current_target 一并交付），T8 消费 ✓
- `ControllerSwitcher.services_ready/list_controllers/list_result/load_controller/switch/switch_ok`：T6 定义，T8 消费（list→load→switch 异步链）✓
- `KeyboardReader.read_key(timeout) -> str|None`：T5 → T8/T9 ✓
- `FrameBuilder(recorder_config, gripper_config).features()/build(...)`：T9 定义与消费 ✓
- 配置键名跨任务一致：`home.master/slave`、`safety.limits`（关节名键）、`teleop.command_rate_hz/watchdog_timeout_s/restore_controller_on_exit`、`gripper.enabled/action_server/...`、`recorder.*` ✓
- 节点参数（config_file/mode/force_home）三节点声明方式一致（declare_parameter + launch parameters）✓
