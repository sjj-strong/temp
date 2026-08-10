# 帧构建（frame_builder.py）

> 路径：`ur_teleop/ur_teleop/frame_builder.py` —— 纯逻辑（仅 numpy），把实时关节/位姿/夹爪/指令组装成 LeRobot `LeRobotDataset.add_frame` 所需的 `observation.state` + `action` 数组。

## 概述

FrameBuilder 是 record 模式的"单帧组装器"：输入 UR 关节、EE 位姿、UR 夹爪弧度、7 维遥操作指令，输出 numpy 帧（含 `task` 字段）。特征布局由 `features()` 按 recorder 配置开关决定（默认 state 14 维、action 7 维）。缺数据时**宁缺毋滥**：必要数据缺失返回 None 跳过本帧；EE 位姿缺失则以 NaN 段记录而非零占位。

## 公开接口

```python
EE_NAMES = ["ee_x", "ee_y", "ee_z", "ee_qx", "ee_qy", "ee_qz", "ee_qw"]

def __init__(self, recorder_config: dict, gripper_config: dict):
    # _state_threshold = recorder_config.get("state_threshold_rad", 0.4)
    # _task            = recorder_config.get("task", "teleoperation")
    # _use_videos      = bool(recorder_config.get("use_videos", True))

def features(self) -> tuple[dict, list[str], list[str]]:
    # → (features, state_names, action_names)

def build(self, ur_joints, ee_pose, gripper_state_rad, teleop_cmd) -> dict | None:
    ...
```

### `features()`（frame_builder.py:25-53）

按配置开关拼特征（各开关默认 True）：

- **state（默认 14 维）**：
  - `record_ur_joints` → 6 × `UR_JOINT_NAMES`（shoulder_pan…wrist_3）；
  - `record_ur_ee_pose` → 7 × `EE_NAMES`；
  - `record_ur_gripper` → `"gripper_state"`。
- **action（默认 7 维）**：
  - `record_action_joints` → `cmd_<UR关节名>` × 6；
  - `record_action_gripper` → `"cmd_gripper"`。
- 每个特征为 `{"dtype": "float32", "shape": (n,), "names": [...]}`；某段全关时该段不出现（如全部 state 开关关闭则无 `observation.state`）。
- **相机**：`recorder.cameras` 每项生成 `observation.images.<image_key 或 cam_name>`，`dtype` 为 `"video"`（`use_videos=True`）或 `"image"`，`shape=(height, width, 3)`（height 默认 480、width 默认 640），`names=["height","width","channels"]`。

### `build(ur_joints, ee_pose, gripper_state_rad, teleop_cmd)`（frame_builder.py:55-75）

```python
if ur_joints is None or teleop_cmd is None:
    return None                                  # 必要数据缺失 → 整帧跳过
# state：
#   record_ur_joints   → ur_joints[:6]
#   record_ur_ee_pose  → ee_pose（None 时 [nan]*7）
#   record_ur_gripper  → 1.0 if gripper_state_rad > state_threshold else 0.0
# action：
#   record_action_joints  → teleop_cmd[:6]
#   record_action_gripper → float(teleop_cmd[6]) if len(teleop_cmd) > 6 else 0.0
return {
    "observation.state": np.array(state_parts, dtype=np.float32),
    "action":            np.array(action_parts, dtype=np.float32),
    "task":              self._task,
}
```

## 关键逻辑

- **state 14 维组成**：6 个 UR 关节（rad）+ 7 维 EE 位姿（xyz + 四元数）+ 1 维 `gripper_state`；**action 7 维**：`cmd_` 前缀的 6 个关节指令 + `cmd_gripper`。
- **EE 缺失 → NaN 段**：`ee_pose is None` 时填入 7 个 `np.nan`（绝不填零占位——NaN 在 LeRobot/训练侧会被显式识别为无效段）。data_recorder 在 `ee_pose_source != "none"` 且首次查询失败时记一次警告（data_recorder.py:198-200）。
- **夹爪真实状态二值化**：`gripper_state_rad` 是 UR 侧 `robotiq_85_left_knuckle_joint` 的实测弧度（不是指令目标），超过 `state_threshold_rad`（默认 0.4，即约半个行程）记 1.0（闭），否则 0.0（开）。注意阈值 0.4 与夹爪闭合指令 0.79 之间存在余量，二值化对轻微抖动不敏感。
- **action 的 `cmd_gripper`**：取 `teleop_cmd[6]`（`/teleop/commands` 第 7 维，由 teleop_node 填 `get_gripper_command_signal(current_target)`，即 1.0/0.0）；长度不足 7 时补 0.0（防御性）。
- **切片而不校验长度**：`ur_joints[:6]` / `teleop_cmd[:6]` 静默截断——形状契约由发布方（teleop_node 恒发 6/7 维）保证，本模块不做长度断言。

### `_gripper` 死字段（注明）

构造参数 `gripper_config` 被存为 `self._gripper`（frame_builder.py:20），但 `features()`/`build()` 全程不读取它——所有行为只由 `recorder_config` 驱动。该字段为历史接口遗留（data_recorder 仍按 `FrameBuilder(rec, cfg.get("gripper", {}))` 传入），目前无功能影响；改动签名会波及 data_recorder，故保留。

## 数据流 / 消费方

- 唯一消费者：data_recorder `_record_frame`（data_recorder.py:191-217）——录制定时循环里取 `_ur_joints` / `_teleop_cmd` / `_ur_gripper_rad`（来自 `/joint_states`）与 `_get_ee_pose()`，调 `build()`，`None` 则跳过本帧；非 None 则拼上相机图像后 `add_frame(frame)`。
- 特征声明：data_recorder 在初始化时用 `features()` 创建 LeRobot 数据集（`LeRobotDataset.create`），因此**先 features() 后 build()，两者必须开关一致**（同一份 recorder 配置）。
- 输入来源：`ur_joints`/`teleop_cmd` 来自 `/teleop/commands` 订阅（record 模式只录指令，不录主臂关节）。

## 错误处理 / 已知边界

- 缺失 `ur_joints` 或 `teleop_cmd` → 返回 None 整帧跳过（宁可丢帧不写坏帧）。
- EE 缺失不丢帧：NaN 段记录（刻意设计，见上）。
- 不校验 feature 开关与 build 切片的一致性——若开关在会话中途被改（本包不会），shape 会与数据集声明不符。
- 四元数不归一化、不校验模长——原样透传 tf/topic 查询结果。

## 测试覆盖（tests/test_frame_builder.py）

- `test_features_14_and_7`：默认 state 14 维（末位 `gripper_state`）、action 7 维（末位 `cmd_gripper`）。
- `test_build_full_frame`：完整帧组装，`gripper_state` 0.79 rad > 0.4 → 1.0；action 末维 = 1.0；`task` 透传。
- `test_gripper_state_threshold_open`：0.0 rad → 0.0。
- `test_missing_joints_returns_none`：`ur_joints` 或 `teleop_cmd` 为 None → None。
- `test_missing_ee_pose_uses_nan`：`ee_pose=None` → state 第 7 维（6 关节之后）为 NaN。
- `test_camera_feature_included`：cameras 配置生成 `observation.images.wrist`，shape (480, 640, 3)。
