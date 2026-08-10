# 夹爪控制（gripper_controller.py）

> 路径：`ur_teleop/ur_teleop/gripper_controller.py` —— 纯逻辑（无 rclpy），带迟滞的二元状态机：把 Alicia 夹爪位置（m，0=开）映射为 Robotiq 2F-85 的开/合目标。

## 概述

GripperController 是"目标决策"层：只根据主臂夹爪实时位置输出 `OPEN`/`CLOSED`/`UNKNOWN` 三态，不做任何 ROS 通信。执行层在 teleop_node：`_gripper_tick`（10 Hz）轮询 `update()`，目标变化时经 `ParallelGripperCommand` action 下发。夹爪单位约定：**Alicia 侧是位置（米，0=开）**，**Robotiq 侧是 knuckle 指令（弧度，0=开，0.79=闭）**，换算在本模块完成（`open_pos_rad`/`close_pos_rad` 参数化，见下）。

## 公开接口

### `GripperTarget(Enum)`

```python
OPEN = 0      # 目标张开
CLOSED = 1    # 目标闭合
UNKNOWN = 2   # 无变化/未启用/未知
```

### `GripperController(gripper_config: dict)`

构造参数全部经 `.get()` 带默认值（gripper_controller.py:19-29）：

| 键 | 类型 | 默认值 | 含义 |
|---|---|---|---|
| `enabled` | bool | `false` | 开关；关闭时 `update()` 恒 UNKNOWN |
| `action_server` | str | `/robotiq_gripper_controller/gripper_cmd` | action server 名（节点用） |
| `open_pos_rad` | float | `0.0` | 张开 knuckle 指令（rad） |
| `close_pos_rad` | float | `0.79` | 闭合 knuckle 指令（rad） |
| `close_threshold_m` | float | `0.0125` | 判 CLOSED 的阈值（Alicia 侧，m） |
| `open_threshold_m` | float | `0.005` | 判 OPEN 的阈值（Alicia 侧，m） |
| `max_effort` | float | `50.0` | 目标 effort |

属性：`open_position` / `close_position` / `max_effort` / `current_target`（最近一次 `update()` 后的目标，初始为 `UNKNOWN`）。

### `update(alicia_gripper_m: float) -> GripperTarget`（gripper_controller.py:47-59）

```python
if not self.enabled:
    return GripperTarget.UNKNOWN
if alicia_gripper_m > self._close_threshold:
    new_target = GripperTarget.CLOSED
elif alicia_gripper_m < self._open_threshold:
    new_target = GripperTarget.OPEN
else:
    new_target = self._current          # 死区：保持上一个目标
changed = new_target != self._current
self._current = new_target
return new_target if changed else GripperTarget.UNKNOWN
```

### `get_knuckle_command(target) -> float` / `get_gripper_command_signal(target) -> float`

```python
get_knuckle_command(target):     # close_pos_rad 若 CLOSED，否则 open_pos_rad
    return self._close_pos if target == GripperTarget.CLOSED else self._open_pos
get_gripper_command_signal(target):    # 1.0 若 CLOSED，否则 0.0
    return 1.0 if target == GripperTarget.CLOSED else 0.0
```

## 关键逻辑：阈值状态机

- **迟滞/死区**：`close_threshold_m`（0.0125）与 `open_threshold_m`（0.005）之间的区间为死区，输入落在这里时保持上一目标，避免单阈值在开合边界来回抖动。
- **UNKNOWN 语义**：仅两种情形——未启用，或目标未变化。它表示"无需动作"，不是"状态未知"；`current_target` 仍保留真实目标（供 `_publish_commands` 拼接第 7 维信号）。
- 状态机只改变量，不缓存输入本身；无时间维度（没有"保持 X 毫秒后才动作"）。

## 数据流 / 消费方（teleop_node `_gripper_tick`）

teleop_node.py:321-342，10 Hz（`gripper.fsm_rate_hz`，默认 10.0，仅 enabled 时建 timer）：

1. `_e_stop` 或未启用 → 直接返回（冻结）。
2. **server 探测（一次）**：`_gripper_probed` 标志位；若 `_gripper_action` 为 None 或 `server_is_ready()` 为 False → 记警告并把 `self._gripper.enabled` 置 False 永久禁用 FSM（此时 `update()` 恒 UNKNOWN，天然静默）。
3. **goals in-flight 门控**：`self._gripper_future` 非 None 且未 done → 本 tick 跳过（同一时刻最多一个未完成 goal）。
4. 锁内 `update(self._master_gripper_m)`（`_master_gripper_m` 来自 `/joint_states` 的 `Gripper` 关节位置，m）。
5. 结果非 UNKNOWN → `_send_gripper_goal(target)`：goal 的 `command.name=[UR_GRIPPER_JOINT]`、`position=[get_knuckle_command(target)]`、`effort=[max_effort]`，`send_goal_async` 存为 `_gripper_future`。

此外 `_publish_commands`（teleop_node.py:346-352）把 `get_gripper_command_signal(current_target)`（1.0/0.0）拼到 `/teleop/commands` 第 7 维（即 FrameBuilder action 的 `cmd_gripper`），与夹爪 action 下发相互独立。

## 错误处理 / 已知边界

- **阈值倒置未校验**：构造不检查 `close_threshold_m > open_threshold_m`（默认 0.0125 > 0.005 正常）。若配置倒置（close ≤ open），两个分支会重叠：对落在 `(close_threshold_m, open_threshold_m)` 区间的输入，`>` 分支先判 → **CLOSED 胜出**。这是 `if/elif` 顺序决定的刻意行为（重叠区偏向闭合而非张开），文档明确记录，不视为缺陷。
- **UNKNOWN → 张开**：`get_knuckle_command(UNKNOWN)` 走 else 分支返回 `open_pos_rad`（0.0）——安全默认（缺省张开而非闭合），测试锁定。
- 门控失效后的补偿：goal 在飞时输入变化会被跳过，下个 tick 再判——接受至多一个 tick 的决策延迟（刻意，避免堆积 goals）。
- 死区两端是严格不等式（`>` / `<`）；恰好等于阈值时保持上一目标。

## 测试覆盖（tests/test_gripper_controller.py）

- `test_disabled_by_config`：未启用 → 恒 UNKNOWN。
- `test_open_to_closed_transition`：0.0 → OPEN、0.03 → CLOSED；knuckle 指令 0.79/0.0、信号 1.0/0.0 双出口。
- `test_hysteresis_deadband_keeps_previous_target`：0.03→CLOSED 后，0.010（死区）→ UNKNOWN 且保持；0.004 越过 open 阈值 → OPEN；回 0.010 → 保持 OPEN；0.02 → CLOSED。
- `test_current_target_follows_updates`：`current_target` 跟随。
- `test_knuckle_default_safe_open`：UNKNOWN → 0.0（张开）。
