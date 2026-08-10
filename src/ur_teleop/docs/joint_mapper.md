# 关节映射（joint_mapper.py）

> 路径：`ur_teleop/ur_teleop/joint_mapper.py` —— 纯逻辑（无 rclpy），Alicia-D → UR10e 的逐关节仿射映射（sign/scale/偏移）与 safety 限位 clamp。

## 概述

JointMapper 把主臂关节角（rad，按 `alicia_joint_order` 排列）映射为从臂关节指令（rad，按 `ur_joint_order` 排列）。公式、clamp、限位表全部内聚在本模块；不接触 ROS 话题。映射基准（home 偏移）由 `SessionOffset` 会话级捕获后注入（见 session_offset.md）。

## 公开接口

### 输入形状：`build_mapping_config(cfg)`（teleop_node.py:46）

JointMapper 的第一个参数是"mapping + safety 合并"后的 dict，由 teleop_node 的模块级函数构造：

```python
def build_mapping_config(cfg: dict) -> dict:
    """mapping + safety merged into the shape JointMapper expects."""
    return dict(
        cfg["mapping"],
        safety={
            "clamp_margin_rad": cfg["safety"]["clamp_margin_rad"],
            "limits": cfg["safety"]["limits"],
        },
    )
```

即 JointMapper 从同一个 dict 里读 `alicia_joint_order` / `ur_joint_order` / `sign` / `scale` / `safety`，而非接收整份 cfg。

### `JointMapper(mapping_config: dict, master_home: list[float], slave_home: list[float])`

构造期校验（任一失败抛 `ValueError`，joint_mapper.py:21-33）：

- `alicia_joint_order` 与 `ur_joint_order` 各必须 6 项；
- `sign` 与 `scale` 各必须 6 项；
- `master_home` 与 `slave_home` 各必须 6 项；
- `ur_joint_order` 中每个关节必须出现在 `safety.limits` 里（报错信息带关节名，如 `UR joint 'bogus_joint' missing from safety.limits`）。

属性/读取接口：

```python
ur_joint_order -> list[str]      # 从臂关节顺序（副本）
alicia_joint_order -> list[str]  # 主臂关节顺序（副本）
num_joints -> int                # len(ur_joint_order)，恒 6
get_master_home() -> list[float] # 副本
get_slave_home() -> list[float]  # 副本
```

### `master_to_slave(master_q: list[float]) -> list[float]`

```python
# joint_mapper.py:47-58，逐关节：
ur_cmd[i] = slave_home[i] + sign[i] * scale[i] * (master_q[i] - master_home[i])
ur_cmd[i] = clamp_to_limits(ur_joint_order[i], ur_cmd[i])
```

- 输入长度必须等于 `len(alicia_joint_order)`，否则 `ValueError(f"Expected {N} master joints, got {M}")`。
- 输出按 `ur_joint_order` 排列（与 `/forward_position_controller/commands` 的 6 维顺序一致）。

### `_clamp(joint_name: str, value: float) -> float`（joint_mapper.py:60-66）

```python
limits = self._limits.get(joint_name)
if limits is None or len(limits) < 2:
    return value                      # 无限位 → 直通（构造期已保证不会发生）
lo = float(limits[0]) + self._margin  # 下界向内收缩 clamp_margin_rad
hi = float(limits[1]) - self._margin  # 上界向内收缩 clamp_margin_rad
return max(lo, min(hi, value))        # 上下界双向 clamp
```

## 关键逻辑

### 映射公式如何生效

- **sign**：`+1` 同向，`-1` 反向（`master` 偏移量取负）。
- **scale**：纯比例系数，作用于主臂相对 home 的偏移量（修复过 scale 不生效的历史 bug，设计文档 §7）；默认 1.0。
- **单位换算**：主臂/从臂均为弧度，公式内无换算；比例与偏移天然为 rad。夹爪另有独立换算（config.py 与 GripperController），不经过本模块。
- **home 偏移**：`master_home` 为捕获时主臂位置、`slave_home` 为捕获时从臂位置；主臂回到捕获位时输出恰为 `slave_home`（恒等性，见 test_offset 集成用例）。

### clamp 与 safety.limits

- 限位表来自 `safety.limits`（ur_teleop.yaml）：每关节 `[lo, hi]` 弧度。默认表（rad）：

| 关节 | 下限 | 上限 |
|---|---|---|
| `shoulder_pan_joint` | -6.283 | 6.283 |
| `shoulder_lift_joint` | -6.283 | 6.283 |
| `elbow_joint` | -3.142 | 3.142 |
| `wrist_1_joint` | -6.283 | 6.283 |
| `wrist_2_joint` | -6.283 | 6.283 |
| `wrist_3_joint` | -6.283 | 6.283 |

- 上下界都向内收缩 `clamp_margin_rad`（默认 0.1 rad），使指令在抵达物理限位前预留安全余量；两侧对称处理。

## 数据流 / 消费方

1. teleop_node `_capture_offset`（teleop_node.py:219-232）：settle 完成后捕获 offset，构造 `JointMapper(build_mapping_config(self._cfg), self._offset.master_home, self._offset.slave_home)`——session 一旦 ARMED 后 mapper 固定，会话内不重建。
2. `_active`（teleop_node.py:306）：`cmd = self._mapper.master_to_slave(self._master_q)`，结果经 `_publish_commands` 发往 `/forward_position_controller/commands`（6 维）与 `/teleop/commands`（6 维 + 夹爪信号）。

## 错误处理 / 已知边界

- 构造期 4 类 `ValueError` 属编程错误（配置已由 `load_config` + `build_mapping_config` 保证形状），节点内不捕获——mapper 构造失败即启动失败。
- `master_to_slave` 长度不符属调用侧 bug，同样直接抛。
- clamp 是纯数值截断，不检测"主臂相对 home 的偏移超限"这类物理合理性；超限部分被静默截断，无日志（调用方可通过对比输入输出发现）。
- 无额外滤波/限速：50 Hz 下发的是 clamp 后的瞬时指令，跳变防护由 UR 端 forward controller 行为承担。

## 测试覆盖（tests/test_joint_mapper.py）

- `test_at_home_maps_to_slave_home`：主臂在 home → 输出 == slave_home（恒等）。
- `test_offset_mapping`：主臂整体偏移 +0.1 → 输出整体偏移 +0.1。
- `test_scale_is_applied`：`scale=[2,...]` 关节偏移量翻倍，其余关节不变（scale 真实生效的回归用例）。
- `test_negative_sign_flips_direction`：`sign=-1` 反向。
- `test_clamp_to_safety_limits`：越限输入被 clamp 到 `上限 - margin`（`1.0 - 0.1`）。
- `test_wrong_master_length_raises` / `test_missing_joint_raises`：长度校验与限位表缺失关节报错。
