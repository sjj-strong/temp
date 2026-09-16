# ruckig_node（遥操从臂 Ruckig 平滑节点）

> 路径：`ur_teleop/ruckig_node.py` — 基于 [Ruckig](https://github.com/pantor/ruckig) 的在线轨迹生成（OTG）节点，对 teleop_node 映射后的 UR 目标做 jerk-limited 平滑，再按配置下发给 UR 前向位置或关节阻抗控制器。
> **性质：遥操链路的平滑环节**。teleop_node 把映射后的 UR 目标发给本节点，本节点以 `control_hz`（默认 **500 Hz**，与 `controller_manager` 一致）平滑后下发配置选择的从臂控制器。

## 在遥操链路中的位置

```
/joint_states（Alicia Joint1-6 @100 Hz + Gripper）
        │
        ▼
  teleop_node ──► JointMapper.master_to_slave()（home offset + sign/scale + 安全 clamp）
        │
        ▼  /ruckig/target_joint_positions（6 维 UR 目标，~50 Hz，Float64MultiArray）
        ▼
  ruckig_node（Ruckig OTG，500 Hz，jerk-limited）
        │   ├─ /joint_states（UR 实测 q/dq，按名重排，用于首次初始化）
        │   └─ 首次全关节有效时 initialize_ruckig()：初始状态 = 当前 UR 状态，目标 = 当前位置 → 启动不产生运动
        ▼
  `teleop.controller=forward_position`：
  /forward_position_controller/commands（6 维位置，500 Hz）

  `teleop.controller=joint_impedance`：
  /joint_impedance_controller/target_joint_state（JointState，500 Hz）
                                                        ← 均仅 ruckig_node 发布
```

夹爪不经 ruckig：teleop_node 用 Alicia `Gripper` 关节做二值（开/合）判断，直接发 Robotiq action。

## 关键机制

- **按名重排**：`/joint_states` 与主臂交织在同一话题（双发布者），`joint_state_callback` 用 `UR_JOINT_INDEX` 按名取 UR 6 关节，忽略其余；`robot_joint_valid` 保证 6 关节都至少收到一次才初始化。
- **初始化语义**：`initialize_ruckig()` 把 Ruckig 初始状态设为 UR 当前实测（位置 + 速度），并把启动目标设为当前位置——**节点启动本身不会让机器人运动**，收到新目标才平滑跟踪。
- **目标校验**：`target_callback` 拒绝维度 ≠ 6 与含 NaN/Inf 的目标（打 error 日志，不更新）。
- **OTG 步进**：`control_loop()` 每 tick 更新 `inp.target_position` → `otg.update(inp, out)` → 发布 `out.new_position` → `out.pass_to_input(inp)`（本步输出成为下步当前状态）。`Result` 非 `Working/Finished` 时打 error 并跳过该 tick。
- **运动限制**：max_velocity/max_acceleration/max_jerk 为 6 维数组，作用于 Ruckig 插值（jerk-limited 平滑，无跳变、无尖峰）。

## 配置

`control_hz` 通过 launch 参数 `ruckig_control_hz`（或 ROS 参数 `control_hz`）设置，默认 **500.0**（`teleop.launch` 从 `ur_teleop.yaml` 的 `ruckig.control_hz` 读取，缺省 500）。

max_velocity / max_acceleration / max_jerk：若 `ur_teleop.yaml` 含 `ruckig:` 段则从其读取，否则回退代码内 `_DEFAULT_*`（`[0.30]×6 / [0.80]×6 / [4.0]×6`，第一阶段真机测试保守参数）。列表长度 ≠ 6 时同样回退默认。需要调参时在 `ur_teleop.yaml` 加回 `ruckig:` 段即可。

`teleop.controller` 决定输出接口：默认 `forward_position` 保持历史行为；设为 `joint_impedance` 时发布 `sensor_msgs/msg/JointState`，其中 `name` 固定为 UR 六关节标准顺序，`position` 为 Ruckig 输出。阻抗控制器的 `velocity` 可省略，控制器会使用其受限内部参考速度。

## 启动方式

```bash
# 正常遥操（推荐）：home 完成后启动，ruckig 从当前（已 home）UR 状态初始化
#   T1: ros2 launch ur_teleop home.launch.py sim:=true
#   T2: ros2 launch ur_teleop teleop.launch.py mode:=teleop        # 含 teleop_node + ruckig_node
# 关节阻抗仿真：先在 YAML 设置 teleop.controller: joint_impedance，随后
#   T1: ros2 launch ur_teleop home.launch.py sim:=true controller:=joint_impedance
#   T2: ros2 launch ur_teleop teleop.launch.py
# 频率可覆盖：
#   ros2 launch ur_teleop teleop.launch.py ruckig_control_hz:=500

# 独立手动测试（无 teleop，自己往 /ruckig/target_joint_positions 灌目标）：
ros2 run ur_teleop ruckig_node --ros-args \
  -p config_file:=/ros2_ws/install/ur_teleop/share/ur_teleop/config/ur_teleop.yaml \
  -p control_hz:=500.0
ros2 topic pub -r 1 /ruckig/target_joint_positions std_msgs/msg/Float64MultiArray \
  "{data: [0.5, -1.5, 0.2, -1.2, 0.3, 0.0]}"
```

## 注意事项

- **唯一发布者**：配置选择的输出话题（`/forward_position_controller/commands` 或 `/joint_impedance_controller/target_joint_state`）只能由 ruckig_node 发布；teleop_node 已改为只发 `/ruckig/target_joint_positions`。**不要同时用 `cell.launch … ruckig:=true` 和 `teleop.launch`**——两个 ruckig_node 会抢同一话题。
- **必须在 home 之后启动**：teleop.launch 在 home 完成后才拉起 ruckig_node，此时 UR 已在 home 位，ruckig 从真实状态初始化，避免 home 阶段轨迹控制器移动机器人导致 Ruckig 内部状态过期（否则切换到 forward controller 时首帧会跳变）。
- **`cell.launch ruckig:=true` 仅用于纯手动测试**（不跑 teleop/home），方便单独验证 Ruckig 效果或标定运动学参数。
- **依赖**：`pip install ruckig`（无 rosdep key，系统 python 与 lerobot venv 均需安装）。
- **真机注意**：`/joint_states` 的 velocity 由 UR driver 提供（RTDE），mock 下为 0；初始化用实测速度作为 Ruckig 当前速度，真机突然启动时若实测速度跳变，Ruckig 会按当前速度规划。

## 测试覆盖（tests/test_ruckig_node.py，12 项）

无 ROS 消息依赖（fake msg 用 SimpleNamespace），`object.__new__(RuckigNode)` 绕过 __init__、注入属性后直驱回调方法，Ruckig 用真实库对象：

- 常量：`UR_JOINT_NAMES`/`UR_JOINT_INDEX` 与 6 关节顺序一致；
- `joint_state_callback`：乱序按名重排、混入主臂关节跳过、velocity 缺失防护、部分关节未收到时拒绝初始化；
- `target_callback`：正常更新、维度 ≠ 6 拒绝、NaN/Inf 拒绝；
- `initialize_ruckig`：初始状态 = 当前 UR 状态、启动目标 = 当前位置（启动不运动）；
- `control_loop`：首步指令 = 当前位置（无跳变）、`pass_to_input` 状态延续、2000 步后指令收敛到目标（真实 OTG 跟踪验证）。
