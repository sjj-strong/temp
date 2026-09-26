# 整体 pipeline 与框架逻辑

> 路径：`/ros2_ws/src/ur_teleop/docs/pipeline.md` — 本包架构核心文档：设计目标、节点拓扑与生命周期、话题/服务/动作清单、数据流、8 态状态机总览与错误处理总览。

本文档回答「系统如何组织与运转」：两阶段启动、进程边界、进程间协议、状态机骨架。各模块实现细节见对应文档（teleop_node / home_node / joint_mapper / data_recorder 等）。

## 概述

UR10e 从臂 + Alicia-D 主臂遥操作：主臂关节 → 会话 offset 映射 → ruckig 在线轨迹平滑（500 Hz，与 controller_manager 同频）→ 配置选择的从臂前向位置或关节阻抗控制；sim/real 两种形态（sim = UR 端 mock + rviz，主臂始终真实）；teleop/record 两种模式（record 基于 lerobot 保存数据集）。所有编排由 `teleop_node` 的 8 态 FSM 驱动，进程间仅以话题/服务/动作通讯，无多线程拆解。

## 设计目标

- **两阶段启动**：cell（UR 栈 + 夹爪 + FT）持久运行，home（阶段 1）与 teleop（阶段 2）分阶段执行。cell 由阶段 1 拉起后持续运行，两阶段共享同一个 controller_manager，避免 UR 栈重启丢失位姿。
- **真实主臂 only**：删除了 fake_alicia 与 gui 滑动条等开发模式，主臂必须是真实 Alicia 驱动。
- **ROS2 nodes + topic 通讯、无多线程拆解**：所有节点单线程 executor；定时器回调内不做阻塞操作（修复 spec 问题 7）；控制器切换是状态机轮询的异步 Future 链，无嵌套 spin（修复问题 10）；键盘用 select 非阻塞读。
- **进程隔离**：teleop_node 与 data_recorder 是独立进程——recorder 崩溃不影响遥操，已保存 episodes 保留。
- **会话内 offset**：offset 不写盘，CAPTURE_OFFSET 时自动捕获（修复问题 2/4）。
- **单一配置入口**：`ur_teleop.yaml` 合并旧五个 yaml；launch 参数优先、yaml 兜底（修复问题 1/13/14）。

## 节点拓扑与生命周期

```
┌─────────────────────────────── 阶段 1 ───────────────────────────────┐
│ home.launch.py                                                       │
│   ├── include cell.launch.py ── 持久运行（两阶段共享）                  │
│   │     ├── ur_control.launch.py（ur_robot_driver）                   │
│   │     │      sim : use_mock_hardware=true（mock UR）                 │
│   │     │      real: 真机 + dashboard                                  │
│   │     │      └── description_launchfile（默认 ur10e_robotiq_ft_description 组合  │
│   │     │          模型 rsp：UR+FT300+2F-85，xacro 含官方 ros2_control │
│   │     │          宏，mock/real 插件自动切换；未安装回退官方纯 UR）    │
│   │     ├── rviz2（仅 sim 且 launch_rviz=true，-d ur_teleop.rviz）      │
│   │     ├── robotiq_control.launch.py（仅 real：夹爪，com_port）        │
│   │     └── ft_sensor_standalone.launch.py（仅 real：FT300，ftdi_id）  │
│   └── home_node（一次性）：cell 就绪 → UR home 轨迹 + Alicia home      │
│         → 验证到位 → 打印 HOME REACHED → 退出 0/1（cell 保持运行）       │
└──────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────── 阶段 2 ───────────────────────────────┐
│ teleop.launch.py（连接已运行的 cell，不含 cell）                       │
│   ├── teleop_node（常驻）：WAITING_CELL → … → ACTIVE                  │
│   ├── ruckig_node（常驻）：消费 teleop 映射目标，500 Hz 平滑后下发      │
│   └── mode=record 时：data_recorder（常驻，键盘唯一归属）               │
└──────────────────────────────────────────────────────────────────────┘
```

- **cell 生命周期**：由 `home.launch.py` 启动并持续运行；`teleop.launch.py` 不带 cell，teleop_node 在 WAITING_CELL 中等 `/joint_states` 与 `/controller_manager` 服务，30 s 超时明确报错「请先运行 home.launch」（teleop_node.py:156-164）。
- **home_node 生命周期**：一次性进程。等 cell 就绪（从臂 `/joint_states` 出现 + trajectory action server 就绪，`cell_ready()`，home_node.py:73-74）→ 向 UR 发 home 轨迹（accept 未接受时 10 s 窗口内每 0.5 s 重试，见 launch.md/teleop 行为）→ 对 Alicia 持续发布 `/joint_commands` → 双臂在容差内保持 `verify_duration_s` 后打印 HOME REACHED 退出；失败退出非零码，cell 保持。
- **teleop_node 生命周期**：常驻。8 态 FSM（见下），Ctrl-C 时执行退出恢复流程。
- **ruckig_node 生命周期**：常驻，随 `teleop.launch.py` 启动（home 完成之后）。从当前（已 home）UR `/joint_states` 初始化 Ruckig 状态、目标=当前位置（启动不产生运动），随后以 500 Hz 将 `/ruckig/target_joint_positions` 平滑后按 `teleop.controller` 发至前向位置或阻抗控制器。务必在 home 之后启动，否则 home 阶段轨迹控制器移动机器人会使 Ruckig 内部状态过期（详见 ruckig_node.md）。

## 话题 / 服务 / 动作清单

| 名称 | 接口 | 方向 | 触发 | 用途 |
|---|---|---|---|---|
| `/joint_states` | sensor_msgs/JointState | alicia_d_driver、UR cell → teleop_node / home_node | 主臂 100 Hz、UR 100 Hz | 主/从**交织在同一话题**；接收方按关节名逐侧更新（避免整包覆盖导致另一侧瞬时缺失） |
| `/joint_commands` | sensor_msgs/JointState | home_node → alicia_d_driver | home 阶段持续发布 | Alicia home 指令（Joint1..6 弧度 + Gripper 0-1000 反向值） |
| `/demonstration` | std_msgs/Bool | teleop_node → alicia_d_driver | ACTIVE 起始发 true、退出发 false | 拖拽模式开关（false 恢复力矩） |
| `/teleop/enable` | std_msgs/Bool | data_recorder → teleop_node | 边沿触发（record 模式 Enter） | ARMED 门控；ARMED 前到达则锁存（`_enable_pending`） |
| `/teleop/status` | std_msgs/Bool | teleop_node → data_recorder | ACTIVE 时 true、INACTIVE 时 false | 录制门控 |
| `/teleop/e_stop` | std_msgs/Bool | 外部急停输入 → teleop_node | 电平 | 冻结标志：关节指令与夹爪 FSM 均暂停 |
| `/teleop/commands` | std_msgs/Float64MultiArray | teleop_node → data_recorder | 50 Hz（ACTIVE/INACTIVE） | 7 维 = 6 关节指令 + 夹爪指令信号 |
| `/ruckig/target_joint_positions` | std_msgs/Float64MultiArray | teleop_node → ruckig_node | 50 Hz（ACTIVE/INACTIVE） | 6 维映射目标（UR 序），ruckig 的平滑输入 |
| `/forward_position_controller/commands` | std_msgs/Float64MultiArray | ruckig_node → UR cell | 500 Hz，`forward_position` | 6 维 jerk-limited 平滑后的前向位置指令 |
| `/joint_impedance_controller/target_joint_state` | sensor_msgs/JointState | ruckig_node → UR cell | 500 Hz，`joint_impedance` | 带 UR 六轴名称的平滑位置参考 |
| `/scaled_joint_trajectory_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectory（action） | home_node → UR cell | home 阶段一次 | UR home 轨迹 |
| `/robotiq_gripper_controller/gripper_cmd` | control_msgs/ParallelGripperCommand（action） | teleop_node → 夹爪控制器 | 10 Hz FSM（仅 real） | 夹爪开/合目标 |
| `/controller_manager/list_controllers` | controller_manager_msgs/ListControllers（srv） | teleop_node → controller_manager | SWITCHING 阶段 | 检查 forward_position_controller 是否已加载 |
| `/controller_manager/load_controller` | controller_manager_msgs/LoadController（srv） | teleop_node → controller_manager | 未加载时 | 自动加载前向控制器 |
| `/controller_manager/switch_controller` | controller_manager_msgs/SwitchController（srv，STRICT） | teleop_node → controller_manager | SWITCHING / 退出恢复 | trajectory ⇄ forward_position 切换 |
| `/robot_description` | std_msgs/String（TRANSIENT_LOCAL） | cell 的 rsp → controller_manager / rviz2 | cell 启动时一次 | URDF 描述：默认组合模型（UR+FT300+2F-85），见 launch.md「rviz 模型」 |

teleop → recorder 协议仅 3 个话题（commands / status / enable），无请求-应答；`/demonstration` 与 `/joint_commands` 是 Alicia 侧协议；`/teleop/e_stop` 由外部急停输入（如控制面板脚本）发布。

## 数据流

```
Alicia ──100 Hz /joint_states(Joint1..6 弧度 + Gripper 米)──> teleop_node._joint_cb
        （与 UR cell 的从臂关节交织在同一话题，按侧更新各自缓存）
                                        │
                                        ▼
   JointMapper.master_to_slave(master_q)
   ur_cmd[i] = slave_home[i] + sign[i]·scale[i]·(master_q[m_i] − master_home[i])
                                        │ clamp 到 [limit_min+margin, limit_max−margin]
                                        ▼
   _publish_commands（50 Hz 定时器 _tick，仅 ACTIVE/INACTIVE 发映射目标）
   ├── /ruckig/target_joint_positions（6 维映射目标，UR 序）→ ruckig_node
   └── /teleop/commands（7 维 = 6 指令 + 夹爪指令信号）→ data_recorder（record 模式）

   ruckig_node（500 Hz Ruckig OTG，jerk-limited）：按 max_velocity/acceleration/jerk
        把目标平滑成轨迹 → forward_position 或 joint_impedance 输出话题 → UR cell

   夹爪（real）：10 Hz _gripper_tick —— GripperController 迟滞 FSM(master_gripper_m)
        → ParallelGripperCommand action → robotiq 夹爪
   record 模式：data_recorder 订阅 /teleop/commands + /teleop/status + /joint_states，
        FrameBuilder 组装 state 14 维 + action 7 维（+ 相机）→ lerobot 数据集
```

要点：

- **指令链路与频率**：teleop `_tick` 50 Hz 全程运行，仅 ACTIVE/INACTIVE 发布——把映射目标（INACTIVE 时为当前位置 hold，避免跳变）发到 `/ruckig/target_joint_positions`。ruckig_node 以 500 Hz Ruckig OTG 把目标平滑成 jerk-limited 轨迹，按 `teleop.controller` 下发至 `/forward_position_controller/commands` 或 `/joint_impedance_controller/target_joint_state`；平滑由 Ruckig 承担。teleop 与 ruckig 都仅在 teleop 处于 ACTIVE/INACTIVE 时才有数据流；其余状态两者均不发指令（teleop_node.py:299-317）。
- **watchdog**：`_last_master_stamp` 在每次收到主臂 `/joint_states` 时刷新；超过 `teleop.watchdog_timeout_s`（默认 0.5 s）→ INACTIVE（`/teleop/status=false`），主臂恢复自动回 ACTIVE，无需重新 Enter。
- **映射参数**：offset 在 CAPTURE_OFFSET 捕获（`SessionOffset`，不写盘）；映射使用 `mapping.*` + `safety.*` 合并后的配置（`build_mapping_config`，teleop_node.py:46-54）。公式与 clamp 细节见 joint_mapper.md。
- **record 数据流**：teleop_node → `/teleop/commands`（7 维）→ data_recorder；data_recorder → `/teleop/enable` → teleop_node。数据集与帧结构见[数据采集](data_recorder.md)。

## 状态机总览（teleop_node，8 态）

```
WAITING_CELL → VERIFY_HOME → SETTLING → CAPTURE_OFFSET → ARMED ⇄ SWITCHING → ACTIVE
                                                                              ↕ watchdog
                                                                         INACTIVE
```

| 状态 | 一句话 |
|---|---|
| `WAITING_CELL` | 等 `/joint_states`（主/从都出现）与 controller_manager 三服务就绪；30 s 超时 → 报错退出（非零码） |
| `VERIFY_HOME` | 双臂实测与配置 home 的 max 误差 ≤ `home.at_home_tolerance_rad`（默认 0.05）；不满足打印两侧误差并等待重试 |
| `SETTLING` | 双臂实测变化 < `settle_motion_threshold_rad`（默认 0.01）持续 `settle_time_s`（默认 2.0 s）才继续 |
| `CAPTURE_OFFSET` | 捕获主/从实测为会话 offset（不写盘），构建 JointMapper |
| `ARMED` | 等 Enter（teleop 模式）或 enable 锁存（record 模式） |
| `SWITCHING` | list →（未加载则 load）→ switch（trajectory → 配置选择的遥操控制器）异步链，失败重试最多 5 次 |
| `ACTIVE` | `/demonstration=true`、`/teleop/status=true`，50 Hz 映射目标发布（ruckig 500 Hz 平滑下发配置控制器）+ 10 Hz 夹爪 FSM |
| `INACTIVE` | 主臂数据超时：暂停映射（改发当前位置）、status=false；主臂恢复自动回 ACTIVE |

关键转换与机制（实现为准，spec §5 基础上含最终修复）：

- **enable 门控**：teleop 模式由 teleop_node 在 ARMED 内读键盘 Enter；record 模式由 data_recorder 发 `/teleop/enable`（Bool，边沿触发）。
- **enable 锁存（`_enable_pending`）**：`/teleop/enable` 在 ARMED **之前**到达（recorder 启动即发的场景）时先置 `_enable_pending=true`，进入 ARMED 后立即开始切换，不再等 Enter（teleop_node.py:139-145、234-239）。这是 spec 之后的最终修复，以代码为准。
- **watchdog 恢复**：INACTIVE 不要求重新 Enter，主臂数据恢复即回 ACTIVE（teleop_node.py:309-315）。
- **e_stop 冻结**：`/teleop/e_stop` **不是独立状态**，而是冻结标志：置位时 `_tick` 与 `_gripper_tick` 直接 return，关节指令与夹爪 FSM 均暂停；释放后从当前位恢复（teleop_node.py:147-155、321-323）。
- **退出恢复**（Ctrl-C）：先发 `/demonstration=false`（恢复力矩）→ 若在 ACTIVE/INACTIVE/SWITCHING 且 `teleop.restore_controller_on_exit`（默认 true）则切回 trajectory controller（5 s 截止，teleop_node.py:360-382）→ recorder `finalize()`（record 模式）。
- **SWITCHING 失败路径**：list 异常 → 视为未加载走 load；load 失败 → 回 ARMED；switch 失败重试 5 次后回 ARMED 并提示检查 controller_manager（teleop_node.py:250-297）。

## 键盘归属

| 模式 | 键盘归属 | 键 | 含义 |
|---|---|---|---|
| teleop（mode=teleop） | teleop_node | Enter | ARMED → ACTIVE（开始映射控制；teleop 50 Hz 喂目标，ruckig 500 Hz 平滑下发） |
| record（mode=record） | data_recorder | Enter | 开始 episode 1 并发送 `/teleop/enable` |
| record | data_recorder | `S` | 保存并结束当前 episode |
| record | data_recorder | `D` | 丢弃当前 episode（重置 buffer） |
| record | data_recorder | `Q` | 退出并 finalize 数据集 |

record 模式下控制在整个会话中持续，episode 边界只影响录制；低于 `recorder.min_frames_per_episode` 的短 episode 自动丢弃。`read_key(0.0)` 非阻塞读，可在 50 Hz 定时器内调用（keyboard.py）。

## 错误处理总览

| 场景 | 行为 |
|---|---|
| cell 未启动 | WAITING_CELL 30 s 超时 → 打印「请先运行 home.launch」→ 退出非零码（`fatal_error`）；home_node 等 `cell_ready()` 30 s 失败 → rc=1 |
| 双臂不在 home | VERIFY_HOME 打印两侧误差、等待重试；`force_home:=true` 把容差置 inf 跳过 |
| UR home 轨迹失败/被拒 | home_node 在 10 s 接受窗口内每 0.5 s 重试（cell 激活窗口 ~1-2 s 首发必被拒）；最终失败打印原因 + 目标/当前关节对比，退出非零码，cell 保持运行 |
| 到位超时 | home_node 报告「未到位」（含两侧目标/当前），rc=1；可用 `force_home:=true` 继续 |
| 主臂数据超时 | watchdog → INACTIVE，改发当前位置避免跳变 |
| 主臂数据恢复 | 自动回 ACTIVE，无需重新 Enter |
| 控制器切换失败 | 重试最多 5 次（未加载先自动 load）；仍失败回 ARMED 并提示检查 controller_manager |
| recorder 崩溃 | teleop 不受影响（进程隔离）；已保存 episodes 保留 |
| lerobot 数据集已存在 | 时间戳后缀新建 repo_id，不覆盖不追加 |
| EE 位姿查询失败 | 该帧 ee 段填 NaN（不填 0 占位）+ 一次性警告 |
| 夹爪 action server 不存在 | 探测一次（`_gripper_probed`），警告并禁用夹爪 FSM |
| e_stop 置位 | 冻结关节指令 + 夹爪 FSM；释放后从当前位恢复 |
| 退出 | `/demonstration=false` → 切回 trajectory controller（`restore_controller_on_exit`）→ recorder `finalize()` |

完整错误路径与代码定位见各节点文档（teleop_node.md / home_node.md / data_recorder.md）。

## 测试覆盖

- 单测 35 个覆盖全部纯逻辑模块（无 ROS）；集成 25 个以假主臂 + mock UR 全栈冒烟，覆盖两阶段启动、状态机门控（VERIFY_HOME 拒绝、Enter 门控）与 record 流程。见包根 README.md §6 与 docs/README.md。
