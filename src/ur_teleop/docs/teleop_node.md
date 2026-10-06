# teleop_node（阶段 2 核心状态机）

> 路径：ur_teleop/teleop_node.py（405 行）
> 职责：双臂 home 验证 → 静止 → 捕获 offset → Enter 门控 → 50 Hz 镜像映射目标发布（ruckig_node 500 Hz 平滑下发 forward）+ 夹爪 FSM（常驻节点，Ctrl-C 退出）。

## 概述

teleop_node 是遥操核心：单线程 executor、**定时器驱动**（`_tick` 50 Hz + `_gripper_tick` 10 Hz），回调内无阻塞（重构问题 7 的修复）。8 态 FSM 覆盖从"等 cell"到"ACTIVE 镜像"的完整生命周期；控制器切换是一条**异步链**（list → load → switch），由状态机轮询 future，绝不嵌套 spin（重构问题 10 的修复）。

进程模型：阶段 1 的 cell 由 `home.launch.py` 启动并**保持运行**，teleop.launch 只连已运行的 cell（阶段 2 不重启 UR 栈，位姿不丢）。`mode=record` 时 teleop.launch 额外拉起 data_recorder，键盘归属权移交给 recorder（teleop_node 不再读 Enter）。`main()` 的启动横幅（teleop_node.py:388-391）：

Alicia 模式中，`gripper.enabled` 同时控制阶段 1 的 Robotiq 夹爪驱动和阶段 2 的夹爪跟随；`cell.ft300_enabled` 控制阶段 1 的独立 FT300 串口驱动。两项均只在对应模式需要时启动：真机按开关启动，仿真不访问 FT300 串口；仿真的夹爪 spawner 仅在前向位置控制器模式且开关开启时启动。`home.launch.py` 可用 `enable_gripper:=true|false`、`enable_ft300:=true|false` 临时覆盖启动开关；若覆盖夹爪开关，阶段 2 的 `gripper.enabled` 仍以 YAML 为准，应保持两处一致。

```text
============================================================
ur_teleop 就绪 — mode=teleop, sim=true
  等待双臂到位 → 静止 → offset → Enter 开始控制
============================================================
```

数据流（spec §4.2）：Alicia `/joint_states`（Joint1-6 + Gripper）→ teleop_node 映射 → `/ruckig/target_joint_positions`（6 维映射目标，UR 序）→ ruckig_node（500 Hz Ruckig OTG 平滑）→ `/forward_position_controller/commands` → UR cell；同时 `/teleop/commands`（7 维）+ `/teleop/status` → data_recorder；recorder 以 `/teleop/enable` 应答（record 模式键盘唯一归属）。teleop 不再直接写 forward controller——该话题由 ruckig_node 唯一发布。

## 状态机

`State` 枚举（teleop_node.py:35-43）：`WAITING_CELL / VERIFY_HOME / SETTLING / CAPTURE_OFFSET / ARMED / SWITCHING / ACTIVE / INACTIVE`。`_tick()` 按当前状态分发到对应方法（teleop_node.py:153-182）；状态迁移统一走 `_log_state()`，日志一行式：`[teleop] 状态: <NAME>`（teleop_node.py:184-186）。

| 转换 | 触发条件 | 超时 / 失败路径 |
|---|---|---|
| `WAITING_CELL → VERIFY_HOME` | 双侧 `/joint_states` 均有数据 **且** `_switcher.services_ready()`（controller_manager 三服务就绪） | 30 s 未满足 → `_fatal_error=True` + 错误日志 + `rclpy.try_shutdown()`，exit 1 |
| `VERIFY_HOME → SETTLING` | 双侧误差 ≤ `home.at_home_tolerance_rad` | 不满足：停在 VERIFY_HOME，每次 tick 打 warn 打印两侧误差；launch 参数 `force_home:=true` 把容差置 `inf` 跳过 |
| `SETTLING → CAPTURE_OFFSET` | 连续 `home.settle_time_s` 内运动 ≤ `home.settle_motion_threshold_rad` | 运动中每 tick 重置 `_settle_start`，永不满足则停在 SETTLING |
| `CAPTURE_OFFSET → ARMED` | 单 tick 内完成：`_offset.capture()` + 构造 `JointMapper` | 无（纯内存操作） |
| `ARMED → SWITCHING` | `_enable_pending` 锁存消费，或（`mode=teleop` 且键盘 Enter） | 无超时：一直等门控 |
| `SWITCHING → ACTIVE` | list→load→switch 全链成功（发布 `/demonstration=true`、`/teleop/status=true`） | 见下方"SWITCHING 失败路径" |
| `SWITCHING → ARMED` | `_switch_future is None`（客户端未就绪）/ load 响应 `ok=false` / switch 连续失败 5 次 | load 失败不计入重试计数 |
| `ACTIVE → INACTIVE` | `now − _last_master_stamp > watchdog_timeout_s`（主臂数据超时） | 进入即发布 `/teleop/status=false` |
| `INACTIVE → ACTIVE` | `now − _last_master_stamp ≤ watchdog_timeout_s`（主臂恢复） | 自动恢复，无需重新 Enter |

各状态 tick 行为：

- **WAITING_CELL**：`_start_time` 起 30 s 上限；双侧数据 + 三服务就绪即前进。超时置 `_fatal_error`（main 据此返回 exit 1）；
- **VERIFY_HOME**：锁内取双侧 q 算 `max` 误差，对比 `home.master` / `home.slave`；通过则重置 `_settle_start` 与 `_last_pose` 再进 SETTLING；
- **SETTLING**：`pose = master_q + slave_q` 拼接，逐 tick 与 `_last_pose` 比 max 运动量；运动超阈值重置计时，静止满 `settle_time_s` 前进；
- **ARMED**：先消费 `_enable_pending`，再看键盘（仅 teleop 模式）；record 模式下 `_kb` 不存在有效输入路径，完全由 enable 驱动；
- **ACTIVE / INACTIVE**：见"watchdog"节。

`/teleop/e_stop` **不是状态**：它是全机冻结标志（spec §5 注），置位时 `_tick` 与 `_gripper_tick` 都提前 return。

## 关键机制

### enable 门控 + enable_pending 锁存（最终评审修复）

`/teleop/enable` 边沿触发（teleop_node.py:139-145）：

- 收到 `data=True` 且正处于 `ARMED` → 打日志 `"[teleop] enable 收到 — 开始控制"` → `_begin_switch()`；
- 收到 `data=True` 但状态不是 ARMED → 打日志 `"[teleop] enable 已收到但状态为 <STATE>，等待 ARMED 后执行"` → 置 `_enable_pending = True` **锁存**；
- `_armed()`（teleop_node.py:234-242）先消费锁存：`_enable_pending = False` → 打日志 `"[teleop] enable 已在 ARMED 前收到 — 开始控制"` → `_begin_switch()`，之后才看键盘。

生产场景：data_recorder 在第一次 Enter 时发一次 enable（data_recorder.py:162-165，`_start_episode()` 内），此时 teleop 尚在 WAITING_CELL/VERIFY_HOME，无锁存时该信号被丢弃且 recorder 不重发 → record 会话永远进不了 ACTIVE。对应回归测试 `test_enable_single_pub_before_armed_latches`（F 组：恰好发布一次 enable 后不再重发，60 s 内必须到 ACTIVE）。失败切换不循环：SWITCHING 各失败分支都回到 ARMED（等待人工重新 enable/Enter），不会自触发重试死循环。

### watchdog（watchdog_timeout_s，默认 0.5 s）

`_last_master_stamp` 在 `_joint_cb` 收到主臂数据时更新。`_active()`（teleop_node.py:299-307）里超时 → warn `"[teleop] 主臂数据超时 → INACTIVE（改发当前位置）"` + `_publish_status(False)` + 进 INACTIVE。`_inactive()`（teleop_node.py:309-317）里主臂恢复（时间戳新鲜）→ 日志 `"[teleop] 主臂恢复 → ACTIVE"` + `_publish_status(True)` + 回 ACTIVE；未恢复则**持续发当前位置**（`_slave_q`，无数据时全 0 兜底）避免跳变。watchdog 是唯一的主臂存活信号：Alicia 串口断开由 driver 自行重连，teleop 只感知数据流。

### e_stop 冻结双 tick

`_estop_cb` 只置 `_e_stop` 标志并打 warn `[teleop] e_stop=ON/OFF`（teleop_node.py:147-149）。`_tick` 与 `_gripper_tick` 第一行都检查 `_e_stop` 提前 return —— **关节指令与夹爪 FSM 同时冻结**（重构问题 8 的修复）。释放后从当前位恢复，无特殊复位逻辑。

### CAPTURE_OFFSET：会话级 offset

`_capture_offset()`（teleop_node.py:219-232）：用当前实测值 `SessionOffset.capture(master_q, slave_q)`，然后 `JointMapper(build_mapping_config(cfg), offset.master_home, offset.slave_home)` 构造映射器，日志 `"[teleop] offset 捕获完成 master=… slave=…"`。offset **会话内一次性、不写盘**（重构决策 3）；此后 `_mapper.master_to_slave()` 的映射公式为 `ur_cmd[i] = slave_home[i] + sign[i]·scale[i]·(master_q[m_i] − master_home[i])`，clamp 到 `[limit_min + clamp_margin_rad, limit_max − clamp_margin_rad]`（`build_mapping_config`，teleop_node.py:46-54，把 `mapping` 与 `safety` 合并成 JointMapper 期望的形状）。record 模式下这里打 `"等待 recorder 的 Enter（/teleop/enable）..."`，teleop 模式打 `"按 Enter 开始控制"`。

### SWITCHING：异步链 + 异常防护 + 5 次重试

`_begin_switch()`（teleop_node.py:244-248）：`_switch_attempt=0`、`_switch_phase="list"`、发 `list_controllers()`，进 SWITCHING。`_switching()`（teleop_node.py:250-297）逐 phase 推进：

- **list**：`ControllerSwitcher.list_result(fut)` 解析（返回 `{name: state}` 字典）；**异常视为未加载**（`except: controllers = {}`）走 load 路径；`forward_position_controller` 未加载 → `load_controller(fwd)`，已加载 → 直接 switch；
- **load**：`fut.result().ok`（异常视为 False）；失败打 error `"加载 forward_position_controller 失败"` 回 ARMED，**不计入 `_switch_attempt`**（A2 断言 attempt==0）；
- **switch**：`ControllerSwitcher.switch_ok(fut)`（异常视为 False）；成功 → 日志 `"[teleop] 控制器切换完成 → ACTIVE"`、`_publish_demo(True)`、`_publish_status(True)`、进 ACTIVE；失败 → `_switch_attempt += 1`，< 5 次打 warn `"[teleop] 切换失败（第 N 次），重试..."` 重发 switch，≥ 5 次打 error `"控制器切换 5 次失败，回到 ARMED；检查 controller_manager"` 回 ARMED。

in-flight future 未完成时 `_tick` **立即返回不阻塞**（A6 断言耗时 < 0.5 s）。切换是严格模式（`strictness=STRICT`）：trajectory 控制器切出、fwd 切入一次完成。

### 退出恢复（shutdown，Ctrl-C）

`shutdown()`（teleop_node.py:360-382）：先 `_publish_demo(False)`（Alicia 恢复力矩）；若 `teleop.restore_controller_on_exit` 且状态在 `(ACTIVE, INACTIVE, SWITCHING)`，发 `switch([traj], [fwd])` 切回 trajectory 控制器——**内建一个临时 SingleThreadedExecutor 有界轮询 5 s**（spin_once 0.1 s）等结果，结果由 `switch_ok` 判定（异常视为失败），日志 `"[teleop] 退出恢复切回 trajectory controller 成功/失败/未确认"`。此时主 executor 已不在 spin（`rclpy.spin` 被 KeyboardInterrupt 打断），临时 executor 不会嵌套。客户端未就绪时 `switch()` 返回 None → 立即跳过（A15 断言 < 2 s 返回）。

### 夹爪探测一次（server_is_ready）

`_gripper_tick()`（teleop_node.py:321-335）首次运行做**一次性探测**：`_gripper_action is None` 或 `server_is_ready()` 失败 → warn `"[teleop] 夹爪 action server 不存在，禁用夹爪 FSM"` + `_gripper.enabled = False` + return；探测后置 `_gripper_probed=True`，后续 tick 直接走流程。探测失败不抛异常、不重试（A14 断言）。夹爪目标来自 `GripperController.update(_master_gripper_m)`（Alicia `Gripper` 米 → 迟滞 FSM → `GripperTarget.OPEN/CLOSED`，死区内返回 `UNKNOWN` 不动作），目标变化才发 action goal（`get_knuckle_command` rad + `max_effort`，goal 名字 `UR_GRIPPER_JOINT`），上一 goal 未完成时本 tick 跳过。

### D3：双侧 /joint_states 按侧合并

同 home_node：`_joint_cb`（teleop_node.py:124-137）按关节名子集判别归属，**逐侧更新** `_master_q`（附 `_master_gripper_m`、`_last_master_stamp`）与 `_slave_q`（附 `_slave_gripper_rad`），互不覆盖；都在 `_lock` 内更新，读侧（FSM 各方法）同样锁内取。

## 话题 / 接口

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/joint_states` | `sensor_msgs/msg/JointState` | 主/从双臂，按侧合并 |
| 订阅 | `/teleop/enable` | `std_msgs/msg/Bool` | 边沿触发（record 模式由 recorder 发布） |
| 订阅 | `/teleop/e_stop` | `std_msgs/msg/Bool` | 冻结标志：置位冻结 `_tick` + `_gripper_tick` |
| 发布 | `/ruckig/target_joint_positions` | `std_msgs/msg/Float64MultiArray` | **6 维**映射目标（UR 序），仅 ACTIVE/INACTIVE 时 50 Hz 发布；ruckig_node 据此 500 Hz 平滑下发 `/forward_position_controller/commands` |
| 发布 | `/teleop/commands` | `std_msgs/msg/Float64MultiArray` | **7 维** = 6 关节 + 夹爪指令信号（`get_gripper_command_signal`） |
| 发布 | `/teleop/status` | `std_msgs/msg/Bool` | ACTIVE=true，仅状态迁移时发一次 |
| 发布 | `/demonstration` | `std_msgs/msg/Bool` | ACTIVE 置 true（Alicia 拖拽模式），shutdown 置 false |
| Action | `<gripper.action_server>` | `control_msgs/action/ParallelGripperCommand` | 夹爪 goal（`gripper.enabled` 时创建） |
| Service | `/controller_manager/{list_controllers,load_controller,switch_controller}` | controller_manager_msgs | `ControllerSwitcher` 异步调用 |

`_publish_commands()`（teleop_node.py `_publish_commands`）双发：映射目标 6 维给 ruckig_node（`/ruckig/target_joint_positions`）；`/teleop/commands` 7 维给 recorder（第 7 维是夹爪指令信号，不参与映射）。**teleop 不再直接写 `/forward_position_controller/commands`**——该话题由 ruckig_node 唯一发布。`_publish_status` / `_publish_demo` 都是单次发布（状态迁移时发一次），集成测试用长驻 `--once` echo 才能捕获。

## 配置键

**参数**（teleop_node.py:60-62）：`config_file`（默认 `default_config_path()`）、`mode`（launch 参数**优先**、yaml `mode` 兜底：`self._mode = self.get_parameter("mode").value or cfg["mode"]`）、`force_home`（true 时 `home.at_home_tolerance_rad` 置 `float("inf")`）。

| 段 | 键 | 默认值 | 消费处 |
|---|---|---|---|
| `teleop.` | `command_rate_hz` | 50 | `_tick` 定时器频率 = 映射目标发布频率（forward 实际下发由 ruckig_node 500 Hz） |
| `teleop.` | `watchdog_timeout_s` | 0.5 | ACTIVE/INACTIVE 主臂超时判定 |
| `teleop.` | `restore_controller_on_exit` | true | `shutdown()` 是否切回 trajectory 控制器 |
| `home.` | `master` / `slave` | 必填 | VERIFY_HOME 目标 |
| `home.` | `at_home_tolerance_rad` | 0.05 | VERIFY_HOME 容差（force_home 覆盖为 inf） |
| `home.` | `settle_time_s` | 2.0 | SETTLING 静止时长 |
| `home.` | `settle_motion_threshold_rad` | 0.01 | 运动超过则重置计时 |
| `gripper.` | `enabled` / `action_server` / `close_threshold_m` / `open_threshold_m` / `open_pos_rad` / `close_pos_rad` / `max_effort` | 见 yaml | 夹爪 FSM 与 action goal（sim 默认 `enabled: false`） |
| `gripper.` | `fsm_rate_hz` | 10.0 | `_gripper_tick` 定时器频率（代码默认，yaml 未写） |
| `mapping.` | `alicia_joint_order` / `ur_joint_order` / `sign` / `scale` | 见 yaml | `build_mapping_config` 传入 JointMapper |
| `safety.` | `clamp_margin_rad` / `limits` | 见 yaml | 同上，clamp 边界 |

## 错误处理

| 场景 | 行为（日志原文） |
|---|---|
| 30 s 无 cell | `error("30 s 内未检测到 cell（/joint_states + controller_manager）。请先运行 home.launch。")` + `try_shutdown()`，exit 1 |
| 双臂不在 home | warn `"[teleop] 双臂不在 home（master err=… rad, slave err=… rad），请先运行 home.launch；确认已到位可用 force_home:=true 跳过"`，停在 VERIFY_HOME |
| 控制器切换失败 | load 失败 / switch 5 次失败回 ARMED；future 为 None 打 `"controller_manager 服务未就绪，无法切换"` |
| 主臂断流 | ACTIVE → INACTIVE（hold 当前位置 + status=false）；恢复自动回 ACTIVE |
| 夹爪 server 缺失 | 探测一次 → warn 禁用 FSM，不抛异常 |
| e_stop 置位 | 冻结双 tick；释放后从当前位恢复 |
| Ctrl-C | demo=false（恢复力矩）→ 切回 trajectory 控制器（5 s 有界等待）→ 退出码 1 iff `_fatal_error` |

## 测试覆盖（集成测试 A 组 15 项，tests/test_integration.py）

in-process 白盒用 `fsm_node` fixture（settle_time_s 加速为 0.1，不 spin 直驱）+ `_FakeFuture`（可控 done/result/exception，test_integration.py:418-443）；子进程组用 mock cell + `fake_master`（home 静止 → 收到 `/teleop/status=true` 后转正弦）。

- **A1** `test_switching_chain_list_load_switch_active`：list→load→switch→ACTIVE 全链相位顺序；
- **A2** `test_switching_load_failure_returns_to_armed`：load ok=false → ARMED 且 `_switch_attempt==0`；
- **A3** `test_switching_five_failures_caps_to_armed`：switch 连续失败 5 次 → ARMED，attempt 计数 5；
- **A4a** `test_switching_list_exception_falls_through_to_load`：list 抛异常 → 视为未加载走 load；
- **A4b** `test_switching_switch_exception_retries_then_armed`：switch 抛异常视为失败重试，异常不逃逸 `_tick`；
- **A5** `test_switching_none_future_returns_to_armed`：future=None 直接 ARMED；
- **A6** `test_switching_inflight_future_does_not_block`：in-flight 时 `_tick` 立即返回（< 0.5 s）；
- **A7** `test_teleop_alone_exits_1_without_cell`（子进程）：无 cell ~30 s 超时 exit 1 + 错误日志；
- **A8** `test_verify_home_tolerance_boundary` + `test_verify_home_force_home_skips_check`：恰好在容差通过、逐关节 tol+ε 拒绝、缺数据拒绝；force_home 置 inf 跳过；
- **A9** `test_settling_motion_resets_timer`：运动超阈值重置计时，静止满 settle 才前进；
- **A10** `test_capture_offset_builds_working_mapper`：offset 后 master home 映射 ≈ slave home、平移量守恒；
- **A11+A13** `test_watchdog_inactive_then_recover`（子进程）：杀 fake_master → status=false / 指令冻结为 hold；重启 → status=true / 指令恢复正弦；ACTIVE 时 `/demonstration=true`；
- **A12** `test_estop_freezes_then_resumes`（子进程）：e_stop ON → 指令静默（drain 后无新数据）；OFF → 恢复；
- **A14** `test_gripper_probe_disables_fsm_without_action_server`：无 action server 时首次 `_gripper_tick` 禁用 FSM，后续不抛；
- **A15** `test_shutdown_restore_without_controller_manager_is_prompt`：无 CM 时 `shutdown()` 立即返回不抛；
- **F** `test_enable_single_pub_before_armed_latches`（子进程）：ARMED 前恰好一次 enable → 锁存 → 60 s 内 ACTIVE。

另有两个全栈用例（test_integration.py:308-347）覆盖 Enter 门控与 VERIFY_HOME 拒绝：`test_enter_gate_then_mirror`（无 enable 不 ACTIVE；enable 后 50 Hz 映射目标发布到 `/ruckig/target_joint_positions`、clamp 生效、跟随正弦）与 `test_verify_home_rejects_off_home_master`（`fake_master --off-home` 时停在 VERIFY_HOME 不进 ACTIVE）。
