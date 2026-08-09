# ur_teleop 功能包重构设计

日期：2026-08-09
状态：已获用户逐节批准

## 1. 背景与目标

`ur_teleop` 是 UR10e 从臂 + Alicia-D 主臂的遥操功能包：

- 主臂 Alicia（`alicia_d_driver`）以 100 Hz 发布 `/joint_states`（`Joint1..6` 弧度 + `Gripper` 米）
- 遥操核心订阅主臂关节，映射后以 50 Hz 发布到 UR 前向 pos control（`/forward_position_controller/commands`，`Float64MultiArray` 6 维）
- 支持 teleop / record 两种模式（record 基于 lerobot 保存数据，键盘控制采集）
- 支持 sim / real 两种方式：sim = UR 端 mock + rviz；**主臂始终是真实 Alicia**
- 两阶段启动：先移动双臂到 home，再运行遥操程序；到位后等待静止、捕获 offset，按 Enter 才开始控制

本次为**重构**：保留核心能力，修复现状问题，简化框架。**不新增功能**（除修复缺陷外）。

### 允许使用的依赖包（用户指定，不得越界）

- `/ros2_ws/src/Alicia-D-ROS2`（alicia_d_driver / alicia_d_descriptions）
- `/ros2_ws/src/Universal_Robots_ROS2_Driver`（ur_robot_driver）
- `/ros2_ws/src/Universal_Robots_ROS2_Description`（ur_description）
- `/ros2_ws/src/ur10e_robotiq_ft`（URDF 集成 robotiq + FT300，已验证）
- `/ros2_ws/src/ros2_robotiq_gripper`（夹爪驱动，独立于 UR 环境）
- `/ros2_ws/src/rq_fts_ros2_driver`（FT300 驱动）
- `/ros2_ws/src/lerobot`（录制，editable 装于 /opt/lerobot_venv）

需要其他功能包必须先向用户询问。

## 2. 现状问题（重构动机，均需修复）

1. **launch 参数覆盖全部失效**：teleop_node 只声明 5 个参数，`use_mock_hardware`/`calibrate_on_start` 等覆盖无效；data_recorder 从不 declare_parameter，`repo_id/root/fps` 的 launch 覆盖永远不生效
2. **标定逻辑双份实现**：`calibration.py` 与 `teleop_node._calibrate_on_start()` 几乎逐行重复
3. **`teleop.launch.py` 引用不存在的 `ur10e_robotiq_cell` 包**，全栈 launch 无法运行
4. **标定写盘路径错误**：`_calibrate_on_start` 用 ROS 参数名查 YAML 字典，永远走硬编码路径
5. **源码树绝对路径硬编码**：默认配置文件路径全是 `/ros2_ws/src/ur_teleop/config/...`（非 symlink 安装即失效）
6. **`JointMapper.scale` 被忽略**：YAML 与文档公式都有 scale，实现只乘 sign
7. **定时器回调内阻塞**：hold 阶段 while+sleep 阻塞 1 s，单线程 executor 下夹爪 FSM、watchdog、joint_states 回调全部停摆
8. **E-stop 不冻结夹爪**：关节检查 e_stop，夹爪 FSM 不检查
9. **watchdog 不收敛**：超时只发 status，不暂停映射；主臂恢复后状态不重置
10. **嵌套 spin**：ControllerSwitcher 的 spin_until_future_complete、标定的 spin_once 循环均与外部 spin 嵌套
11. **recorder 录脏数据**：EE 位姿查不到填 0 占位；夹爪 state 用指令代理
12. **launch 重复**：teleop/calibrate 主臂三分支重复；view/display 重复；`if False` 死代码；URDF 路径硬编码
13. **配置死键**：`master_joint_state_topic`/`slave_joint_state_topic` 从未被读
14. **launch 把业务 YAML 当 ROS 参数文件传**：顶层键从未声明，刷 undeclared parameter 报错
15. **零测试**：package.xml 声明了 test_depend，无 tests 目录
16. **依赖声明缺失**：yaml、numpy、cv_bridge、lerobot（可选）未写入 package.xml

## 3. 已确认的决策（brainstorming 结论）

| 决策点 | 结论 |
|---|---|
| 夹爪跟随 | 保留（Alicia Gripper 米 → Robotiq 夹爪 action），real 模式启用，sim 模式禁用 |
| Home 编排 | **两阶段**：先 home 程序（移双臂到 home 并验证），再遥操程序（settle → offset → Enter 门控） |
| 主臂开发模式 | 只保留 real（删 fake_alicia、gui 滑动条） |
| 进程结构 | 方案 2 干净版：独立 data_recorder 进程 + 最小协议（commands/status/enable）+ 键盘单一归属 |
| 模式切换 | 配置文件 `mode: teleop|record` |
| 配置入口 | 单一 `ur_teleop.yaml`（合并现有 5 个 yaml） |
| Offset | 会话内自动捕获，不写盘（移除静态 calibration_offset.yaml 与独立 calibrate 节点） |

## 4. 总体架构

### 4.1 进程与 launch

```
┌─────────────────────────────── 阶段 1 ───────────────────────────────┐
│ home.launch.py                                                       │
│   ├── include cell.launch.py（持续运行）                               │
│   │     sim : ur_robot_driver ur10e.launch.py(mock) + rviz(裸UR臂)    │
│   │     real: ur10e.launch.py(真机) + robotiq_control + rq_fts 驱动   │
│   └── home 节点：UR 轨迹到 home + Alicia 到 home → 验证 → 打印 READY → 退出│
└──────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────── 阶段 2 ───────────────────────────────┐
│ teleop.launch.py（连接已运行的 cell）                                  │
│   ├── teleop_node（常驻）                                              │
│   └── mode=record 时：data_recorder（常驻）                            │
└──────────────────────────────────────────────────────────────────────┘
```

- cell 由阶段 1 启动并持续运行，两阶段共享同一个 controller_manager，避免 UR 栈重启丢失位姿
- `teleop.launch.py` 不带 cell：WAITING_CELL 等 `/joint_states` + `/controller_manager` services，30 s 超时明确报错"请先运行 home.launch"

### 4.2 数据流

```
Alicia ─/joint_states(Joint1-6+Gripper)→ teleop_node
Alicia ←/joint_commands、/demonstration─ teleop_node / home
teleop_node ─/forward_position_controller/commands(6维)→ UR cell
teleop_node ─ParallelGripperCommand action→ robotiq 夹爪（real 模式）
teleop_node ─/teleop/commands(7维)+ /teleop/status(Bool)→ data_recorder
data_recorder ─/teleop/enable(Bool)→ teleop_node   （record 模式键盘唯一归属）
```

### 4.3 包结构

```
ur_teleop/
├── ur_teleop/
│   ├── home_node.py          # 阶段 1：移动双臂到 home（一次性）
│   ├── teleop_node.py        # 阶段 2 核心：状态机 + 50Hz 映射 + 夹爪 FSM（编排壳，<250 行）
│   ├── data_recorder.py      # 阶段 2：lerobot 录制 + 键盘（record 模式）
│   ├── joint_mapper.py       # 纯逻辑：映射公式 + clamp（修复 scale bug）
│   ├── gripper_controller.py # 纯逻辑：迟滞 FSM（修复死代码 mark_sent）
│   ├── offset.py             # 纯逻辑：offset 捕获/应用（新抽取，双节点共用）
│   ├── controller_switcher.py# controller_manager service 封装
│   ├── config.py             # 纯逻辑：ur_teleop.yaml 解析 + 校验（缺失键报错）
│   └── keyboard.py           # 非阻塞 stdin 读取（select，teleop/record 共用）
├── launch/  cell.launch.py · home.launch.py · teleop.launch.py
├── config/  ur_teleop.yaml（单一配置入口）
└── tests/   test_joint_mapper.py · test_gripper_controller.py · test_offset.py · test_config.py
            + fake_master.py（仅集成测试用，不安装）
```

纯逻辑类（joint_mapper / gripper_controller / offset / config）**无任何 ROS 导入**，可独立单测。

## 5. teleop_node 状态机

单线程 executor，回调内无阻塞（修复问题 7）。

```
WAITING_CELL → VERIFY_HOME → SETTLING → CAPTURE_OFFSET → ARMED → ACTIVE
                                    (Enter 门控)          ↕ watchdog
                                                       INACTIVE（主臂超时）
```

1. **WAITING_CELL**：等 `/joint_states`（UR 100 Hz 出现）+ `/controller_manager` service，超时（30 s）报错退出
2. **VERIFY_HOME**：检查双臂实测位置在配置 home 容差内（`at_home_tolerance`，默认 0.05 rad）。不满足 → 打印两侧误差，等待重试，不自动继续；launch 参数 `force_home:=true` 跳过
3. **SETTLING**：静止等待 `settle_time_s`（默认 2 s，实测变化 < `settle_motion_threshold_rad`（默认 0.01）视为静止）
4. **CAPTURE_OFFSET**：捕获主臂/从臂实测值作为本会话 offset（`master_home_actual`、`slave_home_actual`），不写盘
5. **ARMED**：打印提示，等 Enter
6. **ACTIVE**：控制器切换（trajectory → forward_position）→ `/demonstration=true`（Alicia 拖拽模式）→ 50 Hz 映射发布 + 10 Hz 夹爪 FSM。`/teleop/status=true`
7. **INACTIVE**：主臂数据超时（`watchdog_timeout_s`，默认 0.5 s）→ 暂停映射（改发当前位置，避免跳变）、`/teleop/status=false`；主臂恢复自动回 ACTIVE，无需重新 Enter

注：`/teleop/e_stop` 不是独立状态 —— 它是 ACTIVE/INACTIVE 内的冻结标志（置位时定时器回调直接 return，关节指令与夹爪 FSM 均暂停）。

**退出流程**（Ctrl-C）：`/demonstration=false`（恢复力矩）→ 切回 trajectory controller（`restore_controller_on_exit`，默认 true）→ recorder `finalize()`。

## 6. Enter 门控与键盘归属

| 模式 | 键盘归属 | 第一个 Enter 的含义 |
|---|---|---|
| teleop（mode=teleop） | teleop_node 自己读 | ARMED→ACTIVE |
| record（mode=record） | **data_recorder** | 发 `/teleop/enable` → teleop_node ACTIVE；同时开始 episode 1 |

**协议**（仅 3 个 topic，无请求-应答）：
- `teleop → recorder`：`/teleop/commands`（Float64MultiArray 7 维 = 6 关节指令 + 夹爪指令）、`/teleop/status`（Bool，ACTIVE 时 true）
- `recorder → teleop`：`/teleop/enable`（Bool 边沿触发；reliable）
- 保留 `/teleop/e_stop`（Bool）：外部急停输入，冻结关节指令 + 暂停夹爪 FSM（修复问题 8）

**record 键盘语义**（沿用现有）：Enter=开始 episode（第一个同时发 enable）、`S`=保存并结束当前 episode、`D`=丢弃当前 episode（重置 buffer）、`Q`=退出并 finalize。控制在整个会话中持续，episode 边界只影响录制。`min_frames_per_episode` 以下短 episode 自动丢弃。

## 7. 组件职责

| 组件 | 做什么 | 依赖 |
|---|---|---|
| `home_node` | 等 cell 就绪 → UR 发 home 轨迹（scaled_joint_trajectory_controller action）+ Alicia 发布 home 到 `/joint_commands` → 验证到位（容差+超时）→ READY 退出 | cell、Alicia 驱动、config |
| `teleop_node` | 状态机编排：验证 home → settle → offset → Enter → 50 Hz 映射发布 | JointMapper、Offset、GripperController、ControllerSwitcher |
| `data_recorder` | 键盘 → episode 生命周期；50 Hz 帧组装（state 14 维 + action 7 维 + 相机）→ lerobot | lerobot、config |
| 纯逻辑类 | 无任何 ROS 导入，可单测 | 仅 config 数据 |

**关键实现决策**：
- `joint_mapper`：`ur_cmd[i] = slave_home[i] + sign[i]·scale[i]·(master_q[m_i] − master_home[i])`，clamp 到 `[limit_min + margin, limit_max − margin]`；scale 真正生效（修复问题 6），默认 1.0
- 控制器切换只在 ARMED→ACTIVE 边界做一次（trajectory → forward_position），带重试（5 次 + 自动 load，沿用现 controller_switcher 逻辑），用纯 service 调用 + 超时，不用嵌套 spin（修复问题 10）
- Alicia 单位转换集中在 config 层：状态侧 Gripper（米）↔ 命令侧 0–1000 反向值（stroke 按 50mm/100mm）
- 夹爪：`gripper.enabled=false`（sim 默认）时 FSM 不启动；`action_type_is_available` 探测一次，不存在则警告降级
- recorder：EE 位姿查询失败该帧 ee 段填 NaN + 一次性警告（不填 0 占位）；夹爪 state 从 UR `/joint_states` 的 `robotiq_85_left_knuckle_joint` 读取**真实状态**，action 的夹爪维用 `/teleop/commands[6]` 指令（修复问题 11 的"夹爪 state 用指令代理"）

## 8. 配置

单一 `config/ur_teleop.yaml`（合并 teleop_params / recorder_params / robotiq_gripper / joint_mapping / calibration_offset 五个 yaml；缺失键解析时报错，不用静默默认值）：

```yaml
mode: teleop            # teleop | record —— 配置文件配置模式
sim: true               # cell 端：sim(mock+rviz) / real(真机+夹爪+FT)
cell:
  ur_type: ur10e
  robot_ip: 192.168.1.1
  gripper_port: /dev/ttyUSB1     # real 模式，include robotiq_control 用
  ft_port: /dev/ttyUSB0          # real 模式，include rq_fts 驱动用
  launch_rviz: true
home:
  master: [0.0, -1.2, 0.5, 0, 0, 0]     # 目标 home（用户提前设置，本包不处理含义）
  slave:  [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
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
  limits: [[min, max]]     # 6 组弧度极限，来自 ur10e joint_limits
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
  ee_pose_source: tf             # tf | topic(/tcp_pose) | none
  cameras: {}                    # {"wrist": "/wrist_cam/image_raw", ...}
  min_frames_per_episode: 2
```

**launch 与配置的关系**：
- 三个 launch 以 `config_file` 为唯一必传参数（默认 `share/ur_teleop/config/ur_teleop.yaml`）
- cell.launch.py 显式声明 `sim`、`robot_ip`、`gripper_port`、`ft_port` 四个参数；**launch 参数优先，yaml 兜底**
- 不再存在 calibrate/view/display launch；rviz 由 cell 管理
- 模式选择：`mode: teleop|record` 决定 teleop.launch.py 是否拉起 data_recorder

## 9. 错误处理

| 场景 | 行为 |
|---|---|
| cell 未启动 | WAITING_CELL 30 s 超时打印"请先运行 home.launch"后退出（非零码） |
| 双臂不在 home | VERIFY_HOME 打印两侧误差，等待重试；`force_home:=true` 跳过 |
| UR home 轨迹失败 | home 节点打印原因 + 目标/当前关节对比，退出非零码；cell 保持运行 |
| Alicia 串口断开 | driver 自行重连；watchdog 感知 → INACTIVE（改发当前位置避免跳变） |
| 主臂数据恢复 | 自动回 ACTIVE，无需重新 Enter |
| 到位超时 | home 节点报告"未到位"，`force_home:=true` 可继续 |
| recorder 崩溃 | teleop 不受影响（进程隔离）；已保存 episodes 保留 |
| lerobot 数据集已存在 | 时间戳后缀新建 repo_id，不覆盖不追加 |
| EE 位姿查询失败 | 该帧 ee 段填 NaN + 一次性警告 |
| 夹爪 action server 不存在 | 探测一次，警告并禁用夹爪 FSM |
| e_stop 置位 | 冻结关节指令 + 夹爪 FSM 暂停；释放后从当前位恢复 |

日志纪律：状态迁移用 `[teleop] 状态: XXX` 一行式日志；错误场景给出"原因 + 下一步操作"。

## 10. 测试

**单元测试**（pytest，无 ROS 依赖，`colcon test`）：
- `test_joint_mapper.py`：映射公式（sign/scale/偏移）、clamp、关节缺失报错
- `test_gripper_controller.py`：迟滞 FSM 死区边界
- `test_offset.py`：offset 捕获 → 应用后 home 处指令 == slave 实测
- `test_config.py`：yaml 解析、缺失键报错、Gripper 单位转换

**集成测试**（`tests/fake_master.py`，仅测试用、不安装）：
- 假主臂发布 `/joint_states`（正弦波）→ cell(mock) + home + teleop → 断言 `/forward_position_controller/commands` 50 Hz 持续更新且在 safety 范围内
- 状态机覆盖：VERIFY_HOME 拒绝（主臂不在 home 时断言不进入 ACTIVE）、Enter 门控（无 Enter 不发布命令）
- record 流程：假主臂 + mock UR 录 1 个 episode 到临时 root 并 finalize

**验证命令**：`colcon test`（单测）+ 文档化冒烟流程：`home.launch` → `teleop.launch` → Enter 后 rviz 中 mock UR 跟随主臂。

## 11. 成功标准

1. `colcon build` + `colcon test` 全绿（单测）
2. sim 冒烟：home → teleop → Enter → mock UR 跟随主臂，无回调阻塞告警
3. record 冒烟：录 1 个 episode，parquet 数据 14+7 维结构与预期一致，finalize 正常
4. 问题 1–16 全部修复（配置/launch 参数链路上无失效入口）
5. 不依赖任何允许清单外的功能包
