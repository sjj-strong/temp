# ur_teleop_rtde

Alicia-D 主臂 → UR10e 从臂遥操作功能包（ROS 2 Jazzy，ament_python），**控制走 RTDE servoJ 直连**，不依赖 UR ROS2 driver / controller_manager。

- 主臂 Alicia（`alicia_d_driver`）100 Hz 发布 `/joint_states`（`Joint1..6` 弧度 + `Gripper` 米）
- UR 侧用 **`ur-rtde` python 库**：`initPeriod() → servoJ() → waitPeriod()` 500 Hz 伺服循环（GELLO 同款参数）
- 夹爪用 **`pyrobotiqgripper` v3.2.6** 串口直连（USB-RS485），绕过 ROS2 与 RTDE
- UR 状态（关节 / TCP pose / 夹爪 0-1）通过 RTDE 读取后以 ROS 话题发布
- 支持 **teleop / record** 两种模式（record 基于 **LeRobot 数据集格式**保存，键盘控制采集）
- **两阶段启动**：先 `home.launch.py` 移双臂到 home 并验证，再 `teleop.launch.py` 运行遥操

## 1. 安装与依赖

环境要求：ROS 2 Jazzy；以下依赖功能包需已在工作区：

- `/ros2_ws/src/Alicia-D-ROS2`（alicia_d_driver，主臂）
- `/ros2_ws/src/ur10e_robotiq_ft_description`（**可选**，rviz 组合模型 UR+FT300+2F-85；未装时 cell 回退官方纯 UR 模型）
- `/ros2_ws/src/Universal_Robots_ROS2_Driver`（**可选**，仅 cell 的 rviz 显示需要 UR driver；遥操作控制走 RTDE 直连）
- `/ros2_ws/src/rq_fts_ros2_driver`（**可选**，仅真实模式 cell 需要 FT300 driver）
- `/ros2_ws/src/lerobot`（**可选**，仅 record 模式需要；editable 装于 `/opt/lerobot_venv`）

**不依赖 `ur_teleop` 包**：`JointMapper`、`SessionOffset`、`GripperController`、
`FrameBuilder`、`KeyboardReader` 与关节名常量已 **vendor 进本包**（`ur_teleop_rtde/`
下的 `joint_mapper.py`/`offset.py`/`gripper_controller.py`/`frame_builder.py`/
`keyboard.py`/`constants.py`，文件头注明来源提交，保持同步）。

Python 依赖（非 ROS 包，需 pip 安装）：

```bash
pip install ur-rtde          # RTDE 控制/读取
pip install pyrobotiqgripper # ==3.2.6，串口夹爪
```

构建：

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --packages-select ur_teleop_rtde --symlink-install
source install/setup.bash
```

record 模式需要 lerobot（editable 装于 `/opt/lerobot_venv`，含 numpy 2 兼容噪声属正常）。data_recorder 的 console script shebang 是系统 python3、看不到 lerobot，**必须用 venv 解释器以模块方式启动**：

```bash
source /opt/lerobot_venv/bin/activate
source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash
python -m ur_teleop_rtde.data_recorder
```

## 2. 两阶段启动

### 阶段 1：回 home

```bash
ros2 launch ur_teleop_rtde home.launch.py        # 默认 sim:=true（mock UR + rviz）
ros2 launch ur_teleop_rtde home.launch.py sim:=false   # 真实硬件
```

home.launch 先启动 **cell**（常驻，供阶段 2 复用）再运行 home_node：

- **cell**（`cell.launch.py`）：UR driver（`ur_control.launch.py`，sim 用 mock / 真实用真机）+ FT300 driver（`ft_sensor_standalone.launch.py`，仅真实模式）+ rviz（组合模型 `ur10e_robotiq_ft_description`）。控制不经过 cell——它只为 rviz 提供 `/robot_description`、`/joint_states` 与 TF。
- **不含 robotiq driver**：夹爪由 teleop_node 的 `pyrobotiqgripper` 独占串口（`/dev/ttyUSB1`）；且 robotiq driver 自带的 robot_state_publisher 会以 world 为根重复发布组合模型已有的 `robotiq_85_*` 帧（TF 冲突）。需要 action server 调试时单独启动 `ros2 launch robotiq_description robotiq_control.launch.py com_port:=/dev/ttyUSB1`，但此时不能同时运行 teleop 的夹爪串口控制。
- home_node：UR 侧 RTDE `moveJ` 以 `home.move_speed_rad_s` / `home.move_accel_rad_s2` 移动到 `home.slave`；Alicia 侧持续发布 `/joint_commands` 使其到达 `home.master`
- 双臂均到位后打印 **HOME REACHED** 并退出；cell 保持运行，之后进入阶段 2

### 阶段 2：遥操作

```bash
# 纯遥操作
ros2 launch ur_teleop_rtde teleop.launch.py mode:=teleop

# 遥操作 + LeRobot 数据录制（另开终端，用 venv 解释器启动 recorder；
# launch 里的 recorder 入口会因 shebang 缺 lerobot 报错退出，属预期行为）
ros2 launch ur_teleop_rtde teleop.launch.py mode:=record
source /opt/lerobot_venv/bin/activate && source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash
python -m ur_teleop_rtde.data_recorder
```

启动后 teleop_node 经历 FSM：`INIT → VERIFY_HOME → SETTLING → CAPTURE_OFFSET → ARMED → ACTIVE`：

1. 等待 Alicia `/joint_states` + RTDE 连通（30 s 超时）
2. 校验双臂在 home（`at_home_tolerance_rad`；已确认到位可用 `force_home:=true` 跳过）
3. 静止 `settle_time_s` 秒后捕获映射 offset
4. **按 Enter**（或 recorder 在开始 episode 时自动发 `/teleop/enable`）→ 进入 ACTIVE，servoJ 开始跟手

`mode` 缺省读 config 里 `mode:` 字段，launch 参数可覆盖。

## 3. 键盘控制

| 按键 | 上下文 | 作用 |
|------|--------|------|
| Enter | teleop_node（ARMED） | 开始控制（进入 ACTIVE） |
| Enter | data_recorder | 开始新 episode 并自动发 `/teleop/enable` |
| S | data_recorder | 保存当前 episode（`save_episode()`） |
| D | data_recorder | 丢弃当前 episode（`clear_episode_buffer()`） |
| Q | data_recorder | 保存当前 episode + `finalize()` 数据集并退出 |
| Ctrl-C | 全部 | 优雅退出：`servoStop()` → 断开 RTDE/串口 → `/demonstration=False` |

## 4. 话题一览

### 订阅

| 话题 | 类型 | 来源 | 用途 |
|------|------|------|------|
| `/joint_states` | `JointState` | Alicia-D | 主臂 6 关节 + 夹爪位置（米） |
| `/teleop/enable` | `Bool` | data_recorder / 外部 | ARMED → ACTIVE 触发 |
| `/teleop/e_stop` | `Bool` | 外部安全 | 冻结关节指令与夹爪 FSM |

### 发布

| 话题 | 类型 | 频率 | 用途 |
|------|------|------|------|
| `/ur_teleop_rtde/ur_joints` | `JointState` | 50 Hz | UR 实际关节（RTDE `getActualQ`） |
| `/ur_teleop_rtde/tcp_pose` | `PoseStamped` | 50 Hz | UR TCP pose（RTDE `getActualTCPPose`，RPY→四元数） |
| `/ur_teleop_rtde/gripper_state` | `Bool` | 50 Hz | **True=开, False=合**（夹爪实际位置 >0.5 或滞回 FSM 目标） |
| `/ur_teleop_rtde/status` | `Bool` | 变化时 | True 表示 ACTIVE 控制中 |
| `/ur_teleop_rtde/target_joints` | `Float64MultiArray` | 50 Hz | 当前发给 servoJ 的 6 维目标 |
| `/teleop/commands` | `Float64MultiArray` | 100 Hz | 7 维：6 关节指令 + 夹爪命令信号 |
| `/demonstration` | `Bool` | 变化时 | Alicia 拖拽模式开关（进入 ACTIVE=True） |
| `/joint_states` | `JointState` | 50 Hz | 夹爪实时位置（仅 `robotiq_85_left_knuckle_joint`，rsp 按关节名与 UR/Alicia 发布合并 → rviz 夹爪动画） |

## 5. 配置参考：`config/ur_teleop_rtde.yaml`

单一配置入口（`ur_teleop_rtde/config.py` 启动时校验，必需键缺失/越界报错）。

| 键 | 默认 | 说明 |
|----|------|------|
| `mode` | `teleop` | `teleop` / `record` |
| `robot.robot_ip` | `192.168.1.10` | UR 控制器 IP（RTDE 端口 5900/5901） |
| `cell.sim` | `true` | cell 启动模式：`true`=mock UR driver（离线 rviz），`false`=真实 UR driver + FT300 |
| `cell.launch_rviz` | `true` | cell 是否启动 rviz（组合模型） |
| `cell.ftdi_id` | `""` | FT300 USB-FTDI 标识（`""` = 自动探测） |
| `home.master` / `home.slave` | 见 yaml | 双臂 home 位姿（6 关节 rad） |
| `home.at_home_tolerance_rad` | `0.05` | home 校验容差 |
| `home.settle_time_s` | `2.0` | 静止等待时长 |
| `home.move_speed_rad_s` / `move_accel_rad_s2` | `1.0` / `1.0` | RTDE moveJ 速度/加速度 |
| `mapping.safety.clamp_margin_rad` | `0.1` | 关节限位夹取裕量 |
| `mapping.safety.limits` | 见 yaml | 6 关节硬限位（越界 clamp） |
| `rtde.servoj_dt` | `0.002` | servoJ 周期（500 Hz） |
| `rtde.servoj_lookahead` | `0.2` | 取值范围 `[0.03, 0.2]` |
| `rtde.servoj_gain` | `100` | 取值范围 `[100, 2000]` |
| `rtde.state_read_every_n` | `5` | 每 N 个 servoJ 周期读一次实际状态（→100 Hz） |
| `teleop.command_rate_hz` | `100` | 主循环频率（对齐 Alicia 发布率） |
| `teleop.watchdog_timeout_s` | `0.5` | 主臂数据超时 → SAFETY_HOLD |
| `gripper.com_port` | `/dev/ttyUSB1` | Robotiq USB-RS485 串口 |
| `gripper.speed` / `gripper.force` | `255` / `50` | 夹爪开合速度 / 闭合力（0-255） |
| `gripper.close_threshold_m` / `open_threshold_m` | `0.0125` / `0.005` | Alicia 夹爪位置滞回阈值（米） |
| `recorder.repo_id` | `my_user/ur_teleop_rtde` | LeRobot 数据集 repo_id |
| `recorder.root` | `""` | `""` = `$HF_LEROBOT_HOME/{repo_id}`，或绝对路径 |
| `recorder.fps` | `50` | 录制帧率 |
| `recorder.use_videos` | `true` | MP4 视频 / PNG 图像 |
| `recorder.min_frames_per_episode` | `2` | 少于该帧数的 episode 自动丢弃 |
| `recorder.task` | `teleoperation` | 每帧的 task 标签 |
| `recorder.cameras` | `{}` | 可选相机：`{name: {topic, image_key, height, width}}` |

## 6. 数据格式（record 模式）

LeRobot 数据集（v0.5.1 API，`FrameBuilder` 为本包 vendored 自 ur_teleop 的副本）：

- `observation.state`：`float32[14]` = 6 UR 关节 + 7 EE pose `[x,y,z,qx,qy,qz,qw]` + 1 夹爪（1=开, 0=合）
- `action`：`float32[7]` = 6 关节指令 + 1 夹爪命令信号
- `task`：字符串标签

EE pose 直接读 RTDE `getActualTCPPose()`（无需 TF 查找）。夹爪 0/1 由 `gripper_state` Bool 转换（阈值 `state_threshold_rad: 0.4`）。

## 7. 安全设计

- **SAFETY_HOLD**：主臂数据超时（`watchdog_timeout_s`）或 RTDE 循环异常时保持最后位置并停止跟手；主臂数据恢复或重连成功后自动回到 ACTIVE
- **关节限位**：`mapping.safety.limits` 硬限位 + `clamp_margin_rad` 裕量，映射输出越界即 clamp
- **E-stop**：`/teleop/e_stop`（Bool True）冻结关节指令与夹爪 FSM
- **偏移捕获**：进入 ACTIVE 前必须静止捕获 offset，避免启动瞬间跳变
- **servoJ 参数**：`lookahead_time=0.2, gain=100` 为 GELLO 验证过的安全组合；`gain` 增大可提升跟随但更激进

## 8. 测试

```bash
cd /ros2_ws/src/ur_teleop_rtde
/usr/bin/python3 -m pytest tests/ -q   # 23 tests：config 校验 / RTDE mock 循环 / LeRobot mock 生命周期
```

注意用系统 python3（非 lerobot venv）：venv 内 pytest 与 launch_testing 插件冲突。

## 9. 与 ur_teleop 的差异

| | ur_teleop | ur_teleop_rtde |
|---|---|---|
| UR 控制接口 | ROS2 `forward_position_controller` | **RTDE servoJ 直连**（`ur-rtde`） |
| UR 状态来源 | UR ROS2 driver 话题 / TF | **RTDE `getActualQ` / `getActualTCPPose`** |
| 夹爪 | robotiq 动作服务器 | **pyrobotiqgripper 串口直连** |
| 控制器切换 | controller_manager 8 态 FSM | 无（6 态 FSM：INIT→VERIFY_HOME→SETTLING→CAPTURE_OFFSET→ARMED→ACTIVE） |
| 通用模块 | — | **vendor 自 ur_teleop**（不 import 该包）：JointMapper / SessionOffset / GripperController / FrameBuilder / KeyboardReader / 常量 |
| cell 组成 | UR driver + robotiq driver + FT300 + rviz | **UR driver + FT300 + rviz**（robotiq driver 不含——串口被 pyrobotiqgripper 独占，且其 rsp 与组合模型 TF 冲突） |
