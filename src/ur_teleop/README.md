# ur_teleop

UR10e 从臂 + Alicia-D 主臂遥操作功能包（ROS 2 Jazzy，ament_python）。

- 主臂 Alicia（`alicia_d_driver`）100 Hz 发布 `/joint_states`（`Joint1..6` 弧度 + `Gripper` 米）
- 遥操核心订阅主臂关节，映射后以 50 Hz 发布映射目标；`ruckig_node` 以 **500 Hz**（与 controller_manager 同频）jerk-limited 平滑后下发 UR 前向 pos control（`/forward_position_controller/commands`，6 维）
- 支持 **teleop / record** 两种模式（record 基于 lerobot 保存数据，键盘控制采集）
- 支持 **sim / real** 两种方式：sim = UR 端 mock + rviz；**主臂始终是真实 Alicia**
- **两阶段启动**：先 `home.launch.py` 移双臂到 home 并验证，再 `teleop.launch.py` 运行遥操；到位后静止等待、捕获 offset，按 Enter 才开始控制

## 1. 安装与依赖

环境要求：ROS 2 Jazzy；以下依赖功能包需已在工作区：

- `/ros2_ws/src/Alicia-D-ROS2`（alicia_d_driver / alicia_d_descriptions）
- `/ros2_ws/src/Universal_Robots_ROS2_Driver`（ur_robot_driver）
- `/ros2_ws/src/Universal_Robots_ROS2_Description`（ur_description）
- `/ros2_ws/src/ur10e_robotiq_ft_description`（URDF 集成 robotiq + FT300）
- `/ros2_ws/src/ros2_robotiq_gripper`（夹爪驱动，real 模式）
- `/ros2_ws/src/rq_fts_ros2_driver`（FT300 驱动，real 模式）
- `/ros2_ws/src/lerobot`（**可选**，仅 record 模式需要；editable 装于 `/opt/lerobot_venv`）

本包自身依赖见 `package.xml`（rclpy、sensor_msgs、std_msgs、control_msgs、controller_manager_msgs、geometry_msgs、trajectory_msgs、tf2_ros、tf2_geometry_msgs、python3-yaml、python3-numpy、cv_bridge；lerobot 可选）。安装后可 `rosdep install` 自动补齐。

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --packages-select ur_teleop --symlink-install
source install/setup.bash
```

record 模式使用前需激活 lerobot 虚拟环境（含 `source /opt/lerobot_venv/bin/activate` 后的 shell 会话中启动，纯逻辑模块本身不依赖 lerobot）：

```bash
source /opt/lerobot_venv/bin/activate
```

## 2. 关键配置：`config/ur_teleop.yaml`

单一配置入口（合并了旧版五个 yaml）。必需键缺失解析时报错；可选键带默认值（见下表默认列）。launch 参数优先、yaml 兜底。各键含义：

| 键                                                                              | 含义                                                                                                                                 |
| ------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `mode`                                                                        | `teleop` \| `record` —— 配置文件的模式；launch 参数 `mode:=` 可覆盖                                                          |
| `sim`                                                                         | cell 端开关：`true` = mock UR + rviz；`false` = 真机 + 夹爪 + FT300                                                              |
| `cell.ur_type`                                                                | `ur10e`                                                                                                                            |
| `cell.robot_ip`                                                               | 真机 IP（real 模式），launch 参数`robot_ip:=` 优先                                                                                 |
| `cell.gripper_port`                                                           | Robotiq 夹爪串口（real 模式，如`/dev/ttyUSB1`）                                                                                    |
| `cell.ftdi_id`                                                                | rq_fts 驱动 FT300 的 ftdi_id（real 模式；launch 参数即`ftdi_id`）                                                                  |
| `cell.launch_rviz`                                                            | cell 是否拉起 rviz                                                                                                                   |
| `home.master` / `home.slave`                                                | 主臂 / 从臂目标 home 位姿（弧度 6 维，用户按现场设置）                                                                               |
| `home.master_gripper_value`                                                   | Alicia 夹爪 home 指令（0–1000 反向值）                                                                                              |
| `home.at_home_tolerance_rad`                                                  | 到位容差（默认 0.05 rad），VERIFY_HOME 判定用                                                                                        |
| `home.settle_time_s` / `home.settle_motion_threshold_rad`                   | 静止等待时长 / 静止判据（实测变化 < 阈值）                                                                                           |
| `home.move_timeout_s` / `home.move_duration_s` / `home.verify_duration_s` | home 轨迹超时 / 轨迹时长 / 到位验证观察时长                                                                                          |
| `mapping.alicia_joint_order`                                                  | 主臂关节名顺序（`Joint1..Joint6`）                                                                                                 |
| `mapping.ur_joint_order`                                                      | 从臂关节名顺序（`shoulder_pan_joint` 等 6 个）                                                                                     |
| `mapping.sign` / `mapping.scale`                                            | 每关节符号 / 缩放（映射公式中真正生效）                                                                                              |
| `safety.clamp_margin_rad`                                                     | 关节极限 clamp 的安全余量                                                                                                            |
| `safety.limits`                                                               | 关节名键值 dict（`[min, max]` 弧度，来自 ur10e joint_limits）                                                                      |
| `teleop.command_rate_hz`                                                      | 映射目标发布频率（默认 50 Hz；forward 实际下发由 ruckig 500 Hz 平滑）                                                                |
| `teleop.watchdog_timeout_s`                                                   | 主臂数据超时（默认 0.5 s）→ INACTIVE 暂停映射                                                                                       |
| `teleop.restore_controller_on_exit`                                           | 退出时是否切回 trajectory controller（默认 true）                                                                                    |
| `gripper.enabled`                                                             | 夹爪跟随开关：sim 默认`false`，real 设 `true`                                                                                    |
| `gripper.action_server`                                                       | Robotiq 夹爪 action 名（`/robotiq_gripper_controller/gripper_cmd`）                                                                |
| `gripper.close_threshold_m` / `open_threshold_m`                            | 夹爪 FSM 迟滞死区阈值（Alicia Gripper 米）                                                                                           |
| `gripper.open_pos_rad` / `close_pos_rad` / `max_effort`                   | 夹爪开/合目标与最大力矩                                                                                                              |
| `recorder.repo_id`                                                            | 数据集 id（如`my_user/ur_teleop`）                                                                                                 |
| `recorder.root`                                                               | 数据集根目录，空 =`HF_LEROBOT_HOME`                                                                                                |
| `recorder.fps`                                                                | 录制帧率（默认 50）                                                                                                                  |
| `recorder.robot_type`                                                         | lerobot 数据集 robot 类型                                                                                                            |
| `recorder.use_videos`                                                         | 相机是否以 video 编码（true）或逐帧 image（false）                                                                                   |
| `recorder.ee_pose_source`                                                     | EE 位姿来源：`tf` \| `topic`(/tcp_pose) \| `none`；配合 `ee_pose_topic` / `ee_pose_parent_frame` / `ee_pose_child_frame` |
| `recorder.cameras`                                                            | 相机映射`{"wrist": {topic, image_key, height, width}, ...}`                                                                        |
| `recorder.task`                                                               | lerobot 任务名（默认`teleoperation`）                                                                                              |
| `recorder.min_frames_per_episode`                                             | 低于此帧数的短 episode 自动丢弃                                                                                                      |
| `recorder.state_threshold_rad`                                                | robotiq 夹爪 rad → 0/1 state 的阈值（默认 0.4）                                                                                     |

## 3. 两阶段使用流程

**阶段 1**（home.launch.py）：启动 cell（持续运行）+ 移双臂到 home 并验证 → 打印 HOME REACHED 后退出（cell 保持运行）。
**阶段 2**（teleop.launch.py）：连接已运行的 cell，teleop_node 状态机 WAITING_CELL → VERIFY_HOME → SETTLING → CAPTURE_OFFSET → ARMED（Enter 门控）→ ACTIVE；同时启动 `ruckig_node`（500 Hz 平滑映射目标后下发 `/forward_position_controller/commands`）；`mode=record` 时额外拉起 data_recorder。

### sim（主臂真实，UR 端 mock + rviz）

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch ur_teleop home.launch.py
# 看到 HOME REACHED 后（cell 仍在运行）开第二个终端：
ros2 launch ur_teleop teleop.launch.py
# 状态到 ARMED 后按 Enter → rviz 中 mock UR 跟随主臂
```

rviz 显示模型：**默认组合模型**（`ur10e_robotiq_ft_description` 包，UR + FT300 + Robotiq 2F-85 完整装配，含 gripper_tcp 参考帧）——ur 官方 `ur_description` 只有纯 UR。组合模型的 xacro 复用官方 `ur_ros2_control` 宏，sim 下带 mock 硬件（含夹爪关节），real 下带真机插件；`ur10e_robotiq_ft_description` 未安装时自动回退官方纯 UR 模型。切换方式与细节见 `docs/launch.md`「rviz 模型：URDF 描述文件来源」。

### real（真机）

```bash
ros2 launch ur_teleop home.launch.py sim:=false robot_ip:=<你的UR-IP>
# HOME REACHED 后第二个终端：
ros2 launch ur_teleop teleop.launch.py
```

> `sim` 切换到 false 后`cell.launch.py` 会走真机分支：UR 用官方默认 bare 模型(真机驱动 + recipe 文件路径内置)，夹爪/FT300 各自启动独立的 controller_manager。
> `robot_ip`/`gripper_port`/`ftdi_id`/`launch_rviz` 在 `home.launch.py` 声明，launch 参数优先、yaml 兜底。夹爪和 FT300 的端口如果 yaml 已配置正确则无需传参。

### record 模式

```bash
# 需先激活 lerobot venv：
source /opt/lerobot_venv/bin/activate
ros2 launch ur_teleop teleop.launch.py mode:=record
```

- `mode:=` 是 teleop.launch.py 参数，优先于 yaml 的 `mode` 键。
- record 模式下键盘归属 data_recorder，按 Enter 同时发送 `/teleop/enable` 开始控制并开始 episode 1。

### 通用参数

- `config_file:=<路径>`：三个 launch 都有，默认 `share/ur_teleop/config/ur_teleop.yaml`。
- `force_home:=true`（teleop.launch.py）：跳过 VERIFY_HOME（双臂不在 home 容差内时跳过验证直接继续；home 轨迹未到位时也可用）。

退出：Ctrl-C。退出流程自动执行 `/demonstration=false`（恢复力矩）→ 切回 trajectory controller（`restore_controller_on_exit`）→ recorder `finalize()`。

## 4. 键盘键位

| 模式                  | 键盘归属      | 键    | 含义                                                                         |
| --------------------- | ------------- | ----- | ---------------------------------------------------------------------------- |
| teleop（mode=teleop） | teleop_node   | Enter | ARMED → ACTIVE（开始映射控制；teleop 50 Hz 喂目标，ruckig 500 Hz 平滑下发） |
| record（mode=record） | data_recorder | Enter | 开始 episode 1 并发送`/teleop/enable`（teleop_node 进入 ACTIVE）           |
| record                | data_recorder | `S` | 保存并结束当前 episode                                                       |
| record                | data_recorder | `D` | 丢弃当前 episode（重置 buffer）                                              |
| record                | data_recorder | `Q` | 退出并 finalize 数据集                                                       |

record 模式下控制在整个会话中持续，episode 边界只影响录制；低于 `min_frames_per_episode` 的短 episode 自动丢弃。外部急停输入 `/teleop/e_stop`（Bool，置位冻结关节指令与夹爪 FSM）。

## 5. record 数据格式（LeRobot Dataset）

每帧由 FrameBuilder 组装（见 `ur_teleop/frame_builder.py`）：

- **observation.state：14 维 float32** = 6 个 UR 关节（按 `mapping.ur_joint_order`，实测值） + 7 维 EE 位姿 + 1 维夹爪 state。
  - state 特征名：`[shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint, ee_x, ee_y, ee_z, ee_qx, ee_qy, ee_qz, ee_qw, gripper_state]`
  - EE 位姿查询失败时该帧 ee 段填 **NaN**（不填 0 占位）+ 一次性警告。
  - `gripper_state` 为 0/1（读 UR `/joint_states` 的 `robotiq_85_left_knuckle_joint` 真实状态，按 `recorder.state_threshold_rad` 阈值二值化）。
- **action：7 维 float32** = 6 个关节指令 + 1 维夹爪指令（取 `/teleop/commands[6]` 指令信号）。
  - action 特征名：`[cmd_shoulder_pan_joint, cmd_shoulder_lift_joint, cmd_elbow_joint, cmd_wrist_1_joint, cmd_wrist_2_joint, cmd_wrist_3_joint, cmd_gripper]`
- **相机**（可选，`recorder.cameras` 配置）：`observation.images.<image_key>`，`use_videos: true` 时以 video 编码。

数据流（record 模式）：teleop_node → `/teleop/commands`（7 维）+ `/teleop/status`（Bool）→ data_recorder；data_recorder → `/teleop/enable`（Bool 边沿触发）→ teleop_node。数据集已存在时以时间戳后缀新建 repo_id，不覆盖不追加。

## 6. 测试

**单元测试**（35 个，无 ROS 依赖）：

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon test --packages-select ur_teleop --python-testing pytest --event-handlers console_direct+
# 期望 35 passed
```

**集成测试**（25 个，全栈冒烟：假主臂 + mock UR + home + teleop + record，约 5 分钟）。需已 source ROS 环境与工作区 install（套件 import `ur_teleop.config`），并激活 lerobot venv。隔离机制：模块级 `PYTHONUNBUFFERED` 保证子进程日志逐行可达；`tmp_path` 隔离配置文件与按路径进程清扫；孤儿 `controller_manager` 清扫。**必须设独立 `ROS_DOMAIN_ID`**——套件内所有节点共享该 domain，若与本机常驻的 mock/真机栈同域，外来 `/joint_states` 会与套件假主臂/mock UR 交织，导致 home 验证超时等假失败：

```bash
export ROS_DOMAIN_ID=77   # 与机器上常驻 ROS 栈隔离，任选空闲 id
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
source /opt/lerobot_venv/bin/activate
cd /ros2_ws
python3 -m pytest src/ur_teleop/tests/test_integration.py -m integration -p no:launch_testing -p no:launch_ros
# 期望 25 passed
```

## 7. 重构问题 → 修复对照表

| #  | 原问题（spec §2）                                     | 本包如何解决                                                                                                                                        |
| -- | ------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1  | launch 参数覆盖全部失效                                | 每个节点`declare_parameter` + launch `parameters` 传参；单一 `ur_teleop.yaml`；launch 参数优先、yaml 兜底（`config.py` + 三 launch）        |
| 2  | 标定逻辑双份实现                                       | 删除`calibration.py`；offset 改会话内自动捕获（`offset.py`，teleop/home 共用，无重复）                                                          |
| 3  | teleop.launch.py 引用不存在的`ur10e_robotiq_cell` 包 | 重写三 launch：`cell.launch.py`（引真实 `ur10e_robotiq_ft_description`）/ `home.launch.py` / `teleop.launch.py`                             |
| 4  | 标定写盘路径错误                                       | offset 不写盘（会话内捕获后直接应用）                                                                                                               |
| 5  | 源码树绝对路径硬编码                                   | setup.py`data_files` 安装 + 默认 `share/ur_teleop/config/ur_teleop.yaml`（`config.py::default_config_path`）                                  |
| 6  | `JointMapper.scale` 被忽略                           | `joint_mapper.py` 公式 `ur_cmd = slave_home + sign·scale·(master − master_home)`，scale 真正生效，clamp 到 `[limit+margin, limit−margin]` |
| 7  | 定时器回调内阻塞                                       | teleop_node 单线程 executor，回调无阻塞；键盘用 select 非阻塞读（`keyboard.py`）                                                                  |
| 8  | E-stop 不冻结夹爪                                      | `/teleop/e_stop` 冻结标志同时暂停关节指令与夹爪 FSM                                                                                               |
| 9  | watchdog 不收敛                                        | INACTIVE 状态：超时暂停映射（改发当前位置避免跳变），主臂恢复自动回 ACTIVE，无需重新 Enter                                                          |
| 10 | 嵌套 spin                                              | `controller_switcher.py` 纯 service 调用 + 超时（5 次重试 + 自动 load），无嵌套 spin                                                              |
| 11 | recorder 录脏数据                                      | `frame_builder.py`：EE 查询失败填 NaN + 一次性警告（不填 0）；夹爪 state 读 UR 真实 knuckle 关节，action 夹爪维用 `/teleop/commands[6]` 指令    |
| 12 | launch 重复                                            | 三 launch 精简：cell 一次管理 rviz/URDF/控制器；删除 view/display/calibrate launch 与`if False` 死代码                                            |
| 13 | 配置死键                                               | 单一 yaml 只含被消费键（`master_joint_state_topic` 等已删）；`config.py` 缺失键解析报错                                                         |
| 14 | launch 把业务 YAML 当 ROS 参数文件传                   | launch 仅传`config_file` 字符串参数，节点内 `load_config` 自行解析                                                                              |
| 15 | 零测试                                                 | `tests/`：35 单测（joint_mapper / gripper_controller / offset / config / frame_builder / keyboard）+ 25 集成（`fake_master.py` 假主臂全栈冒烟） |
| 16 | 依赖声明缺失                                           | `package.xml` depend 补全 yaml / numpy / cv_bridge / tf 等；lerobot 以注释说明可选（record 模式需要，`/opt/lerobot_venv`）                      |
