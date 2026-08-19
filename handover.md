# UR Teleop Handover — 2026-08-12

## 概述

Alicia-D 主臂 → UR10e 从臂遥操作系统，含 Robotiq 2F-85 夹爪 + FT300 力传感器。
通过 `ur_teleop` 包实现主从映射、轨迹平滑（Ruckig）、数据录制（LeRobot）。

## 启动流程

```bash
# === 前置：清孤儿进程 ===
pkill -9 ros2_control_node; sleep 1

# === 阶段 1：启动 cell（持久） + 回 home ===
ros2 launch ur_teleop home.launch.py sim:=false robot_ip:=169.254.138.15
# HOME REACHED 后，home_node 自动退出，cell 保持运行

# === 阶段 2：遥操作 ===
ros2 launch ur_teleop teleop.launch.py
# 按 Enter 开始控制
```

**注意**：home.launch.py 退出后 `ros2_control_node` 仍然在后台运行（cell 持久）。
下次重跑前必须 `pkill -9 ros2_control_node`，否则会孤儿化冲突。

## 架构

```
home.launch.py
├── cell.launch.py (persistent)
│   ├── ur_control.launch.py [UR driver]
│   │   ├── ros2_control_node (UR controller_manager)
│   │   ├── ur_rsp.launch.py → robot_state_publisher (/robot_description)
│   │   ├── controller_stopper_node (管理控制器激活/停用)
│   │   ├── urscript_interface
│   │   └── spawners: scaled_joint_trajectory_controller (active) + 其他
│   ├── robotiq_control.launch.py [real 模式]
│   │   ├── ros2_control_node → robotiq_controller_manager (/robotiq_robot_description)
│   │   └── robot_state_publisher → /robotiq_robot_description (remapped)
│   ├── ft_sensor_standalone.launch.py [real 模式]
│   ├── alicia_d_driver.launch.py
│   └── rviz2 [sim 模式]
└── home_node (one-shot, 到位后退出)
    └── /scaled_joint_trajectory_controller/follow_joint_trajectory

teleop.launch.py
├── teleop_node (主从映射 + 状态机)
├── ruckig_node (jerk-limited 平滑，500 Hz)
└── data_recorder (record 模式)
    └── /forward_position_controller/commands (ruckig 输出)
```

## 控制器切换流程

```
home_node 阶段:
  scaled_joint_trajectory_controller → active
  forward_position_controller       → inactive

home_node 退出后:
  controller_stopper_node 将 scaled_joint_trajectory_controller → inactive
  (因为它不在 consistent_controllers 列表中)

teleop 启动后:
  teleop_node 发起 STRICT switch:
    activate:   forward_position_controller
    deactivate: scaled_joint_trajectory_controller (如果还 active)
  → ruckig_node 通过 /forward_position_controller/commands 下发平滑轨迹
```

## 关键配置 (ur_teleop.yaml)

| 段 | 关键参数 | 当前值 | 说明 |
|---|---|---|---|
| `sim` | `sim` | `true` | yaml 兜底；launch arg `sim:=false` 覆盖 |
| `cell` | `robot_ip` | `169.254.138.15` | UR 控制器 IP |
| `cell` | `ftdi_id` | `ttyUSB3` | FT300 串口 |
| `cell` | `gripper_port` | `/dev/ttyUSB1` | 夹爪串口 |
| `home` | `slave` | 6 关节 rad 值 | **UR home 目标——必须与当前物理位姿接近** |
| `home` | `at_home_tolerance_rad` | `0.1` | 对齐控制器 goal tolerance |
| `home` | `move_timeout_s` | `60.0` | 含轨迹执行时间 |
| `mapping` | `sign` | `[1,-1,-1,-1,-1,-1]` | Alicia→UR 关节方向映射 |
| `mapping` | `scale` | `[0.8...]` | 运动比例 |
| `ruckig` | `control_hz` | `500.0` | 与 controller_manager update_rate 一致 |
| `teleop` | `command_rate_hz` | `500` | 遥操作发布频率 |

## 本次 session 修复的问题

### 1. real 模式 ros2_control_node crash（recipe 缺失）
- **现象**：`URPositionHardwareInterface` 报 "Neither output recipe file nor output recipe have been defined"，进程 abort
- **根因**：`cell.launch.py` 中 `DeclareLaunchArgument("description_launchfile")` 与 `ur_control.launch.py` 内部同名参数冲突，sim 分支的 `rsp_mock.launch.py` 值漏到 real 分支。`rsp_mock.launch.py` 没传 recipe 参数。
- **修复**：
  - `cell.launch.py`: 重命名为 `description_sim`，real 分支显式传 `ur_rsp.launch.py`
  - `rsp_mock.launch.py`: 补齐 `script_filename`/`output_recipe_filename`/`input_recipe_filename`
  - `home.launch.py`: 移除已迁移的 `_description_launchfile()` 和参数声明
- **Commits**: 79a9b5e, eec4efc, 40d182e

### 2. 删除冗余 gripper_sim_controller.yaml
- **根因**：与 `ur_controllers_sim.yaml` 中 `robotiq_gripper_controller` 段 100% 重复
- **Commit**: 1319ccf

### 3. home_node 到位超时
- **现象**：robot_mode=7 (RUNNING)，控制器 active，轨迹发出，但 home_node 超时
- **根因**：
  - `at_home_tolerance_rad: 0.05` 严于控制器 `goal: 0.1`，控制器报告 SUCCESS 后 home_node 仍验证失败
  - `move_timeout_s: 30` 不够：`scaled_joint_trajectory_controller` 速度约束 0.2 rad/s，大跨度位移需 25s+
  - **更根本的问题**：UR 当前 wrist_3 在 -1.72 rad，home 目标在 +2.98 rad，差 4.7 rad (269°)——必须物理挪近
- **修复**：`at_home_tolerance_rad` → 0.1，`move_timeout_s` → 60.0
- **Commit**: c85fc5d

### 4. teleop_node 控制器切换失败
- **现象**：`controller_manager` 报 "scaled_joint_trajectory_controller can not be deactivated since it is not active"，STRICT switch abort
- **根因**：`controller_stopper_node` 在 home_node 退出后已将 trajectory controller 切为 inactive，teleop_node 仍尝试 deactivate 它
- **修复**：switch 前检查实际状态，只 deactivate 真正 active 的控制器；重试时重新查询
- **Commit**: e25be0a

### 5. record 模式键盘无响应
- **现象**：teleop.launch mode:=record 启动后按 Enter 无反应，teleop_node 一直停在 ARMED 等 `/teleop/enable`
- **根因**：data_recorder 进程启动即崩溃——`ros2 launch` 用 `/usr/bin/python3` 启动节点（shebang），lerobot 只装在 `/opt/lerobot_venv` → import 失败 → RuntimeError。recorder 拥有键盘，它死了键盘自然无人处理
- **修复**：`data_recorder.py` 的 `main()` 检测到 `LeRobotDataset is None` 时用 `/opt/lerobot_venv/bin/python` 做 `os.execv` 原地重启（PID 不变，launch 进程管理不受影响，PYTHONPATH 和 ros args 继承）；`UR_TELEOP_REEXEC` env 护栏防无限重启
- **注意**：venv python 是 `/usr/bin/python3` 的 symlink，`realpath` 比较恒等，不能作为"已在 venv"判据
- **Commit**: 9c41f5b

### 6. record 模式 s/d/q 键无反应（Enter 正常）
- **现象**：Enter 能识别，但 s/d/q 按下无反应
- **根因**：终端默认 canonical 模式，普通按键被行缓冲，只有回车（产生 `\n`）才整行交付。`KeyboardReader` 打开 `/dev/tty` 后没切模式，单键命令永远等不到回车
- **修复**：`keyboard.py` 打开 `/dev/tty` 后 `termios.tcgetattr` + `tty.setcbreak` 切逐键立即交付（保留 Ctrl-C），`atexit` 恢复终端属性
- **Commit**: 51b77eb

### 7. venv numpy 混用满屏 traceback（非致命）
- **现象**：data_recorder 启动时打印大量 numpy 1.x/2.x 不兼容 traceback 样式的输出
- **根因**：venv（`include-system-site-packages=true`，numpy 2.2.6）可见系统 numexpr/bottleneck（numpy 1.x 编译）。numpy 2.x 的 `__getattr__` 硬编码 `sys.stderr.write(msg + tb_msg)`——不是 warning，`filterwarnings` 拦不住
- **修复**：`pip install --ignore-installed numexpr bottleneck` 装进 venv site-packages 遮蔽系统版本；**注意装完要把 numpy 钉回 2.2.6**（--ignore-installed 顺带拉了 numpy 2.5.2，与 lerobot 要求的 `<2.3` 冲突）

## 重要陷阱

### LaunchConfiguration 全局改写
`IncludeLaunchDescription` 的 `launch_arguments` 按名字全局共享——父级同名 LaunchConfiguration 会被改写。
- 不要在不同 include 分支用同名 LaunchConfiguration key
- sim/real 分支分离的参数用不同名字（如 `description_sim` vs `description_launchfile`）

### ros2_control_node 孤儿进程
`home.launch` 退出后 cell 进程仍在后台。再次启动前必须 `pkill -9 ros2_control_node`。

### colcon 脚本安装路径
`setup.cfg` 必须用 `$base/lib/<pkg>`，否则 `ros2 run` 报 "No executable found"。

### Alicia-D driver 安装路径
launch 文件装在 share 根目录，include 时不要加 `launch` 子目录。

### ROS Python 不在 venv
节点跑在 `/usr/bin/python3`，pip 依赖要装给系统 python。

### UR 当前位置必须接近 home
`home_node` 通过 `scaled_joint_trajectory_controller` 运动，速度约束 0.2 rad/s。
如果 UR 位姿离 home 太远（>1 rad），轨迹执行时间超长且可能触发超时。
**最佳实践**：示教器 freedrive 先把 UR 推到接近 home，再用 home.launch 精确到位。

## 日常操作速查

```bash
# 查看控制器状态
ros2 control list_controllers 2>/dev/null | grep -E "scaled|forward|NAME"

# 查看 UR 当前关节
ros2 topic echo /joint_states --once 2>/dev/null | grep -A1 "shoulder_pan"

# 查看 robot_mode (7=RUNNING)
ros2 topic echo /io_and_status_controller/robot_mode --once 2>/dev/null

# 查看 speed_scaling (应为 100.0)
ros2 topic echo /speed_scaling_state_broadcaster/speed_scaling --once 2>/dev/null

# 手动发 home 轨迹
ros2 action send_goal /scaled_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{...}"

# 后台运行 cell 避免被 SIGINT 杀死
setsid ros2 launch ur_teleop home.launch.py sim:=false robot_ip:=169.254.138.15 &
```

## 文件索引

| 文件 | 用途 |
|---|---|
| `src/ur_teleop/config/ur_teleop.yaml` | **唯一运行时配置**（home/mapping/safety/teleop/ruckig/gripper/recorder） |
| `src/ur_teleop/config/ur_controllers_sim.yaml` | sim 模式控制器定义（含 gripper controller） |
| `src/ur_teleop/launch/home.launch.py` | 阶段 1：启动 cell + home_node |
| `src/ur_teleop/launch/cell.launch.py` | cell 编排（UR + gripper + FT + Alicia + rviz） |
| `src/ur_teleop/launch/teleop.launch.py` | 阶段 2：teleop + ruckig + recorder |
| `src/ur_teleop/launch/rsp_mock.launch.py` | sim 模式的组合模型 robot_state_publisher |
| `src/ur_teleop/ur_teleop/home_node.py` | home 到位节点（发送 scaled 轨迹 + 验证） |
| `src/ur_teleop/ur_teleop/teleop_node.py` | 遥操作状态机（主从映射 + 控制器切换） |
| `src/ur_teleop/ur_teleop/ruckig_node.py` | Ruckig 平滑节点 |
| `src/ur_teleop/ur_teleop/controller_switcher.py` | controller_manager 异步客户端 |
| `src/ur_teleop/ur_teleop/joint_mapper.py` | Alicia→UR 关节映射 |
| `src/ur_teleop/ur_teleop/config.py` | 配置加载/验证 + 关节名常量 |
| `src/ur10e_robotiq_ft_description/urdf/ur10e_robotiq_ft.urdf.xacro` | 组合 URDF（UR+FT300+2F-85） |
