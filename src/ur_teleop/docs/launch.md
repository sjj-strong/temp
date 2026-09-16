# launch 文件解析

> 路径：`/ros2_ws/src/ur_teleop/launch/` — 三个 launch（cell / home / teleop）的职责、包含关系、参数声明与「launch 参数 > yaml 默认」优先级机制。

## 概述

- `cell.launch.py`：UR cell（持久），sim/real 两模式，可被 home.launch 与 teleop.launch 复用（实际仅 home.launch include 它）。
- `home.launch.py`：阶段 1 = cell + home_node（移双臂到 home 并验证后退出，cell 保持）。
- `teleop.launch.py`：阶段 2 = teleop_node + ruckig_node（+ mode=record 时 data_recorder），连接已运行的 cell，**不含 cell**。
- 三个 launch 都以 `config_file` 为唯一必传参数（默认 `share/ur_teleop/config/ur_teleop.yaml`）；除 `config_file`/`mode`/`force_home`/`description_launchfile` 外，其余参数的默认值都来自 yaml（`_yaml_default` 机制）。

## cell.launch.py

职责：启动 UR 栈（ur_control.launch.py）、按形态附加组件、管理 rviz。sim/real 由 `sim` 参数（字符串）决定，判断用 `PythonExpression("'sim' == 'true'")` 而非直接取真值——规避 `"false"` 字符串在 Python 中为真的陷阱（cell.launch.py:52）。

包含关系：

| include / Node | 来源包 | 传参 | 条件 |
|---|---|---|---|
| `ur_control.launch.py` | ur_robot_driver | `ur_type`、`robot_ip`、`use_mock_hardware=sim`、`mock_sensor_commands="false"`、`launch_dashboard_client="false"`、`launch_rviz="false"`、`description_launchfile`（rviz 模型，见下）、`initial_joint_controller="scaled_joint_trajectory_controller"`、`activate_joint_controller="true"` | 总是 |
| `rviz2`（Node） | rviz2 | `-d` `config/rviz/ur_teleop.rviz` | `sim=='true'` 且 `launch_rviz=='true'` |
| `robotiq_control.launch.py` | robotiq_description | `com_port=gripper_port`、`launch_rviz="false"` | `UnlessCondition(sim=='true')`（仅 real） |
| `ft_sensor_standalone.launch.py` | robotiq_ft_sensor_hardware | `ftdi_id`、`frame_id="robotiq_ft_frame_id"` | `UnlessCondition(sim=='true')`（仅 real） |

要点：

- `launch_rviz="false"` 传给 ur_control（避免双重 rviz）；`launch_dashboard_client="false"` 关闭 dashboard 客户端；rviz 由本 launch 自己的 Node 管理（sim 下）。
- `initial_joint_controller="scaled_joint_trajectory_controller"` + `activate_joint_controller="true"`：cell 启动即激活 trajectory 控制器——home_node 的 `follow_joint_trajectory` action 依赖它；teleop ACTIVE 时再切到 `forward_position_controller`。
- 注意 cell 启动的控制器激活窗口约 1-2 s，期间 trajectory 控制器会 REJECT goal，故 home_node 有 10 s 接受重试（home_node.py:89-109）。

参数声明（cell.launch.py:55-63）：

| 参数 | 默认值来源 | 含义 |
|---|---|---|
| `config_file` | `share/ur_teleop/config/ur_teleop.yaml` | 配置入口路径（本 launch 的 yaml 默认值读取源） |
| `sim` | yaml 顶层 `sim`（兜底 `"true"`） | `"true"` = mock UR + rviz；`"false"` = 真机 + 夹爪 + FT300 |
| `robot_ip` | `cell.robot_ip`（兜底 `"0.0.0.0"`） | 真机 IP（real 模式） |
| `gripper_port` | `cell.gripper_port`（兜底 `"/dev/ttyUSB1"`） | Robotiq 夹爪串口（real 模式） |
| `ftdi_id` | `cell.ftdi_id`（兜底 `""`） | rq_fts 驱动 FT300 的 ftdi_id（real 模式） |
| `launch_rviz` | `cell.launch_rviz`（兜底 `"true"`） | sim 下是否拉起 rviz |
| `ur_type` | `cell.ur_type`（兜底 `"ur10e"`） | UR 型号；**仅 cell.launch.py 声明**（见下） |
| `description_launchfile` | `_description_launchfile()`（组合模型 rsp，见下） | rviz/controller_manager 的 URDF 来源（cell.launch.py:30-37、62-63） |

## rviz 模型：URDF 描述文件来源

rviz（以及 controller_manager）读的是 **`/robot_description` 话题**（`std_msgs/String`，TRANSIENT_LOCAL QoS，由 robot_state_publisher 发布）。该话题的内容由 `description_launchfile` 决定，经 ur_control.launch.py 的 rsp include 链传递（ur_control 只显式传 `robot_ip`/`ur_type`，其余如 `use_mock_hardware` 走 launch config 继承）。

默认取 **组合模型**（用户视角：ur 官方只有纯 UR，组合模型在 ur10e_robotiq_ft_description）：

- **`ur10e_robotiq_ft_description/launch/rsp.launch.py`**（默认）：xacro `ur10e_robotiq_ft.urdf.xacro` → **UR + FT300 + Robotiq 2F-85 完整装配**（连接链 `world → UR → tool0 → FT300 → ft300_sensor → 2F-85 → gripper_tcp`）。该 xacro 复用官方 `ur_ros2_control` 宏（ur_robot_driver/urdf/ur.ros2_control.xacro），与官方 `ur.urdf.xacro` 参数集一致——mock 模式下发 mock 硬件（UR + 夹爪两个 `mock_components/GenericSystem`，`/joint_states` 含夹爪关节、TF 完整），real 模式下发真机插件（URPositionHardwareInterface，夹爪仍由 robotiq_control.launch.py 独立管理）。
- **官方 `ur_robot_driver/launch/ur_rsp.launch.py`**（回退）：纯 UR 模型。当 `ur10e_robotiq_ft_description` 包未安装时自动回退（`_description_launchfile()` 内 `get_package_share_directory` 抛异常即 fallback，cell.launch.py:30-37）。

显式指定：`ros2 launch ur_teleop cell.launch.py sim:=true description_launchfile:=/path/to/ur_rsp.launch.py`（home.launch.py 同参数）。

## home.launch.py

职责：include cell（持久）+ 启动 home_node（一次性，打印 HOME REACHED 后退出，cell 保持）。

- 声明并转发 `config_file` / `sim` / `robot_ip` / `gripper_port` / `ftdi_id` / `launch_rviz` / `description_launchfile` 到 cell.launch.py（home.launch.py:40-70）；home_node 只收 `config_file` 参数（节点内 `load_config` 自行解析，home.launch.py:71-74）。
- **不转发 `ur_type`**：home.launch.py 未声明该参数，`ur_type` 只能经 yaml `cell.ur_type` 配置（或直接运行 cell.launch.py 传参）。这是有意保留的最小面——大多数现场不改型号。
- home_node 流程见 home_node.md：等 `cell_ready()`（30 s）→ UR home 轨迹（接受重试 + 结果等待）→ Alicia `/joint_commands` 持续命令 → 双臂保持 `verify_duration_s` 后打印 HOME REACHED → 退出 0/1。

参数声明（home.launch.py:40-56）：

| 参数 | 默认值来源 | 含义 |
|---|---|---|
| `config_file` | 同 cell | 配置入口路径 |
| `sim` / `robot_ip` / `gripper_port` / `ftdi_id` / `launch_rviz` / `description_launchfile` | 同 cell 各键（`_yaml_default` 同源；description_launchfile 用同一 `_description_launchfile()`） | 原样转发给 cell.launch.py |

## teleop.launch.py

职责：阶段 2。启动 teleop_node（常驻）+ ruckig_node（常驻，500 Hz 平滑），`mode=record` 时由 `IfCondition` 门控额外拉起 data_recorder。**不含 cell**——WAITING_CELL 30 s 超时即提示先运行 home.launch。

ruckig_node 必须在 home 完成后启动：teleop.launch 连接的是已 home 的 cell，ruckig 从当前（已 home）UR `/joint_states` 初始化 Ruckig 状态，避免 home 阶段轨迹控制器移动机器人导致状态过期（切换到 forward 时首帧跳变）。teleop_node 把映射目标发到 `/ruckig/target_joint_positions`，ruckig_node 以 `ruckig_control_hz`（默认 500 Hz）平滑后下发 `/forward_position_controller/commands`——详见 ruckig_node.md。**不要同时用 `cell.launch … ruckig:=true`**，否则两个 ruckig_node 抢同一话题。

参数声明：

| 参数 | 默认值来源 | 含义 |
|---|---|---|
| `config_file` | `share/ur_teleop/config/ur_teleop.yaml` | 配置入口路径 |
| `mode` | yaml 顶层 `mode`（兜底 `"teleop"`；`choices=["teleop", "record"]`） | 运行模式；同时门控 data_recorder |
| `force_home` | `"false"`（硬编码，无 yaml 对应键） | `true` = 跳过 VERIFY_HOME（teleop_node 把 `at_home_tolerance_rad` 置 `inf`，teleop_node.py:64-65） |
| `ruckig_control_hz` | yaml `ruckig.control_hz`（兜底 `"500.0"`） | ruckig_node OTG 频率，默认与 controller_manager 同频 500 Hz |

控制器由 `ur_teleop.yaml` 的 `teleop.controller` 选择。默认 `forward_position` 沿用
`/forward_position_controller/commands`；设为 `joint_impedance` 时，`home.launch.py`
转发同名 launch 参数给 cell，cell 用关节阻抗包的参数文件将
`joint_impedance_controller` 预加载为 inactive。随后 teleop 的 STRICT 切换会停用
trajectory 与另一遥操运动控制器，并激活配置选择的控制器；Ruckig 输出改为
`/joint_impedance_controller/target_joint_state` 的 `JointState`。若使用自定义
`config_file:=...`，还应显式传 `controller:=joint_impedance`，原因与下方的 launch
默认值读取机制相同。

当 `sim:=true controller:=joint_impedance` 时，cell 不会启动官方 UR 的
`GenericSystem`，而会启动 `JointImpedanceMockSystem`：home 阶段由同名
`scaled_joint_trajectory_controller` 使用 position 接口，遥操阶段严格切换到 effort
接口并按单位惯量/黏性阻尼积分关节状态。因此该分支可验证完整的“遥操映射 → Ruckig
→ 阻抗力矩 → 关节状态”闭环。该专用模型仅含 UR 六轴，不含仿真 Robotiq/FT；若启用
夹爪，夹爪 action server 不存在时会被遥操节点自动禁用，不影响六轴闭环。

`mode` 与 `ruckig_control_hz` 的读取与 cell 类参数不同：teleop.launch.py 内联 `yaml.safe_load(...)` 一次性读 `mode`（兜底 `"teleop"`）与 `ruckig.control_hz`（兜底 `"500.0"`）作默认值；teleop_node 收到 `mode` 参数后 `mode or cfg["mode"]` 兜底（launch 参数优先、yaml 兜底，teleop_node.py:67）。

## 参数优先级：launch 参数 > yaml 默认

三个 launch 的默认值机制是 `_yaml_default(config_file, *path, fallback)`（cell.launch.py:19-27、home.launch.py:14-22）：**launch 文件加载时**读包 share 目录的 yaml，沿 `path` 链取值作为 `DeclareLaunchArgument` 的 `default_value`；键缺失或读取异常则用 `fallback`。用户传参（如 `sim:=false`）覆盖默认值，未传时落到 yaml 默认。

各键的读取路径（launch 默认值 ← yaml 键）：

| launch 默认值 | yaml 路径 | fallback |
|---|---|---|
| `sim` | 顶层 `sim` | `"true"` |
| `robot_ip` | `cell.robot_ip` | `"0.0.0.0"` |
| `gripper_port` | `cell.gripper_port` | `"/dev/ttyUSB1"` |
| `ftdi_id` | `cell.ftdi_id` | `""` |
| `launch_rviz` | `cell.launch_rviz` | `"true"` |
| `ur_type` | `cell.ur_type`（仅 cell.launch.py） | `"ur10e"` |
| `description_launchfile` | 组合模型 rsp（`ur10e_robotiq_ft_description`，未安装回退官方 `ur_rsp.launch.py`） | `_description_launchfile()` |
| `mode` | 顶层 `mode`（仅 teleop.launch.py，内联读取） | `"teleop"` |
| `ruckig_control_hz` | `ruckig.control_hz`（仅 teleop.launch.py，内联读取） | `"500.0"` |

**已知张力（`config_file:=` 只影响节点参数）**：`_yaml_default` 读的是**安装 share 的 yaml**（`get_package_share_directory("ur_teleop")/config/ur_teleop.yaml`），而 `config_file:=` 参数只传给节点（teleop_node / home_node / data_recorder 经 `load_config` 读取）。因此 `config_file:=/path/to/other.yaml` 会换掉节点配置，但 **launch 默认值（sim/robot_ip 等）仍取自 share 里的 yaml**。若同时想换 launch 默认，需显式传对应参数（如 `sim:=false robot_ip:=...`），不能指望 `config_file:=` 一并改变。这是当前实现的有意取舍：launch 参数显式且完整，yaml 仅作兜底默认。

## 启动命令序列

### 默认前向位置控制

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

# 终端 1：sim，等待 HOME REACHED，保持 cell 运行。
ros2 launch ur_teleop home.launch.py sim:=true

# 终端 2：进入 ARMED 后按 Enter。
ros2 launch ur_teleop teleop.launch.py
```

### 关节阻抗仿真

在 `ur_teleop.yaml` 中设置 `teleop.controller: joint_impedance`。阻抗仿真使用专用的
`JointImpedanceMockSystem`，只包含 UR 六轴，因此建议同时设置 `gripper.enabled: false`。

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --symlink-install --packages-select joint_impedance_controller ur_teleop
source install/setup.bash

# 终端 1：home 使用 position 接口；HOME REACHED 后保持运行。
ros2 launch ur_teleop home.launch.py sim:=true controller:=joint_impedance

# 终端 2：遥操阶段严格切换到 effort 接口的阻抗控制器。
ros2 launch ur_teleop teleop.launch.py
```

Ruckig 会向 `/joint_impedance_controller/target_joint_state` 发布带关节名的
`sensor_msgs/msg/JointState`。如果使用 `config_file:=/绝对路径/配置.yaml`，两个 launch
都要传同一个 `config_file`，并在 `home.launch.py` 命令额外传
`controller:=joint_impedance`；`teleop.launch.py` 没有 `controller` launch 参数。

### 关节阻抗真机验证

配置 `teleop.controller: joint_impedance` 后，只有完成现场安全检查且真机控制器支持
effort 接口时才可执行：

```bash
# 终端 1：HOME REACHED 后保持运行。
ros2 launch ur_teleop home.launch.py sim:=false controller:=joint_impedance \
  robot_ip:=192.168.1.1 gripper_port:=/dev/ttyUSB1 ftdi_id:=<你的ftdi_id>

# 终端 2：进入 ARMED 后按 Enter。
ros2 launch ur_teleop teleop.launch.py
```

真机阻抗安全门固定锁定前五轴，只允许 `wrist_3_joint` 在启动切换时的位置附近
`±teleop.real_impedance_wrist_3_max_delta_rad`（默认 `±0.02 rad`）旋转。不得下发其他关节、末端位姿、轨迹或笛卡尔速度控制指令。

## 相机发布与数据录制

相机由独立的 `camera.launch.py` 启动，不包含在 `cell.launch.py`、`home.launch.py` 或
`teleop.launch.py` 中，也不会启动 UR、Alicia、控制器或数据录制节点。请在独立终端
启动相机后，再启动遥操与录制。`ur_teleop.yaml` 的 `cameras.realsense.enabled` 与
`cameras.opencv.enabled` 分别控制两类相机，可同时为 `true`。两者默认关闭。

- RealSense 复用 `data_collection/launch/dual_realsense.launch.py`；序列号、是否启用
  D435i、RGB/深度、分辨率和命名空间均由 `cameras.realsense` 配置。
- USB/OpenCV 复用 `data_collection/launch/opencv_cameras.launch.py`；设备路径、发布
  话题、帧名、分辨率、帧率与节点名在本包 `config/opencv_cameras.yaml` 配置。

例如要在录制时保存 D455 RGB 与 USB 前视图，`recorder.cameras` 的 topic 必须与发布端
一致：`/camera/d455/color/image_raw` 和 `/camera/usb_front/color/image_raw`。两者均为
`sensor_msgs/msg/Image`。

```bash
# 环境（sim 与 real 均需）
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

# 阶段 1 —— sim（主臂真实，UR 端 mock + rviz）
ros2 launch ur_teleop home.launch.py
# 看到 HOME REACHED 后（cell 仍在运行）开第二个终端：
ros2 launch ur_teleop teleop.launch.py
# 状态到 ARMED 后按 Enter → rviz 中 mock UR 跟随主臂

# 阶段 1 —— real（真机）
ros2 launch ur_teleop home.launch.py sim:=false robot_ip:=192.168.1.1 gripper_port:=/dev/ttyUSB1 ftdi_id:=<你的ftdi_id>
# HOME REACHED 后第二个终端：
ros2 launch ur_teleop teleop.launch.py

# 阶段 2 —— record（需先激活 lerobot venv）
source /opt/lerobot_venv/bin/activate
ros2 launch ur_teleop teleop.launch.py mode:=record
```

补充：

- `sim`/`robot_ip`/`gripper_port`/`ftdi_id`/`launch_rviz` 只在 home.launch.py（含 cell）声明；也可直接改 yaml 的 `sim`/`cell.*` 而不传参。
- `force_home:=true`：双臂不在 home 容差内时跳过验证直接继续；home 轨迹未到位时也可用。
- 退出：Ctrl-C。teleop_node 自动执行 `/demonstration=false` → 切回 trajectory controller（`restore_controller_on_exit`）→ recorder `finalize()`。

## 测试覆盖

- 集成测试 `test_integration.py` 通过 launch API 拉起 home/teleop 全栈（假主臂 + mock UR + record），覆盖两阶段启动与 `force_home` 等参数路径；launch 文件本身的参数默认值读取（`_yaml_default` 与 `config_file` 张力）由测试的 `tmp_path` 隔离配置复现验证。见 docs/README.md。
