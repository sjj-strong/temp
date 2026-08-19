# ur10e_robotiq_ft_description

> 路径：`/ros2_ws/src/ur10e_robotiq_ft_description` — UR10e + Robotiq FT300 力/力矩传感器 + Robotiq 2F-85 夹爪的**组合机器人描述包**（`ament_cmake`）。本身不含任何驱动节点，只提供组合 xacro 与展示 / 状态发布 launch，作为整个 robot cell 的 URDF 来源。

## 概述

官方 `ur_description` 只描述纯 UR 机械臂，没有 FT300 与夹爪。本包把三者串成一条完整运动链并发布到 `/robot_description` 话题，供 rviz、controller_manager、MoveIt 等消费。连接链（见 `urdf/ur10e_robotiq_ft.urdf.xacro:7`）：

```
world → UR 机械臂 → tool0 → FT300 → ft300_sensor → Robotiq 2F-85 → gripper_tcp
```

两个 launch：

| launch | 启动的节点 | 用途 |
|---|---|---|
| `view.launch.py` | `robot_state_publisher` + `joint_state_publisher_gui` + `rviz2` | **纯可视化校验**：拖滑块动关节、检查装配，无需真机、不起控制器 |
| `rsp.launch.py` | 仅 `robot_state_publisher` | 作为 `/robot_description` 源：`ur_teleop` 的 `cell.launch.py` / `home.launch.py` 默认 `description_launchfile`（drop-in 替换官方 `ur_rsp.launch.py`） |

## 文件结构

| 路径 | 作用 |
|---|---|
| `urdf/ur10e_robotiq_ft.urdf.xacro` | 组合模型主 xacro：include 上游 UR / FT300 / 2F-85 宏，串接运动链，复用官方 `ur_ros2_control` 宏 |
| `launch/view.launch.py` | 可视化 launch（RSP + jsp_gui + rviz） |
| `launch/rsp.launch.py` | 仅 RSP 的 launch，参数集对齐官方 `ur_robot_driver/launch/ur_rsp.launch.py` |
| `rviz/display.rviz` | rviz 配置：Grid + RobotModel（订阅 `/robot_description`）+ TF，Fixed Frame = `world` |
| `package.xml` / `CMakeLists.txt` | 包元数据；`CMakeLists.txt` 仅 `install(DIRECTORY urdf launch rviz ...)` |

## 1. 安装与依赖

环境：ROS 2 Jazzy。本包**不构建任何节点**，编译只安装 `urdf/launch/rviz` 资源；但 **xacro 在解析期需要以下包可在工作区找到**（`$(find ...)`，见 xacro:49-52 与 `rsp.launch.py` 的 recipe/script 默认值）：

| xacro / launch 引用的包 | 来源仓库（本工作区） | 提供 |
|---|---|---|
| `ur_description` | `Universal_Robots_ROS2_Description` | `urdf/ur_macro.xacro`、`config/<ur_type>/*.yaml`（关节限位 / 运动学 / 物理 / 可视） |
| `ur_robot_driver` | `Universal_Robots_ROS2_Driver/ur_robot_driver` | `urdf/ur.ros2_control.xacro`（官方 ros2_control 宏）、RTDE recipe 资源 |
| `ur_client_library` | `Universal_Robots_Client_Library` | `external_control.urscript`（`rsp.launch.py` 的 `script_filename` 默认） |
| `robotiq_ft_sensor_description` | `rq_fts_ros2_driver/robotiq_ft_sensor_description` | `urdf/robotiq_ft300.urdf.xacro` |
| `robotiq_description` | `ros2_robotiq_gripper/robotiq_description` | `urdf/robotiq_2f_85_macro.urdf.xacro` |
| 运行时 | —— | `robot_state_publisher`、`joint_state_publisher_gui`、`rviz2`、`xacro` |

> ⚠️ **依赖缺口（如实记录）**：`ur_robot_driver` 与 `ur_client_library` 是 xacro include 与 `rsp.launch.py` 默认值的硬依赖，但当前 `package.xml` 的 `exec_depend` **未列入**这两项。若在未编译它们的情况下单独 `--packages-select ur10e_robotiq_ft_description`，`view.launch.py` / xacro 解析会因 `$(find ur_robot_driver)` 失败而报错。规避：先编译整个工作区，或至少先编 `ur_robot_driver` + `ur_client_library`。

编译：

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --packages-select ur10e_robotiq_ft_description --symlink-install
source install/setup.bash
```

## 2. 快速使用：可视化校验（`view.launch.py`）

最常用的入口——快速看到完整装配，无真机、无控制器：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch ur10e_robotiq_ft_description view.launch.py
```

行为：rviz 加载 `rviz/display.rviz`，显示组合模型；`joint_state_publisher_gui` 弹窗，拖动滑块即可控制 UR 六关节 + 夹爪手指关节。

`view.launch.py` 暴露的参数（`view.launch.py:28-41`）：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `name` | `ur10e_robotiq_ft` | 机器人名（写入 xacro 顶层 `name`） |
| `ur_type` | `ur10e` | UR 型号，需在 `ur_description/config/` 下有对应目录（如 `ur5e` / `ur10e` / `ur16e` / `ur20` / `ur30`） |

换型号示例：

```bash
ros2 launch ur10e_robotiq_ft_description view.launch.py ur_type:=ur5e
```

> `view.launch.py` 不向 xacro 传 `use_mock_hardware`，xacro 取默认 `false`。但由于本 launch **不起 controller_manager**，URDF 里的 `<ros2_control>` 标签不会被激活，仅作为模型文本发布到 `/robot_description`——不影响可视化。要真正激活 ros2_control（mock 或真机硬件接口）请走第 3 节的 `rsp.launch.py` + `ur_control.launch.py` 链。

## 3. 作为 description 源被 `ur_teleop` 使用（`rsp.launch.py`）

这是本包在 cell 中的主要角色。`ur_teleop` 的 `cell.launch.py` / `home.launch.py` 默认 `description_launchfile` 就是本包的 `rsp.launch.py`（`ur_teleop/launch/cell.launch.py:30-37`：`get_package_share_directory("ur10e_robotiq_ft_description")` 成功则用之，否则回退官方 `ur_rsp.launch.py`）。透传到 xacro 的 `use_mock_hardware` 决定硬件形态：

- **`use_mock_hardware:=true`**（sim）：UR 发 mock 硬件，夹爪 `include_ros2_control=true` 自带 fake 硬件 → `/joint_states` 含夹爪手指关节、TF 完整。
- **`use_mock_hardware:=false`**（real）：UR 发真机硬件接口，夹爪 `include_ros2_control=false`（URDF 仅含链路 / visual，无 ros2_control 标签），夹爪由 `robotiq_control.launch.py` 独立管理。

通常你**不直接运行** `rsp.launch.py`，而是通过 `ur_teleop` 间接使用：

```bash
# sim（cell + home，UR 端 mock + rviz）
ros2 launch ur_teleop home.launch.py

# real（真机）
ros2 launch ur_teleop home.launch.py sim:=false robot_ip:=192.168.1.1 \
    gripper_port:=/dev/ttyUSB1 ftdi_id:=<你的ftdi_id>
```

需要独立验证 `rsp.launch.py` 本身（仅 RSP + /robot_description）：

```bash
ros2 launch ur10e_robotiq_ft_description rsp.launch.py use_mock_hardware:=true
# 另一终端
ros2 topic echo /robot_description --once | head
```

`rsp.launch.py` 声明的参数集完全对齐官方 `ur_robot_driver/launch/ur_rsp.launch.py`（`rsp.launch.py:70-150`）：`name` / `ur_type` / `tf_prefix` / `robot_ip` / `use_mock_hardware` / `mock_sensor_commands` / `headless_mode` / `verify_robot_model` / `use_tool_communication` 与全部 `tool_*` / `reverse_*` / 端口 / recipe / script / 各 `*_params_file` / `transmission_hw_interface` / `initial_positions_file`。需要时可按官方参数名直接传，例如 `robot_ip:=... tf_prefix:=ur_`。

## 4. 关键 xacro 参数（`ur10e_robotiq_ft.urdf.xacro`）

| 参数 | 默认值 | 含义 |
|---|---|---|
| `name` | `ur10e_robotiq_ft` | 机器人名 |
| `ur_type` | `ur10e` | UR 型号（选 `ur_description/config/` 下对应目录） |
| `tf_prefix` | `""` | TF / 关节名前缀（多机器人场景） |
| `ft_xyz` / `ft_rpy` | `0 0 0.0` / `0 0 0` | FT300 相对 `tool0` 的安装变换 |
| `gripper_xyz` / `gripper_rpy` | `0 0 0` / `-3.1415 0 0` | 2F-85 相对 `ft300_sensor` 的安装变换（默认绕 X 翻 180°） |
| `tcp_xyz` / `tcp_rpy` | `0 0 0.15` / `0 0 0` | `gripper_tcp` 相对 `robotiq_85_base_link` 的偏移 |
| `use_mock_hardware` | `false` | mock / real 切换（同时驱动夹爪 `include_ros2_control`，见第 3 节） |
| `robot_ip` / `reverse_ip` / 各端口 / recipe / script | 见 xacro | ros2_control 参数集，与官方 `ur.urdf.xacro` 一致 |
| `initial_positions_file` | `$(find ur_description)/config/initial_positions.yaml` | 初始关节位姿 |

> **安装变换为未标定的占位默认值，仅用于模型验证**（xacro:14 注释）。真机标定出 FT300 / 夹爪的实际安装位姿后，用 launch 参数或直接改 xacro 默认值覆盖。

直接用 xacro 命令传参（绕过 launch）：

```bash
ros2 run xacro xacro \
    $(ros2 pkg prefix --share ur10e_robotiq_ft_description)/urdf/ur10e_robotiq_ft.urdf.xacro \
    ur_type:=ur10e use_mock_hardware:=true ft_xyz:="0 0 0.005" \
    > /tmp/ur10e_robotiq_ft.urdf
```

## 5. 校验生成的 URDF

确认 xacro 解析无误、运动链完整：

```bash
# 解析为纯 URDF
ros2 run xacro xacro \
    $(ros2 pkg prefix --share ur10e_robotiq_ft_description)/urdf/ur10e_robotiq_ft.urdf.xacro \
    ur_type:=ur10e use_mock_hardware:=true > /tmp/ur10e_robotiq_ft.urdf

# 校验运动链（需安装 ros-jazzy-urdf）
check_urdf /tmp/ur10e_robotiq_ft.urdf
```

期望输出：从 `world` 到 `gripper_tcp` 的完整树，无断链；UR 六关节 + 2F-85 手指关节均为 `revolute` 可动。

## 6. 发布的话题与坐标系

| 名称 | 类型 / 内容 | 发布者 |
|---|---|---|
| `/robot_description` | `std_msgs/String`（TRANSIENT_LOCAL QoS），URDF 文本 | `robot_state_publisher`（两个 launch 均发布） |
| `/joint_states` | `sensor_msgs/JointState`，UR 六关节 + 夹爪手指关节 | `joint_state_publisher_gui`（仅 `view.launch.py`）；sim 下由 `ur_control.launch.py` 的 controller_manager 接管 |

TF 树（Fixed Frame = `world`）：

```
world
 └─ <name>_base_link / base … tool0           # UR 机械臂（ur_macro）
     └─ ft300_* … ft300_sensor                 # Robotiq FT300
         └─ robotiq_85_base_link … (finger)    # Robotiq 2F-85
             └─ gripper_tcp                    # 夹爪 TCP 参考点
```

> `tool0` 是 UR 法兰；FT300 装在法兰上，2F-85 装在 FT300 传感器面，`gripper_tcp` 是夹爪基座上的固定 TCP 参考点（`tcp_xyz` 默认 `0 0 0.15`）。这三段安装位姿即第 4 节的 `ft_*` / `gripper_*` / `tcp_*` 参数，真机标定后需覆盖。
