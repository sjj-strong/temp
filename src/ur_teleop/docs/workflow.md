# 完整使用流程

操作顺序：准备设备和配置 → 启动相机 → 回 Home → 遥操作 → 数据采集。Alicia 与 Xbot 二选一，同一机器人只运行一套控制栈。

## 1. 准备环境与配置

按[硬件准备](hardware.md)安装依赖并连接设备。首次构建：

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --packages-select ur_teleop --symlink-install
```

每个 ROS 终端加载：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

本文直接使用源码配置路径，修改配置后重启对应进程即可，无需为配置修改重新构建。

| 输入源 | 配置文件 | 启动前检查 |
| --- | --- | --- |
| Alicia | `/ros2_ws/src/ur_teleop/config/alicia_teleop.yaml` | 主臂串口、双臂 Home、映射、控制器和夹爪开关 |
| Xbot | `/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml` | 手柄标定、UR Home、机器人 IP 和阻抗参数文件 |

设备地址与 Home 必须匹配实际设备。Xbot 先完成[标定与输入检查](xbot_control.md#配置与校准)。需要更新 UR Home 时使用[位置读取工具](capture_slave_home.md)。

## 2. 启动相机（可选）

先用[相机工具](camera_inspector.md)确认设备和采集模式，关闭调参预览，再运行：

```bash
ros2 launch ur_teleop camera.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/camera.yaml
```

`camera.yaml` 顶层每个键为相机名；每台相机独立设置设备、预览及 `resize`。在遥操作配置的 `recorder.cameras` 中按名称选择录制相机，并设置：

```yaml
recorder:
  camera_config_file: /ros2_ws/src/ur_teleop/config/camera.yaml
```

相机发布和录制必须读取同一相机配置。尺寸设置见[相机配置](launch.md#相机)。

## 3. 回 Home

选择以下一个分支。Home 成功后显示 `HOME REACHED`，保持该终端运行。

### Alicia 仿真

在 Alicia 配置中设置 `sim: true`、`teleop.controller: joint_impedance`、`gripper.enabled: false`。UR 使用 mock，Alicia 主臂仍是真实设备并会回 Home。

```bash
ros2 launch ur_teleop home.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/alicia_teleop.yaml \
  sim:=true controller:=joint_impedance enable_gripper:=false enable_ft300:=false
```

### Alicia 真机

在 Alicia 配置中设置 `sim: false`，核对 `cell.robot_ip`、`cell.alicia_port`、`home.master` 和 `home.slave`。以下使用关节阻抗控制，关闭未连接的夹爪和 FT300；此时 YAML 中也应关闭对应开关。

```bash
ros2 launch ur_teleop home.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/alicia_teleop.yaml \
  sim:=false controller:=joint_impedance \
  robot_ip:=169.254.138.15 alicia_port:=/dev/ttyACM0 \
  enable_gripper:=false enable_ft300:=false
```

IP 和串口是示例，需替换为实际值。接入外设时同步开启 YAML 和 Home 参数，详见[启动参数](launch.md)。

### Xbot 仿真

在 Xbot 配置中设置 `sim: true`，并将 `xbot.controller_config_file` 设置为：

```yaml
xbot:
  controller_config_file: /ros2_ws/src/cartesian_impedance_controller/config/ur10e_xbot_sim_cartesian_impedance.yaml
```

```bash
ros2 launch ur_teleop home.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml
```

### Xbot 真机

在 Xbot 配置中设置 `sim: false`，核对 `cell.robot_ip`、`home.slave` 和外设开关，选择与真机匹配的阻抗参数，例如：

```yaml
xbot:
  controller_config_file: /ros2_ws/src/cartesian_impedance_controller/config/ur10e_ft300_cartesian_impedance.yaml
```

```bash
ros2 launch ur_teleop home.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml
```

Xbot 的仿真/真机选择由 YAML `sim` 决定，不能用 `sim:=false` 覆盖。

真机 Home 启动后，在示教器启动 External Control 程序，再在 Home 终端按 Enter。确认后会执行六轴 Home 运动；Alicia 分支还会移动主臂。Home 失败时检查反馈和配置，不要跳过位置验证。

## 4. 遥操作

新终端选择与 Home 相同的配置。以下命令同时适用于对应配置的仿真和真机：

```bash
# Alicia
ros2 launch ur_teleop teleop.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/alicia_teleop.yaml mode:=teleop
```

```bash
# Xbot：与 Alicia 二选一
ros2 launch ur_teleop teleop.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml mode:=teleop
```

Alicia 等待静止和偏移捕获，提示后按 Enter 开始。Xbot 自动接管阻抗控制器，就绪后先松开 RB，再按住 RB 操作；按键见[手柄操作](xbot_control.md#按键与参考系)。

## 5. 数据采集

先 Ctrl-C 停止遥操作终端，保留硬件和相机终端。如果 Alicia 已离开 Home，重新运行 Home 后再启动采集。Xbot 在阻抗控制器仍 active 时可从当前位置接管。

采集终端加载 ROS 环境和 LeRobot 环境，然后选择一条命令。仿真和真机使用相同入口，机器人模式由前面的 Home 配置决定：

```bash
source /opt/lerobot_venv/bin/activate
# Alicia
ros2 launch ur_teleop teleop.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/alicia_teleop.yaml mode:=record
```

```bash
# Xbot：与 Alicia 二选一
ros2 launch ur_teleop teleop.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml mode:=record
```

| 操作 | Alicia | Xbot |
| --- | --- | --- |
| 开始一段 | Enter | Menu |
| 保存 | S | Y |
| 丢弃 | D | B |
| 保存并结束 | Q | View 长按 |

修改 `recorder.root`、`task`、数据字段和相机选择后再采集。仿真无力数据时关闭 `record_wrench`。格式与参数见[数据采集](data_recorder.md)和[录制参数](recorder_config.md)。

## 6. 结束

先保存并结束采集，等待数据集写入完成，再停止遥操作、相机和硬件终端。不要同时运行 teleop 与 record 两套遥操作进程。软件停止和退出的行为见[Alicia](teleop_node.md#停止与退出)与[Xbot](xbot_control.md#保护与限制)。
