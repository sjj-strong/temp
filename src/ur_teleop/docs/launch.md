# 启动与相机

每个终端先加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

两种控制方式均先运行 `home.launch.py`，看到 `HOME REACHED` 后保持终端运行，再启动 `teleop.launch.py`。后者不启动机器人硬件。

| 配置 `teleop.control_source` | Home 阶段 | 遥操作阶段 |
| --- | --- | --- |
| `alicia`（默认） | `cell.launch.py`，双臂回 Home | `teleop_node`，按配置使用 Ruckig |
| `xbot` | `xbot_cell.launch.py`，仅 UR 回 Home | `joy_node` + `xbot_teleop_node` |

`mode:=record` 在遥操作阶段增加录制器，详见[数据采集](data_recorder.md)。Xbot 的完整配置、校准和命令见[手柄说明](xbot_control.md)。

## Alicia 启动

```bash
# 终端 1：UR 仿真；Alicia 仍使用真实主臂
ros2 launch ur_teleop home.launch.py sim:=true

# 终端 2：Home 到位后启动，进入 ARMED 后按 Enter
ros2 launch ur_teleop teleop.launch.py
```

使用自定义配置时，两个阶段传入同一 `config_file:=/绝对路径/配置.yaml`。

| 参数 | 适用入口 | 说明 |
| --- | --- | --- |
| `config_file` | Home / teleop | 节点配置文件 |
| `sim`、`robot_ip`、`gripper_port`、`ftdi_id` | Alicia Home | 仿真选择与设备地址 |
| `launch_alicia`、`alicia_port`、`launch_rviz` | Alicia Home | 主臂启动、串口、显示 |
| `controller` | Alicia Home | forward_position 或 joint_impedance |
| `mode` | teleop | teleop 或 record，未指定时读取所选配置 |
| `use_ruckig`、`ruckig_control_hz` | Alicia teleop | 平滑开关及频率 |
| `force_home` | Alicia teleop | 跳过 Home 位置验证；Xbot 不支持 |

Alicia Home 的参数默认值来自安装目录的 `ur_teleop.yaml`；自定义 `config_file` 不会替换这些默认值，必要时显式传入对应参数。teleop 阶段按所选配置读取默认值。Xbot Home 直接读取所选配置，不使用上述 Alicia 专属参数覆盖。

## Alicia 控制器选择

`teleop.controller: forward_position` 使用 Ruckig 平滑目标后发送至 `/forward_position_controller/commands`。

设置为 `joint_impedance` 时，Home 命令还需传 `controller:=joint_impedance`。轨迹控制器先使用 position 接口回 Home，遥操作再严格切换到 effort 阻抗接口：

```bash
ros2 launch ur_teleop home.launch.py sim:=true controller:=joint_impedance
# Home 到位后，在另一终端执行
ros2 launch ur_teleop teleop.launch.py
```

该仿真分支使用仅含 UR 六轴的 mock，建议关闭 `gripper.enabled`。Ruckig 向 `/joint_impedance_controller/target_joint_state` 发布目标；关闭 `ruckig.enabled` 时由遥操作直接发送。

真机需配置正确的 IP、串口、Home 与 `sim: false`，确认控制器支持所需接口后再运行。运行期间不得由其他节点向同一运动控制器发送指令。Home 失败应检查实际关节值与 `home.slave`，不要用跳过验证代替故障处理。

## 相机

相机由 `camera.launch.py` 单独启动，不包含在 Home 或 teleop 中，也不启动机器人。配置 `cameras.realsense.enabled`、`cameras.opencv.enabled` 选择发布源：

```bash
ros2 launch ur_teleop camera.launch.py \
  launch_realsense:=true \
  launch_opencv_cameras:=true \
  launch_image_viewers:=true
```

- RealSense 使用 `data_collection/launch/dual_realsense.launch.py`，设置位于 `cameras.realsense`。
- USB/OpenCV 使用 `data_collection/launch/opencv_cameras.launch.py`，设备与图像参数位于 `config/opencv_cameras.yaml`。
- `recorder.cameras` 的 topic 必须与发布端一致，消息类型为 `sensor_msgs/Image`。
- `cameras.visualization.topics` 定义预览话题，拼接结果为 `/camera_mosaic/image_raw`，由 rqt 显示。
