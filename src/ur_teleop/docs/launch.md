# 启动与相机

每个终端先加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

真机启动 `home.launch.py` 后，Home 节点会打印提示：请在示教器上启动“外部控制（External Control）”程序，完成后在启动终端按回车。未按回车时不会发送 Home 指令；收到回车后打印确认日志，再等待控制器就绪并执行 Home。仿真不需要示教器，跳过此确认。

两种控制方式均先运行 `home.launch.py`，看到 `HOME REACHED` 后保持终端运行，再启动 `teleop.launch.py`。后者不启动机器人硬件。

| 配置 `teleop.control_source` | Home 阶段 | 遥操作阶段 |
| --- | --- | --- |
| `alicia`（默认） | `cell.launch.py`，双臂回 Home | `teleop_node`，按配置使用 Ruckig |
| `xbot` | `xbot_cell.launch.py`，仅 UR 回 Home | `joy_node` + `xbot_teleop_node` |

`mode:=record` 在遥操作阶段增加录制器，详见[数据采集](data_recorder.md)。Xbot 的完整配置、校准和命令见[手柄说明](xbot_control.md)。

完整的硬件、相机调试、teleop 测试到 record 采集步骤见[完整流程](workflow.md)，控制器参数见[控制器说明](controllers.md)。

## Alicia 启动

```bash
# 终端 1：当前 Alicia 配置选择关节阻抗；UR 仿真，Alicia 仍是真实主臂
ros2 launch ur_teleop home.launch.py sim:=true controller:=joint_impedance

# 终端 2：Home 到位后启动，进入 ARMED 后按 Enter
ros2 launch ur_teleop teleop.launch.py mode:=teleop
```

使用自定义配置时，两个阶段传入同一 `config_file:=/绝对路径/配置.yaml`。

| 参数 | 适用入口 | 说明 |
| --- | --- | --- |
| `config_file` | Home / teleop | 节点配置文件 |
| `sim`、`robot_ip`、`gripper_port`、`ftdi_id` | Alicia Home | 仿真选择与设备地址 |
| `enable_gripper`、`enable_ft300` | Alicia Home | 仅覆盖阶段 1 外设启动，不覆盖 teleop/recorder YAML |
| `launch_alicia`、`alicia_port`、`launch_rviz` | Alicia Home | 主臂启动、串口、显示 |
| `controller` | Alicia Home | forward_position 或 joint_impedance |
| `mode` | teleop | teleop 或 record，未指定时读取所选配置 |
| `ruckig_control_hz` | Alicia teleop | 覆盖 Ruckig ROS control_hz，不改变 teleop 映射频率 |
| `use_ruckig` | Alicia teleop | 仅控制是否创建 Ruckig 节点；不覆盖 teleop 的 YAML 路径选择，需保持一致 |
| `force_home` | Alicia teleop | 跳过 Home 位置验证；Xbot 不支持 |

Alicia Home 的参数默认值来自安装目录的 `alicia_teleop.yaml`；自定义 `config_file` 不会替换这些默认值，必要时显式传入对应参数。teleop 阶段按所选配置读取默认值。Xbot Home 直接读取所选配置，不使用上述 Alicia 专属参数覆盖。

## Alicia 控制器选择

`teleop.controller: forward_position` 使用 Ruckig 平滑目标后发送至 `/forward_position_controller/commands`。

设置为 `joint_impedance` 时，应确保 Home 的 `controller` 与 YAML 一致；自定义配置或覆盖时显式传 `controller:=joint_impedance`。轨迹控制器先使用 position 接口回 Home，遥操作再严格切换到 effort 阻抗接口：

```bash
ros2 launch ur_teleop home.launch.py sim:=true controller:=joint_impedance
# Home 到位后，在另一终端执行
ros2 launch ur_teleop teleop.launch.py mode:=teleop
```

该仿真分支使用仅含 UR 六轴的 mock，建议在临时配置关闭 `gripper.enabled` 并在 Home 传 `enable_gripper:=false`。完整示例见[流程](workflow.md)。Ruckig 向 `/joint_impedance_controller/target_joint_state` 发布目标；关闭 `ruckig.enabled` 时由遥操作直接发送。

本文未执行真实机器人运动验证，完整 Home/遥操作示例只用于 mock，真机测试仍仅允许 wrist_3_joint。真机连接的代码分支需配置正确的 IP、串口、Home 与 `sim: false`，确认控制器支持所需接口后再运行。运行期间不得由其他节点向同一运动控制器发送指令。Home 失败应检查实际关节值与 `home.slave`，不要用跳过验证代替故障处理。

## 相机

相机由 `camera.launch.py` 单独启动，不包含在 Home 或 teleop 中，也不启动机器人。两种遥操作共用 `config/camera.yaml`，顶层每个键就是自定义相机名，无需 `cameras` 包装层：

```yaml
front:
  type: usb
  enabled: true
  visualize: true
  device: /dev/video0
  width: 640
  height: 480
  fps: 30
  fourcc: MJPG

wrist:
  type: realsense
  enabled: true
  visualize: false
  serial_no: "001234567890"
  enable_color: true
  enable_depth: false
  width: 640
  height: 480
  fps: 30
```

```bash
ros2 launch ur_teleop camera.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/camera.yaml
```

- 名称必须以字母开头，仅包含字母、数字和下划线；默认彩色话题为 `/camera/<名称>/color/image_raw`，可用该相机的 `topic` 覆盖。
- `type` 为 `usb` 或 `realsense`。USB 的 `device` 填设备路径或端口索引，优先使用稳定的 `/dev/v4l/by-path`；RealSense 的 `serial_no` 必须加引号，各设备独立启动，不固定相机型号或数量。
- `enabled` 默认 `true`，关闭后不启动也不加入预览。`visualize` 默认 `false`，仅开启时加入拼图；没有相机需要预览时，不启动拼图节点或 rqt。
- 宽高和帧率必须为正整数，默认 640×480、30 FPS。USB 可配置 `fourcc`、`publish_compressed`、`compressed_quality`、`auto_exposure`、`exposure_time_absolute` 和 `frame_id`；RealSense 用 `enable_color`、`enable_depth` 控制图像流。
- 原有按类别开关及序列号的命令行参数已移除；入口仅使用 `config_file`，每台相机的参数在文件内修改。已有旧配置须按上例迁移。
- 预览话题自动从选中的相机生成，无需单独维护话题列表；拼接结果为 `/camera_mosaic/image_raw`。预览使用彩色图像，RealSense 需要开启 `enable_color`。
- 每台相机的 `resize` 控制保存时是否缩放，`resize_width`、`resize_height` 设置保存尺寸；`width`、`height` 是采集尺寸。`alicia_teleop.yaml` / `xbot_teleop.yaml` 的 `recorder.cameras` 只按名称选择录制相机，尺寸和话题自动从同一相机文件读取，见[录制配置](data_recorder.md)。

修改 Ruckig 路径应使用 YAML `ruckig.enabled` 并重启阶段 2。`use_ruckig:=false` 配合仍启用的 YAML 会令 teleop 继续发向缺失的 Ruckig 节点；反向不一致可能产生直接输出与 Ruckig 双发布，不应作为推荐命令。Home cell 由 launch 启动后，Home 节点退出不会自动关闭 cell。
