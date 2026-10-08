# 数据采集与帧格式

`ros2 launch ur_teleop teleop.launch.py config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml mode:=record` 启动手柄、遥操作和录制器。组合单元须先按[手柄启动说明](xbot_control.md)启动并完成 Home；相机先按[调试说明](camera_inspector.md)确认参数，再按[相机发布说明](launch.md#相机)单独启动。从硬件准备到先测试后采集的命令见[完整流程](workflow.md)。录制数据写入 `recorder.root`，不会自动上传。

完整参数、LeRobot 创建选项和 `num_episodes` 用法见[Record 配置参数](recorder_config.md)。

## 数据集目录重名

首次使用配置的 `recorder.root` 创建数据集；目录已存在时，新建带时间后缀的目录，不覆盖已有数据。后缀格式为 `年-月-日-时-分-秒-微秒`（`YYYY-MM-DD-HH-MM-SS-ffffff`），目录名和 `repo_id` 均以 `-` 连接。

例如，原目录 `/ros2_ws/dataset/xbot` 已存在时，新目录可为 `/ros2_ws/dataset/xbot-2026-10-08-22-30-15-123456`；相应 `repo_id` 为 `my_user/ur10e_xbot-2026-10-08-22-30-15-123456`。未设置 `recorder.root` 时，也会为 `repo_id` 添加同样后缀。

## 操作

| 操作 | Xbot 手柄 | Alicia 键盘 |
| --- | --- | --- |
| 开始 episode | Menu | Enter |
| 保存 | Y | S |
| 丢弃 | B | D |
| 保存并结束录制 | View 长按 | Q |

不足 `min_frames_per_episode` 帧时自动丢弃，不计入采集数量。`recorder.num_episodes: 0` 不限数量；设为正整数后，成功保存达到该数量会自动 finalize 并退出采集器。目标未达到时，保存或丢弃后可开始下一段。

## Alicia 数据格式

`record_ur_joints` 保存六关节实测位置，`record_ur_ee_pose` 保存 TCP 的 xyz+xyzw；两者均开启时 `observation.state` 为 13 维。`record_action_joints` 保存六关节目标，`record_action_gripper` 可在 action 末尾附加一个二值夹爪指令。

## Xbot action

`recorder.action_mode: abs` 保存手柄节点最终发布给阻抗控制器的绝对目标 `x,y,z,qx,qy,qz,qw`。`rel` 保存该目标相对**同一控制周期实测 TCP** 的 `dx,dy,dz,drx,dry,drz`；姿态增量是参考坐标系中的最短旋转向量，满足 `q_target = dq × q_actual`。工作空间裁剪发生在编码之前，因此 action 与最终下发目标一致。摇杆回中时仍保存保持目标；此时相对 action 可能非零。

Xbot 两种 action 模式均逐帧保存字符串 `action.reference_link`：真机通常为 `base`，仿真为 `base_link`。`record_action_gripper: true` 时再附加 `cmd_gripper`（打开 `0`、闭合 `1`）；无夹爪时建议设为 `false`。修改模式、坐标系或字段配置后请重启遥操作和录制器，新建数据集。

## 可选 observation

Xbot 的下列开关相互独立。启用的数值字段按表格顺序拼接为 `observation.state`，特征 `names` 给出各元素名称；全部关闭时不创建该键。

| 开关 | 内容 | 维度 |
| --- | --- | ---: |
| `record_joint_position` | `/joint_states.position`，按 UR 六关节顺序 | 6 |
| `record_joint_velocity` | `/joint_states.velocity` | 6 |
| `record_joint_effort` | `/joint_states.effort`，UR 上可能是电机电流，不能视为实测关节力矩 | 6 |
| `record_tcp_pose` | 配置的 TCP link 相对控制器参考 link 的 xyz+xyzw | 7 |
| `record_wrench` | 原始 `force.xyz, torque.xyz` | 6 |

启用 TCP 时，还保存 `observation.tcp_reference_link` 和 `observation.tcp_link`。`ee_pose_child_frame` 指定 TCP link；Xbot 的父 link 自动采用控制器参考 link。末端位姿固定从 TF 获取，无需配置来源或位姿话题；Xbot 的 `record_tcp_pose: false` 关闭位姿录制及 TF 监听。

启用力数据时，`cell.ft300_enabled: true` 订阅 `/robotiq_force_torque_sensor_broadcaster/wrench`；设为 `false` 则订阅 UR 内置传感器 `/force_torque_sensor_broadcaster/ft_data`。数值保持消息原始坐标，不做变换；`observation.wrench_reference_link` 保存该消息的 `header.frame_id`。FT300 在组合 URDF 的 ros2_control 硬件接口中运行，组合启动仅额外加载 broadcaster，不启动争用串口的独立驱动。仿真录制如无力话题，应将 `record_wrench` 设为 `false`。

每台相机的采集与保存尺寸统一在 `camera.yaml` 中设置：

```yaml
usb_front:
  type: usb
  enabled: true
  device: /dev/video0
  width: 640                   # 采集宽度
  height: 480                  # 采集高度
  fps: 30
  resize: true                 # 是否缩放保存图像
  resize_width: 320            # 保存宽度
  resize_height: 240           # 保存高度
```

`resize: true` 在录制器接收图像后缩放到 `resize_width × resize_height`，图像、视频与数据集尺寸描述同步改变。`resize: false` 或省略则保存原图，采用采集的 `width/height`；目标尺寸默认 320×240，必须为正整数。修改后重启录制器，并使用新数据集避免旧尺寸冲突。图像发布和预览仍使用采集尺寸。

遥操作配置只选择相机，名称必须与 `camera.yaml` 顶层键一致，不再重复填写话题和尺寸：

```yaml
recorder:
  cameras:
    usb_front:
      enabled: true
      image_key: front         # 可选；省略时使用 usb_front
```

默认读取安装目录中的 `config/camera.yaml`。相机发布入口使用自定义文件时，在遥操作配置中设置 `recorder.camera_config_file: /绝对路径/camera.yaml`，让录制器读取同一文件；相对路径以遥操作配置文件所在目录为基准。`use_videos` 决定视频或逐帧图像特征，`recorder.cameras.<名称>.enabled` 只控制是否录制该相机（省略视为启用）。话题和尺寸以相机文件为准。

## 数据就绪条件

Xbot 开始及写帧时要求就绪心跳、action 和启用字段均有效且未超过 `data_timeout_s`；开启 TCP 时还要求有效 TF。数据缺失、非有限或过期时拒绝开始或跳过整帧。

Alicia 使用最新缓存，没有同等数据龄检查；必要关节或 action 缺失时跳过帧，TCP 查询失败时填 NaN 并警告。两种模式开始前均要求所选相机至少收到一帧。

## 查看数据话题

`/teleop/commands` 为目标动作，`/teleop/record_event` 为手柄录制事件，`/teleop/xbot_ready` 为就绪心跳。

```bash
ros2 topic echo --once /teleop/commands
ros2 topic echo --once /force_torque_sensor_broadcaster/ft_data
# 启用 FT300 时改查：
ros2 topic echo --once /robotiq_force_torque_sensor_broadcaster/wrench
```


## 夹爪录制开关

Alicia 与 Xbot 仅使用 `recorder.record_action_gripper` 控制夹爪录制：`true` 在 action 末尾保存二值 `cmd_gripper`（打开 `0`、闭合 `1`），`false` 不保存。不保存夹爪实测 observation。录制无需等待夹爪反馈。该开关不控制夹爪执行，执行仍由 `gripper.enabled` 控制。更改字段后请使用新数据集。



## 采集日志与进度

顶层 `debug: false` 显示操作提示、配置频率、首次控制接口、采集进度和故障。改为 `true` 后增加输入、位姿、状态及控制器诊断，修改后重启相关进程。

进度中的 `frames` 是成功写入帧数，`collect_hz` 是实际写入频率，`target_hz` 是 `recorder.fps`。配置控制频率不等于实测机器人执行频率。数据未就绪时写入频率会下降；每段由用户保存或丢弃，没有固定帧数。

采集环境需安装 `tqdm`。正常结束时等待数据集写入完成，再关闭其他终端。
