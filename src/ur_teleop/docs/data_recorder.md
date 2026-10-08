# 数据采集与帧格式

`ros2 launch ur_teleop teleop.launch.py config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml mode:=record` 启动手柄、遥操作和录制器。组合单元须先按[手柄启动说明](xbot_control.md)启动并完成 Home；相机先按[调试说明](camera_inspector.md)确认参数，再按[相机发布说明](launch.md#相机)单独启动。从硬件准备到先测试后采集的命令见[完整流程](workflow.md)。录制数据写入 `recorder.root`，不会自动上传。

## 操作

| 操作 | Xbot 手柄 | Alicia 键盘 |
| --- | --- | --- |
| 开始 episode | Menu | Enter |
| 保存 | Y | S |
| 丢弃 | B | D |
| 保存并结束录制 | View 长按 | Q |

不足 `min_frames_per_episode` 帧时自动丢弃；保存或丢弃后可开始下一段。

## Xbot action

`recorder.action_mode: abs` 保存手柄节点最终发布给阻抗控制器的绝对目标 `x,y,z,qx,qy,qz,qw`。`rel` 保存该目标相对**同一控制周期实测 TCP** 的 `dx,dy,dz,drx,dry,drz`；姿态增量是参考坐标系中的最短旋转向量，满足 `q_target = dq × q_actual`。工作空间裁剪发生在编码之前，因此 action 与最终下发目标一致。摇杆回中时仍保存保持目标；此时相对 action 可能非零。

两种模式均逐帧保存字符串 `action.reference_link`：真机通常为 `base`，仿真为 `base_link`。`record_action_gripper: true` 时再附加 `cmd_gripper`（打开 `0`、闭合 `1`）；无夹爪时建议设为 `false`。修改模式、坐标系或字段配置后请重启遥操作和录制器，新建数据集。

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

每台相机由 `recorder.cameras.<名称>.enabled` 单独控制；省略 `enabled` 视为启用。启用后用 `topic`、`image_key`、`height`、`width` 定义 `observation.images.<image_key>`。`use_videos` 决定视频或逐帧图像特征。例如：

```yaml
recorder:
  cameras:
    front:
      enabled: true
      topic: /camera/usb_front/color/image_raw
      image_key: front
      height: 480
      width: 640
```

录制开始和写帧时只要求**启用**的字段有新鲜数据；任一启用字段缺失、非有限或超过 `data_timeout_s` 时跳过整帧，不写缺键或 NaN。手柄就绪心跳与位姿 action 始终是必要条件。Alicia 的 observation 不再附加夹爪实测状态；关节 action 可通过同一个开关附加夹爪指令。

## 话题与验证

Xbot 手柄节点通过 `/teleop/commands` 发布 action 数组，布局标签携带模式、参考 link 和 TCP link；录制器校验标签与原始数组维度。`/teleop/record_event` 传递开始、保存、丢弃和结束事件；`/teleop/xbot_ready` 提供就绪心跳。

```bash
ros2 topic echo --once /teleop/commands
ros2 topic echo --once /force_torque_sensor_broadcaster/ft_data
# 启用 FT300 时改查：
ros2 topic echo --once /robotiq_force_torque_sensor_broadcaster/wrench
```

单元测试位于 `tests/test_xbot_core.py`、`tests/test_xbot_recorder.py` 和 `tests/test_frame_builder.py`。

## 夹爪录制开关

Alicia 与 Xbot 仅使用 `recorder.record_action_gripper` 控制夹爪录制：`true` 在 action 末尾保存二值 `cmd_gripper`（打开 `0`、闭合 `1`），`false` 不保存。不再录制夹爪实测 observation；已移除 `record_ur_gripper` 和 `state_threshold_rad`。录制无需等待夹爪反馈。该开关不控制夹爪执行，执行仍由 `gripper.enabled` 控制。已有数据集的 observation 维度会变化，请使用新数据集。

录制器仅在保存关节位置、速度或 effort 时订阅关节反馈；夹爪指令录制不参与该订阅判断。

## 采集日志与进度

两个遥操作配置均新增顶层 `debug` 布尔开关，默认关闭：

```yaml
debug: false
```

`mode: record` 下，正常日志只保留按键/手柄操作、控制配置频率和采集进度。事件正文使用 JSON，ROS 自身保留日志等级、时间和节点名。例如：

```text
{"event": "control_frequency", "command_hz": 50.0}
{"event": "collection_frequency", "target_hz": 20}
{"event": "keyboard", "action": "start", "episode": 1, "message": "Y=保存 B=丢弃 View长按=退出"}
episode=1 | frames=100 | elapsed=00:05, collect_hz=19.8, target_hz=20
{"event": "keyboard", "action": "save", "episode": 1, "frames": 100}
```

`tqdm` 进度行只统计 `add_frame` 成功的帧，`collect_hz` 每秒按成功写入帧数/实际经过时间计算；数据未就绪暂停采集时会降到 0。`target_hz` 是 `recorder.fps`，`control_frequency.command_hz` 是控制节点的配置频率，不能当作实测机器人执行频率。Episode 由用户按键结束，没有固定总帧数，显示帧数和耗时，不显示百分比。保存、丢弃或退出时关闭进度行。采集器取消每 5 秒重复打印的状态提示，保留启动时的按键说明及实际操作事件。

改为 `debug: true` 后显示：

| 位置 | 受开关控制的现有诊断 |
| --- | --- |
| `xbot_teleop_node.py` | 位姿/目标差、RB/LB、输入量、数据龄、控制器状态、工作空间原点、夹爪请求/接受反馈、状态变化 |
| `teleop_node.py` | Alicia 状态机、offset、enable、控制器切换成功、夹爪就绪、启动/恢复信息 |
| `ruckig_node.py` | 控制周期、目标控制器、初始关节角与初始化信息 |
| `data_recorder.py` | 数据集创建、enable 发布、数据集 finalize 信息 |
| `xbot_cell.launch.py` | 控制器参数路径与笛卡尔控制器“已接收目标 pose”的 INFO 日志 |

关闭 debug 不隐藏错误或故障警告。此开关管理本包诊断及 Xbot 笛卡尔控制器的 INFO 输出，不改变其他 ROS 驱动的日志配置，也不改变机器人控制行为。修改配置后重启遥操作/采集节点；笛卡尔控制器日志等级在 Home 阶段加载，修改它需重启对应启动流程。

新增依赖为 `python3-tqdm`，LeRobot 虚拟环境也需安装 `tqdm`。启动采集器时启用终端模拟，以便显示实时进度。接口参考：[tqdm 文档](https://tqdm.github.io/docs/tqdm/)。
