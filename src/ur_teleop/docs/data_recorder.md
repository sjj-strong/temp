# 数据采集与帧格式

`ros2 launch ur_teleop teleop.launch.py config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml mode:=record` 启动手柄、遥操作和录制器。组合单元须先按[手柄启动说明](xbot_control.md)启动并完成 Home；相机按其驱动说明单独启动。录制数据写入 `recorder.root`，不会自动上传。

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

两种模式均逐帧保存字符串 `action.reference_link`：真机通常为 `base`，仿真为 `base_link`。`record_action_gripper: true` 时再附加 `cmd_gripper`；无夹爪时建议设为 `false`。修改模式、坐标系或字段配置后请重启遥操作和录制器，新建数据集。

## 可选 observation

Xbot 的下列开关相互独立。启用的数值字段按表格顺序拼接为 `observation.state`，特征 `names` 给出各元素名称；全部关闭时不创建该键。

| 开关 | 内容 | 维度 |
| --- | --- | ---: |
| `record_joint_position` | `/joint_states.position`，按 UR 六关节顺序 | 6 |
| `record_joint_velocity` | `/joint_states.velocity` | 6 |
| `record_joint_effort` | `/joint_states.effort`，UR 上可能是电机电流，不能视为实测关节力矩 | 6 |
| `record_tcp_pose` | 配置的 TCP link 相对控制器参考 link 的 xyz+xyzw | 7 |
| `record_ur_gripper` | 夹爪实测开合状态，阈值由 `state_threshold_rad` 指定 | 1 |
| `record_wrench` | 原始 `force.xyz, torque.xyz` | 6 |

启用 TCP 时，还保存 `observation.tcp_reference_link` 和 `observation.tcp_link`。`ee_pose_child_frame` 指定 TCP link；Xbot 的父 link 自动采用控制器参考 link。`ee_pose_source` 可选 `tf` 或 `topic`，关闭 TCP 观测时也可设 `none`。

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

录制开始和写帧时只要求**启用**的字段有新鲜数据；任一启用字段缺失、非有限或超过 `data_timeout_s` 时跳过整帧，不写缺键或 NaN。手柄就绪心跳与位姿 action 始终是必要条件。Alicia 的旧 `observation.state` 和关节 action 格式保持原有配置。

## 话题与验证

Xbot 手柄节点通过 `/teleop/commands` 发布 action 数组，布局标签携带模式、参考 link 和 TCP link；录制器校验标签与原始数组维度。`/teleop/record_event` 传递开始、保存、丢弃和结束事件；`/teleop/xbot_ready` 提供就绪心跳。

```bash
ros2 topic echo --once /teleop/commands
ros2 topic echo --once /force_torque_sensor_broadcaster/ft_data
# 启用 FT300 时改查：
ros2 topic echo --once /robotiq_force_torque_sensor_broadcaster/wrench
```

单元测试位于 `tests/test_xbot_core.py`、`tests/test_xbot_recorder.py` 和 `tests/test_frame_builder.py`。
