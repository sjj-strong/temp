# 数据采集与帧格式

`teleop.launch.py mode:=record` 启动独立的 `data_recorder` 进程，按 `recorder.fps` 录制。相机需[单独启动](launch.md#相机)。

## 录制操作

| 操作 | Alicia 键盘 | Xbot 手柄 |
| --- | --- | --- |
| 开始 episode | Enter | Menu |
| 保存并结束 episode | S | Y |
| 丢弃并结束 episode | D | B |
| 保存当前有效 episode 并 finalize | Q | View 长按 2 秒 |

Alicia 首次开始时发送一次 `/teleop/enable`。Xbot 不读取键盘、不发送此使能，可先按 RB 预摆位，再按 Menu 录制；View 长按还会锁定运动，需重启节点才能继续。

帧数不足 `recorder.min_frames_per_episode`（默认 2）时，保存操作自动丢弃。保存或丢弃后，可再次开始下一段。未开始过 episode 时退出不会创建数据集。

## 数据格式

`FrameBuilder.features()` 定义特征，`build()` 组装每帧。默认 observation 为 14 维，action 为 7 维：

| 字段 | 内容 |
| --- | --- |
| `observation.state` | 6 个 UR 关节角（rad）+ TCP 的 xyz 和 xyzw 四元数 + 夹爪状态 |
| Alicia `action` | 6 个关节目标 `cmd_<关节名>` + `cmd_gripper` |
| Xbot `action` | `vx, vy, vz, wx, wy, wz, cmd_gripper` |
| `observation.images.<image_key>` | RGB 图像，形状为 height × width × 3 |
| `task` | `recorder.task` |

Xbot 前六维是在 `base_link` 中用于目标积分的速度（m/s、rad/s），不是测得速度；TCP 模式先转换到 base 再记录，停止时为零。Xbot 强制记录完整七维 action，不能与 Alicia 的关节 action 混用。

夹爪 observation 由实测关节角与 `state_threshold_rad`（默认 0.4）比较得到，0 开、1 闭；action 末维是夹爪目标信号。Xbot 使用已接受的目标，初值取实测开合状态。支持夹爪控制器单独发布的 `JointState`。

## 数据有效性

- 两种模式开始前均要求所有配置相机收到图像。关节或动作缺失时不组帧。
- Alicia 的 EE 位姿缺失时填 NaN；相机仅检查缓存是否存在，不检查帧龄。
- Xbot 开始前要求新鲜的就绪心跳、动作、UR/夹爪状态、TCP 位姿及所有相机，`recorder.data_timeout_s` 默认 0.5 秒；录制中失效的周期跳过。
- Xbot 恢复后继续当前 episode，数据集时间按帧率排列，不保留故障期间的墙钟间隔。需要连续时间数据时，应丢弃断连影响的 episode。
- `add_frame` 失败时记录错误并跳过该帧。

## 配置与存储

| 配置项 | 用途 |
| --- | --- |
| `repo_id` / `root` / `robot_type` | 数据集标识、存储目录、机器人类型 |
| `fps` / `min_frames_per_episode` | 录制频率、最少有效帧数 |
| `ee_pose_source` | `tf`、`topic` 或 `none` |
| `ee_pose_parent_frame` / `ee_pose_child_frame` | TF 查询框架；Xbot 为 base_link / gripper_tcp |
| `cameras` | 各相机的 topic、image_key、height、width |
| `use_videos` | true 为视频特征，false 为图像特征 |
| `record_ur_joints` / `record_ur_ee_pose` / `record_ur_gripper` | observation 字段开关 |
| `record_action_joints` / `record_action_gripper` | Alicia action 字段开关 |
| `image_writer_processes` / `image_writer_threads` | 图像写入并发参数 |

以上均位于 `recorder` 下。Xbot 默认使用 `repo_id: my_user/ur10e_xbot`、`root: /ros2_ws/dataset/xbot`，与 Alicia 数据集分开。已有数据集不覆盖、不追加，另建带时间戳的标识和目录。

LeRobot 不可导入时，入口会尝试使用 `/opt/lerobot_venv/bin/python` 重启；仍不可用则报错。实际视频编码和落盘需在采集环境验收。

## 接口

订阅 `/joint_states`、`/teleop/commands`、配置的图像话题，以及 TF 或 TCP 位姿话题。

Xbot 另订阅 `/teleop/xbot_ready`（Bool）和 `/teleop/record_event`（String：start/save/discard/finalize）。事件回调只入队，数据集操作在录制主线程执行；超过两秒的非 finalize 事件忽略。结束时发布 `/teleop/record_finished`（Bool）。

测试见 `tests/test_frame_builder.py`、`tests/test_xbot_recorder.py`；原有 LeRobot 集成测试位于 `tests/test_integration.py`。
