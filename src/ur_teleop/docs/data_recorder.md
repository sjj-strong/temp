# data_recorder（record 模式数据采集）

> 路径：ur_teleop/data_recorder.py（263 行）
> 职责：键盘控制 episode 生命周期，50 Hz 组装 lerobot 帧（state 14 维 + action 7 维 + 相机）写入数据集（独立进程，`mode=record` 时由 teleop.launch 拉起）。

## 概述

data_recorder 是**独立进程**（进程隔离：崩溃不影响 teleop，已保存 episodes 保留；spec §9"recorder 崩溃"行）。键盘单一归属权在 record 模式下移交给它：**第一个 Enter 同时开 episode 1 并发 `/teleop/enable`**，teleop_node 收到后开始控制（配合其 `enable_pending` 锁存，即使 enable 早于 ARMED 到达也不丢）。

主循环结构（data_recorder.py:234-258）：`rclpy.spin` 跑在**独立后台线程**（订阅回调），主线程按 `recorder.fps` 周期做两件事——录制中则 `_record_frame()`（帧组装 + add_frame），然后非阻塞 `KeyboardReader.read_key(0.0)` 处理按键。无 ROS 定时器：主循环是 `period = 1.0/fps` 的 sleep 周期循环，后台 spin 线程与主循环键盘轮询互不干扰。注意 `LeRobotDataset` 导入失败时构造直接抛 `RuntimeError`（data_recorder.py:29-30）。

## 生命周期（按键语义）

| 按键 | 动作 |
|---|---|
| Enter（`\n`/`\r`） | `_start_episode()`：开 episode（第一个同时单发 `/teleop/enable`） |
| `s` | `_save_episode()`：保存当前 episode 并结束 |
| `d` | `_discard_episode()`：丢弃当前 episode（清 buffer 重置） |
| `q` | 退出：`finalize()` 后结束进程 |

```text
启动 → ARMED 等待
  Enter → 相机预检 → 建数据集（首次）→ 单发 enable → 录帧（50 Hz）
    s → save_episode（< min_frames 自动丢弃）→ 可再 Enter 开下一 episode
    d → clear_episode_buffer（不落盘）→ 可再 Enter 重开
    q → 若在录先保存 → dataset.finalize() → 退出
```

按键由 `KeyboardReader`（ur_teleop/keyboard.py，纯 stdlib）读取：`select` 非阻塞单字节读（`timeout=0`），`\n`/`\r` 归一为 `"enter"`，其余按键小写返回；`os.read(fd, 1)` 直接从 fd 读，避免缓冲流 read 把整块 kernel 数据读进 Python 缓冲区后卡在 select 之外。**控制持续整个会话**（enable 不重发），episode 边界只影响录制。

### 与 teleop_node 的交互时序（record 模式）

```text
recorder 主线程                  teleop_node 状态机
   Enter（键盘）
   ├─ 相机预检 → _init_dataset()
   ├─ publish /teleop/enable（会话唯一一次）
   └─ _recording=True，按 fps 录帧
                                  enable 到达（可能早于 ARMED）
                                    → _enable_pending 锁存
                                  … 完成 VERIFY_HOME/SETTLING/CAPTURE_OFFSET
                                  ARMED 消费锁存 → SWITCHING → ACTIVE
                                  → /teleop/status=true、/demonstration=true
                                  → 50 Hz 发 /ruckig/target_joint_positions（6 维映射目标）+ /teleop/commands（7 维）
                                  → ruckig_node 500 Hz 平滑 → /forward_position_controller/commands → UR cell
recorder 订阅 /teleop/commands → action 7 维帧数据
```

若 enable 丢失（无锁存的旧实现），recorder 不会重发 → 会话永远停在 ARMED；`_enable_sent` 单发与 teleop `_enable_pending` 锁存两者必须同时成立（见测试覆盖 F 组）。

## 关键机制

### `_enable_sent` 单发（与 teleop enable_pending 锁存的配合）

`_start_episode()`（data_recorder.py:152-168）里 `_enable_sent` 保证 `/teleop/enable` **整个会话只发一次**（发布前打日志 `"已发送 /teleop/enable → teleop_node 开始控制"`）。两个环节互相兜底：

1. recorder 侧：`_enable_sent=True` 后不再重发 → teleop 若错过该信号，会话永远进不了 ACTIVE；
2. teleop 侧：`_enable_pending` 锁存 ARMED 前收到的 enable（teleop_node.py:139-145）→ ARMED 后自动消费开始控制。

生产时序是"recorder 先发、teleop 后 ARMED"（teleop 需先等 cell + settle + offset），所以锁存是 record 模式能否 ACTIVE 的关键。

### FrameBuilder 消费（14 维 state + 7 维 action）

`FrameBuilder`（ur_teleop/frame_builder.py，纯逻辑无 rclpy）由 recorder 传入 `recorder` + `gripper` 配置构造，每帧：

| 字段 | 维度 | 内容 |
|---|---|---|
| `observation.state` | 14 | 6 UR 关节 + 7 EE 位姿（`ee_x,ee_y,ee_z,ee_qx,ee_qy,ee_qz,ee_qw`）+ 1 夹爪 state |
| `action` | 7 | `cmd_<UR关节名> ×6` + `cmd_gripper` |
| `task` | 文本 | `recorder.task`（默认 `teleoperation`） |

- **夹爪 state 是真实状态**：`robotiq_85_left_knuckle_joint` rad > `state_threshold_rad`（默认 0.4）→ 1.0，否则 0.0（spec §7 修复"用指令代理"问题）；
- **action 夹爪维来自指令**：`/teleop/commands[6]`（`get_gripper_command_signal` 输出的 0/1 信号），`len < 7` 时补 0.0；
- **EE 位姿缺失 → NaN 段**（绝不填 0 占位）；`ur_joints` 或 `teleop_cmd` 缺失 → `build()` 返回 None，该帧跳过；
- 相机帧按 `observation.images.<image_key>`（默认 `cam_name`）键写入（data_recorder.py:204-212）。

`features()` 的开关（`record_ur_joints` / `record_ur_ee_pose` / `record_ur_gripper` / `record_action_joints` / `record_action_gripper`，均默认 True）决定最终维度；`use_videos=true` 时相机特征 dtype 为 `video`，否则 `image`。

### lerobot 0.5.x 显式 root 布局

`LeRobotDataset.create()`（data_recorder.py:128-148）传显式 `root`（`recorder.root` 为空则走 HF_LEROBOT_HOME 默认）：

```text
<root>/meta/info.json                        ← create 时即落盘
<root>/data/chunk-000/file-000.parquet       ← 首个 save_episode 时落盘
```

`repo_id` 仅作元数据，**不存在** `<root>/<repo_id>` 之类的目录（集成测试据此断言）。数据集已存在（`FileExistsError`）→ 以 `f"{repo_id}_{%Y%m%d_%H%M%S}"` 时间戳后缀**新建** repo_id（warn `"数据集已存在，新建带时间戳: …"`），不覆盖不追加（spec §9）。`create` 还透传 `image_writer_processes`（默认 0）/ `image_writer_threads`（默认 2）。

### 关键保护

- **相机帧预检**：`_start_episode` 先查 `_cameras` 配置的每个相机是否收到过帧，缺帧 → error `"相机未收到帧，拒绝开始 episode: [...]。检查相机 topic 后重新按 Enter"` 并 return，**不建数据集不发 enable**（data_recorder.py:155-159）；
- **中途断流**：录制中某相机缺帧 → 该 episode **每相机仅警告一次** `"相机 X 无帧，本 episode 该相机的图像帧将被跳过"`，其余帧照录（`_missing_cam_warned` 集合在 episode 边界重置）；
- **零 episode finalize 安全**：`finalize()`（data_recorder.py:219-224）在 `_dataset is None` 时直接跳过（从未 Enter 过也安全）；录制中退出先自动保存；
- **min_frames_per_episode**（默认 2）：S 时帧数不足 → warn `"少于 N 帧，自动丢弃"` + `clear_episode_buffer()`，不落盘；
- **add_frame 异常**：捕获后打 error `"add_frame 失败: {e}"`，帧跳过，不崩溃；
- **EE 查询失败**：`_get_ee_pose()`（data_recorder.py:107-124）三源——`tf`（`lookup_transform` 到 `ee_pose_parent_frame`/`ee_pose_child_frame`，超时 0.5 s）、`topic`（`ee_pose_topic` 默认 `/tcp_pose` 订阅缓存）、`none`（恒 None → 恒 NaN 段）；查询失败时整会话只警告一次 `"EE 位姿查询失败，该段以 NaN 记录（仅警告一次）"`（`none` 模式不警告）。

## 话题 / 接口

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/joint_states` | `sensor_msgs/msg/JointState` | 取 UR 6 关节 + `robotiq_85_left_knuckle_joint` 夹爪 state |
| 订阅 | `/teleop/commands` | `std_msgs/msg/Float64MultiArray` | 7 维 action 来源 |
| 订阅 | `/tcp_pose`（可配） | `geometry_msgs/msg/PoseStamped` | `ee_pose_source=topic` 时的 EE 位姿 |
| 订阅 | `<cameras.*.topic>` | `sensor_msgs/msg/Image` | cv_bridge 转 rgb8 缓存 |
| 发布 | `/teleop/enable` | `std_msgs/msg/Bool` | 会话单发（第一次 Enter） |

启动（teleop.launch 的 `mode` 条件为 `record` 时自动拉起）：

```bash
ros2 launch ur_teleop teleop.launch.py mode:=record
# 独立运行（必须用 lerobot venv 的解释器：ros2 run 入口脚本 shebang 是系统 python3，缺 lerobot 会崩）
source /opt/lerobot_venv/bin/activate
python -m ur_teleop.data_recorder --ros-args -p config_file:=/path/to/ur_teleop.yaml
```

## 配置键（recorder.*，ur_teleop.yaml）

| 键 | 默认值 | 含义 |
|---|---|---|
| `recorder.repo_id` | `my_user/ur_teleop` | 数据集 repo_id（元数据；已存在则加时间戳后缀） |
| `recorder.root` | `""` | 显式数据集根目录；空 = HF_LEROBOT_HOME |
| `recorder.fps` | 50 | 录制帧率（主循环周期） |
| `recorder.robot_type` | `ur10e_alicia_teleop` | 数据集 robot_type 元数据 |
| `recorder.use_videos` | true | 相机特征 dtype：video / image |
| `recorder.ee_pose_source` | `tf` | `tf` / `topic`（`ee_pose_topic`，默认 `/tcp_pose`）/ `none` |
| `recorder.ee_pose_parent_frame` / `ee_pose_child_frame` | `base_link` / `gripper_tcp` | tf 查询框架 |
| `recorder.cameras` | `{}` | `{"wrist": {topic, image_key, height, width}}` |
| `recorder.min_frames_per_episode` | 2 | 低于此帧数的 episode 保存时自动丢弃 |
| `recorder.image_writer_processes` / `image_writer_threads` | 0 / 2 | 传给 `LeRobotDataset.create` |
| `recorder.task` | `teleoperation` | 随帧写入的 task 文本 |
| `recorder.state_threshold_rad` | 0.4 | **frame_builder 消费**：夹爪 rad → 0/1 state 阈值 |
| `recorder.record_ur_joints` / `record_ur_ee_pose` / `record_ur_gripper` / `record_action_joints` / `record_action_gripper` | 均 true | **frame_builder 消费**：features 维度开关 |

## 错误处理

| 场景 | 行为（日志原文） |
|---|---|
| lerobot 未安装 | 构造即 `RuntimeError("LeRobot 未安装：source /opt/lerobot_venv/bin/activate")` |
| 相机缺帧（episode 开始前） | error `"相机未收到帧，拒绝开始 episode: [...]。检查相机 topic 后重新按 Enter"`，不启动 |
| 相机断流（录制中） | warn 每相机每 episode 一次，图像帧跳过 |
| 数据集已存在 | warn `"数据集已存在，新建带时间戳: <new_id>"`，不覆盖不追加 |
| 帧数 < min_frames | warn `"少于 N 帧，自动丢弃"`，clear buffer 不落盘 |
| EE 查询失败 | warn 一次，该段填 NaN（`none` 模式无警告） |
| add_frame 异常 | error `"add_frame 失败: {e}"`，帧跳过 |
| 从未开过 episode 就 Q | `finalize()` 跳过 dataset 分支，安全退出 |
| Q 时仍在录制 | 先 `_save_episode()` 再 finalize |

## 测试覆盖（tests/test_integration.py，record 组）

两者都是子进程冒烟：mock cell + fake_master + teleop_node + 以 `python -m ur_teleop.data_recorder` 启动的 recorder（stdin 管道驱动按键，`\n` 即 Enter）。均需 lerobot venv（缺则 `pytest.skip`），断言本地显式 root 布局与日志。

- **`test_record_one_episode`**（test_integration.py:350-398）：Enter → 断言 `_enable_and_wait()` 进入 ACTIVE（enable 由 recorder 的 Enter 先行发出，status 迁移可能先于测试订阅发生，靠 `/ruckig/target_joint_positions` 指令流兜底判定）→ 录 ~150 帧 → **D 丢弃** → Q finalize。断言 `meta/info.json` 存在（create 即落盘）且 **`data/chunk-000` 不存在**（D 负断言：被丢弃的 episode 不落盘，chunk 仅在 save_episode 时创建）、日志含 `finalize`；
- **`test_record_save_episode`**（D 组，test_integration.py:962-1010）：Enter → ACTIVE → 录帧 → **S 保存** → Q finalize。断言 `data/chunk-000/file-000.parquet` 存在、日志含 `已保存` 与 `finalize`。

按键驱动的两个实测陷阱（测试注释已记录）：`d\n` 会被读成丢弃 + 立即开新 episode（Q 时新 episode 被 finalize 保存，实测 48 帧）；`s\n` 同理会保存后又开新 episode。测试必须**单字节写按键、不带 `\n`**。

运行方式：

```bash
colcon test --packages-select ur_teleop --pytest-args "-m integration"
```
