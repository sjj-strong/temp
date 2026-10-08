# Record 配置参数

Alicia 使用 `config/alicia_teleop.yaml`，Xbot 使用 `config/xbot_teleop.yaml`；两者的 `recorder` 都由采集器读取。以下创建参数已按当前实际安装的 `LeRobotDataset.create` 接口核对，源码位于 [lerobot_dataset.py](../../lerobot/src/lerobot/datasets/lerobot_dataset.py)。升级 LeRobot 后应重新核对接口。

## 采集数量与采集循环

```yaml
recorder:
  num_episodes: 50
  fps: 20
  min_frames_per_episode: 2
```

`num_episodes` 是本包参数，不直接传给 LeRobot：`0` 不限制段数；正整数表示本次新数据集需要成功保存的 episode 数。丢弃、过短片段和保存失败不计数。每段仍按原有 Enter/Menu 开始、S/Y 保存，不自动生成或开始下一段。保存够目标后采集器发布结束通知、finalize 并退出，硬件终端和其他遥操作节点不会随采集器退出而自动关闭。

进度显示 `episode=3/50`、本段帧数和实际采集 Hz；未指定目标时显示 `episode=3`。Q/View 长按仍可提前保存并结束。Xbot 收到结束通知后锁定手柄输入；Alicia 保持原有遥操作退出方式，结束时仍需 Ctrl-C 停止相应终端。

| 参数 | 类型/默认值 | 作用 |
| --- | --- | --- |
| `num_episodes` | 非负整数，`0` | 成功保存的目标段数，0 不限 |
| `fps` | 正整数，代码默认 `50` | 采集循环目标 Hz，同时写入数据集元数据；当前 Alicia 配置 60、Xbot 配置 20 |
| `min_frames_per_episode` | 正整数，`2` | 低于此帧数的片段自动丢弃 |
| `task` | 字符串，`teleoperation` | 每帧保存的任务描述 |
| `data_timeout_s` | 有限正数，`0.5` | Xbot 的输入、启用 observation 和相机数据新鲜度阈值，单位秒 |

## LeRobot 数据集创建参数

下列字段直接传给当前 `LeRobotDataset.create`；其中 `repo_id`、`root`、`robot_type` 和 `fps` 由采集器组装，其余通过同名字段透传。

| 参数 | 类型/默认值 | 作用与生效条件 |
| --- | --- | --- |
| `repo_id` | 字符串 | 数据集标识，例如 `my_user/ur10e_xbot`；不自动上传 |
| `root` | 路径或空 | 本地目录；空时使用 `$HF_LEROBOT_HOME/{repo_id}`；目录重名自动添加连字符时间后缀 |
| `robot_type` | 字符串或 `null` | 数据集元数据中的机器人类型 |
| `use_videos` | bool，`true` | 相机图像存 MP4；false 存图像；没有启用相机时不产生视频 |
| `tolerance_s` | 有限非负数，`0.0001` | LeRobot 时间戳间隔校验容差，单位秒；不是本包数据超时阈值 |
| `image_writer_processes` | 非负整数，`0` | 异步图像写入进程数；0 使用线程 |
| `image_writer_threads` | 非负整数，`2` | 异步图像写入线程数；本包保留原默认 2，LeRobot 当前接口默认 0 |
| `video_backend` | 非空字符串或 `null` | 数据集视频读取后端，如 `pyav`；null 自动选择，不决定视频编码器 |
| `batch_encoding_size` | 正整数，`1` | 非流式视频每批编码的 episode 数；finalize 刷新剩余片段 |
| `vcodec` | 非空字符串，`libsvtav1` | 视频编码器，当前 LeRobot 支持的值由其实现及环境决定，如 `libsvtav1`、`h264`、`hevc`、`auto` |
| `metadata_buffer_size` | 正整数，`10` | 写入前缓冲的 episode 元数据数量；finalize 刷新剩余记录 |
| `streaming_encoding` | bool，`false` | 开启后在采集期间实时编码视频；需要视频图像特征，不走常规图像暂存/批量编码路径 |
| `encoder_queue_maxsize` | 正整数，`30` | 流式编码时每台相机待编码的最大缓冲帧数 |
| `encoder_threads` | 正整数或 `null` | 每个视频编码器的线程数；null 自动选择 |

`features` 也属于 LeRobot 创建接口，但不提供任意 YAML 覆盖：本包根据下面的数据字段开关和相机配置自动生成，保证特征维度与实际帧一致。读取既有数据集使用的 `revision`、`episodes`、`delta_timestamps`、`image_transforms` 等不是 `create` 参数，不放入 record 创建配置。

## 数据字段

| 参数 | 适用范围 | 作用 |
| --- | --- | --- |
| `record_action_joints` | Alicia | 是否保存 UR 关节目标，默认 true；Xbot 固定保存完整位姿 action |
| `record_action_gripper` | 两种输入源 | 是否在 action 末尾保存二值夹爪指令，不控制夹爪执行 |
| `record_ur_joints` | Alicia | 是否保存 UR 实测关节位置，默认 true |
| `record_ur_ee_pose` | Alicia | 是否保存 TF 查询的实测末端位姿，默认 true |
| `action_space` | Xbot | 固定为 `cartesian_pose`；Alicia 使用关节目标 |
| `action_mode` | Xbot | `abs`：最终绝对目标；`rel`：相对同周期实测位姿的增量 |
| `record_joint_position` | Xbot | 是否保存六关节实测位置 |
| `record_joint_velocity` | Xbot | 是否保存六关节实测速度 |
| `record_joint_effort` | Xbot | 是否保存 effort 原始值，UR 上可能是电流，不当作实测力矩 |
| `record_tcp_pose` | Xbot | 是否保存实测 TCP 的 xyz+xyzw |
| `record_wrench` | Xbot | 是否保存六维原始力/力矩与参考坐标系 |
| `ee_pose_parent_frame` | Alicia | 末端位姿的父 link；Xbot 自动使用控制器参数里的参考 link |
| `ee_pose_child_frame` | 两种输入源 | observation 的 TCP link，默认 `tool0` |
| `cameras` | 两种输入源 | 需要保存的相机映射；空映射不保存图像 |

每个 `cameras.<名称>` 支持 `enabled`、`topic`、`image_key`、`height`、`width`，例如：

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

相机发布端口、曝光和预览由独立相机配置/调参工具管理，不属于 LeRobot 创建参数。完整字段格式与按键见[数据采集](data_recorder.md)，相机准备见[相机调试](camera_inspector.md)。`debug` 是配置顶层开关，不在 `recorder` 内；其作用见[采集日志](data_recorder.md#采集日志与进度)。
