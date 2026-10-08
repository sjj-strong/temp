# 数据流与生命周期

本包通过 `teleop.control_source` 选择 Alicia 或 Xbot。两者都先由 `home.launch.py` 启动持久硬件单元和一次性 Home 节点，再由 `teleop.launch.py` 连接已运行单元；阶段 2 不重启机器人硬件。相机由 `camera.launch.py` 独立启动。可复制命令见[完整流程](workflow.md)。

## Alicia

`/joint_states` 中主臂与 UR 消息交织，节点按各自关节名称分别缓存。Home 后验证双臂位置、等待静止并捕获本次会话 offset，再等待 Enter（teleop）或 `/teleop/enable`（record）切换控制器。

```text
Alicia 关节 → JointMapper（offset、sign、scale、限位）→ teleop_node
  ├─ 启用 Ruckig：/ruckig/target_joint_positions → ruckig_node → 从臂控制器
  ├─ 关闭关节阻抗的 Ruckig：直接发送 JointState → 关节阻抗控制器
  └─ /teleop/commands → data_recorder（仅 record 模式启动）
Alicia Gripper（米）→ 迟滞状态机 → ParallelGripperCommand → Robotiq
```

| 配置 | 输出 |
| --- | --- |
| `forward_position` | Ruckig → `/forward_position_controller/commands`，六维位置数组 |
| `joint_impedance`、`ruckig.enabled: true` | Ruckig → `/joint_impedance_controller/target_joint_state`，JointState |
| `joint_impedance`、`ruckig.enabled: false` | teleop 直接发布同一 JointState 话题 |

映射频率由 `teleop.command_rate_hz` 决定，代码缺省 50 Hz，当前 Alicia YAML 为 500 Hz。Ruckig 的阶段 2 launch 缺省 500 Hz；夹爪轮询代码缺省 10 Hz。配置频率不等于实测频率。Ruckig 在 UR 状态初始化成功后会持续发布当前目标，即使尚未收到 teleop 目标；它不订阅 teleop 状态或软件停止信号。

Alicia 八态和故障处理见[遥操作节点](teleop_node.md)。主臂超时改发 UR 当前反馈位置；数据恢复自动继续。软件停止只暂停 teleop 的两个定时器处理，不能取消已经发送的夹爪请求或保证下游停止运动。

## Xbot

Home 仅移动 UR；单元预加载笛卡尔阻抗控制器为 inactive。阶段 2 启动 `joy_node` 与 `xbot_teleop_node`，不启动 Alicia 或 Ruckig。反馈有效且 UR 在 Home 时自动切换阻抗；已 active 时从当前实测位姿接管。RB 仅使能输入更新，不负责切换控制器。

```text
/joy + UR 关节/TF → Xbot 目标计算 → 转换到控制器参考 link → 工作空间裁剪
  ├─ /cartesian_impedance_controller/target_pose → 笛卡尔阻抗
  └─ /teleop/commands（abs/rel action）→ data_recorder
```

松开 RB 或摇杆回中保持最后目标，故障时重置目标与重新按 RB 的门控见[手柄说明](xbot_control.md)。数据集 action 在控制器参考 link 编码，默认源码配置为 `rel`。夹爪独立使用 A 键切换已接受的二值目标。

## 录制与线程

`mode:=record` 增加独立采集进程。Alicia 的 Enter 开始首段时发一次 `/teleop/enable`；Xbot 通过 `/teleop/record_event` 接收 Menu/Y/B/View 操作，`/teleop/xbot_ready` 为就绪心跳。Alicia 发布的 `/teleop/status` 当前没有被采集器订阅，不能把它解释为实际录制门控。

采集器在后台线程 spin ROS 回调，在主线程读操作并写数据集。遥操作和 Ruckig 通常各自单线程 spin；“所有进程都单线程、回调都不阻塞”不适用于整个包。

Alicia observation 在启用关节和 TCP 时为 6+7=13 维，action 在启用关节目标和夹爪信号时为 6+1=7 维。Xbot observation 按开关拼接；动作 abs 为 7 维、rel 为 6 维，夹爪可再加 1 维。相机选择、保存尺寸、数据就绪差异见[数据采集](data_recorder.md)。

结束录制不会自动关闭硬件或其他遥操作节点；按[完整流程](workflow.md)依次结束各终端。
