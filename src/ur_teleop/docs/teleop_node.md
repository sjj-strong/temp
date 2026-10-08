# Alicia 遥操作节点

实现：`ur_teleop/teleop_node.py`。本节点只负责 Alicia 关节映射、控制器切换和夹爪跟随；Xbot 见[手柄说明](xbot_control.md)。先完成 Home，保持硬件终端，再启动阶段 2，命令见[启动说明](launch.md)。

## 状态机

| 状态 | 行为和转换 |
| --- | --- |
| `WAITING_CELL` | 等主从关节缓存与 controller_manager 的 list/load/switch 服务；30 秒超时退出，返回非零码 |
| `VERIFY_HOME` | 两侧最大关节误差不超过 `home.at_home_tolerance_rad`；不满足时等待并警告 |
| `SETTLING` | 双侧逐周期变化不超过 `settle_motion_threshold_rad`，持续 `settle_time_s` |
| `CAPTURE_OFFSET` | 复制双方实际位置作为映射基准，构造 JointMapper，进入 ARMED |
| `ARMED` | teleop 等 Enter，record 等 `/teleop/enable`；提前到达的 true 使能会锁存 |
| `SWITCHING` | 异步 list → 必要时 load → STRICT switch，future 未完成时立即返回 |
| `ACTIVE` | 发布映射关节目标与录制动作；主臂超过 watchdog 阈值进入 INACTIVE |
| `INACTIVE` | 发布 UR 当前反馈位置作为保持目标；主臂数据恢复后自动回 ACTIVE |

切换目标由 `teleop.controller` 决定。查询阶段只停用实际 active 的轨迹控制器和另一个冲突遥操作控制器；加载成功分支请求停用轨迹控制器。切换失败会重新查询状态再重试，累计 5 次失败回 ARMED；加载失败或服务未就绪也回 ARMED，等待重新使能。

`force_home` 仅把 teleop 的 Home 验证容差设为无穷，不改变 Home 节点或实际位置。完整六轴 Home/遥操作测试仅使用 mock，工作区真机测试仅允许 `wrist_3_joint`。

## 参数与输出

ROS 参数：`config_file`（安装目录 Alicia 配置）、`mode`（空则读 YAML）、`force_home`（false）。

| YAML 参数 | 代码缺省 | 当前 Alicia 配置 |
| --- | --- | --- |
| `teleop.controller` | `forward_position` | `joint_impedance` |
| `teleop.command_rate_hz` | 50 | 500 |
| `teleop.watchdog_timeout_s` | 0.5 秒 | 0.5 秒 |
| `teleop.restore_controller_on_exit` | true | true |
| `home.at_home_tolerance_rad` | 0.05 rad | 0.1 rad |
| `home.settle_time_s` | 2 秒 | 2 秒 |
| `home.settle_motion_threshold_rad` | 0.01 rad | 0.01 rad |

前向位置始终使用 Ruckig；关节阻抗依据 YAML `ruckig.enabled` 选择 Ruckig 或直接发布。输出类型、频率与唯一发布者见[数据流](pipeline.md)。`/teleop/commands` 始终包含六个关节目标和一个二值夹爪信号，数据集是否保存这些字段另由 recorder 开关决定。

## 夹爪与停止

`gripper.enabled` 为真时建立夹爪定时器，频率由 `gripper.fsm_rate_hz` 决定（缺省 10 Hz）。action server 未就绪时逐 tick 重试，节点启动后超过 30 秒仍不就绪才警告并禁用。此定时器不检查 ACTIVE，因此不能认为夹爪一定受 Enter 门控；只受启用开关、软件停止和请求未完成门控约束。

目标变化时发送 `ParallelGripperCommand`；记录的 future 是发送请求的 future，其 done 不代表夹爪运动已完成。具体迟滞和边界见[夹爪控制](gripper_controller.md)。

`/teleop/e_stop=true` 暂停关节与夹爪定时器处理，但不会清除已有 Ruckig 目标、停用控制器或取消已发送 action。退出先发布 `/demonstration=false`，然后在 ACTIVE/INACTIVE/SWITCHING 且恢复开关开启时尝试切回轨迹控制器，最多等待 5 秒；失败或未确认会输出 error。

状态、offset 和成功切换日志由 `debug` 控制；错误、按键提示、配置频率与首次[接口日志](control_interface_logging.md)始终可见。
