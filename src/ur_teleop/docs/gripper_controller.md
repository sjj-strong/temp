# 夹爪控制

实现：`ur_teleop/gripper_controller.py`（Alicia 纯逻辑）、`teleop_node.py` 和 `xbot_teleop_node.py`（ROS 执行）。默认打开 0.0 rad、闭合 0.4 rad，都是 Robotiq knuckle 目标；实际动作和 effort 单位由控制器定义。

## Alicia 迟滞

`GripperController.update(alicia_gripper_m)` 的输入单位为米。大于 close_threshold_m 判闭合，小于 open_threshold_m 判张开，两个阈值之间（含等号）保持原目标。只有目标变化才返回 OPEN/CLOSED，否则返回 UNKNOWN。初始 current_target 也是 UNKNOWN。

| `gripper` 参数 | 代码缺省 |
| --- | --- |
| `enabled` | false（纯逻辑模块缺省，YAML 可显式开启） |
| `action_server` | /robotiq_gripper_controller/gripper_cmd |
| `open_pos_rad`、`close_pos_rad` | 0.0、0.4 |
| `open_threshold_m`、`close_threshold_m` | 0.005、0.0125 |
| `max_effort` | 50.0 |
| `fsm_rate_hz` | teleop 轮询缺省 10 Hz |

阈值没有倒置校验，应保证 open 小于 close。`get_knuckle_command()` 对 CLOSED 返回闭合角，其余返回打开角；`get_gripper_command_signal()` 对 CLOSED 返回 1，其他返回 0。

## ROS 请求

Alicia 夹爪定时器在 action server 未就绪时重试，节点启动后超过 30 秒仍未就绪才禁用。它不要求 teleop ACTIVE；软件停止或 enabled=false 才直接返回。保存的 future 是 `send_goal_async` 请求，等待的是发送请求结果，不是实际运动完成，也没有完整的结果处理/自动失败重发。

发送 `ParallelGripperCommand.Goal` 时，command.name 为 robotiq_85_left_knuckle_joint，position 为开合角，effort 为配置值。目标没有变化时不再发送，发送失败后不能假定会自动补发。

Xbot 不读取 Alicia 的迟滞阈值；A 键有效边沿、RB 运动已使能、夹爪反馈新鲜且服务就绪时切换上一次已接受的目标。首次反馈按开闭角中点初始化二值指令，目标拒绝时保持旧指令。请求接受后等待 action 结果，再释放忙碌门控；停止时会尝试取消已保存的 goal。

## 启动开关和数据

`gripper.enabled` 控制执行；Alicia Home 的 `enable_gripper` CLI 仅覆盖阶段 1，阶段 2 仍读取 YAML，应保持一致。Alicia 关节阻抗 mock 只包含六轴，不加载夹爪；前向位置 mock 在开关开启时可加载夹爪。Xbot 使用组合模型，按开关加载夹爪控制器；未加载时外形/TF 仍存在。

`recorder.record_action_gripper` 决定是否保存二值 cmd_gripper（开 0、闭 1），不保存夹爪实测 observation，也不把录制开关解释为硬件开关。详见[数据采集](data_recorder.md)。
