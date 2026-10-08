# Home 节点

实现：`ur_teleop/home_node.py`。通过 `home.launch.py` 启动一次性 Home 节点和持续运行的 cell；完成后节点退出，cell 保持运行。Alicia 移动主从双臂，Xbot 仅移动 UR。入口命令及两阶段顺序见[完整流程](workflow.md)。完整 Home 是六轴轨迹，不能用于本工作区的真机测试。

## 执行顺序

1. 真机先提示启动示教器 External Control 并等回车；仿真跳过。等待确认没有固定超时，期间仍处理 ROS 回调。
2. 最多 30 秒等待 UR 六轴状态和轨迹 action server，二者须持续就绪至少 2 秒；不查询 controller_manager 状态。
3. 向 `/scaled_joint_trajectory_controller/follow_joint_trajectory` 发送单点 Home：名称按 UR 标准六轴顺序，位置为 `home.slave`，到达时间为 `move_duration_s`。
4. action 接受窗口共 10 秒；每次等待最多 5 秒，未接受后等待 0.5 秒重试。已接受后按 `move_timeout_s` 等结果，只有 `SUCCESSFUL` 继续。
5. Alicia 此时持续发布 `/joint_commands`（六个主臂弧度目标 + Gripper 0–1000 指令值）；Xbot 不发布 Alicia Home 指令。
6. 在另一段 `move_timeout_s` 窗口内验证双方（Xbot 仅 UR）都在容差内并连续保持 `verify_duration_s`，输出 `HOME REACHED` 后返回 0；失败返回 1 并打印目标/反馈。

各阶段超时分别计时，`move_timeout_s` 不是从启动开始的总体截止时间。控制器拒绝首条 goal 是可能的启动情况，不是必然现象。

## 参数

ROS 参数为 `config_file`、`sim`。`sim` 缺省读取所选 YAML，Home launch 对 Alicia 传入 CLI sim，对 Xbot 使用配置 sim。

| `home` 字段 | 代码缺省 | 说明 |
| --- | --- | --- |
| `slave` | 必填六项 | UR Home，rad |
| `master` | Alicia 必填六项 | Alicia Home，rad |
| `master_gripper_value` | 1000 | Alicia Gripper 指令，1000 表示张开 |
| `at_home_tolerance_rad` | 0.05 | 最大逐关节位置误差，rad；当前两个 YAML 为 0.1 |
| `move_duration_s` | 8 | 轨迹点到达时间，秒 |
| `move_timeout_s` | 30 | action 结果和随后到位验证分别使用的等待窗口，秒；当前 YAML 为 60 |
| `verify_duration_s` | 2 | 到位后连续验证时间，秒 |

`settle_time_s` 和 `settle_motion_threshold_rad` 由 Alicia teleop 的 offset 捕获使用，不参与 Home 节点验证。`force_home` 也只是 teleop 参数，不能让失败的 Home 变成成功。
