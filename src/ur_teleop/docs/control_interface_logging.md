# 控制接口日志

首次实际调用发布器或夹爪 Action 后，输出 INFO 级 JSON 事件 `control_interface`，即使 `debug: false` 也显示。每个节点中的同一函数和接口只记录一次，不逐帧输出。发布成功指调用 `publish()` 返回，不表示硬件已执行；夹爪日志表示异步请求已发出，不表示目标已被接受。

字段：`function` 为业务函数及 ROS API，`endpoint` 为话题或 Action 名称，`message_type` 为消息或 Action 类型，`controller` 为所选控制器。话题使用发布器返回的实际名称，包含重映射。

| 路径 | 函数接口 | ROS 接口 | 类型 |
|---|---|---|---|
| Alicia 经 Ruckig | `TeleopNode._publish_commands → Publisher.publish` | `/ruckig/target_joint_positions` | `std_msgs/msg/Float64MultiArray` |
| Ruckig 前向位置输出 | `RuckigNode.control_loop → Publisher.publish` | `/forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray` |
| Ruckig 关节阻抗输出 | `RuckigNode.control_loop → Publisher.publish` | `/joint_impedance_controller/target_joint_state` | `sensor_msgs/msg/JointState` |
| Alicia 直接关节阻抗 | `TeleopNode._publish_commands → Publisher.publish` | `/joint_impedance_controller/target_joint_state` | `sensor_msgs/msg/JointState` |
| Xbot 笛卡尔阻抗 | `XbotTeleopNode.tick → Publisher.publish` | `/cartesian_impedance_controller/target_pose` | `geometry_msgs/msg/PoseStamped` |
| 夹爪 | `_gripper_tick` 或 `toggle_gripper → ActionClient.send_goal_async` | 配置的 `gripper.action_server` | `control_msgs/action/ParallelGripperCommand` |

当前 Alicia 配置选择关节阻抗并启用 Ruckig，因此首次控制时 teleop 与 Ruckig 各输出一条接口日志。`/teleop/commands` 是数采话题，不是机械臂控制接口。

验证仅运行离线测试，未向真实机械臂下发控制指令。
