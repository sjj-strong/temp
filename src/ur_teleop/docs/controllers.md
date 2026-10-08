# 遥操作使用的控制器

本包按输入源选择控制器。两种输入源都先由 `scaled_joint_trajectory_controller` 完成 Home，再进入对应遥操作控制器。

| 阶段/输入源 | 配置选择 | 控制器与接口 | 对应文档 |
| --- | --- | --- | --- |
| Home（两种输入源） | Home 启动自动选择 | `scaled_joint_trajectory_controller`，FollowJointTrajectory action | [Home 节点](home_node.md)、[UR 驱动](../../Universal_Robots_ROS2_Driver/ur_robot_driver/doc/usage/startup.rst) |
| Alicia 前向位置 | `teleop.controller: forward_position` | `forward_position_controller`，关节位置数组；Ruckig 平滑 | [启动参数](launch.md)、[Ruckig](ruckig_node.md) |
| Alicia 关节阻抗 | `teleop.controller: joint_impedance` | `joint_impedance_controller`，目标 JointState；按配置使用 Ruckig | [关节阻抗控制器](../../joint_impedance_controller/docs/README.md) |
| Xbot 笛卡尔阻抗 | `teleop.controller: cartesian_impedance` | `cartesian_impedance_controller`，目标 PoseStamped；不启动 Ruckig | [控制器概览](../../cartesian_impedance_controller/docs/01_控制器概览.md)、[使用方式](../../cartesian_impedance_controller/docs/03_使用方式.md) |

## 配置文件

Alicia 默认源码配置选择 `joint_impedance`。其 Home 命令应显式带 `controller:=joint_impedance`，确保 cell 与遥操作节点选择相同控制器。关节阻抗参数与接口见[关节阻抗文档](../../joint_impedance_controller/docs/README.md)。若改用前向位置，应同时修改 YAML 和 Home 命令的 `controller` 参数。

Xbot 的 `xbot.controller_config_file` 是阻抗参数入口：控制和录制共用其参考坐标系；`damping: null` 自动按 `2√K` 计算阻尼，显式填写则使用给定值。仿真应指向 `ur10e_xbot_sim_cartesian_impedance.yaml`，参考 link 为 `base_link`；真机应使用匹配实际硬件的配置，通常为 `base`。当前源码选择 `ur10e_cartesian_impedance_high.yaml`，不要把它直接用作 mock 参数。

刚度、阻尼、误差/wrench/力矩限幅见[参数与安全](../../cartesian_impedance_controller/docs/04_参数与安全.md)；已有配置档位见[刚度阻尼三档配置](../../cartesian_impedance_controller/docs/07_刚度阻尼三档配置.md)。控制器参考 link 会决定目标转换与数据集 action 的坐标系。

## 切换与排查

Home 完成后保持硬件终端运行。遥操作阶段切换到配置控制器；Xbot 在反馈和 Home 条件满足后自动切换，RB 控制是否更新目标；Alicia 在 Enter/采集使能后切换。不要手动另起一套 controller_manager。

```bash
# 只读查看当前状态：
ros2 control list_controllers -c /controller_manager
ros2 topic echo --once /joint_states
```

状态广播器应保持 active，轨迹与遥操作控制器按阶段切换。Xbot Home 阶段阻抗控制器应为 inactive。切换行为见[控制器切换](controller_switcher.md)，组合 mock 说明见[xbot_effort_mock](../../ur10e_robotiq_ft_description/docs/xbot_effort_mock.md)。`debug` 日志开关见[采集日志](data_recorder.md#采集日志与进度)。
