# 控制器自动切换

正常使用由遥操作自动切换控制器，无需手动调用服务。启动命令见[完整流程](workflow.md)。

| 输入源 | Home 控制器 | 遥操作控制器 | 切换时机 |
| --- | --- | --- | --- |
| Alicia | scaled_joint_trajectory_controller | forward_position_controller 或 joint_impedance_controller | teleop 按 Enter，record 首次开始录制 |
| Xbot | scaled_joint_trajectory_controller | cartesian_impedance_controller | Home 和反馈满足条件后自动切换 |

Alicia 切换失败会重新查询并重试，累计五次失败后回到等待使能状态；加载失败同样等待重新使能。Xbot 切换失败或超时会锁定遥操作，需排查后重启。

## 查看状态

仿真与真机均可运行：

```bash
ros2 control list_controllers -c /controller_manager
```

Home 阶段轨迹控制器应为 `active`；遥操作阶段所选控制器应为 `active`，冲突控制器应为 `inactive`。

服务找不到时确认 Home 终端仍运行、各终端 ROS 域一致。切换失败时核对硬件支持的命令接口与[控制器配置](controllers.md)，并停止重复的控制进程。
