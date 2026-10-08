# Ruckig 关节平滑

Ruckig 对 Alicia 的六关节目标施加速度、加速度和加加速度限制。正常使用由 `teleop.launch.py` 自动启动，无需单独运行；仿真和真机用法一致，见[使用流程](workflow.md)。Xbot 不使用 Ruckig。

## 配置

```yaml
ruckig:
  enabled: true
  control_hz: 500.0
  max_velocity: [0.30, 0.30, 0.30, 0.30, 0.30, 0.30]
  max_acceleration: [0.80, 0.80, 0.80, 0.80, 0.80, 0.80]
  max_jerk: [4.0, 4.0, 4.0, 4.0, 4.0, 4.0]
```

限速数组按 UR 六关节顺序填写，单位分别为 rad/s、rad/s²、rad/s³。示例值需按实际任务调整。

- `joint_impedance` 可用 `enabled: false` 改为直接发布目标。
- `forward_position` 始终使用 Ruckig。
- 修改后重启遥操作。推荐通过 YAML 设置启用状态；launch 的 `use_ruckig` 只控制节点启动，不能同步修改遥操作发布路径。

## 输出与注意事项

| 控制器 | 输出话题 |
| --- | --- |
| forward_position | `/forward_position_controller/commands` |
| joint_impedance | `/joint_impedance_controller/target_joint_state` |

收到完整 UR 反馈后，节点以当前位置初始化并持续输出；不等待遥操作使能，也不监听软件停止。已有目标会继续平滑执行。同一输出话题只保留一个目标发布器。

需要独立运行时，先启动并确认对应硬件和控制器，停止其他 Ruckig 发布器，然后执行（仿真/真机通用）：

```bash
ros2 run ur_teleop ruckig_node --ros-args \
  -p config_file:=/ros2_ws/src/ur_teleop/config/alicia_teleop.yaml \
  -p control_hz:=500.0
```

独立入口的频率默认 100 Hz，需显式传 `control_hz`；通过遥操作 launch 启动时读取 YAML 频率。初始化失败时检查完整六关节反馈；目标无效时检查六项数组和有限数值。
