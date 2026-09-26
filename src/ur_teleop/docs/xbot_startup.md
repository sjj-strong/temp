# Xbot 模式启动与回 home

Xbot 模式由 `config/xbot_teleop.yaml` 选择。该文件继承现有 `ur_teleop.yaml` 的 UR home、机器人 IP、夹爪端口和传感器设置，因此不会另存一份可能过期的 `home.slave`。原有 `ur_teleop.yaml` 未设置 `teleop.control_source` 时仍按 Alicia 模式运行。

启动阶段 1 时，将 Xbot 配置文件传给原有入口：

```bash
ros2 launch ur_teleop home.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml
```

`home_node` 仍使用 `/scaled_joint_trajectory_controller/follow_joint_trajectory` 将 UR 移至 `home.slave`，但只等待和验证 UR 关节，不订阅 Alicia 状态作为门控，也不发布 Alicia home 命令。看到 `HOME REACHED — UR 已到位` 后，保持该终端运行，再启动阶段 2。

仿真单元使用 `ur10e_robotiq_ft_description` 的组合模型和专用 effort mock，支持轨迹控制器与笛卡尔阻抗控制器互斥切换。实机单元复用组合包的 `real_bringup.launch.py`：UR、FT300 和夹爪均由原有链路启动，不再另起一个独立 FT300 串口驱动。笛卡尔阻抗控制器在 home 阶段只加载为 inactive；后续由手柄模式切换激活。

`gripper_tcp` 的真实安装偏移须在实机启用手柄前测量并与组合描述核对。仿真 effort mock 仅验证接线和状态流转，不验证真实力矩响应。
