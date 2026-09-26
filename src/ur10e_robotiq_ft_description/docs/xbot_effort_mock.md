# Xbot 遥操作的仿真描述

组合 Xacro 保持同一条 `base_link → gripper_tcp` 运动链，实机仍使用原有 UR 驱动和 FT300 硬件。仅在 Xbot 仿真启动时传入 `xbot_effort_mock:=true`、`use_mock_hardware:=true`、`ft_sensor_use_fake_mode:=true`，UR 六轴的 `ros2_control` 硬件才改为 `joint_impedance_controller/JointImpedanceMockSystem`。它同时支持 home 阶段的位置接口，以及笛卡尔阻抗阶段的 effort 接口，并将 effort 积分为关节状态。

夹爪继续使用组合描述中原有的模拟硬件；`gripper_tcp` 与实机使用同一个链接定义。该仿真只用于检查状态机、坐标变换和控制器接线，不代表真实 UR 的动力学或接触行为。若不传 `xbot_effort_mock`，组合描述行为不变。

可用以下命令核对展开结果：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
xacro /ros2_ws/src/ur10e_robotiq_ft_description/urdf/ur10e_robotiq_ft.urdf.xacro \
  name:=ur10e_robotiq_ft300 use_mock_hardware:=true \
  ft_sensor_use_fake_mode:=true xbot_effort_mock:=true | \
  rg 'JointImpedanceMockSystem|gripper_tcp'
```
