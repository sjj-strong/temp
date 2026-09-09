# UR10e、2F-85 与 FT300 实机启动

`real_bringup.launch.py` 复用官方 `ur_robot_driver/ur_control.launch.py` 启动 UR10e，并额外启动 Robotiq 2F-85 的独立控制器。
FT300 不启动独立节点，而是由组合 URDF 内的 `RobotiqFTSensorHardware` 加载到 UR 的 `/controller_manager`；因此不能同时启动 `ft_sensor_standalone.launch.py`。

## 启动前检查

1. 容器可访问 FT300 设备 `/dev/ttyUSB2`，且当前用户拥有读写权限。
2. 夹爪端口与 FT300 端口不同；以下示例假设夹爪为 `/dev/ttyUSB0`。
3. 机器人示教器已启动包含 External Control 的程序。

## 启动

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch ur10e_robotiq_ft_description real_bringup.launch.py \
  robot_ip:=169.254.138.15 \
  ft_sensor_ftdi_id:=ttyUSB2 \
  gripper_com_port:=/dev/ttyUSB0 \
  launch_rviz:=true
```

成功时，UR 硬件必须显示 `ur_robot_driver/URPositionHardwareInterface`，FT300 必须显示 `use_fake_mode -> 0` 与 `ftdi_id -> ttyUSB2`。不应出现 `mock_components/GenericSystem` 或持续的 `tcp_pose_broadcaster: Invalid pose [nan, ...]`。

夹爪运行在 `/robotiq_controller_manager`，UR 与 FT300 运行在 `/controller_manager`。两个 controller manager 是正常且必要的，不能把 `robotiq_gripper_controller` 加入 UR 的 controller YAML。
