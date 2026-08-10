# Hardware 与 Controller

## 一个 controller manager

`mock_control.launch.py` 只 include 一次上游 `ur_control.launch.py`，并把组合 controller YAML 传给同一个 `/controller_manager`。Robotiq 夹爪和 FT300 broadcaster 通过 spawner 加入这个 manager；不存在第二个 `/robotiq_controller_manager`。

这个设计让 UR、夹爪和 FT300 共享同一份 `/robot_description`、resource manager 与 `/joint_states`，并避免重复 hardware/interface 注册。

## 三个 hardware component

| Component | Mock 实现 | 主要 interface |
|---|---|---|
| `ur` | 上游 UR `use_mock_hardware:=true`，运行时为 `GenericSystem` | 六轴 position command，position/velocity state，speed scaling 等 UR interface |
| `robotiq_2f85` | Robotiq `use_fake_hardware:=true`，运行时为 `GenericSystem` | `robotiq_85_left_knuckle_joint` position command/state，五个 mimic joint state |
| `robotiq_ft_sensor` | `robotiq_fts_ros2_control` 的 `use_fake_mode:=true` SensorInterface | `force.x/y/z` 与 `torque.x/y/z` state |

查看实际状态：

```bash
source /ros2_ws/install/setup.bash
ros2 control list_hardware_components --controller-manager /controller_manager
ros2 control list_hardware_interfaces --controller-manager /controller_manager
```

## 主要 controller

| Controller | Plugin | 用途 |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | 聚合 UR 和夹爪关节状态到 `/joint_states` |
| `scaled_joint_trajectory_controller` | `ur_controllers/ScaledJointTrajectoryController` | 接收 UR 六轴 `FollowJointTrajectory` |
| `robotiq_gripper_controller` | `parallel_gripper_action_controller/GripperActionController` | 接收主动夹爪关节 `ParallelGripperCommand` |
| `force_torque_sensor_broadcaster` | `force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster` | 上游 UR 内建 TCP F/T interface |
| `robotiq_force_torque_sensor_broadcaster` | `force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster` | 外部 Robotiq FT300 SensorInterface |

运行时检查：

```bash
source /ros2_ws/install/setup.bash
ros2 control list_controllers --controller-manager /controller_manager
ros2 action list -t
```

MoveIt 把 `ur_manipulator` 映射到 `/scaled_joint_trajectory_controller/follow_joint_trajectory`，把 `gripper` 映射到 `/robotiq_gripper_controller/gripper_cmd`。

## UR 内建 F/T 与外部 FT300

两条 wrench 链路是不同的传感器语义，不能混用：

| 链路 | Sensor/interface 名 | Frame | Controller 配置的 topic | 完整 topic |
|---|---|---|---|---|
| UR 内建 TCP F/T | `tcp_fts_sensor` | `tool0_controller` | `ft_data` | `/force_torque_sensor_broadcaster/ft_data` |
| 外部 Robotiq FT300 | `robotiq_ft_sensor` | `robotiq_ft_frame_id` | `wrench` | `/robotiq_force_torque_sensor_broadcaster/wrench` |

UR 内建链路是上游 UR controller 配置的 TCP F/T 接口；FT300 链路则来自安装在 `tool0` 和夹爪之间的外部传感器 hardware component。上层程序应根据物理传感器来源、frame 和 topic 选择数据，不要因为两者都是 `WrenchStamped` 就视为同一测量。

在当前 Mock 中，外部 FT300 的 fake wrench 是六维零值，只能验证 hardware→broadcaster→topic 路由。它不代表物理接触、重力、负载、噪声或标定质量。

## 常用状态检查

```bash
source /ros2_ws/install/setup.bash
ros2 control list_hardware_components --controller-manager /controller_manager
ros2 control list_hardware_interfaces --controller-manager /controller_manager
ros2 control list_controllers --controller-manager /controller_manager
ros2 topic echo --once --timeout 10 /joint_states
ros2 topic echo --once --timeout 10 /robotiq_force_torque_sensor_broadcaster/wrench
```

预期 `ur`、`robotiq_2f85`、`robotiq_ft_sensor` 均为 `active`；`joint_state_broadcaster`、`scaled_joint_trajectory_controller`、`robotiq_gripper_controller` 和 `robotiq_force_torque_sensor_broadcaster` 也应为 `active`。
