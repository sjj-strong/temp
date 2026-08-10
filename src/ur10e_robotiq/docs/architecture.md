# 系统架构

## 范围

当前系统是一条用于 Ubuntu 24.04、ROS 2 Jazzy 的 Mock 集成链：UR10e 与 2F-85 分别使用 `mock_components/GenericSystem`，FT300 使用 `robotiq_ft_sensor_hardware/RobotiqFTSensorHardware` 的 fake mode。它验证描述、TF、接口、控制器、action、wrench 发布及 MoveIt 规划执行的软件连接，不代表真实硬件或接触动力学已经验证。

## 控制与数据拓扑

```text
MoveIt -> scaled_joint_trajectory_controller -> UR GenericSystem
MoveIt/Action -> robotiq_gripper_controller -> Robotiq GenericSystem
FT300 SensorInterface(fake) -> robotiq_force_torque_sensor_broadcaster -> WrenchStamped
                         \____________________ one /controller_manager ____________________/
```

MoveIt 的 `ur_manipulator` 轨迹映射到 `scaled_joint_trajectory_controller/follow_joint_trajectory`；`gripper` 映射到 `robotiq_gripper_controller/gripper_cmd`。FT300 broadcaster 只发布状态，不是 MoveIt motion controller。

## 单一 `/controller_manager`

`mock_control.launch.py` 只 include 一次上游 `ur_robot_driver/launch/ur_control.launch.py`。该 launch 创建唯一的 `controller_manager/ros2_control_node`，本项目的两个额外 spawner 也明确连接 `/controller_manager`，不会为夹爪或 FT300 再启动 manager。

同一份组合 URDF 在 `include_ros2_control:=true` 时包含三个硬件组件：

| 组件名 | 类型 | 插件/模式 |
| --- | --- | --- |
| `ur` | `system` | `mock_components/GenericSystem` |
| `robotiq_2f85` | `system` | `mock_components/GenericSystem` |
| `robotiq_ft_sensor` | `sensor` | `robotiq_ft_sensor_hardware/RobotiqFTSensorHardware`，`use_fake_mode:=true` |

UR 官方 launch 负责其既有 controller spawning；集成 launch 额外 spawn `robotiq_gripper_controller` 和 `robotiq_force_torque_sensor_broadcaster`。三套硬件接口和全部控制器因此共享同一生命周期与更新循环。

## `/robot_description` 单一真源

唯一模型源文件是：

```text
ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro
```

它统一 include UR、FT300、2F-85 的上游 macro，并且只在这里定义三段安装变换与 `gripper_tcp`。两种启动方式都消费它：

- `display.launch.py` 以 `include_ros2_control:=false` 展开该 Xacro，把同一个 `robot_description` 参数交给 `robot_state_publisher` 和 `joint_state_publisher_gui`。
- `mock_control.launch.py` 把集成的 `robot_state_publisher.launch.py` 作为上游 UR control launch 的 description wrapper；wrapper 以 `include_ros2_control:=true` 展开同一 Xacro，由 `robot_state_publisher` 发布 `/robot_description`，唯一的 controller manager 从这份描述加载三个硬件组件。
- `ur10e_robotiq_moveit.launch.py` 先运行 `wait_for_robot_description`，再启动 `move_group`；MoveIt 配置只生成组合 SRDF 与规划参数，未调用第二次 URDF Xacro。

因此，同一次 Mock 运行中没有“控制模型”和“规划模型”两份独立 URDF。尤其 `gripper_rpy=-3.1415 0 0` 只在组合 Xacro 中定义，Display、Mock control 与 MoveIt 均继承该默认值。

## 启动边界

| 模式 | Xacro 开关 | 启动内容 | 不启动的内容 |
| --- | --- | --- | --- |
| Display | `include_ros2_control:=false` | robot_state_publisher、joint_state_publisher_gui、RViz | ros2_control、controller manager、UR driver |
| Mock control | `include_ros2_control:=true`，三个 fake/mock 开关为 `true` | 官方 UR control 链、唯一 controller manager、三个硬件组件及控制器 | Dashboard、真实 UR 通信、真实夹爪与真实 FT300 通信 |
| MoveIt | 消费已发布的 `/robot_description` | move_group、可选 RViz、OMPL/KDL 与控制器映射 | 第二份 URDF、第二个 controller manager |

Mock control 必须先于 MoveIt 启动。Display 适合独立检查 link、joint、mimic 与 TF；它不是 MoveIt 执行链的前置进程。
