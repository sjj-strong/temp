# MoveIt 配置与使用

## RobotModel 与规划组

SRDF 定义两个 group：

- `ur_manipulator`：从 `base_link` 到 `gripper_tcp` 的 chain，包含 UR 六个主动关节。运动学插件为 `kdl_kinematics_plugin/KDLKinematicsPlugin`，search resolution 为 `0.005`，timeout 为 `0.005 s`。
- `gripper`：只包含主动关节 `robotiq_85_left_knuckle_joint`；其余夹爪关节由 mimic 关系驱动。命名状态为 `open=0.0` 和 `close=0.7929`。

end effector 名为 `robotiq_2f85`，挂在 `robotiq_85_base_link`，属于 `ur_manipulator`。机械臂位姿目标的末端 link 是 `gripper_tcp`，不是 `tool0` 或 `ft300_sensor`。

## `/robot_description` 单一真源

`mock_control.launch.py` 通过组合 Xacro 展开完整 UR10e + FT300 + 2F-85 模型，并发布 `/robot_description`。MoveIt launch 先运行 `ur_robot_driver/wait_for_robot_description`；只在收到该 topic 后才启动 `move_group` 和可选 RViz。

`MoveItConfigsBuilder` 只显式加载本包的 SRDF/规划配置，不调用 `.robot_description()`，也不另行执行 Xacro。因此 controller manager、robot state publisher、MoveIt 和 RViz 消费同一份组合 `/robot_description`，避免双重模型漂移。

## Controller mapping

MoveIt 使用 `moveit_simple_controller_manager/MoveItSimpleControllerManager`：

| MoveIt group | Controller | Action 类型 | Action 路径 | Joint |
|---|---|---|---|---|
| `ur_manipulator` | `scaled_joint_trajectory_controller` | `FollowJointTrajectory` | `/scaled_joint_trajectory_controller/follow_joint_trajectory` | UR 六轴 |
| `gripper` | `robotiq_gripper_controller` | `ParallelGripperCommand` | `/robotiq_gripper_controller/gripper_cmd` | `robotiq_85_left_knuckle_joint` |

夹爪主动关节在 URDF 中保留 velocity limit；MoveIt `joint_limits.yaml` 另加 `has_acceleration_limits: true` 和 `max_acceleration: 1.0`，用于时间参数化。

## 启动

终端 A 先启动 Mock control：

```bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=177
export ROS_HOME=/tmp/ur10e-task7-main-ros-home
mkdir -p "$ROS_HOME"
ros2 launch ur10e_robotiq_description mock_control.launch.py
```

终端 B 再启动 MoveIt：

```bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=177
export ROS_HOME=/tmp/ur10e-task7-main-ros-home
ros2 launch ur10e_robotiq_moveit_config ur10e_robotiq_moveit.launch.py launch_rviz:=true
```

## Plan 与 Execute

RViz MotionPlanning 的初始 group 为 `ur_manipulator`：

1. 选择 `ur_manipulator`，设置一个无碰的 joint goal 或拖动 `gripper_tcp` interactive marker 设置 pose goal。
2. 先点击 `Plan` 检查路径；确认无碰后点击 `Plan & Execute`。
3. 选择 `gripper` group，调用命名状态 `open` 或 `close`，再规划并执行。
4. 执行后用 `ros2 control list_controllers` 确认两个目标 controller 仍为 `active`，并用 `ros2 topic echo --once --timeout 10 /joint_states` 检查关节值。

`Plan` 成功只证明生成了轨迹；`Execute` 还依赖 controller mapping、action server、时间参数化和 controller 的 active 状态。

## 自碰矩阵

最终 SRDF 的自碰豁免由官方 MoveIt Setup Assistant updater 以 `100000` trials 独立生成两次并比较；稳定集合为 `17 Adjacent + 79 Never`。安装链保留以下 `Adjacent`：

- `wrist_3_link` / `ft300_mounting_plate`
- `ft300_mounting_plate` / `ft300_sensor`
- `ft300_sensor` / `robotiq_85_base_link`

另仅加入 updater `--default` inspection 报告的两对内部夹爪 `Default`：

```xml
<disable_collisions link1="robotiq_85_left_finger_tip_link" link2="robotiq_85_left_inner_knuckle_link" reason="Default"/>
<disable_collisions link1="robotiq_85_right_finger_tip_link" link2="robotiq_85_right_inner_knuckle_link" reason="Default"/>
```

除这两对外没有其他 `Default`，也没有为让规划通过而人工添加非相邻 arm-to-gripper 豁免。
