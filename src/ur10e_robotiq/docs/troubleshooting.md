# 排障指南

所有命令默认先执行：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

若同时运行多套 ROS 图，所有终端必须使用同一个 `ROS_DOMAIN_ID`。以下解决方法均针对
当前 Mock 集成；不要据此连接真机。

## Xacro include 失败

- **现象：** `xacro.XacroException`、`package not found` 或 include 文件不存在。
- **检查命令：**
  `xacro /ros2_ws/src/ur10e_robotiq/ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro include_ros2_control:=false > /tmp/ur10e_robotiq_check.urdf`
- **常见根因：** 未 source Jazzy/工作区，依赖包未构建，或上游包路径发生变化。
- **解决方法：** 先构建
  `colcon build --symlink-install --packages-up-to ur10e_robotiq_description`，重新 source
  `/ros2_ws/install/setup.bash`；再用 `ros2 pkg prefix ur_description`、
  `ros2 pkg prefix robotiq_description` 和
  `ros2 pkg prefix robotiq_ft_sensor_description` 定位缺失依赖。不要复制宏到组合包规避 include。

## package not found

- **现象：** `ros2 launch` 报 `Package 'ur10e_robotiq_description' not found` 或 MoveIt 包找不到。
- **检查命令：**
  `ros2 pkg prefix ur10e_robotiq_description && ros2 pkg prefix ur10e_robotiq_moveit_config`
- **常见根因：** 当前 shell 没有 source 最新 install，或目标包构建失败/被跳过。
- **解决方法：** 在 `/ros2_ws` 执行
  `colcon build --symlink-install --packages-up-to ur10e_robotiq_moveit_config`，随后在每个新终端重新 source install。

## hardware plugin 找不到

- **现象：** `ros2_control_node` 报 class/plugin 无法创建，例如
  `RobotiqFTSensorHardware` 或 `mock_components/GenericSystem` 不存在。
- **检查命令：**
  `ros2 pkg prefix hardware_interface && ros2 pkg prefix robotiq_ft_sensor_hardware && ros2 pkg prefix ur_robot_driver`
- **常见根因：** hardware package 未构建、plugin XML 未安装，或启动 shell 的 overlay 过期。
- **解决方法：** 执行
  `colcon build --symlink-install --packages-up-to robotiq_ft_sensor_hardware ur10e_robotiq_description`，重新 source；保持组合 Xacro 中已验证的 plugin 名称，不用相似字符串替换。

## controller manager 崩溃

- **现象：** `/controller_manager/list_controllers` 消失，所有 spawner 连接失败，或
  `ros2_control_node` 在启动阶段退出。
- **检查命令：**
  `ros2 node list | rg '^/controller_manager$'`，并查看启动终端中最早出现的
  `ERROR`/`FATAL`，而不是只看后续 spawner 超时。
- **常见根因：** `robot_description` 展开失败、hardware 初始化异常、controller YAML 解析失败，或同域中误启动第二个 manager。
- **解决方法：** 先用
  `xacro /ros2_ws/src/ur10e_robotiq/ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro include_ros2_control:=true use_mock_hardware:=true use_fake_hardware:=true use_fake_mode:=true > /tmp/ur10e_robotiq_mock_check.urdf`
  和 `check_urdf /tmp/ur10e_robotiq_mock_check.urdf` 静态检查；再确认只运行一个
  `mock_control.launch.py`，并保持所有 spawner 指向 `/controller_manager`。停止旧进程后重新启动整套 Mock。

## hardware configure/activate 失败

- **现象：** component 停留在 `unconfigured`/`inactive`，日志出现
  `Failed to configure` 或 `Failed to activate`。
- **检查命令：**
  `ros2 control list_hardware_components --controller-manager /controller_manager`
- **常见根因：** Mock 参数未传入、FT300 未使用 fake mode、接口数量错误，或真机路径被误启用。
- **解决方法：** 使用项目的 `mock_control.launch.py`，核实组合 Xacro 展开时
  `use_mock_hardware:=true use_fake_hardware:=true use_fake_mode:=true`；不要在 Mock 验证中改成真实 IP 或关闭 fake mode。

## joint interface mismatch

- **现象：** controller configure 失败，报 command/state interface 不存在或已被占用。
- **检查命令：**
  `ros2 control list_hardware_interfaces --controller-manager /controller_manager`
- **常见根因：** controller 的 joint 名称或 interface 与组合 URDF 不一致，或两个 active controller 同时 claim 相同 command interface。
- **解决方法：** UR 使用六轴 `position` command 与 `position/velocity` state；夹爪只控制
  `robotiq_85_left_knuckle_joint/position`，其余关节由 mimic 关系驱动。恢复
  `ur_controllers_mock.yaml` 中的已验证列表，并只激活一个 UR trajectory controller。

## gripper action 不存在

- **现象：** `/robotiq_gripper_controller/gripper_cmd` 不在 action 列表，MoveIt 报夹爪 controller 不可用。
- **检查命令：**
  `ros2 action list -t | rg '/robotiq_gripper_controller/gripper_cmd|ParallelGripperCommand'`，以及
  `ros2 control list_controllers --controller-manager /controller_manager`
- **常见根因：** `robotiq_gripper_controller` 未加载/激活，插件包缺失，或误用旧的 `GripperCommand` 类型。
- **解决方法：** 构建并 source `parallel_gripper_controller` 依赖，重新启动 Mock；确认 controller 类型为
  `parallel_gripper_action_controller/GripperActionController`，action 类型为
  `control_msgs/action/ParallelGripperCommand`。

## FT wrench 不发布

- **现象：** `/robotiq_force_torque_sensor_broadcaster/wrench` 不存在，或 `echo --once` 超时。
- **检查命令：**
  `ros2 control list_controllers --controller-manager /controller_manager`，
  `ros2 topic info /robotiq_force_torque_sensor_broadcaster/wrench -v`，
  `ros2 topic echo --once --timeout 10 /robotiq_force_torque_sensor_broadcaster/wrench`
- **常见根因：** FT hardware/broadcaster 未 active、topic 名写错、ROS domain 不一致，或 DDS/QoS 环境异常。
- **解决方法：** 确认 `robotiq_ft_sensor` 和 broadcaster 都为 `active`；使用完整 namespace。Mock 正常消息应是 frame
  `robotiq_ft_frame_id` 的六维零值；它不是接触力读数。

## MoveIt controller not found

- **现象：** Plan 成功但执行时报 `Unable to identify any set of controllers` 或 action server 不可用。
- **检查命令：**
  `ros2 action list -t | rg '/scaled_joint_trajectory_controller/follow_joint_trajectory|/robotiq_gripper_controller/gripper_cmd'`
- **常见根因：** MoveIt controller 名称/action namespace 与 ros2_control 不一致，或先启动 MoveIt、后启动 description/controller。
- **解决方法：** 先启动 `mock_control.launch.py`，等待 controllers active，再启动 MoveIt；保持
  `moveit_controllers.yaml` 中 `FollowJointTrajectory/follow_joint_trajectory` 与
  `ParallelGripperCommand/gripper_cmd` 映射不变。

## RobotModel 错误

- **现象：** RViz 显示 `No robot model loaded`、模型缺件或 `/robot_description` 超时。
- **检查命令：**
  `ros2 topic info /robot_description -v` 和
  `ros2 param get /robot_state_publisher robot_description`
- **常见根因：** robot_state_publisher 未运行、MoveIt 与 Mock 不在同一 domain，或错误地启动了第二份不同 URDF。
- **解决方法：** 用 `mock_control.launch.py` 发布唯一组合 description；MoveIt launch 应等待该 topic，不能再增加第二条 Xacro 命令。仅看模型时可单独用 `display.launch.py`。

## TF disconnected

- **现象：** RViz 报 `No transform`，`world`、FT300、夹爪或 `gripper_tcp` 形成断开的树。
- **检查命令：**
  `ros2 run tf2_ros tf2_echo world gripper_tcp` 和
  `ros2 run tf2_tools view_frames`
- **常见根因：** `/joint_states` 未发布、robot_state_publisher 使用了不同 description、或 joint/link 名被改动。
- **解决方法：** 确认 `joint_state_broadcaster` active，并验证链
  `world -> base_link -> base_link_inertia -> shoulder_link -> upper_arm_link -> forearm_link -> wrist_1_link -> wrist_2_link -> wrist_3_link -> flange -> tool0 -> ft300_mounting_plate -> ft300_sensor -> robotiq_85_base_link -> gripper_tcp`；不要用静态 TF 覆盖模型关节。

## SRDF link/joint 不存在

- **现象：** MoveIt 加载时报 SRDF 引用未知 link/joint，planning group 为空。
- **检查命令：**
  `xacro /ros2_ws/src/ur10e_robotiq/ur10e_robotiq_moveit_config/srdf/ur10e_robotiq.srdf.xacro > /tmp/ur10e_robotiq_check.srdf`，再执行
  `xacro /ros2_ws/src/ur10e_robotiq/ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro include_ros2_control:=false > /tmp/ur10e_robotiq_check.urdf` 和
  `check_urdf /tmp/ur10e_robotiq_check.urdf`。
- **常见根因：** SRDF 和组合 URDF 来自不同版本，或上游 link/joint 名变化。
- **解决方法：** 从同一工作区重新构建 description 与 MoveIt config；保持 arm chain
  `base_link -> gripper_tcp` 和 gripper 主动关节 `robotiq_85_left_knuckle_joint`。修改模型后必须重新生成并审查 collision matrix。

## Plan 成功而 Execute 失败

- **现象：** MoveIt 已生成轨迹，但执行返回 controller error，或夹爪在
  `AddTimeOptimalParameterization` 阶段失败。
- **检查命令：**
  `ros2 control list_controllers --controller-manager /controller_manager`、
  `ros2 action list -t`、`ros2 topic echo --once /joint_states`，同时查看 `move_group` 的精确 error code。
- **常见根因：** controller/action 未 active、起始状态过期/碰撞、action mapping 错误；夹爪若缺少 acceleration limit，会在时间参数化阶段失败。
- **解决方法：** 确认两个执行 controller 均 active、当前状态有效且 action 类型匹配；组合 MoveIt 配置必须保留
  `robotiq_85_left_knuckle_joint` 的 `max_acceleration: 1.0`。不要通过扩大碰撞豁免来掩盖执行错误。
