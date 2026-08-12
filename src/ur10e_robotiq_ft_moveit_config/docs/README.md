# `ur10e_robotiq_ft_moveit_config` 使用说明

本包为 UR10e、Robotiq 2F-85 和 Robotiq FT300 提供单一的 ROS 2 Control 与 MoveIt 启动入口。
它只包含集成层的 xacro、控制器和 launch 配置；不会修改以下官方功能包：

- `Universal_Robots_ROS2_Driver`
- `ros2_robotiq_gripper`
- `rq_fts_ros2_driver`

组合模型的末端链路为：`tool0 → FT300 → robotiq_ft_frame_id → 2F-85 → gripper_tcp`。

## 1. 构建与环境

在工作空间根目录执行：

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur10e_robotiq_ft_moveit_config
source install/setup.bash
```

运行时必须先执行上述 `source install/setup.bash`，确保 xacro 能找到本包以及 UR、夹爪和 FT300 的依赖包。

## 2. Mock 模式

无真实硬件时使用 mock 模式。它加载 UR、夹爪和 FT300 各自的 fake/mock 硬件系统，并启动 MoveIt：

```bash
ros2 launch ur10e_robotiq_ft_moveit_config demo.launch.py launch_rviz:=true
```

也可直接使用主启动文件：

```bash
ros2 launch ur10e_robotiq_ft_moveit_config bringup.launch.py \
  use_mock_hardware:=true \
  launch_rviz:=true
```

启动后核对 controller 状态：

```bash
ros2 control list_controllers
```

至少应看到以下 controller 为 `active`：

- `joint_state_broadcaster`
- `scaled_joint_trajectory_controller`
- `robotiq_gripper_trajectory_controller`
- `robotiq_force_torque_sensor_broadcaster`

## 3. 真实硬件模式

启动前确认：

1. UR10e 网络可达，示教器已按 UR 官方流程加载并运行 External Control 程序。
2. 2F-85 串口设备可访问，例如 `/dev/ttyUSB0`。
3. 已通过官方 FT300 包确认 `ftdi_id`，不要把占位符 `_` 用于真实硬件。

示例：

```bash
ros2 launch ur10e_robotiq_ft_moveit_config bringup.launch.py \
  robot_ip:=169.254.138.15 \
  gripper_com_port:=/dev/ttyUSB1 \
  ftdi_id:=ttyUSB3 \
  launch_rviz:=true
```

常用参数：

| 参数                  | 默认值           | 说明                                         |
| --------------------- | ---------------- | -------------------------------------------- |
| `robot_ip`          | `0.0.0.0`      | UR 控制柜 IP；真实模式必须指定。             |
| `gripper_com_port`  | `/dev/ttyUSB0` | 2F-85 串口设备。                             |
| `ftdi_id`           | `_`            | FT300 的 FTDI 标识；真实模式必须指定真实值。 |
| `use_mock_hardware` | `false`        | `true` 时切换三套硬件到 mock/fake 模式。   |
| `ft_max_retries`    | `100`          | FT300 初始化/读取的最大重试次数。            |
| `ft_read_rate`      | `10`           | FT300 读取频率。                             |
| `tf_prefix`         | 空               | TF 与 UR 关节前缀；单机器人保持默认值。      |
| `reverse_ip`        | `0.0.0.0`      | UR 驱动反向连接所使用的本机 IP。             |
| `launch_rviz`       | `true`         | 是否启动带 MoveIt 配置的 RViz。              |

## 4. MoveIt 与 ROS 接口

MoveIt 中的 `ur_manipulator` 规划组控制 UR 六个关节；`gripper` 规划组控制 `robotiq_85_left_knuckle_joint`。

| 功能             | 接口                                                               |
| ---------------- | ------------------------------------------------------------------ |
| UR MoveIt 执行   | `/scaled_joint_trajectory_controller/follow_joint_trajectory`    |
| 夹爪 MoveIt 执行 | `/robotiq_gripper_trajectory_controller/follow_joint_trajectory` |
| 关节状态         | `/joint_states`                                                  |
| FT300 wrench     | `/robotiq_force_torque_sensor_broadcaster/ft300_wrench`          |
| FT300 测量坐标系 | `robotiq_ft_frame_id`                                            |

夹爪使用单关节 `FollowJointTrajectory` controller，而不是官方 standalone launch 中的
`ParallelGripperCommand` action。这是为了让 MoveIt Simple Controller Manager 可以直接执行夹爪规划。

UR 六轴使用官方 MoveIt 配置中的保守加速度上限 `5.0 rad/s²`；夹爪主动关节使用 `1.0 rad/s²`。
这些限制是轨迹时间参数化的必需条件，首次真实硬件执行仍应在 RViz 中将速度与加速度缩放保持较低值。

可单独测试夹爪轨迹接口：

```bash
ros2 action send_goal \
  /robotiq_gripper_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [robotiq_85_left_knuckle_joint], points: [{positions: [0.0], time_from_start: {sec: 1}}]}}"
```

## 5. 常见问题

| 现象                          | 检查项                                                                                                 |
| ----------------------------- | ------------------------------------------------------------------------------------------------------ |
| `controller_manager` 未出现 | 确认已`source install/setup.bash`，并查看 launch 中 xacro 展开错误。                                 |
| UR controller 未激活          | 确认示教器处于 Remote Control、External Control 正在运行，且`robot_ip`、网络与 `reverse_ip` 正确。 |
| 夹爪无法启动                  | 检查`gripper_com_port`、串口权限和夹爪供电；先用官方夹爪包验证设备。                                 |
| FT300 无数据                  | 检查`ftdi_id`、USB 权限和 `ft_read_rate`；先用官方 FT300 包验证。                                  |
| MoveIt 无法做笛卡尔/姿态规划  | 确认`ur_manipulator` 的 KDL 运动学插件已加载，并检查终端的 MoveIt 日志。                             |

真实硬件首次动作应使用较低的 MoveIt 速度与加速度缩放比例，并确认碰撞模型、TCP 和装配变换已经过现场标定。
