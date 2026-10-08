# 关节阻抗控制器

本功能包为 UR 六轴机械臂提供关节空间阻抗控制。控制器申请六个 `joint/effort` 命令接口；真实 UR 硬件驱动在 effort 模式下将力矩命令送入 `direct_torque(...)`。UR 控制柜已执行重力补偿，因此控制律不添加重力项。

控制律为：

```text
tau = K * (q_ref - q) + D * (dq_ref - dq)
```

每个周期还会依次执行逐关节参考位置速度限制、由受限参考位置差分得到参考速度并进行一阶低通滤波、位置误差检查、绝对力矩限幅和力矩变化率限制。目标消息中的 `velocity` 仅做格式和有限性校验，不直接进入控制律，因此不能绕过参考限速。激活时以当前位置为保持目标；目标超时使用单调时钟判断并重新锁存当前位置；状态无效、越界或写接口失败时写入零力矩并返回错误。

## 参数与保护

| 参数                                            | 含义                                                                             |
| ----------------------------------------------- | -------------------------------------------------------------------------------- |
| `max_reference_speed`                         | 六个关节内部参考位置的最大变化速度，单位 rad/s。每轴独立限制。                   |
| `reference_velocity_filter_time_constant`     | 由受限参考位置差分得到的参考速度的一阶低通时间常数，单位 s；`0` 表示关闭滤波。 |
| `max_position_error`                          | 每关节参考位置与实测位置的最大允许偏差，单位 rad。                               |
| `joint_position_min` / `joint_position_max` | 当前状态和外部目标均须满足的六关节位置安全边界，单位 rad。                       |
| `max_torque` / `max_torque_rate`            | 每关节绝对力矩和力矩变化率硬上限，单位 Nm、Nm/s。                                |

## UR 官方接口依据

- `Universal_Robots_ROS2_Description/urdf/inc/ur_joint_control.xacro` 为六个关节声明了 `effort` 命令接口。
- `Universal_Robots_ROS2_Driver/ur_robot_driver/src/hardware_interface.cpp` 导出这些 effort 接口，并在切换时进入力矩控制模式。
- `Universal_Robots_Client_Library/resources/external_control.urscript` 把该模式的六关节力矩发给 `direct_torque(...)`。
- 驱动代码会拒绝低于 PolyScope 5.23.0 / 10.10.0 的 effort 模式；本控制器不会绕过该检查。

## 接口

- 仿真目标话题：`/joint_impedance_sim/joint_impedance_controller/target_joint_state`
- 实机默认目标话题：`/joint_impedance_controller/target_joint_state`
- 消息类型：`sensor_msgs/msg/JointState`
- `name` 和 `position` 必须包含全部六个 UR 关节；`velocity` 可为空，空值按零处理。
- 命令接口：六个 `<joint>/effort`
- 状态接口：六个 `<joint>/position` 和 `<joint>/velocity`

## RViz 仿真测试

构建后启动：

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to joint_impedance_controller
source install/setup.bash
ros2 launch joint_impedance_controller rviz_test.launch.py
```

仿真默认使用 `/joint_impedance_sim` 命名空间，会同时隔离 controller manager、`robot_description`、关节状态和 TF，因此可以与已启动的 UR 实机驱动并存。
修改过本包后，必须停止旧的 `rviz_test.launch.py`、重新构建并再次 `source install/setup.bash`，然后启动新 launch；已运行的进程不会自动更新命名空间。

以 20 Hz 持续发布目标。下面保持 UR 模型的其他初始关节角，只将 `wrist_3_joint` 旋转到 0.2 rad：

```bash
ros2 topic pub --rate 20 /joint_impedance_sim/joint_impedance_controller/target_joint_state \
  sensor_msgs/msg/JointState \
  "{name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], position: [0.0, -1.57, 0.0, -1.57, 0.0, 0.2]}"
```

仿真复用官方 UR10e 描述模型，但使用本包的 `JointImpedanceMockSystem`。该模拟硬件按单位惯量模型将 effort 积分为关节速度和位置，同时回传有限的 effort 状态，因此 RViz 能显示阻抗闭环运动，`/joint_impedance_sim/joint_state_broadcaster/joint_states` 也能完整记录位置、速度和力矩，且不会连接机器人。它还支持与 effort 互斥的 position 命令接口，供 `ur_teleop` 的 home 阶段轨迹控制器使用；controller_manager 切换到阻抗控制器后才开始 effort 积分。官方 Jazzy `GenericSystem` 不提供这一可用于阻抗闭环的 effort 动力学，不能直接用于遥操阻抗仿真。可用以下命令确认接口与控制器：

```bash
ros2 control list_controllers -c /joint_impedance_sim/controller_manager
ros2 control list_hardware_interfaces -c /joint_impedance_sim/controller_manager \
  | grep -E 'effort|position|velocity'
```

## RViz 中机器人不运动

先确认发布的是仿真命名空间下的新话题，而不是实机控制器的全局话题：

```bash
ros2 topic info -v \
  /joint_impedance_sim/joint_impedance_controller/target_joint_state
```

`Subscription count` 必须为 `1`，且订阅者命名空间必须是 `/joint_impedance_sim`。再检查仿真的关节状态是否改变：

```bash
ros2 topic echo --once \
  /joint_impedance_sim/joint_state_broadcaster/joint_states
```

若关节数值不变，检查控制器必须都为 `active`：

```bash
ros2 control list_controllers -c /joint_impedance_sim/controller_manager
```

若关节数值已变但 RViz 不动，检查状态话题必须有 `robot_state_publisher` 订阅：

```bash
ros2 topic info -v \
  /joint_impedance_sim/joint_state_broadcaster/joint_states
```

如果 `ros2 node list` 中仍出现全局 `/joint_impedance_controller`，说明旧仿真进程还在运行。回到启动它的终端按 `Ctrl-C`，然后重新执行本节的构建、source 和 launch 命令。不要通过全局 `/controller_manager` 判断仿真控制器状态，该名称可能属于正在运行的 UR 实机驱动。

## 保存测试日志

脚本会创建时间戳目录，同时保存 launch 文本日志、ROS 节点日志和 rosbag：

```bash
ros2 run joint_impedance_controller run_rviz_test.sh

# 或指定目录
ros2 run joint_impedance_controller run_rviz_test.sh /tmp/my_joint_impedance_test
```

对应输出为 `launch.log`、`ros/` 和 `bag/`。停止测试时使用 `Ctrl-C`，等待 rosbag 打印 `Recording stopped` 后再关闭终端，以确保缓存已落盘；脚本会将这种正常中断转换为成功退出。
脚本会在启动 launch 前设置 `ROS_LOG_DIR`，因此 launch 自身和各节点日志都会保存到该测试目录中。

也可以直接启动并指定：

```bash
ros2 launch joint_impedance_controller rviz_test.launch.py \
  record_bag:=true \
  bag_output:=/tmp/joint_impedance_bag \
  ros_log_dir:=/tmp/joint_impedance_logs
```

## 实机边界

当前仅完成 RViz/mock 和单元测试，不代表完成实机安全验证。实机使用前必须校验机器人标定、关节顺序、急停、保护停止和限幅，并确保所有位置/速度/轨迹控制器已停用。根据工作区安全约束，首次实机验证只允许给 `wrist_3_joint` 下发小幅目标，其余关节目标必须保持为激活时的当前位置。

官方 `force_mode_controller` 对应控制柜内置笛卡尔 `force_mode(...)`，不是本控制器使用的接口。本控制器使用的是关节 `effort` 接口，对应 `direct_torque(...)`。

实机驱动正常运行后，先以 inactive 状态加载：

`type` 必须位于 YAML 中 `/**/joint_impedance_controller` 的参数块。Jazzy
的 spawner 仅从该控制器块读取类型；若放到 `controller_manager` 块，会在
manager 日志中报 `The 'type' param was not defined`。

```bash
ros2 run controller_manager spawner joint_impedance_controller \
  --controller-manager /controller_manager \
  --param-file "$(ros2 pkg prefix joint_impedance_controller)/share/joint_impedance_controller/config/ur10e_joint_impedance.yaml" \
  --inactive
```

核对六个 effort 接口均为 available 后，严格切换运动控制器：

```bash
ros2 control switch_controllers \
  --deactivate scaled_joint_trajectory_controller \
  --activate joint_impedance_controller \
  --strict
```

不要同时激活 `forward_effort_controller`、轨迹、位置或速度控制器。UR 的 direct torque 要求兼容的软件版本；本工作区驱动会在切换 effort 模式时检查版本，不满足条件会拒绝切换。


## 实机使用命令

以下流程仅适用于已完成风险评估的 UR10e。它只用于首次小幅验证
`wrist_3_joint`：不得将示例扩展为其他关节、末端位姿、轨迹或笛卡尔速度控制。开始前确保急停可用、机器人周边无人且无接触负载；示教器上的 External Control 程序必须已安装、选中并可运行。

### 1. 构建并启动真实 UR 驱动

在 ROS PC 上构建并加载本包与 UR 驱动：

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to joint_impedance_controller
source install/setup.bash
```

将 `<机器人IP>` 替换为控制柜 IP，在独立终端启动真实硬件驱动。`use_mock_hardware` 必须保持为 `false`；启动后在示教器上运行 External Control 程序，确认驱动已连接且机器人处于正常运行状态：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur10e \
  robot_ip:=<机器人IP> \
  use_mock_hardware:=false \
  launch_rviz:=false
```

实际机器人必须使用由 `ur_calibration` 为该序列号生成的标定参数；不要把默认运动学参数当作标定结果。PolyScope 版本还必须不低于 `5.23.0`（PolyScope X 不低于 `10.10.0`），否则驱动会拒绝 effort 模式。

### 2. 核对驱动和接口

在第二个终端加载工作区环境后，先确认不是 mock 硬件，并核对控制器状态和六个 effort 接口：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 control list_controllers -c /controller_manager
ros2 control list_hardware_interfaces -c /controller_manager | \
  rg 'shoulder_pan_joint/effort|shoulder_lift_joint/effort|elbow_joint/effort|wrist_1_joint/effort|wrist_2_joint/effort|wrist_3_joint/effort'
ros2 topic echo --once /joint_states
```

六个 effort 接口都必须显示为 `available`。默认的
`scaled_joint_trajectory_controller` 应为 `active`，而 `forward_effort_controller`、
`forward_position_controller`、`forward_velocity_controller` 与 `force_mode_controller` 必须不是 `active`。若看到 `mock_components/GenericSystem`、连接失败、保护停止或接口缺失，立即停止流程，排除问题后从本节重新开始。

### 3. 加载并切换控制器

先以 inactive 状态加载，确认它出现于控制器列表中：

```bash
ros2 run controller_manager spawner joint_impedance_controller \
  --controller-manager /controller_manager \
  --param-file "$(ros2 pkg prefix joint_impedance_controller)/share/joint_impedance_controller/config/ur10e_joint_impedance.yaml" \
  --inactive

ros2 control list_controllers -c /controller_manager
```

确认参数、关节顺序、限位和急停后，严格切换。激活瞬间控制器会将当前六关节位置锁存为保持参考，并先输出零力矩：

```bash
ros2 control switch_controllers \
  --deactivate scaled_joint_trajectory_controller \
  --activate joint_impedance_controller \
  --strict

ros2 control list_controllers -c /controller_manager
```

输出中 `joint_impedance_controller` 必须是 `active`，`scaled_joint_trajectory_controller` 必须不是 `active`。切换失败时不要重复发布目标；先检查上一步列出的互斥控制器与控制柜状态。

### 4. 仅验证 wrist_3_joint

先从 `/joint_states` 记录**刚激活后**的六个关节位置。下面命令中的前五项必须原样填写为该次读取的当前位置；最后一项仅允许在当前 `wrist_3_joint` 位置基础上增加或减小不超过 `0.02` rad。以 20 Hz 持续发布是为了避免 `command_timeout`（默认 `0.5` s）触发保持：

```bash
ros2 topic pub --rate 20 /joint_impedance_controller/target_joint_state \
  sensor_msgs/msg/JointState \
  "{name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], position: [<当前肩关节>, <当前肩升降关节>, <当前肘关节>, <当前腕1关节>, <当前腕2关节>, <当前wrist_3关节±0.02>]}"
```

发布期间只观察 `wrist_3_joint` 的小幅旋转；若发现任何其他关节运动、振荡、异常声音、保护停止或人员进入风险区域，立即按 `Ctrl-C` 停止发布并执行下一节的停用命令。不要在本流程中修改前五个关节的目标值，也不要发送轨迹、位姿或笛卡尔速度命令。

### 5. 停用并恢复默认控制器

验证结束或出现异常时，先停止目标发布（发布终端按 `Ctrl-C`），再停用阻抗控制器并恢复轨迹控制器：

```bash
ros2 control switch_controllers \
  --deactivate joint_impedance_controller \
  --activate scaled_joint_trajectory_controller \
  --strict

ros2 control list_controllers -c /controller_manager
```

停用时控制器会写入零力矩。若上述切换失败或机器人未进入预期安全状态，应使用示教器急停/保护停止处置，不要继续发送 ROS 控制命令。

通过 `ur_teleop` 启动时，`damping: null` 会在传给 spawner 前转换为逐轴 `D_i = 2√K_i`（等效质量／惯量归一为 1）。显式六维阻尼数组优先，保持原值。直接使用其他启动文件时仍需提供数值数组，ROS 参数文件不能直接解析 null。
