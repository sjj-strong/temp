# 关节阻抗控制器

本功能包为 UR 六轴机械臂提供关节空间阻抗控制。控制器申请六个 `joint/effort` 命令接口；真实 UR 硬件驱动在 effort 模式下将力矩命令送入 `direct_torque(...)`。UR 控制柜已执行重力补偿，因此控制律不添加重力项。

控制律为：

```text
tau = K * (q_ref - q) + D * (dq_ref - dq)
```

每个周期还会依次执行参考位置速度限制、位置误差检查、绝对力矩限幅和力矩变化率限制。激活时以当前位置为保持目标；目标超时使用单调时钟判断并重新锁存当前位置；状态无效、越界或写接口失败时写入零力矩并返回错误。

## UR 官方接口依据

- `Universal_Robots_ROS2_Description/urdf/inc/ur_joint_control.xacro` 为六个关节声明了 `effort` 命令接口。
- `Universal_Robots_ROS2_Driver/ur_robot_driver/src/hardware_interface.cpp` 导出这些 effort 接口，并在切换时进入力矩控制模式。
- `Universal_Robots_Client_Library/resources/external_control.urscript` 把该模式的六关节力矩发给 `direct_torque(...)`。
- 驱动代码会拒绝低于 PolyScope 5.23.0 / 10.10.0 的 effort 模式；本控制器不会绕过该检查。

## 接口

- 目标话题：`/joint_impedance_controller/target_joint_state`
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

以 20 Hz 持续发布目标。下面保持 UR 模型的其他初始关节角，只将 `wrist_3_joint` 旋转到 0.2 rad：

```bash
ros2 topic pub --rate 20 /joint_impedance_controller/target_joint_state \
  sensor_msgs/msg/JointState \
  "{name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], position: [0.0, -1.57, 0.0, -1.57, 0.0, 0.2]}"
```

仿真复用官方 UR10e 描述模型，但使用本包的 `JointImpedanceMockSystem`。该模拟硬件按单位惯量模型将 effort 积分为关节速度和位置，同时回传有限的 effort 状态，因此 RViz 能显示阻抗闭环运动，`/joint_states` 也能完整记录位置、速度和力矩，且不会连接机器人。官方 Jazzy `GenericSystem` 的 `calculate_dynamics=true` 模式不接受 effort-only 控制模式，不能直接用于此测试。可用以下命令确认接口与控制器：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces | grep -E 'effort|position|velocity'
```

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
