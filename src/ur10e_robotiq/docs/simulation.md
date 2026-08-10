# Mock 使用手册

本文介绍 UR10e + Robotiq FT300 + Robotiq 2F-85 组合模型的 Display 和 Mock 软件链路。当前验证范围是 ROS 2 Jazzy 下的 Mock，不是 Gazebo/物理仿真，也不会连接真实 UR、FT300 或夹爪。

## 构建

从已安装的 ROS 和工作区环境开始。为避开已知的默认 `/ros2_ws/build` symlink-install 旧产物冲突，下面使用新建的 `/tmp` build/log 目录：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws
task8_build_root=$(mktemp -d /tmp/ur10e-task8-build.XXXXXX)
colcon --log-base "$task8_build_root/log" build \
  --symlink-install \
  --build-base "$task8_build_root/build" \
  --install-base /ros2_ws/install \
  --packages-up-to ur10e_robotiq_moveit_config
source /ros2_ws/install/setup.bash
```

## Display 模式

Display 只用于检查 URDF、TF 和 mimic joint；它强制 `include_ros2_control:=false`，不会启动 controller manager。在有可用图形显示的终端执行：

```bash
source /ros2_ws/install/setup.bash
ros2 launch ur10e_robotiq_description display.launch.py
```

RViz 的 fixed frame 是 `world`。`joint_state_publisher_gui` 可用于拖动 UR 六轴和夹爪主动关节 `robotiq_85_left_knuckle_joint`。Display 与后文 Mock/MoveIt 是不同的启动流程，不要在同一 ROS domain 同时启动它们。

## Mock control

终端 A：

```bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=177
export ROS_HOME=/tmp/ur10e-task7-main-ros-home
mkdir -p "$ROS_HOME"
ros2 launch ur10e_robotiq_description mock_control.launch.py
```

该 launch 通过一个 `/controller_manager` 管理 UR、Robotiq 2F-85 和 FT300 三个 hardware component。在另一终端查看状态：

```bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=177
export ROS_HOME=/tmp/ur10e-task7-main-query-ros-home
mkdir -p "$ROS_HOME"
ros2 control list_hardware_components --controller-manager /controller_manager
ros2 control list_controllers --controller-manager /controller_manager
ros2 topic echo --once --timeout 10 /joint_states
```

## MoveIt

保持终端 A 的 Mock control 运行，再于终端 B 启动 MoveIt 和 RViz：

```bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=177
export ROS_HOME=/tmp/ur10e-task7-main-ros-home
ros2 launch ur10e_robotiq_moveit_config ur10e_robotiq_moveit.launch.py launch_rviz:=true
```

MoveIt launch 会先等待终端 A 发布的 `/robot_description`，收到后才启动 `move_group` 和 RViz。如果只需无 GUI 规划服务，可将末尾改为 `launch_rviz:=false`。

## ParallelGripperCommand

保持 Mock control 运行，在终端 C 直接调用夹爪 controller action。打开：

```bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=177
export ROS_HOME=/tmp/ur10e-task7-main-action-ros-home
mkdir -p "$ROS_HOME"
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: [robotiq_85_left_knuckle_joint], position: [0.0], velocity: [], effort: []}}" \
  --feedback
```

关闭：

```bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=177
export ROS_HOME=/tmp/ur10e-task7-main-action-ros-home
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: [robotiq_85_left_knuckle_joint], position: [0.7929], velocity: [], effort: []}}" \
  --feedback
```

`0.0` 与 `0.7929` 分别是 SRDF 的 `open` 和 `close` 状态。夹爪只向主动关节发命令，其余五个关节由 URDF mimic 关系联动。

## FT300 topic

保持 Mock control 运行，在查询终端执行：

```bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=177
export ROS_HOME=/tmp/ur10e-task7-main-query-ros-home
ros2 topic info /robotiq_force_torque_sensor_broadcaster/wrench -v
ros2 topic echo --once --timeout 10 /robotiq_force_torque_sensor_broadcaster/wrench
```

该 topic 类型为 `geometry_msgs/msg/WrenchStamped`，frame 为 `robotiq_ft_frame_id`。Mock 中的六维 wrench 是有限零值，只证明 SensorInterface、broadcaster 和 topic 软件链路可用，不表示真实力/力矩、接触或噪声。

## Mock 限制

- UR 和夹爪由 mock/fake hardware 执行，没有动力学、摩擦、负载、碰撞响应或真实速度比例行为。
- FT300 fake mode 只发布零 wrench，不能用于力控或接触算法验证。
- MoveIt 的规划与 controller action 路由可真实执行，但执行结果只代表 Mock 软件链路。
- RViz 是可视化/交互工具，不是物理仿真器。
