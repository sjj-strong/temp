# UR10e + FT300 + 2F-85 Mock Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不修改任何上游源码的前提下，交付可显示、可 Mock 控制、可发布 FT300 fake wrench、可控制 2F-85、可由 MoveIt 规划并执行的 UR10e 组合机器人。

**Architecture:** `ur10e_robotiq_description` 是唯一组合 Xacro 和 ros2_control 描述来源。它复用 UR、FT300、2F-85 的上游 macro，并由一个官方 UR control launch 创建唯一 `/controller_manager`。`ur10e_robotiq_moveit_config` 只提供组合 SRDF、MoveIt 参数、控制器映射和 RViz，不重新生成第二份 URDF。

**Tech Stack:** Ubuntu 24.04、ROS 2 Jazzy、xacro、robot_state_publisher、ros2_control、mock_components/GenericSystem、UR Driver 3.8.0、MoveIt 2、KDL、parallel_gripper_action_controller、force_torque_sensor_broadcaster。

## Global Constraints

- 只创建或修改 `/ros2_ws/src/ur10e_robotiq` 内的文件。
- 不修改 `Universal_Robots_ROS2_Driver`、`Universal_Robots_ROS2_Description`、`ros2_robotiq_gripper`、`rq_fts_ros2_driver` 的任何源码。
- `ros2_robotiq_gripper` 的既有未提交修改是用户认可的本地基线，必须保留。
- 所有模式共用 `ur10e_robotiq.urdf.xacro`；禁止维护重复的 display/control/MoveIt 机器人模型。
- 组合 Mock 系统只允许一个 `/controller_manager`。
- 显示模式的 `include_ros2_control` 必须为 `false`；Mock 模式必须为 `true`。
- UR 和 2F-85 Mock 必须使用各自上游 macro 中的 `mock_components/GenericSystem`；FT300 必须使用上游 `use_fake_mode:=true`。
- 临时安装值固定为 `ft_xyz="0 0 0"`、`ft_rpy="0 0 0"`、`gripper_xyz="0 0 0"`、`gripper_rpy="-3.1415 0 0"`、`tcp_xyz="0 0 0.15"`、`tcp_rpy="0 0 0"`，并明确为未标定值。`gripper_rpy` 只在组合 Xacro 中定义，Display、Mock control 与 MoveIt 统一继承该默认值。
- MoveIt 的臂组 tip 必须是 `gripper_tcp`；夹爪组只能包含 `robotiq_85_left_knuckle_joint`。
- 每个任务结束时记录变更文件、原因、实际执行命令、观察输出、PASS/FAIL 和剩余问题；没有运行的检查写“未执行”。

---

## File Structure

| 路径 | 职责 |
| --- | --- |
| `ur10e_robotiq_description/CMakeLists.txt` | 安装 description 包的 `urdf`、`config`、`launch`、`rviz` 资源。 |
| `ur10e_robotiq_description/package.xml` | 声明 Xacro、UR/Robotiq/FT300 description、ros2_control 与显示依赖。 |
| `ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro` | 唯一完整模型、三个安装变换、可选三个 ros2_control 元素。 |
| `ur10e_robotiq_description/config/ur_controllers_mock.yaml` | UR 原 controller 配置副本与 Robotiq gripper/FT300 broadcaster 扩展。 |
| `ur10e_robotiq_description/launch/display.launch.py` | 纯模型显示。 |
| `ur10e_robotiq_description/launch/robot_state_publisher.launch.py` | 对官方 `ur_rsp.launch.py` 的组合模型 wrapper。 |
| `ur10e_robotiq_description/launch/mock_control.launch.py` | 对官方 `ur_control.launch.py` 的 Mock wrapper，外加两个 spawner。 |
| `ur10e_robotiq_description/rviz/display.rviz` | 纯模型显示 RViz 初始配置。 |
| `ur10e_robotiq_moveit_config/CMakeLists.txt` | 安装 MoveIt 配置、SRDF 与 launch。 |
| `ur10e_robotiq_moveit_config/package.xml` | 声明 MoveIt、KDL、simple controller manager 和 description runtime 依赖。 |
| `ur10e_robotiq_moveit_config/srdf/ur10e_robotiq.srdf.xacro` | arm、gripper、end effector、named state 和重新审查后的 collision matrix。 |
| `ur10e_robotiq_moveit_config/config/*.yaml` | KDL、joint limit、OMPL、MoveIt controller 映射和执行参数。 |
| `ur10e_robotiq_moveit_config/config/moveit.rviz` | MoveIt MotionPlanning RViz 配置。 |
| `ur10e_robotiq_moveit_config/launch/ur10e_robotiq_moveit.launch.py` | 等待组合 description 后启动 move_group、可选 RViz。 |
| `docs/*.md` | 架构、使用、控制器、FT300、排障和真实验证记录。 |

### Task 1: 冻结基线、构建 FT300 依赖并建立 Description 包骨架

**Files:**
- Create: `ur10e_robotiq_description/CMakeLists.txt`
- Create: `ur10e_robotiq_description/package.xml`
- Create: `ur10e_robotiq_description/urdf/.gitkeep`
- Create: `ur10e_robotiq_description/config/.gitkeep`
- Create: `ur10e_robotiq_description/launch/.gitkeep`
- Create: `ur10e_robotiq_description/rviz/.gitkeep`
- Create: `docs/validation.md`

**Interfaces:**
- Consumes: 上游 package `ur_description`、`ur_robot_driver`、`robotiq_description`、`robotiq_ft_sensor_description`、`robotiq_ft_sensor_hardware`。
- Produces: 可被 ament 找到的 `ur10e_robotiq_description`，供任务 2 的 Xacro、任务 3 的 display 与任务 4 的 control launch 使用。

- [ ] **Step 1: 保存四个上游仓库的基线状态**

Run:

```bash
git -C /ros2_ws/src/Universal_Robots_ROS2_Driver status --short
git -C /ros2_ws/src/Universal_Robots_ROS2_Description status --short
git -C /ros2_ws/src/ros2_robotiq_gripper status --short
git -C /ros2_ws/src/rq_fts_ros2_driver status --short
```

Expected: UR 与 FT300 仓库无本任务引入的改动；Robotiq gripper 的既有状态保留并记录为用户基线。

- [ ] **Step 2: 先构建 FT300 package，验证此前 package-not-found 的失败条件已消失**

Run:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to robotiq_ft_sensor_hardware
source /ros2_ws/install/setup.bash
ros2 pkg prefix robotiq_ft_sensor_description
ros2 pkg prefix robotiq_ft_sensor_hardware
```

Expected: 两个 `ros2 pkg prefix` 都返回 `/ros2_ws/install/...`。若构建失败，只记录编译器输出并停在此任务；不修改 FT300 上游文件。

- [ ] **Step 3: 创建最小 ament 文件**

Write `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.22)
project(ur10e_robotiq_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
```

Write `package.xml` with these runtime dependencies:

```xml
<exec_depend>joint_state_publisher_gui</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>rviz2</exec_depend>
<exec_depend>xacro</exec_depend>
<exec_depend>controller_manager</exec_depend>
<exec_depend>force_torque_sensor_broadcaster</exec_depend>
<exec_depend>parallel_gripper_action_controller</exec_depend>
<exec_depend>ur_description</exec_depend>
<exec_depend>ur_robot_driver</exec_depend>
<exec_depend>robotiq_description</exec_depend>
<exec_depend>robotiq_ft_sensor_description</exec_depend>
<exec_depend>robotiq_ft_sensor_hardware</exec_depend>
```

- [ ] **Step 4: 构建骨架并确认唯一 package 名称**

Run:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to ur10e_robotiq_description
source /ros2_ws/install/setup.bash
ros2 pkg prefix ur10e_robotiq_description
```

Expected: 构建成功，prefix 为 `/ros2_ws/install/ur10e_robotiq_description`。

- [ ] **Step 5: 记录 Stage 0/1 结果并提交**

Append `docs/validation.md` with date, ROS distro, exact commands, package prefix outputs and PASS/FAIL. Then run:

```bash
git -C /ros2_ws/src add ur10e_robotiq/ur10e_robotiq_description ur10e_robotiq/docs/validation.md
git -C /ros2_ws/src commit -m "chore: scaffold UR10e Robotiq integration"
```

### Task 2: 实现单一组合 Xacro 与 URDF 静态验证

**Files:**
- Create: `ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro`
- Modify: `docs/validation.md`

**Interfaces:**
- Consumes: `xacro:ur_robot`、`xacro:ur_ros2_control`、`xacro:robotiq_ft300`、`xacro:robotiq_fts_ros2_control`、`xacro:robotiq_gripper`。
- Produces: 参数 `include_ros2_control`、`use_mock_hardware`、`use_fake_hardware`、`use_fake_mode`、六个安装参数，以及 `gripper_tcp` link；供所有后续 launch 与 MoveIt 使用。

- [ ] **Step 1: 写入会失败的最小结构检查命令**

在 Xacro 文件尚不存在时运行：

```bash
source /ros2_ws/install/setup.bash
xacro /ros2_ws/src/ur10e_robotiq/ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro include_ros2_control:=false > /tmp/ur10e_robotiq.urdf
```

Expected: FAIL，提示文件不存在。

- [ ] **Step 2: 写入组合 Xacro 的参数、include 与 UR arm 段**

Use these exact declarations and include paths:

```xml
<xacro:arg name="ur_type" default="ur10e"/>
<xacro:arg name="include_ros2_control" default="false"/>
<xacro:arg name="use_mock_hardware" default="true"/>
<xacro:arg name="use_fake_hardware" default="true"/>
<xacro:arg name="use_fake_mode" default="true"/>
<xacro:arg name="ft_xyz" default="0 0 0"/>
<xacro:arg name="ft_rpy" default="0 0 0"/>
<xacro:arg name="gripper_xyz" default="0 0 0"/>
<xacro:arg name="gripper_rpy" default="-3.1415 0 0"/>
<xacro:arg name="tcp_xyz" default="0 0 0.15"/>
<xacro:arg name="tcp_rpy" default="0 0 0"/>
<xacro:include filename="$(find ur_description)/urdf/ur_macro.xacro"/>
<xacro:include filename="$(find ur_robot_driver)/urdf/ur.ros2_control.xacro"/>
<xacro:include filename="$(find robotiq_ft_sensor_description)/urdf/robotiq_ft300.urdf.xacro"/>
<xacro:include filename="$(find robotiq_ft_sensor_description)/urdf/robotiq_fts.ros2_control.xacro"/>
<xacro:include filename="$(find robotiq_description)/urdf/robotiq_2f_85_macro.urdf.xacro"/>
```

Create `world`, call `ur_robot` with the four official UR parameter YAML paths under `ur_description/config/$(arg ur_type)/`, and set the macro parent to `world`.

- [ ] **Step 3: 连接 FT300、2F-85 与 TCP，并只在控制模式插入 ros2_control**

Use these macro calls and fixed TCP joint:

```xml
<xacro:robotiq_ft300 parent="tool0" tf_prefix="">
  <origin xyz="$(arg ft_xyz)" rpy="$(arg ft_rpy)"/>
</xacro:robotiq_ft300>
<xacro:robotiq_gripper name="robotiq_2f85" prefix="" parent="ft300_sensor"
  include_ros2_control="$(arg include_ros2_control)"
  use_fake_hardware="$(arg use_fake_hardware)">
  <origin xyz="$(arg gripper_xyz)" rpy="$(arg gripper_rpy)"/>
</xacro:robotiq_gripper>
<link name="gripper_tcp"/>
<joint name="robotiq_85_base_link-gripper_tcp" type="fixed">
  <parent link="robotiq_85_base_link"/>
  <child link="gripper_tcp"/>
  <origin xyz="$(arg tcp_xyz)" rpy="$(arg tcp_rpy)"/>
</joint>
```

Wrap `ur_ros2_control` and `robotiq_fts_ros2_control` in the same `xacro:if` that checks `include_ros2_control`; set FT sensor name to `robotiq_ft_sensor`, `use_fake_mode` to `$(arg use_fake_mode)` and UR `use_mock_hardware` to `$(arg use_mock_hardware)`.

- [ ] **Step 4: 生成两个 flattened URDF 并检查模型不变量**

Run:

```bash
source /ros2_ws/install/setup.bash
xacro /ros2_ws/src/ur10e_robotiq/ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro include_ros2_control:=false > /tmp/ur10e_robotiq_display.urdf
check_urdf /tmp/ur10e_robotiq_display.urdf
xacro /ros2_ws/src/ur10e_robotiq/ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro include_ros2_control:=true use_mock_hardware:=true use_fake_hardware:=true use_fake_mode:=true > /tmp/ur10e_robotiq_mock.urdf
check_urdf /tmp/ur10e_robotiq_mock.urdf
rg -n 'gripper_tcp|ft300_sensor|robotiq_85_base_link|shoulder_pan_joint' /tmp/ur10e_robotiq_display.urdf
rg -n '<ros2_control' /tmp/ur10e_robotiq_display.urdf /tmp/ur10e_robotiq_mock.urdf
xmllint --xpath 'string(/robot/joint[@name="robotiq_85_base_joint"]/origin/@rpy)' /tmp/ur10e_robotiq_display.urdf
```

Expected: 两份 `check_urdf` 成功；display 文件没有 `<ros2_control>`；mock 文件恰有三个 `<ros2_control>`；四个关键 link/joint 名都存在；`robotiq_85_base_joint` 的 flattened URDF `rpy` 为 `-3.1415 0 0`。

- [ ] **Step 5: 记录 Stage 2 并提交**

Append generated-URDF commands and outputs to `docs/validation.md`, then run:

```bash
git -C /ros2_ws/src add ur10e_robotiq/ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro ur10e_robotiq/docs/validation.md
git -C /ros2_ws/src commit -m "feat: add combined UR10e FT300 2F85 model"
```

### Task 3: 实现纯模型 Display launch 并验证 TF 与 mimic

**Files:**
- Create: `ur10e_robotiq_description/launch/display.launch.py`
- Create: `ur10e_robotiq_description/rviz/display.rviz`
- Modify: `docs/validation.md`

**Interfaces:**
- Consumes: `ur10e_robotiq.urdf.xacro include_ros2_control:=false`。
- Produces: `/robot_description`、`/joint_states`、完整静态与可动 TF tree；供人工模型和夹爪 mimic 验证。

- [ ] **Step 1: 写入 display launch 的失败检查**

Run before creating the file:

```bash
source /ros2_ws/install/setup.bash
ros2 launch ur10e_robotiq_description display.launch.py
```

Expected: FAIL，提示 launch file 不存在。

- [ ] **Step 2: 创建明确禁用 ros2_control 的 launch**

Implement `display.launch.py` with `Command([FindExecutable(name="xacro"), " ", PathJoinSubstitution([...]), " include_ros2_control:=false"])`; pass the resulting `robot_description` to these nodes:

```python
Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[robot_description])
Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui", parameters=[robot_description])
Node(package="rviz2", executable="rviz2", arguments=["-d", display_rviz])
```

Set RViz fixed frame to `world`, add RobotModel and TF displays, and add the `joint_states` display.

- [ ] **Step 3: 启动 display 并执行可观察检查**

In terminal A:

```bash
source /ros2_ws/install/setup.bash
ros2 launch ur10e_robotiq_description display.launch.py
```

In terminal B:

```bash
source /ros2_ws/install/setup.bash
ros2 run tf2_tools view_frames
ros2 topic echo --once /robot_description
```

Expected: RViz 同时显示 UR10e、FT300、2F-85；GUI 移动 `shoulder_pan_joint` 到 `wrist_3_joint` 时 UR 运动；移动 `robotiq_85_left_knuckle_joint` 时五个 mimic joint 同步；TF tree 连通至 `gripper_tcp`。

- [ ] **Step 4: 记录 Stage 3 并提交**

Append launch log summary and `view_frames` result path to `docs/validation.md`, then run:

```bash
git -C /ros2_ws/src add ur10e_robotiq/ur10e_robotiq_description/launch/display.launch.py ur10e_robotiq/ur10e_robotiq_description/rviz/display.rviz ur10e_robotiq/docs/validation.md
git -C /ros2_ws/src commit -m "feat: add combined robot display launch"
```

### Task 4: 实现单 controller_manager 的 Mock control 启动与配置

**Files:**
- Create: `ur10e_robotiq_description/config/ur_controllers_mock.yaml`
- Create: `ur10e_robotiq_description/launch/robot_state_publisher.launch.py`
- Create: `ur10e_robotiq_description/launch/mock_control.launch.py`
- Modify: `docs/validation.md`

**Interfaces:**
- Consumes: 上游 `ur_control.launch.py`、`ur_rsp.launch.py`、官方 `ur_controllers.yaml`、组合 Xacro。
- Produces: 一个 `/controller_manager`，包含 UR、FT300、Robotiq 三个 hardware component；`robotiq_gripper_controller` 与 `robotiq_force_torque_sensor_broadcaster` 可被 spawner 加载。

- [ ] **Step 1: 写入对官方 YAML 的本地副本和明确扩展**

Copy upstream YAML to the local file, then add under `controller_manager.ros__parameters`:

```yaml
robotiq_gripper_controller:
  type: parallel_gripper_action_controller/GripperActionController
robotiq_force_torque_sensor_broadcaster:
  type: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster
```

Append these exact controller parameters:

```yaml
robotiq_gripper_controller:
  ros__parameters:
    joint: robotiq_85_left_knuckle_joint
    state_interfaces: [position, velocity]
    allow_stalling: true
    stall_timeout: 0.05
    goal_tolerance: 0.02

robotiq_force_torque_sensor_broadcaster:
  ros__parameters:
    sensor_name: robotiq_ft_sensor
    state_interface_names: [force.x, force.y, force.z, torque.x, torque.y, torque.z]
    frame_id: robotiq_ft_frame_id
    topic_name: wrench
```

Keep the upstream UR `force_torque_sensor_broadcaster` block unchanged.

- [ ] **Step 2: 创建能正确传递 Mock 选项的本地 RSP wrapper**

源码已确认官方 `ur_rsp.launch.py` 的 Xacro 命令不转发 `include_ros2_control`、`use_fake_hardware` 或 `use_fake_mode`；不能用 include 得到正确的组合 ros2_control 描述。仅在本 package 创建等价的轻量 wrapper，不修改 `ur_rsp.launch.py`。

Implement a local `robot_state_publisher` Node with this xacro command:

```python
robot_description = {
    "robot_description": ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", combined_xacro,
            " ur_type:=", LaunchConfiguration("ur_type"),
            " robot_ip:=", LaunchConfiguration("robot_ip"),
            " include_ros2_control:=true",
            " use_mock_hardware:=true",
            " use_fake_hardware:=true",
            " use_fake_mode:=true",
        ]),
        value_type=str,
    )
}
```

Declare `ur_type` with default `ur10e` and `robot_ip` with default `0.0.0.0`, then pass `robot_description` to `robot_state_publisher/robot_state_publisher`.

- [ ] **Step 3: 创建 Mock control wrapper**

Include `$(find ur_robot_driver)/launch/ur_control.launch.py` with:

```python
{
    "ur_type": "ur10e",
    "robot_ip": "0.0.0.0",
    "use_mock_hardware": "true",
    "launch_dashboard_client": "false",
    "launch_rviz": "false",
    "controllers_file": controllers_file,
    "description_launchfile": rsp_launch_file,
}
```

Add two `controller_manager/spawner` nodes after the include, each with `--controller-manager /controller_manager` and controller name respectively `robotiq_gripper_controller` and `robotiq_force_torque_sensor_broadcaster`.

- [ ] **Step 4: 验证配置能解析，并启动一套 manager**

Run:

```bash
source /ros2_ws/install/setup.bash
ros2 launch ur10e_robotiq_description mock_control.launch.py
```

Expected: `ros2_control_node` 不报告重复 hardware name、plugin 找不到或 joint interface 冲突；只存在 `/controller_manager/list_controllers`，不存在 `/robotiq_controller_manager/list_controllers`。

- [ ] **Step 5: 记录 Stage 4 并提交**

Append control log summary to `docs/validation.md`, then run:

```bash
git -C /ros2_ws/src add ur10e_robotiq/ur10e_robotiq_description/config/ur_controllers_mock.yaml ur10e_robotiq/ur10e_robotiq_description/launch/robot_state_publisher.launch.py ur10e_robotiq/ur10e_robotiq_description/launch/mock_control.launch.py ur10e_robotiq/docs/validation.md
git -C /ros2_ws/src commit -m "feat: add unified mock ros2 control"
```

### Task 5: 验证 hardware、controller、FT300 与夹爪 Mock 链路

**Files:**
- Modify: `ur10e_robotiq_description/config/ur_controllers_mock.yaml`（仅当实际 controller interface 报错时调整本地副本）
- Modify: `docs/validation.md`

**Interfaces:**
- Consumes: Task 4 的运行系统。
- Produces: 可复现的硬件接口、controller、wrench 和 gripper action 验证证据；供 MoveIt controller mapping 使用。

- [ ] **Step 1: 运行 hardware component 与 interface 检查**

Run while Task 4 launch 在运行：

```bash
source /ros2_ws/install/setup.bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

Expected: 三个 active hardware component；UR 六个 joint 的 position command/state interface；`robotiq_85_left_knuckle_joint/position` command interface；`robotiq_ft_sensor/force.x` 到 `robotiq_ft_sensor/torque.z` 六个 state interface；`joint_state_broadcaster`、`scaled_joint_trajectory_controller`、`robotiq_gripper_controller`、`robotiq_force_torque_sensor_broadcaster` 都为 active。

- [ ] **Step 2: 用 controller activation 失败作为接口契约检查**

若 `robotiq_gripper_controller` 未 active，运行：

```bash
ros2 control set_controller_state robotiq_gripper_controller active
ros2 control list_hardware_interfaces
```

Expected: 失败输出必须明确列出缺失接口。仅在此时修改本地 `ur_controllers_mock.yaml`，使其不请求 mock Xacro 未导出的 effort 或 velocity command interface；再次执行同一命令直到 controller active。

- [ ] **Step 3: 验证 FT300 fake wrench**

Run:

```bash
ros2 topic info /robotiq_force_torque_sensor_broadcaster/wrench -v
ros2 topic echo --once /robotiq_force_torque_sensor_broadcaster/wrench
```

Expected: 类型 `geometry_msgs/msg/WrenchStamped`；`header.frame_id` 为 `robotiq_ft_frame_id`；六个数都是有限数，fake mode 下均为 `0.0` 可通过。

- [ ] **Step 4: 验证夹爪 Open、Mid、Close action 与 joint state**

Run:

```bash
ros2 action list -t
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 0.0}}" --feedback
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.4, max_effort: 0.0}}" --feedback
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.7929, max_effort: 0.0}}" --feedback
ros2 topic echo --once /joint_states
```

Expected: action list 含 `/robotiq_gripper_controller/gripper_cmd`；每个 goal 返回成功；最后消息中 `robotiq_85_left_knuckle_joint` 接近 `0.7929`；RViz 中 mimic links 同步闭合。

- [ ] **Step 5: 验证 UR trajectory Mock**

Run:

```bash
ros2 action list -t | rg scaled_joint_trajectory_controller
ros2 topic echo --once /joint_states
```

Expected: scaled trajectory FollowJointTrajectory action 存在，六个 UR joint 有有限 position state。

- [ ] **Step 6: 记录 Stage 5 并提交**

Append all command output excerpts and controller states to `docs/validation.md`, then run:

```bash
git -C /ros2_ws/src add ur10e_robotiq/ur10e_robotiq_description/config/ur_controllers_mock.yaml ur10e_robotiq/docs/validation.md
git -C /ros2_ws/src commit -m "test: validate gripper and FT300 mock chain"
```

### Task 6: 建立组合 MoveIt 配置与 SRDF

**Files:**
- Create: `ur10e_robotiq_moveit_config/CMakeLists.txt`
- Create: `ur10e_robotiq_moveit_config/package.xml`
- Create: `ur10e_robotiq_moveit_config/srdf/ur10e_robotiq.srdf.xacro`
- Create: `ur10e_robotiq_moveit_config/config/kinematics.yaml`
- Create: `ur10e_robotiq_moveit_config/config/joint_limits.yaml`
- Create: `ur10e_robotiq_moveit_config/config/ompl_planning.yaml`
- Create: `ur10e_robotiq_moveit_config/config/moveit_controllers.yaml`
- Modify: `docs/validation.md`

**Interfaces:**
- Consumes: Task 2 的完整 link/joint 名称与 Task 5 的 active action endpoints。
- Produces: `ur_manipulator`、`gripper`、`robotiq_2f85` semantic model，和两个 MoveIt motion-controller 映射。

- [ ] **Step 1: 创建 MoveIt 包安装规则和依赖**

Write `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.22)
project(ur10e_robotiq_moveit_config)

find_package(ament_cmake REQUIRED)
install(DIRECTORY config srdf launch DESTINATION share/${PROJECT_NAME})
ament_package()
```

Write `package.xml` with runtime dependencies `moveit_configs_utils`, `moveit_kinematics`, `moveit_planners`, `moveit_ros_move_group`, `moveit_ros_visualization`, `moveit_simple_controller_manager`, `ur_robot_driver`, `ur10e_robotiq_description`, `xacro` and `warehouse_ros_sqlite`.

- [ ] **Step 2: 写入 SRDF 的规划组与 end effector 定义**

Use these exact semantic elements:

```xml
<group name="ur_manipulator"><chain base_link="base_link" tip_link="gripper_tcp"/></group>
<group name="gripper"><joint name="robotiq_85_left_knuckle_joint"/></group>
<end_effector name="robotiq_2f85" parent_link="robotiq_85_base_link" group="gripper" parent_group="ur_manipulator"/>
<group_state name="open" group="gripper"><joint name="robotiq_85_left_knuckle_joint" value="0.0"/></group_state>
<group_state name="close" group="gripper"><joint name="robotiq_85_left_knuckle_joint" value="0.7929"/></group_state>
```

Copy the six-joint `home`、`up` 和 `test_configuration` values from upstream UR SRDF. Preserve only justified UR adjacent collision pairs initially; after Task 7 regenerate the matrix and replace the initial list with the inspected result.

- [ ] **Step 3: 复制规划参数并把 arm tip 的 KDL 配置固定为组合组名**

Copy upstream `joint_limits.yaml` and `ompl_planning.yaml` into this package. Write `kinematics.yaml`:

```yaml
ur_manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

Write `moveit_controllers.yaml` with exactly these controller entries:

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
moveit_simple_controller_manager:
  controller_names: [scaled_joint_trajectory_controller, robotiq_gripper_controller]
  scaled_joint_trajectory_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
  robotiq_gripper_controller:
    type: ParallelGripperCommand
    action_ns: gripper_cmd
    default: true
    joints: [robotiq_85_left_knuckle_joint]
```

- [ ] **Step 4: 执行 MoveIt 配置解析失败检查和构建检查**

Before creating the package, run `ros2 pkg prefix ur10e_robotiq_moveit_config` and record package-not-found. After creation, run:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to ur10e_robotiq_moveit_config
source /ros2_ws/install/setup.bash
ros2 pkg prefix ur10e_robotiq_moveit_config
xacro /ros2_ws/src/ur10e_robotiq/ur10e_robotiq_moveit_config/srdf/ur10e_robotiq.srdf.xacro > /tmp/ur10e_robotiq.srdf
rg -n 'gripper_tcp|robotiq_85_left_knuckle_joint|robotiq_2f85' /tmp/ur10e_robotiq.srdf
```

Expected: package prefix resolves; SRDF contains exact group, TCP and end-effector names.

- [ ] **Step 5: 记录 Stage 6 并提交**

Append build and SRDF checks to `docs/validation.md`, then run:

```bash
git -C /ros2_ws/src add ur10e_robotiq/ur10e_robotiq_moveit_config ur10e_robotiq/docs/validation.md
git -C /ros2_ws/src commit -m "feat: add combined robot MoveIt semantics"
```

### Task 7: 实现 MoveIt launch、重新生成 collision matrix 并验证规划/执行

**Files:**
- Create: `ur10e_robotiq_moveit_config/launch/ur10e_robotiq_moveit.launch.py`
- Create: `ur10e_robotiq_moveit_config/config/moveit.rviz`
- Modify: `ur10e_robotiq_moveit_config/srdf/ur10e_robotiq.srdf.xacro`
- Modify: `docs/validation.md`

**Interfaces:**
- Consumes: Task 4 的 `/robot_description`、Task 5 controller actions、Task 6 的 MoveIt package。
- Produces: `move_group` 和 RViz MotionPlanning；可规划和执行 UR 与夹爪轨迹。

- [ ] **Step 1: 创建等待组合 description 的 launch**

Adapt the official UR MoveIt launch, but set the builder to:

```python
MoveItConfigsBuilder(robot_name="ur10e_robotiq", package_name="ur10e_robotiq_moveit_config")
    .robot_description_semantic(Path("srdf") / "ur10e_robotiq.srdf.xacro")
    .to_moveit_configs()
```

Start `ur_robot_driver/wait_for_robot_description` first, then launch `moveit_ros_move_group/move_group`; launch RViz only when `launch_rviz` is true. Do not call `.robot_description()` in the builder and do not add any second Xacro command.

- [ ] **Step 2: 创建 MoveIt RViz 初始配置**

Set fixed frame `world`, add RobotModel, PlanningScene, MotionPlanning and TF displays. Configure MotionPlanning initial group `ur_manipulator`; keep interactive marker end-effector link `gripper_tcp`.

- [ ] **Step 3: 重新生成并审查组合 collision matrix**

With flattened combined URDF loaded in MoveIt Setup Assistant, generate a collision matrix. Replace the SRDF disable-collision section only with pairs reported as `Adjacent` or `Never`; verify manually that the resulting list contains sensible handling for `tool0`/FT300, FT300/2F-85 base and adjacent finger links. Do not disable non-adjacent arm-to-gripper collisions merely to make a plan succeed.

- [ ] **Step 4: 验证 MoveIt RobotModel 和 Planning**

In terminal A:

```bash
source /ros2_ws/install/setup.bash
ros2 launch ur10e_robotiq_description mock_control.launch.py
```

In terminal B:

```bash
source /ros2_ws/install/setup.bash
ros2 launch ur10e_robotiq_moveit_config ur10e_robotiq_moveit.launch.py launch_rviz:=true
```

Expected: RViz RobotModel 显示三部分模型；Planning Group `ur_manipulator` 的 end-effector 为 `gripper_tcp`；joint goal 和无碰撞 pose goal 都能 Plan 成功。

- [ ] **Step 5: 验证 Mock Execute 与 gripper MoveIt mapping**

In RViz, click `Plan & Execute` for one valid UR joint goal and one valid pose goal. Then execute `gripper` group `open` 和 `close` named state. In terminal C run:

```bash
ros2 control list_controllers
ros2 topic echo --once /joint_states
```

Expected: `scaled_joint_trajectory_controller` remains active; UR joint states move toward target after each arm execution; gripper state changes after each gripper execution; no MoveIt error names a missing controller.

- [ ] **Step 6: 记录 Stage 7/8/9 并提交**

Append MoveIt launch logs, plan success, execution result and final controller state to `docs/validation.md`, then run:

```bash
git -C /ros2_ws/src add ur10e_robotiq/ur10e_robotiq_moveit_config ur10e_robotiq/docs/validation.md
git -C /ros2_ws/src commit -m "feat: launch MoveIt for combined robot"
```

### Task 8: 编写与已实现行为一致的项目文档

**Files:**
- Create: `docs/README.md`
- Create: `docs/architecture.md`
- Create: `docs/robot_description.md`
- Create: `docs/simulation.md`
- Create: `docs/moveit.md`
- Create: `docs/controllers.md`
- Create: `docs/ft300.md`
- Create: `docs/troubleshooting.md`
- Modify: `docs/validation.md`

**Interfaces:**
- Consumes: Task 1–7 的真实 package、launch、controller、topic 和测试输出。
- Produces: 用户可独立执行的 Mock 使用手册和可审计的验证记录。

- [ ] **Step 1: 写入 README 和 architecture**

`README.md` 必须列出支持硬件、Ubuntu 24.04/ROS Jazzy、目录、Quick Start、文档索引，并在首屏明确“当前仅完成 Mock 仿真验证”。`architecture.md` 必须含以下 ASCII 拓扑：

```text
MoveIt -> scaled_joint_trajectory_controller -> UR GenericSystem
MoveIt/Action -> robotiq_gripper_controller -> Robotiq GenericSystem
FT300 SensorInterface(fake) -> robotiq_force_torque_sensor_broadcaster -> WrenchStamped
                         \____________________ one /controller_manager ____________________/
```

- [ ] **Step 2: 写入机器人模型和控制器文档**

`robot_description.md` 必须列出完整 link/joint tree、上游 macro 来源、六个临时安装参数及其默认值和“未标定”限制。`controllers.md` 必须说明一个 manager 原则、三个 hardware component、UR 内置 F/T topic 与外部 FT300 wrench topic 的语义区别。

- [ ] **Step 3: 写入 simulation、moveit 和 FT300 使用文档**

`simulation.md` 必须给出从 `source /ros2_ws/install/setup.bash` 开始的 build、display、mock control、MoveIt、夹爪 action 和 FT topic 全命令。`moveit.md` 必须说明 arm/gripper group、`gripper_tcp`、KDL、controller mapping、Plan 和 Execute。`ft300.md` 必须说明 fake wrench 是六维零值软件链路检查，不能表达物理接触。

- [ ] **Step 4: 写入逐症状排障表**

`troubleshooting.md` 为以下每一项给出“现象、检查命令、根因、解决方法”：Xacro include 失败、package not found、hardware plugin 找不到、controller manager 崩溃、hardware configure/activate 失败、joint interface mismatch、gripper action 不存在、FT wrench 不发布、MoveIt controller not found、RobotModel 错误、TF disconnected、SRDF link/joint 不存在、Plan 成功而 Execute 失败。

- [ ] **Step 5: 只用真实输出更新 validation 文档并提交**

For every section in `docs/validation.md`, include date, ROS distro, build command, display, check_urdf, hardware component/interface/controller, wrench, gripper action, MoveIt plan and execution. Then run:

```bash
git -C /ros2_ws/src add ur10e_robotiq/docs
git -C /ros2_ws/src commit -m "docs: document UR10e Robotiq mock integration"
```

### Task 9: 全量构建、上游完整性与官方 UR 回归

**Files:**
- Modify: `docs/validation.md`

**Interfaces:**
- Consumes: 全部两个新 package 和上游 workspace。
- Produces: 最终可复现构建证据、上游未改动证据与官方 UR 基线回归记录。

- [ ] **Step 1: 执行两层构建检查**

Run:

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --symlink-install --packages-up-to ur10e_robotiq_description
colcon build --symlink-install --packages-up-to ur10e_robotiq_moveit_config
colcon build --symlink-install
```

Expected: 三个命令均 exit code 0。

- [ ] **Step 2: 检查上游完整性**

Run:

```bash
git -C /ros2_ws/src/Universal_Robots_ROS2_Driver status --short
git -C /ros2_ws/src/Universal_Robots_ROS2_Description status --short
git -C /ros2_ws/src/ros2_robotiq_gripper status --short
git -C /ros2_ws/src/rq_fts_ros2_driver status --short
```

Expected: UR 和 FT300 状态与 Task 1 基线一致；Robotiq 的输出与 Task 1 用户认可基线一致；没有任何 `ur10e_robotiq` 路径出现在四个上游仓库。

- [ ] **Step 3: 执行不连接真机的官方 package 回归**

Run:

```bash
source /ros2_ws/install/setup.bash
ros2 pkg prefix ur_robot_driver
ros2 pkg prefix ur_moveit_config
colcon build --symlink-install --packages-select ur_description ur_robot_driver ur_moveit_config
```

Expected: 两个 package prefix 可解析，三个官方 package 构建成功。当前不启动依赖真实或独立 UR robot description 的官方 launch；真机 `ur_control.launch.py robot_ip:=169.254.138.15` 和官方 MoveIt launch 仅在用户明确允许连接机器人时执行。

- [ ] **Step 4: 写入最终状态并提交**

Append all build exit codes, upstream status and official regression result to `docs/validation.md`, then run:

```bash
git -C /ros2_ws/src add ur10e_robotiq/docs/validation.md
git -C /ros2_ws/src commit -m "test: record integration regression results"
```

## Plan Self-Review

- Spec coverage: Task 1 覆盖依赖和上游冻结；Task 2–3 覆盖单一描述和 Display；Task 4–5 覆盖一个 controller manager、UR/夹爪/FT300 Mock；Task 6–7 覆盖 SRDF、collision matrix、MoveIt 规划和执行；Task 8 覆盖全部文档；Task 9 覆盖全量构建和上游回归。
- Placeholder scan: 本计划没有未定实现项；临时安装值、启动参数、controller 名称、验证命令和通过条件均已明确。
- Interface consistency: Xacro 的 `gripper_tcp`、FT sensor `robotiq_ft_sensor`、夹爪 joint `robotiq_85_left_knuckle_joint`、两个新增 controller 名称和 MoveIt action namespace 在所有任务中保持一致。
