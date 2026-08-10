# UR10e + FT300 + 2F-85 Mock 集成设计

## 目标

在 ROS 2 Jazzy、MoveIt 2 与 ros2_control 下，新增一个独立集成项目，提供 UR10e、Robotiq FT300 和 Robotiq 2F-85 的单一机器人模型、Mock ros2_control、MoveIt 规划与 Mock 执行链路。当前交付只覆盖软件 Mock 验证，不宣称真实组合硬件已验证。

## 已核实的本地基线

- `ur_description` 3.5.1：`ur_robot` macro 生成 UR10e 的全部模型、关节限制、惯量和 `flange`/`tool0` frame。
- `ur_robot_driver` 3.8.0：`ur_ros2_control` macro 在 `use_mock_hardware:=true` 时使用 `mock_components/GenericSystem`；`ur_control.launch.py` 支持外部 `controllers_file` 与 `description_launchfile`。
- `robotiq_description`：`robotiq_gripper` macro 支持 `parent`、`prefix`、`include_ros2_control` 和 `use_fake_hardware:=true`，后者使用 `mock_components/GenericSystem`。
- `robotiq_ft_sensor_description` 2.0.0：`robotiq_ft300` 生成安装板、FT300 sensor 和 `robotiq_ft_frame_id`；`robotiq_fts_ros2_control` 支持 `use_fake_mode:=true`。
- `robotiq_ft_sensor_hardware` 2.0.0：fake mode 实际导出六个 state interface，并在读取时赋值为有限的零 wrench。

`robotiq_ft_sensor_description` 和 `robotiq_ft_sensor_hardware` 源码已经位于 workspace，但尚未出现在当前 `install` overlay；它们必须在任何组合包构建前成功构建。`ros2_robotiq_gripper` 的现有未提交改动是用户认可的本地基线，必须保留且不修改。

## 边界与不变量

- 只在 `/ros2_ws/src/ur10e_robotiq` 创建或修改文件。
- 不修改 `Universal_Robots_ROS2_Driver`、`Universal_Robots_ROS2_Description`、`ros2_robotiq_gripper` 或 `rq_fts_ros2_driver` 的任何源码。
- 组合系统只有一个 Xacro 单一真源、一个 `robot_description`、一个 `/controller_manager`。
- 显示模式不启动任何硬件、controller manager 或 UR driver。
- Mock 模式加载三个硬件组件：UR GenericSystem、FT300 SensorInterface fake mode、2F-85 GenericSystem。
- MoveIt 只映射运动控制器；FT300 broadcaster 不是 MoveIt motion controller。
- 原始 UR bringup 和 `ur_moveit_config` 不修改；完成后只执行构建与 git 状态回归检查，不在本阶段连接或操作真机。

## 项目结构

```
src/ur10e_robotiq/
├── ur10e_robotiq_description/
│   ├── urdf/ur10e_robotiq.urdf.xacro
│   ├── config/ur_controllers_mock.yaml
│   ├── launch/display.launch.py
│   ├── launch/robot_state_publisher.launch.py
│   └── launch/mock_control.launch.py
├── ur10e_robotiq_moveit_config/
│   ├── config/
│   ├── srdf/ur10e_robotiq.srdf.xacro
│   └── launch/ur10e_robotiq_moveit.launch.py
└── docs/
```

只有上述两个目录是 ROS package。`docs/` 不作为 package 或 bringup package。

## 机器人描述与 TF

顶层 Xacro include `ur_description/urdf/ur_macro.xacro`、`ur_robot_driver/urdf/ur.ros2_control.xacro`、FT300 description/control macro 和 2F-85 macro。它先以官方 `ur_robot` 生成 `world -> base_link -> ... -> tool0`，然后以官方 FT300 macro 将 sensor 接到 `tool0`，再以官方 2F-85 macro 将夹爪接到 `ft300_sensor`。

`gripper_tcp` 是从 `robotiq_85_base_link` 固定出来的 frame，而不是任一手指 link 的子节点。因此 SRDF 的 `base_link -> gripper_tcp` arm chain 只含六个 UR 主动关节；夹爪仍有独立的一个主动关节规划组。

组合 Xacro 公开以下顶层参数：

| 连接 | 参数 | 临时默认值 | 含义 |
| --- | --- | --- | --- |
| `tool0 -> FT300` | `ft_xyz`, `ft_rpy` | `0 0 0`, `0 0 0` | FT300 macro 的外部安装变换 |
| `FT300 sensor -> 2F-85 base` | `gripper_xyz`, `gripper_rpy` | `0 0 0`, `-3.1415 0 0` | 2F-85 macro 的外部安装变换；默认绕 X 轴翻转夹爪 |
| `2F-85 base -> gripper_tcp` | `tcp_xyz`, `tcp_rpy` | `0 0 0.15`, `0 0 0` | 夹爪中心参考点 |

这些默认值只支持结构、TF、Mock 与 MoveIt 流程验证，不能用于实机安装、负载计算或碰撞安全。文档会以“临时、未标定参数”标记，并给出替换命令。`gripper_rpy="-3.1415 0 0"` 只在组合 Xacro 中定义默认值；display、Mock control 与 MoveIt 均继承该单一真源，不在 launch 中重复硬编码安装姿态。

`include_ros2_control:=false` 时只输出模型；`include_ros2_control:=true` 时连续输出 UR、FT300 与 2F-85 的三个 `<ros2_control>` 元素。

## 启动与控制架构

`display.launch.py` 直接执行组合 Xacro（控制关闭），启动 `robot_state_publisher`、`joint_state_publisher_gui` 和 RViz。

`mock_control.launch.py` include 官方 `ur_control.launch.py`，传入 `ur_type:=ur10e`、`use_mock_hardware:=true`、集成本地 controllers YAML 和集成本地 RSP wrapper。官方 control launch 仅向外部 description launchfile 转发 `robot_ip` 与 `ur_type`；wrapper 因而显式将 `use_mock_hardware:=true`、FT fake mode 和 gripper fake mode 传给官方 `ur_rsp.launch.py`，并以组合 Xacro 覆盖 `description_file`。这样保留官方 control-node、默认 UR controller spawning 和 driver 架构，同时保证从 Xacro 到 controller manager 的模型一致。

集成 YAML 从官方 `ur_controllers.yaml` 的副本演进，保留现有 UR 控制器配置，额外声明：

- `robotiq_gripper_controller`：`parallel_gripper_action_controller/GripperActionController`，只控制 `robotiq_85_left_knuckle_joint` 的 position interface。
- `robotiq_force_torque_sensor_broadcaster`：`force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster`，使用 `robotiq_ft_sensor` 的 force/torque 六个 state interface，frame 为 `robotiq_ft_frame_id`。

`mock_control.launch.py` 在官方 spawner 后额外 spawn 这两个 controller。官方的 `force_torque_sensor_broadcaster` 继续表示 UR 内部 TCP F/T；新增 broadcaster 表示外置 FT300。

## MoveIt 架构

新的 `ur10e_robotiq_moveit_config` 参考官方 Jazzy `ur_moveit_config` 的 launch/config 结构，但不重新构造 URDF。它等待已有组合 `robot_description`，并只在自身包中构造组合 SRDF、规划器、运动控制器映射和 RViz 参数。

- `ur_manipulator`：chain `base_link -> gripper_tcp`，KDL solver。
- `gripper`：只含 `robotiq_85_left_knuckle_joint`。
- `robotiq_2f85` end effector：关联 `gripper` 与 `ur_manipulator`，parent link 为 `robotiq_85_base_link`。
- named state：沿用 UR 的 `home`、`up`、`test_configuration`，增加 `gripper` 的 `open`（0.0）和 `close`（0.7929）。
- MoveIt controllers：`scaled_joint_trajectory_controller` 为默认 `FollowJointTrajectory`；`robotiq_gripper_controller` 为 `ParallelGripperCommand`。不加入任一 F/T broadcaster。

组合 collision matrix 不复用裸 UR matrix 作为最终结果。通过当前组合模型重新生成并人工复查；保留两次 100000-trial 生成结果中稳定一致的 `Adjacent`/`Never` 配对，并精确增加生成器报告为 `Default`、且导致全部夹爪状态无效的两对内部几何豁免：`robotiq_85_left_finger_tip_link`/`robotiq_85_left_inner_knuckle_link` 与 `robotiq_85_right_finger_tip_link`/`robotiq_85_right_inner_knuckle_link`。安装链必须保留 `wrist_3_link`/`ft300_mounting_plate`、`ft300_mounting_plate`/`ft300_sensor`、`ft300_sensor`/`robotiq_85_base_link` 的 `Adjacent` 豁免；不得增加任何其他非邻接 arm-to-gripper 豁免。

## 验证策略

严格按以下顺序推进，任何失败都在本阶段解决，不进入下一阶段：

1. 构建并发现 FT300 package，检查其 plugin XML。
2. 生成组合 URDF，运行 `check_urdf`，验证单一 link/joint tree。
3. 启动 display，检查模型、mimic 和 `tf2_tools view_frames`。
4. 启动 mock control，检查三个 hardware component、硬件 interface 和 controller lifecycle。
5. 验证 FT300 wrench 持续发布、frame 正确且数值不是 NaN；零值视为通过。
6. 发送夹爪 action，验证左指关节状态和 RViz mimic 同步变化。
7. 启动 MoveIt，验证 RobotModel、joint goal 和 `gripper_tcp` pose goal 的规划。
8. 执行 MoveIt trajectory，验证 scaled controller、joint states 和 RViz 当前状态更新。
9. 使用 gripper controller mapping 或 action 验证 open/close。
10. 构建整个 workspace、检查四个上游仓库状态，并把真实命令和结果写入 validation 文档。

每个阶段的文档条目固定记录：变更文件、原因、执行命令、观察输出、PASS/FAIL 和剩余问题。未真实运行的验证项标为“未执行”，不会标为 PASS。

## 文档交付

项目 docs 包含 README、architecture、robot_description、simulation、moveit、controllers、ft300、troubleshooting、validation。所有运行命令使用当前两个 package 的真实 launch 名和 ROS 2 Jazzy 控制器类型。真实硬件、接触动力学、Gazebo/MuJoCo、力控和 programmable FT mock 均不属于此阶段。
