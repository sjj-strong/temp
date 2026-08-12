# 机器人描述

## 单一组合模型

`ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro` 是 UR10e、Robotiq FT300 和 Robotiq 2F-85 的唯一组合模型。默认 `ur_type` 为 `ur10e`。模型从 `world` 开始，经 UR10e 的 `tool0` 依次安装 FT300 与 2F-85；`gripper_tcp` 固定连接到 `robotiq_85_base_link`，因此 MoveIt 的 `base_link -> gripper_tcp` 机械臂链只包含六个 UR 主动关节。

`include_ros2_control:=false` 生成纯描述；`include_ros2_control:=true` 还会生成 `ur`、`robotiq_ft_sensor` 和 `robotiq_2f85` 三个 `<ros2_control>` 元素。Display、Mock control 与 MoveIt 不维护重复模型。

## 完整 link/joint tree

下面是默认参数展开后的完整树。方括号内依次为 joint 名、类型；`mimic` 标出相对唯一夹爪主动关节 `robotiq_85_left_knuckle_joint` 的跟随关系。

```text
world
└── [base_joint, fixed] base_link
    ├── [base_link-base_fixed_joint, fixed] base
    └── [base_link-base_link_inertia, fixed] base_link_inertia
        └── [shoulder_pan_joint, revolute] shoulder_link
            └── [shoulder_lift_joint, revolute] upper_arm_link
                └── [elbow_joint, revolute] forearm_link
                    └── [wrist_1_joint, revolute] wrist_1_link
                        └── [wrist_2_joint, revolute] wrist_2_link
                            └── [wrist_3_joint, revolute] wrist_3_link
                                ├── [wrist_3_link-ft_frame, fixed] ft_frame
                                └── [wrist_3-flange, fixed] flange
                                    └── [flange-tool0, fixed] tool0
                                        └── [ft300_fix, fixed] ft300_mounting_plate
                                            └── [ft300_mounting_plate, fixed] ft300_sensor
                                                ├── [measurment_joint, fixed] robotiq_ft_frame_id
                                                └── [robotiq_85_base_joint, fixed] robotiq_85_base_link
                                                    ├── [robotiq_85_base_link-gripper_tcp, fixed] gripper_tcp
                                                    ├── [robotiq_85_left_inner_knuckle_joint, revolute, mimic ×1] robotiq_85_left_inner_knuckle_link
                                                    ├── [robotiq_85_left_knuckle_joint, revolute, active] robotiq_85_left_knuckle_link
                                                    │   └── [robotiq_85_left_finger_joint, fixed] robotiq_85_left_finger_link
                                                    │       └── [robotiq_85_left_finger_tip_joint, revolute, mimic ×-1] robotiq_85_left_finger_tip_link
                                                    ├── [robotiq_85_right_inner_knuckle_joint, revolute, mimic ×-1] robotiq_85_right_inner_knuckle_link
                                                    └── [robotiq_85_right_knuckle_joint, revolute, mimic ×-1] robotiq_85_right_knuckle_link
                                                        └── [robotiq_85_right_finger_joint, fixed] robotiq_85_right_finger_link
                                                            └── [robotiq_85_right_finger_tip_joint, revolute, mimic ×1] robotiq_85_right_finger_tip_link
```

注意：上游 FT300 macro 的 joint 名确实拼写为 `measurment_joint`，这里按实际展开模型记录，不在集成层改名。

## 上游 macro 来源

组合 Xacro 复用下列上游实现，不复制其 link、joint、惯量或硬件插件定义：

| Macro                              | 上游 package 与文件                                                   | 在组合模型中的职责                                                 |
| ---------------------------------- | --------------------------------------------------------------------- | ------------------------------------------------------------------ |
| `xacro:ur_robot`                 | `ur_description/urdf/ur_macro.xacro`                                | UR10e link/joint、`base_link`、`flange`、`tool0` 与几何/惯量 |
| `xacro:ur_ros2_control`          | `ur_robot_driver/urdf/ur.ros2_control.xacro`                        | UR ros2_control；Mock 时选择 GenericSystem                         |
| `xacro:robotiq_ft300`            | `robotiq_ft_sensor_description/urdf/robotiq_ft300.urdf.xacro`       | FT300 安装板、传感器模型与`robotiq_ft_frame_id`                  |
| `xacro:robotiq_fts_ros2_control` | `robotiq_ft_sensor_description/urdf/robotiq_fts.ros2_control.xacro` | FT300 六维 state interface 与 fake mode 硬件插件                   |
| `xacro:robotiq_gripper`          | `robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro`           | 2F-85 link/joint/mimic；内部 include`2f_85.ros2_control.xacro`   |

`xacro:ur_robot` 还读取 `ur_description/config/$(arg ur_type)/` 下的 `joint_limits.yaml`、`default_kinematics.yaml`、`physical_parameters.yaml` 和 `visual_parameters.yaml`。集成文件本身只新增 `world`、三段外部安装关系、`gripper_tcp`，以及按模式启用的 ros2_control macro 调用。

## 六个临时安装参数

以下六个顶层 Xacro 参数及默认值均来自当前组合源码：

| 连接                                     | 参数            | 默认值          |
| ---------------------------------------- | --------------- | --------------- |
| `tool0 -> ft300_mounting_plate`        | `ft_xyz`      | `0 0 0`       |
| `tool0 -> ft300_mounting_plate`        | `ft_rpy`      | `0 0 0`       |
| `ft300_sensor -> robotiq_85_base_link` | `gripper_xyz` | `0 0 0`       |
| `ft300_sensor -> robotiq_85_base_link` | `gripper_rpy` | `-3.1415 0 0` |
| `robotiq_85_base_link -> gripper_tcp`  | `tcp_xyz`     | `0 0 0.15`    |
| `robotiq_85_base_link -> gripper_tcp`  | `tcp_rpy`     | `0 0 0`       |

其中 `gripper_rpy=-3.1415 0 0` 使夹爪默认绕 X 轴翻转，并且只在组合 Xacro 中定义一次；现有三个 launch 不会重复覆盖该姿态。

> **未标定限制：** 六个值都是为了结构、TF、Mock 控制和 MoveIt 软件链路检查设置的临时值，未经过真实法兰、转接板、传感器、夹爪和 TCP 标定。它们不得用于实机安装、碰撞安全、负载/质心计算、轨迹执行或力/力矩解释。

完成机械测量与安全复核后，可以用实际标定值覆盖六个 Xacro 参数做离线检查。
下面这条可直接执行的命令使用当前默认值验证参数转发路径：

```bash
source /ros2_ws/install/setup.bash
xacro /ros2_ws/src/ur10e_robotiq/ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro \
  include_ros2_control:=false \
  ft_xyz:="0 0 0" ft_rpy:="0 0 0" \
  gripper_xyz:="0 0 0" gripper_rpy:="-3.1415 0 0" \
  tcp_xyz:="0 0 0.15" tcp_rpy:="0 0 0" \
  > /tmp/ur10e_robotiq_calibrated.urdf
check_urdf /tmp/ur10e_robotiq_calibrated.urdf
```

这只验证替代参数能否生成 URDF；当前 launch 文件没有公开这六个 launch argument。要让 Display、Mock control 与 MoveIt 一致采用标定值，必须在进入实机阶段前更新唯一组合 Xacro 的默认值，或统一扩展启动参数转发，并重新完成模型、TF、碰撞与执行验证。
