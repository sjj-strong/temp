# UR10e + FT300 + Robotiq 2F-85 集成

> **当前仅完成 Mock 仿真验证。** 本项目尚未在真实 UR10e、FT300 或 2F-85 上完成接线、标定、负载、碰撞安全与运动验证，不可把当前默认安装位姿用于实机。

## 支持硬件（当前仅限模型与 Mock）

本项目在一个机器人描述和一个 ros2_control controller manager 中组合以下设备：

| 支持的组合部件 | 当前覆盖范围 |
| --- | --- |
| Universal Robots UR10e | 上游模型、`mock_components/GenericSystem`、轨迹控制与 MoveIt Mock 执行 |
| Robotiq FT300 | 上游模型、fake SensorInterface、六维零值 wrench 软件链路 |
| Robotiq 2F-85 | 上游模型、mimic 关节、`mock_components/GenericSystem` 与夹爪 action |

运行基线为 **Ubuntu 24.04 + ROS 2 Jazzy**，并使用 MoveIt 2、ros2_control、UR Driver 和对应的 Robotiq 上游包。当前范围不包含真实硬件、Gazebo/MuJoCo 接触动力学、力控或可编程 FT300 仿真。

## 目录

```text
src/ur10e_robotiq/
├── ur10e_robotiq_description/
│   ├── urdf/ur10e_robotiq.urdf.xacro
│   ├── config/ur_controllers_mock.yaml
│   ├── launch/
│   │   ├── display.launch.py
│   │   ├── robot_state_publisher.launch.py
│   │   └── mock_control.launch.py
│   └── rviz/display.rviz
├── ur10e_robotiq_moveit_config/
│   ├── config/
│   ├── srdf/ur10e_robotiq.srdf.xacro
│   └── launch/ur10e_robotiq_moveit.launch.py
└── docs/
```

`ur10e_robotiq_description` 是唯一组合 URDF/Xacro 与 ros2_control 描述来源；`ur10e_robotiq_moveit_config` 只提供 SRDF、规划、运动学、控制器映射和 RViz 配置，不生成第二份 URDF。

## Quick Start

先构建并加载工作区：

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to ur10e_robotiq_moveit_config
source /ros2_ws/install/setup.bash
```

只查看模型时运行：

```bash
ros2 launch ur10e_robotiq_description display.launch.py
```

验证 Mock 控制与 MoveIt 时，在终端 A 启动唯一的 controller manager：

```bash
source /ros2_ws/install/setup.bash
ros2 launch ur10e_robotiq_description mock_control.launch.py
```

等待控制器启动后，在终端 B 启动 MoveIt：

```bash
source /ros2_ws/install/setup.bash
ros2 launch ur10e_robotiq_moveit_config ur10e_robotiq_moveit.launch.py
```

MoveIt 启动文件会等待 Mock 启动链发布的 `/robot_description`，因此不要先启动 MoveIt。纯显示模式和 Mock 控制模式是两条独立启动路径，不应同时启动。

## 文档索引

- [架构](architecture.md)：单一描述、单一 controller manager 和数据流。
- [机器人描述](robot_description.md)：完整 link/joint tree、上游 macro 与未标定安装参数。
- [Mock 仿真](simulation.md)：构建、显示、控制、夹爪与 FT300 检查命令。
- [MoveIt](moveit.md)：规划组、KDL、控制器映射及 Plan/Execute。
- [控制器](controllers.md)：硬件组件、控制器和 action/interface。
- [FT300](ft300.md)：外置传感器 fake wrench 的含义与限制。
- [排障](troubleshooting.md)：按症状定位常见启动和执行问题。
- [验证记录](validation.md)：实际执行命令、观察输出与 PASS/FAIL 证据。
