# ur_teleop 使用文档

本包支持 Alicia 主臂或 Xbot 手柄遥操作 UR，并采集 LeRobot 数据集。两种输入源二选一。

## 开始使用

1. [硬件与环境准备](hardware.md)：依赖、设备连接与初始位置。
2. [完整使用流程](workflow.md)：相机、仿真/真机启动、遥操作与采集。
3. [启动参数](launch.md)：入口参数、覆盖规则与相机配置。

## 配置与操作

| 内容 | 文档 |
| --- | --- |
| Alicia 配置与操作 | [配置参数](alicia_teleop_config.md)、[遥操作](teleop_node.md)、[键盘操作](keyboard.md) |
| Xbot 配置与操作 | [手柄标定和操作](xbot_control.md)、[配置参数](xbot_teleop_config.md)、[独立手柄检查](xbot_joy_test.md) |
| 相机 | [检查与调参](camera_inspector.md)、[发布和保存尺寸](launch.md#相机) |
| 数据采集 | [采集操作与数据格式](data_recorder.md)、[录制参数](recorder_config.md) |
| 初始位置 | [读取从臂位置](capture_slave_home.md)、[回 Home](home_node.md) |

## 控制行为与排查

[控制流程](pipeline.md) · [控制器](controllers.md) · [自动切换](controller_switcher.md) · [关节映射](joint_mapper.md) · [会话偏移](session_offset.md) · [Ruckig 平滑](ruckig_node.md) · [夹爪](gripper_controller.md) · [控制接口日志](control_interface_logging.md)
