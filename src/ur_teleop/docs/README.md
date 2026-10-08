# ur_teleop 文档入口

本目录说明 Xbot/Alicia 遥操作 UR、相关控制器及采集前后操作。

按顺序阅读：[硬件准备](hardware.md) → [完整流程](workflow.md) → [相机调试](camera_inspector.md) → [启动说明](launch.md) → [数据采集](data_recorder.md)。控制器对应关系见[控制器说明](controllers.md)，所有文档及用途见[功能包 README](../README.md#文档导航)。

Xbot 的标定和按键见[手柄操作](xbot_control.md)，逐项参数见[Xbot 配置](xbot_teleop_config.md)，不连接机器人的输入检查见[独立手柄测试](xbot_joy_test.md)。Alicia 的配置和节点逻辑见[配置加载](config.md)、[数据流](pipeline.md)与[遥操作节点](teleop_node.md)。

修改从臂初始位置时使用[只读关节位置工具](capture_slave_home.md)。Home 确认、控制器切换、映射、offset、Ruckig、夹爪与键盘等实现细节从功能包 README 的文档导航进入。
