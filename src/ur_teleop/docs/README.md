# ur_teleop 文档

支持 Alicia 主从臂和 Xbot 手柄两种控制方式，通过 `teleop.control_source: alicia | xbot` 选择，不同时运行。

| 文档 | 内容 |
| --- | --- |
| [启动与相机](launch.md) | 两阶段启动、配置选择、相机发布 |
| [Xbot 手柄](xbot_control.md) | 校准、按键、base/TCP 切换、安全限制与测试 |
| [数据采集](data_recorder.md) | 两种控制方式的录制操作、帧格式与数据集 |
| [配置](config.md) | 配置加载与字段说明 |
| [Alicia 数据流](pipeline.md) | 主从臂拓扑、状态机与接口 |

## 实现参考

以下文档主要说明 Alicia 分支；Xbot 分支见手柄文档。

- [Home](home_node.md)、[遥操作状态机](teleop_node.md)、[控制器切换](controller_switcher.md)
- [关节映射](joint_mapper.md)、[会话偏移](session_offset.md)、[Ruckig 平滑](ruckig_node.md)
- [夹爪迟滞控制](gripper_controller.md)、[键盘读取](keyboard.md)

## 测试

先加载 ROS 和工作区环境：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
PYTHONPATH=/ros2_ws/src/ur_teleop:$PYTHONPATH \
  /usr/bin/python3 -m pytest /ros2_ws/src/ur_teleop/tests -q
```

默认不执行原有 `integration` 测试，Xbot mock 测试需按[手柄文档](xbot_control.md#测试)单独启用。测试不得连接真机。
