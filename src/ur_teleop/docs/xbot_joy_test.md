# 独立手柄功能测试

脚本自动启动 `joy_node` 并监听专用话题 `/xbot_joy_test/joy`，无需启动 Home、仿真、真机、RViz 或数据录制器。读取 `xbot_teleop.yaml` 中的 `xbot.device_id`、`calibration_file`、`deadzone` 和 `joy_timeout_s`；`sim` 与 `mode` 不影响此测试。脚本不创建机器人指令发布器或控制器/夹爪客户端。

## 一个终端直接运行

连接手柄后执行，无需重新构建：

```bash
source /opt/ros/jazzy/setup.bash
python3 /ros2_ws/src/ur_teleop/ur_teleop/xbot_joy_test.py
```

按键按下和松开时立即打印带时间戳的事件；默认每秒打印 5 行状态日志，包含全部原始轴、全部原始按键、标定后的功能键状态、归一化轴值、消息数和数据龄。未标定的按键仍显示原始编号；标定文件不存在或布局不匹配时仍可测试原始输入。停止接收输入后打印超时，恢复后重新建立按键基线。按 `Ctrl+C` 退出并停止脚本启动的手柄驱动。

示例事件：

```text
按键[7] RB（运动使能） 按下
按键[7] RB（运动使能） 松开
```

功能名称只用于核对映射，不执行实际功能。可以依次短按、长按和组合按下 A/B/X/Y、RB/LB、Menu/View，再操作两个摇杆、两个扳机和十字键，观察事件、原始轴和归一化轴值。轴值使用遥操作的标定和死区，但未应用主轴过滤；短于状态打印周期的按键操作仍通过事件日志显示。连接状态表示 Joy 消息是否持续到达，驱动没有上报的输入无法由脚本检测。

## 可选参数

```bash
# 更换设备编号和日志频率
python3 /ros2_ws/src/ur_teleop/ur_teleop/xbot_joy_test.py --device-id 1 --rate 10

# 使用另一份配置
python3 /ros2_ws/src/ur_teleop/ur_teleop/xbot_joy_test.py --config /路径/xbot_teleop.yaml

# 列出 SDL 识别的手柄和设备编号
ros2 run joy joy_enumerate_devices
```

若需要独立提供测试输入，可用 `--no-start-joy`，脚本只订阅 `/xbot_joy_test/joy`。外部驱动应将 `joy` 重映射到该话题。默认测试驱动也只向这个专用话题发布输入，正式遥操作使用的 `/joy` 不会收到测试消息。

构建后也可通过包入口运行：

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur_teleop --symlink-install
source /ros2_ws/install/setup.bash
ros2 run ur_teleop xbot_joy_test
```

## 验证记录

已通过 7 项自动测试、`ur_teleop` 包构建，以及专用话题的实际 ROS 收发验证：按键按下/松开日志、输入超时、自动启动驱动和 `Ctrl+C` 清理均正常。验证环境没有实体手柄，物理按键编号、摇杆方向和重新插拔后的驱动表现需连接设备后核对。
