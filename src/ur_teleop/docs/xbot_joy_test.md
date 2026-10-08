# 独立手柄功能测试

脚本自动启动 `joy_node` 并监听专用话题 `/xbot_joy_test/joy`，无需启动 Home、仿真、真机、RViz 或数据录制器。读取 `xbot_teleop.yaml` 中的 `xbot.device_id`、`calibration_file`、`deadzone`、`joy_timeout_s`、`precision_scale`、`left_stick_xy_free` 和 `view_hold_s`，以及 `gripper.enabled`；`sim` 与 `mode` 不影响此测试。脚本不创建机器人指令发布器或控制器/夹爪客户端。

## 一个终端直接运行

连接手柄后执行，无需重新构建：

```bash
source /opt/ros/jazzy/setup.bash
python3 /ros2_ws/src/ur_teleop/ur_teleop/xbot_joy_test.py
```

日志默认只显示动作预览，隐藏原始按键编号和轴数组。按键按下和松开立即打印，默认每秒 5 次显示持续运动方向、输入强度、RB 使能状态、LB 精细模式、当前参考系和按住的动作；回中立即打印“运动输入归零”。按 `Ctrl+C` 退出并停止脚本启动的手柄驱动。

示例日志：

```text
动作预览：运动使能（RB）按下
动作预览：切换参考系（X）按下，当前参考系=TCP
动作预览：运动已使能；精细模式；参考系=TCP；+X 平移 强度=0.250；当前按住=运动使能、精细模式
动作预览：保存片段（Y）按下，action=save
动作预览：运动输入归零
```

## 映射与动作解释

使用 `xbot.calibration_file` 指定的文件，默认 `/ros2_ws/src/ur_teleop/config/xbot_joy.yaml`，不自动猜测或替换映射。默认动作对应关系如下：

| 输入 | 动作预览 |
| --- | --- |
| RB | 运动使能；未按住时运动方向仍可观察，并明确显示“运动未使能” |
| LB | 精细模式，强度乘 `precision_scale` |
| A | 切换夹爪请求；若配置关闭夹爪或 RB 未按住，提示不可执行原因 |
| X | 切换测试内部的 BASE/TCP 参考系提示 |
| Menu | 开始录制，`action=start` |
| Y / B | 保存 / 丢弃片段，`action=save` / `action=discard` |
| View | 达到 `view_hold_s` 后显示一次结束录制，`action=finalize`；松开后可重新测试 |
| 左摇杆 | 标定正方向对应 +X / +Y 平移 |
| RT / LT | +Z / -Z 平移 |
| 右摇杆 | 标定正方向对应绕 +X / +Y 旋转 |
| 十字键 | 标定正方向对应绕 +Z 旋转 |

运动输入应用遥操作的标定、死区、主轴过滤、向量模长限制和精细模式比例。强度是处理后的归一化输入，不代表实际机器人速度或位移；TCP 方向表示局部轴，不计算真实 TCP 到基座的变换。脚本不发送任何机器人、夹爪或录制指令，也不模拟机器人就绪、夹爪忙碌及数据集保存等反馈。

标定文件声明的轴数/按键数与实际输入不同，不再整组拒绝：每个映射项分别检查，有效动作继续显示，超出设备范围的动作在首次收到输入或可用性变化时单独提示。当前文件声明 19 个按键，若实际反馈仅 11 个（编号 0–10），当前 Menu=10 可用，而 View=11 越界，结束长按不可用；提示以运行时实际布局和标定为准。未映射输入不打印、不猜测动作；若某键没有动作日志，应检查指定文件对应关系。这个宽松解析只用于测试脚本，正式遥操作仍严格检查布局。

标定文件缺失或无效时明确提示无法识别动作，不回退到原始数组日志。输入超时后停止运动预览、清除长按计时；恢复后重新建立按键基线。连接状态表示 Joy 消息是否持续到达，驱动没有上报的输入无法由脚本检测。

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
