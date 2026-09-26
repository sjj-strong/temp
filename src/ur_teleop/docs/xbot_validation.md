# Xbot 验证记录（2026-09-26）

## 已执行

- `colcon build --packages-select ur_teleop ur10e_robotiq_ft_description`：通过。
- 默认单元测试：67 通过、1 个 Xbot mock 测试跳过、27 个原有 integration 测试未选择。
- 设置 `UR_XBOT_MOCK_TEST=1 ROS_DOMAIN_ID=225` 并启动 mock Home 后运行包测试：68 通过，27 个原有 integration 测试未选择。
- Xbot mock 集成测试在恢复轨迹控制器 active、阻抗 inactive、夹爪打开后连续复测三次通过。
- mock Home 日志确认 UR 到达当前配置的 `home.slave`；阻抗初始 inactive，RB 后严格切换成功。
- Joy 合成输入覆盖 base/TCP 切换、夹爪 action 接受、超时停止、急停恢复锁定、Menu/Y/B/View 事件。
- 内存数据集测试覆盖开始、保存、丢弃、finalize、陈旧数据拒录，以及独立夹爪 JointState。

早期集成测试曾出现启动就绪及夹爪状态时序失败；测试已改为显式等待控制器/TF 就绪，且复测前恢复 mock 初态。夹爪间歇性失败未在后三次复测中重现，不能据此推断实机时序和安全性能已验收。

## 复现

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=225
export ROS_HOME=/tmp/ur_xbot_validation
# 确认配置 sim: true，只在独立 mock 域运行
ros2 launch ur_teleop home.launch.py config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml
```

Home 到位后在同一 ROS 域的另一个终端：

```bash
UR_XBOT_MOCK_TEST=1 PYTHONPATH=/ros2_ws/src/ur_teleop:$PYTHONPATH \
  /usr/bin/python3 -m pytest /ros2_ws/src/ur_teleop/tests -q
```

测试会改变 mock 控制器和夹爪状态，重复执行前重新启动 mock Home 或恢复上述初态。禁止对真机运行该合成输入测试。

## 未验收与限制

没有连接真机，没有使用物理 Xbot，没有验证实际相机/LeRobot 视频落盘。必须人工校准输入映射、TCP 安装偏移、逐轴方向、夹爪动作及停止行为。当前阻抗控制器无进程失联超时保护；详情见 [控制说明](xbot_control.md)。
