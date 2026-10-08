# 回 Home

Home 启动机器人控制栈并执行初始位置运动。Alicia 移动主从双臂，Xbot 仅移动 UR；成功显示 `HOME REACHED` 后，保持启动终端运行，再启动遥操作。

## 启动

以下 Alicia 命令使用关节阻抗，关闭夹爪和 FT300；对应 YAML 开关也应关闭。IP、串口和 Home 需按设备设置。

```bash
# Alicia 仿真：UR 为 mock，Alicia 仍是真实主臂
ros2 launch ur_teleop home.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/alicia_teleop.yaml \
  sim:=true controller:=joint_impedance enable_gripper:=false enable_ft300:=false
```

```bash
# Alicia 真机
ros2 launch ur_teleop home.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/alicia_teleop.yaml \
  sim:=false controller:=joint_impedance \
  robot_ip:=169.254.138.15 alicia_port:=/dev/ttyACM0 \
  enable_gripper:=false enable_ft300:=false
```

```bash
# Xbot：仿真设置 YAML sim: true；真机设置 sim: false
ros2 launch ur_teleop home.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml
```

Xbot 还需为仿真或真机选择对应阻抗参数文件，见[完整流程](workflow.md#3-回-home)。

真机启动后按提示在示教器运行 External Control，再在终端按 Enter。确认后会发送六轴 Home 轨迹；Alicia 在 UR 轨迹成功后使主臂回 Home。仿真跳过示教器确认。

### 启动日志中的确认提示

Home 订阅 `/io_and_status_controller/robot_program_running`，在连接状态变化时转述驱动状态，不解析英文日志。收到运行状态后显示 `UR 外部控制已连接（Ready to receive control commands）`；未收到或程序停止时显示等待连接。该提示只表示外部控制程序的状态，轨迹控制器与关节状态仍由后续流程检查。

等待回车期间每 5 秒重复显示带分隔线的 `Home 等待确认` 提示，避免被启动日志刷走。Alicia、Xbot 共用此行为；仿真跳过确认。无需添加配置参数，驱动的错误与警告保持可见。收到回车后停止重复提示，按原流程等待控制器并执行 Home；只有 `HOME REACHED` 才表示 Home 成功，可以进入遥操作阶段。

## 参数

| `home` 参数 | 作用 |
| --- | --- |
| `slave` | UR 六关节目标，rad，标准六轴顺序 |
| `master` | Alicia 六关节目标，rad；Xbot 不需要 |
| `master_gripper_value` | Alicia 夹爪 Home 指令，1000 表示打开 |
| `at_home_tolerance_rad` | 最大逐关节到位误差，rad |
| `move_duration_s` | Home 轨迹点的到达时间，秒 |
| `move_timeout_s` | 等待轨迹结果、随后验证到位的两个独立超时窗口，秒 |
| `verify_duration_s` | 在容差内连续保持的时间，秒 |

`settle_time_s` 和 `settle_motion_threshold_rad` 用于后续遥操作捕获偏移，不参与 Home 验证。

## 失败排查

- 未开始运动：确认示教器程序和终端回车，以及轨迹控制器和关节反馈是否就绪。
- 轨迹失败：查看控制器日志，检查 Home 路径、目标角度和超时配置。
- 到位验证失败：比较实际关节角和配置 Home；Alicia 还需检查主臂反馈。

更新 UR Home 可用[只读位置工具](capture_slave_home.md)。不要用遥操作的 `force_home` 掩盖 Home 失败。
