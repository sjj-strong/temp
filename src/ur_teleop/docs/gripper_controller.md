# 夹爪使用

`gripper.enabled` 控制夹爪执行，`recorder.record_action_gripper` 控制是否保存开合指令。两者独立。夹爪随 Home/遥操作启动，无需单独运行本包的夹爪模块；仿真与真机入口见[完整流程](workflow.md)。

## 配置

| `gripper` 参数 | 作用 |
| --- | --- |
| `enabled` | 是否启用夹爪 |
| `action_server` | 夹爪 Action，通常为 `/robotiq_gripper_controller/gripper_cmd` |
| `open_pos_rad` | 打开目标关节角，例如 0.0 rad |
| `close_pos_rad` | 闭合目标关节角；Xbot 的 Robotiq 2F-85 完全闭合为 0.7929 rad |
| `max_effort` | 下发的最大 effort；含义和单位以夹爪控制器为准 |
| `open_threshold_m` | Alicia 输入小于此值时打开，例如 0.005 m |
| `close_threshold_m` | Alicia 输入大于此值时闭合，例如 0.0125 m |
| `fsm_rate_hz` | Alicia 夹爪输入检查频率，默认 10 Hz |

两个输入阈值只用于 Alicia，须满足打开阈值小于闭合阈值。区间内保持原开合状态，避免抖动；它们与输出目标角度不是同一物理量。Xbot 不使用这两个阈值。

## 操作

Alicia 根据主臂夹爪输入自动切换开合。夹爪检查不以关节遥操作的 Enter 使能为门控。启动后 30 秒内仍找不到 Action 服务时会禁用夹爪；排查驱动后重启遥操作。

Xbot 就绪并按住 RB 时，按 A 切换开合；夹爪忙碌、反馈过期或服务未就绪时忽略操作。录制指令为打开 0、闭合 1，表示目标状态，不代表夹爪已运动到位。

## 硬件与仿真

真机需填写 `cell.gripper_port`。Alicia Home 还需传 `enable_gripper:=true`，与 YAML 开关一致；例如在[真机 Home 命令](workflow.md#alicia-真机)中增加：

```bash
ros2 launch ur_teleop home.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/alicia_teleop.yaml \
  sim:=false controller:=joint_impedance enable_gripper:=true \
  gripper_port:=/dev/ttyUSB0 enable_ft300:=false
```

同时核对实际机器人 IP 和 Alicia 串口；未显式传入时使用安装配置的默认值。Xbot 直接按 YAML 开关启动夹爪。

Alicia 关节阻抗 mock 仅包含 UR 六轴，应关闭夹爪；Xbot mock 可启用虚拟夹爪。无法开合时检查：

```bash
ros2 action list -t
```

确认配置的 Action 存在，并查看驱动和请求失败日志。

完全闭合表示下发全行程目标；夹住物体时，夹爪可能因接触或 effort 限制提前停止，不能保证实际角度到达全闭值。
