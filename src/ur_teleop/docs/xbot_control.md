# Xbot 手柄遥操作

使用 `cartesian_impedance_controller` 控制 `tool0` 法兰位姿，复用 `ur10e_robotiq_ft_description` 的组合 URDF。夹爪末端 `gripper_tcp` 与 `tool0` 不重合。Xbot 不启动 Alicia 或 Ruckig。相关控制器参数文档见[控制器说明](controllers.md)，从设备准备到采集的命令见[完整流程](workflow.md)。

刚度、阻尼和力矩限制在 `xbot.controller_config_file` 指定的控制器文件中设置，详见[控制器说明](controllers.md)。

## 配置与校准

配置入口为 `config/xbot_teleop.yaml`，该文件是完整的 XBot 独立配置，**不继承** `alicia_teleop.yaml`。
其中 `home.slave` 是 UR 的六关节 Home 位姿，按 `shoulder_pan`、`shoulder_lift`、`elbow`、`wrist_1`、
`wrist_2`、`wrist_3` 顺序填写，单位为 rad。XBot 模式不需要 Alicia 的 `home.master`、关节映射或 Alicia
串口字段。Xbot 的仿真/真机选择以配置文件的 `sim` 为准，不使用 Home 的 `sim:=` 参数覆盖。
`cell.launch_rviz: true` 默认让两种模式的 Home 启动同时打开 RViz；无图形界面时可在配置中设为 `false`。

只连接 UR 与手柄测试时，在配置中设置 `sim: false`、`gripper.enabled: false`、`cell.ft300_enabled: false`；启动 teleop 时显式传 `mode:=teleop`。`gripper.enabled` 同时控制夹爪控制栈、仿真夹爪控制器及 A 键命令；`cell.ft300_enabled` 为 `false` 时 FT300 硬件使用虚拟模式，不打开其串口。需要接入设备时分别改为 `true` 并核对 `cell.gripper_port`、`cell.ftdi_id`。组合 URDF 的 FT300 与夹爪外形及 TF 仍保留，运动目标仍为 `tool0`；虚拟 FT300 的读数不可用作真实力反馈。控制器参数 `use_external_ft: false` 保持关闭。

在两个终端分别加载 ROS 环境后运行：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

# 终端 1：仅发布手柄输入
ros2 run joy joy_node --ros-args -p autorepeat_rate:=50.0 -p deadzone:=0.0

# 终端 2：按提示校准，不发送机器人指令
ros2 run ur_teleop xbot_calibrate --output /tmp/xbot_joy_new.yaml
```

标定过程不需要按 Enter：每步先松开所有按键和扳机、摇杆回中并稳定一秒；看到下一条提示后，按键与十字键按下即记录，模拟轴输入推到极限并保持一秒。每项记录后会打印识别出的 Joy 编号；若 30 秒内没有收到清晰输入，脚本会超时提示重试。默认输出路径为 `/ros2_ws/src/ur_teleop/config/xbot_joy.yaml`，该文件通常已存在，工具拒绝覆盖。上例写入 `/tmp/xbot_joy_new.yaml`；完成后把 xbot.calibration_file 指向新文件，或将它保存到自己的持久路径再更新配置。映射使用 ROS Joy 编号，不能直接复制 pygame 轴号。

校准完成后停止手动启动的 `joy_node`，正式启动会自动运行它。

## 仿真与真机启动

先按[完整流程](workflow.md#3-回-home)设置 `sim` 与控制器文件并启动 Home。Home 成功后保持终端运行，在另一终端执行：

```bash
# 仿真与真机共用：机器人模式由 Home 使用的 YAML 决定
ros2 launch ur_teleop teleop.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml mode:=teleop
```

真机需设置 `sim: false`、实际 IP 和 Home；当前高档配置参考坐标系为 `base_link`。仿真设置 `sim: true` 并使用 `ur10e_xbot_sim_cartesian_impedance.yaml`，参考坐标系为 `base_link`。两种模式都使用真实手柄。

真机 Home 命令为：

```bash
ros2 launch ur_teleop home.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml
```

按提示在示教器运行 External Control，再在终端按 Enter，确认后执行六轴 Home。仿真与真机不能同时占用同一套控制栈；使用正确的机器人运动学标定，排除其他 RTDE 客户端占用。

## Home 后接管（仿真与真机共用）

Home 阶段使用轨迹控制器，笛卡尔阻抗控制器保持 inactive。teleop 启动后，在关节/TF 反馈有效且已到 Home 时自动切换阻抗，不需要按 RB，也不依赖手柄消息触发。切换失败或超时锁定遥操作，需检查后重启。

若阻抗控制器已经 active，teleop 在反馈有效后自动从当前实测位姿接管，不必再次回 Home。启动时松开 RB；就绪后只有按住 RB 并操作运动输入才产生位姿增量或夹爪命令。RB 不再负责切换控制器；启动时已按住 RB 或发生故障后，仍需先松开再按下，防止意外运动。

## 按键与参考系

| 输入 | 功能 |
| --- | --- |
| 左摇杆上 / 左 | -X / -Y 平移 |
| RT / LT | -Z / +Z 平移 |
| 右摇杆上 / 左 | 绕 +X / +Y 旋转 |
| 十字键左 / 右 | 绕 +Z / -Z 旋转 |
| RB | 按住允许更新目标，松开仍跟踪末次目标 |
| LB | 按住精细速度，默认 25% |
| A | 启用夹爪且 RB 有效时切换夹爪，忙时忽略 |
| X | 切换 base/TCP 参考系 |

base 模式沿 `base_link` 轴运动，TCP 模式沿实测 `tool0` 局部轴运动；平移和旋转都遵循所选模式。X 键切换当帧不叠加增量。左、右摇杆默认分别保留幅值较大的轴；`left_stick_xy_free: true` 只取消左摇杆的主轴过滤。死区后的单轴满量程为 ±1，平移和旋转命令各自按向量模长归一化；LB 再将命令乘 `precision_scale`。

每周期增量由 `max_linear_speed_m_s`、`max_angular_speed_rad_s` 和 `control_hz` 决定。被操作的位置轴以本周期实测位置加增量生成目标，其余轴保持末次目标；旋转输入以实测姿态加旋转增量生成目标。松开 RB 或输入回中时保持完整末次目标。

手柄节点收到的首条 `/cartesian_impedance_controller/current_pose` 锁定本次运行的工作空间原点。目标在控制器参考坐标系按 `workspace_half_extent_m` 对 XYZ 裁剪，默认相对原点各 ±0.20 m；本次节点运行期间不重新锁定。此处只限制目标位置，不限制姿态；控制器内部仍有位姿误差、wrench 和关节力矩限幅。

内部实测位姿来自 `base_link → tool0`；发布前完整转换到控制器要求的参考 link，当前高档真机和仿真配置均为 `base_link`；其他配置按其 `base_frame` 转换。工作空间裁剪在转换后执行，绝对和相对录制动作也在此参考 link 中编码。缺少有效 TF 或工作空间原点时不允许手柄更新目标。

录制按键、数据格式及数据集位置见[数据采集](data_recorder.md)。

## 保护与限制

- RB 只控制手柄输入是否更新目标；Joy、TF、关节反馈或控制器状态失效时退出使能，恢复后需松开再按 RB。
- 工作空间按启动时原点裁剪最终目标。控制器按 `max_pose_error`、`max_wrench` 和 `max_torque` 限幅。
- `/list_controllers` 异步查询控制器状态；明确失活、切换失败或长期无法确认时锁定遥操作，检查后重启。
- `/teleop/e_stop` 为软件停止请求：重置输入使能并尝试取消夹爪，但已有有效目标仍可能持续发布，不等同于停用控制器或硬件急停。
- Xbot Ctrl-C 退出不自动切回轨迹控制器；Alicia 的 restore_controller_on_exit 不适用于 Xbot。

## 终端诊断

顶层 `debug: true` 显示输入、使能原因、控制器状态、数据龄和当前/目标位姿。`xbot.diagnostic_hz` 只改变日志频率，不改变控制频率。诊断位姿使用 `base_link`，发给真机控制器的目标会转换到配置参考系。

| 现象 | 检查 |
| --- | --- |
| RB 无法使能 | 先松开再按下；检查标定、Joy、关节和 TF 反馈 |
| 控制器锁定 | 查看控制器状态和切换失败日志，排查后重启遥操作 |
| 持续提示基座 TF 无效 | 检查控制器参考系与 `base_link` 的 TF |
| 夹爪无响应 | 启用开关、夹爪反馈、Action 服务和忙碌状态 |
| 采集无法开始 | 查看[数据就绪条件](data_recorder.md#数据就绪条件) |

首次[控制接口日志](control_interface_logging.md)不受 debug 开关影响；“已发布”只表示发送目标，不表示机器人已经到位。

## 每段结束后回 Home

record 模式保存或丢弃当前 episode 后，自动停用笛卡尔阻抗控制器、启用轨迹控制器，按 `home.slave` 执行六轴 Home。轨迹成功且反馈在容差内持续达到 `home.verify_duration_s` 后，切回阻抗控制器并以实际 TCP 重新设定保持目标。

回程期间不响应手柄运动或新 episode。成功后先松开 RB，再按住 RB 操作；Menu 开始下一段。最终保存并结束采集也执行回 Home，请保持 Home 和遥操作终端运行到回程完成。没有正在录制的 episode 时，保存/丢弃不会触发运动。

回程使用 `home.move_duration_s`、`move_timeout_s` 和 `at_home_tolerance_rad`。软件停止、切换失败、轨迹失败或超时会锁定遥操作，不自动切回阻抗；排查后重新启动。回程只移动 UR，不自动打开夹爪。
