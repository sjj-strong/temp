# Xbot 手柄遥操作

使用 `cartesian_impedance_controller` 控制 `tool0`（TCP），复用 `ur10e_robotiq_ft_description` 的组合 URDF。Xbot 不启动 Alicia 或 Ruckig。

Xbot 不定义阻抗刚度、阻尼、wrench、力矩或速度限制。`xbot_cell.launch.py` 在仿真时加载
`cartesian_impedance_controller/config/ur10e_xbot_sim_cartesian_impedance.yaml`，在真机时加载
`cartesian_impedance_controller/config/ur10e_ft300_cartesian_impedance.yaml`；控制参数只在控制器包内维护。

## 配置与校准

配置入口为 `config/xbot_teleop.yaml`，该文件是完整的 XBot 独立配置，**不继承** `ur_teleop.yaml`。
其中 `home.slave` 是 UR 的六关节 Home 位姿，按 `shoulder_pan`、`shoulder_lift`、`elbow`、`wrist_1`、
`wrist_2`、`wrist_3` 顺序填写，单位为 rad。XBot 模式不需要 Alicia 的 `home.master`、关节映射或 Alicia
串口字段。Xbot 的仿真/真机选择以配置文件的 `sim` 为准，不使用 Home 的 `sim:=` 参数覆盖。
`cell.launch_rviz: true` 默认让两种模式的 Home 启动同时打开 RViz；无图形界面时可在配置中设为 `false`。

在两个终端分别加载 ROS 环境后运行：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

# 终端 1：仅发布手柄输入
ros2 run joy joy_node --ros-args -p autorepeat_rate:=50.0 -p deadzone:=0.0

# 终端 2：按提示校准，不发送机器人指令
ros2 run ur_teleop xbot_calibrate
```

每次操作推到极限并保持，步骤间松开按键、摇杆回中。结果默认写入 `/ros2_ws/src/ur_teleop/config/xbot_joy.yaml`，遥操作从同一路径读取；已有文件不覆盖，另存时用 `--output` 并更新 `xbot.calibration_file`。映射使用 ROS Joy 编号，不能直接复制 pygame 轴号。

校准完成后停止手动启动的 `joy_node`，正式启动会自动运行它。

## 仿真启动

在 `config/xbot_teleop.yaml` 中设置 `sim: true`。使用组合模型的 `xbot_effort_mock`，不连接 UR 真机；手柄仍使用真实设备。

两个终端均加载上述 ROS 环境，并设置独立的仿真域，避免与真机的 `/robot_description`、`/controller_manager` 混用：

```bash
export ROS_DOMAIN_ID=225
```

```bash
# 终端 1：模拟 UR 回 Home，到位后保持此终端运行
ros2 launch ur_teleop home.launch.py config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml

# 终端 2：仅遥操作；录制时改为 mode:=record
ros2 launch ur_teleop teleop.launch.py config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml mode:=teleop
```

日志中应加载 `joint_impedance_controller/JointImpedanceMockSystem`；夹爪为 `mock_components/GenericSystem`。此模式的 `HOME REACHED` 仅表示模拟机械臂到位。

## 真机启动

在 `config/xbot_teleop.yaml` 中设置 `sim: false`，核对 `cell.robot_ip`、`cell.gripper_port`、`cell.ftdi_id` 和 `home.slave`。

启动前确认：

- 停止仿真及重复的机器人控制栈，两个终端使用相同的真机 `ROS_DOMAIN_ID`，不得沿用仍有 mock 节点的域。
- 无其他 RTDE 控制客户端占用机器人；出现 `speed_slider_mask ... controlled by another RTDE client` 时先排除占用，不继续遥操作。
- 使用该机器人对应的运动学标定，TCP 为 `tool0`；出现 calibration mismatch 时先处理标定，不继续笛卡尔遥操作。
- 确认 Home 运动路径无障碍，物理急停可用并有人工监护。**Home 启动会自动发送运动目标，不只是启动驱动。**

两个终端均加载上述 ROS 环境后运行：

```bash
# 终端 1：启动真机控制栈并回 Home，到位后保持运行
ros2 launch ur_teleop home.launch.py config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml

# 终端 2：确认真机 Home 成功后启动；录制时改为 mode:=record
ros2 launch ur_teleop teleop.launch.py config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml mode:=teleop
```

实机复用 `real_bringup.launch.py`，FT300 不另开串口驱动。日志应显示 UR 真机硬件 `ur_robot_driver/URPositionHardwareInterface` 成功连接并激活，而非 `JointImpedanceMockSystem`；不能仅凭 `HOME REACHED` 判断连接了真机。

## Home 后接管（仿真与真机共用）

Home 阶段使用轨迹控制器，笛卡尔阻抗控制器保持 inactive。teleop 启动后，在关节/TF 反馈有效且已到 Home 时自动切换阻抗，不需要按 RB，也不依赖手柄消息触发。切换失败或超时锁定遥操作，需检查后重启。

若阻抗控制器已经 active，teleop 在反馈有效后自动从当前实测位姿接管，不必再次回 Home。启动时松开 RB；就绪后只有按住 RB 并操作运动输入才产生位姿增量或夹爪命令。RB 不再负责切换控制器；启动时已按住 RB 或发生故障后，仍需先松开再按下，防止意外运动。

## 按键与参考系

| 输入 | 功能 |
| --- | --- |
| 左摇杆上 / 左 | +X / +Y 平移 |
| RT / LT | +Z / -Z 平移 |
| 右摇杆上 / 左 | 绕 +X / +Y 旋转 |
| 十字键左 / 右 | 绕 +Z / -Z 旋转 |
| RB | 按住允许更新目标，松开仍跟踪末次目标 |
| LB | 按住精细速度，默认 25% |
| A | RB 有效时切换夹爪，忙时忽略 |
| X | 切换 base/TCP 参考系 |

base 模式沿 `base_link` 的轴运动；TCP 模式沿实测 `tool0` 当前局部轴运动。平移和旋转均遵循所选参考系，切换当帧目标不变。

仅在 RB 按住且运动输入非零时，以本周期实测 `tool0` 位姿叠加手柄增量更新目标，不累加上一周期目标。归一化输入直接乘 `xbot.max_translation_delta_m`（米）和 `xbot.max_rotation_delta_rad`（弧度），不乘周期时长；旋转增量用四元数合成。

松开 RB、摇杆回中或无输入地再次按下 RB，均不覆盖最后目标，控制器继续跟踪该目标。RB 是目标更新许可，不是松开即停的开关；有持续非零输入时仍每周期更新目标。遥操作节点会重复发布锁存的目标，不代表生成了新的目标位姿。

默认最大增量为 0.4 mm / 0.002 rad，保持旧实现 50 Hz 下的目标偏移幅度，未提高真机驱动力。增量与更新频率无关，旧速度参数不再接受；需要更大偏移时在安全评估后单独调整增量参数。

内部实测 TF 和录制动作统一使用 `base_link → tool0`。发布到 `/cartesian_impedance_controller/target_pose` 前，读取本模式安装的控制器 YAML 中的 `tf_prefix`、`base_frame`、`tip_frame`，通过 TF 同时转换目标位置和姿态：默认仿真为 `base_link`，真机为 `base`，TCP 始终为 `tool0`。缺少有效基座变换时禁止使能及发布目标。

`Ignoring target_pose outside base frame or with non-finite position` 表示控制器拒收目标，也可能由工作空间越界触发。检查消息坐标系是否与控制器一致；不能仅修改 `frame_id` 而不转换位姿。修改控制器坐标系配置后需在安全停止后重新启动 Home 和 teleop，确保两端加载相同配置。

录制按键、数据格式及数据集位置见[数据采集](data_recorder.md)。

## 保护与限制

- 默认 50 Hz 更新；平移、旋转增量分别按向量模长限制，LB 将两者缩小至 25%。这不是机械臂实际速度限制。
- 松开 RB 只禁止后续目标更新，不停止对最后目标的跟踪。Joy/TF/关节反馈超时、软件急停或定时器卡顿仍退出使能并锁存故障时实测位姿作为保持目标；默认超时 0.25 秒，恢复后须重新按 RB。
- 不再设置额外的目标偏移阈值；手柄偏移幅度仅由平移/旋转增量参数限制，控制器自身的工作空间、力和力矩限制保持不变。
- 故障保持位姿在故障开始时锁定；TF 丢失只能使用最后有效实测位姿。夹爪停止使用 action cancel，实际效果取决于夹爪控制器。
- `/teleop/e_stop` 是软件停止请求，不是安全认证急停。遥操作进程崩溃后，阻抗控制器仍可能追踪最后目标；必须保留物理急停和人工监护。
- `/teleop/xbot_status` 显示参考系及运动状态，`/teleop/xbot_ready` 提供录制就绪心跳。

## 测试

仅在 `sim: true` 的隔离域中运行。两个终端均设置 `ROS_DOMAIN_ID=225` 和相同的 `ROS_HOME`：

```bash
export ROS_DOMAIN_ID=225
export ROS_HOME=/tmp/ur_xbot_validation
```

终端 1 按上文启动 Home；到位后，终端 2 执行：

```bash
UR_XBOT_MOCK_TEST=1 PYTHONPATH=/ros2_ws/src/ur_teleop:$PYTHONPATH \
  /usr/bin/python3 -m pytest /ros2_ws/src/ur_teleop/tests -q
```

测试会切换控制器和夹爪，重复执行前恢复轨迹 active、阻抗 inactive、夹爪打开，或重新启动 mock Home。

mock 已覆盖控制器切换、参考系、夹爪、断连恢复和录制事件。曾出现夹爪时序测试间歇性失败，恢复初态后三次复测未重现。物理手柄、真机动作、相机视频编码及 LeRobot 实际落盘尚未验收。
