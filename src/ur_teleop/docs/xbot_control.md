# Xbot 手柄遥操作

使用 `cartesian_impedance_controller` 控制 `tool0` 法兰位姿，复用 `ur10e_robotiq_ft_description` 的组合 URDF。夹爪末端 `gripper_tcp` 与 `tool0` 不重合。Xbot 不启动 Alicia 或 Ruckig。

Xbot 不定义阻抗刚度、阻尼、wrench、力矩或速度限制。`xbot_cell.launch.py` 在仿真时加载
`cartesian_impedance_controller/config/ur10e_xbot_sim_cartesian_impedance.yaml`，在真机时加载
`cartesian_impedance_controller/config/ur10e_ft300_cartesian_impedance.yaml`；控制参数只在控制器包内维护。

## 配置与校准

配置入口为 `config/xbot_teleop.yaml`，该文件是完整的 XBot 独立配置，**不继承** `ur_teleop.yaml`。
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
ros2 run ur_teleop xbot_calibrate
```

标定过程不需要按 Enter：每步先松开所有按键和扳机、摇杆回中并稳定一秒；看到下一条提示后，按键与十字键按下即记录，模拟轴输入推到极限并保持一秒。每项记录后会打印识别出的 Joy 编号；若 30 秒内没有收到清晰输入，脚本会超时提示重试。结果默认写入 `/ros2_ws/src/ur_teleop/config/xbot_joy.yaml`，遥操作从同一路径读取；已有文件不覆盖，另存时用 `--output` 并更新 `xbot.calibration_file`。映射使用 ROS Joy 编号，不能直接复制 pygame 轴号。

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

在 `config/xbot_teleop.yaml` 中设置 `sim: false`，核对 `cell.robot_ip` 和 `home.slave`。只有启用对应设备时才需要核对 `cell.gripper_port` 或 `cell.ftdi_id`。

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

在当前双设备关闭配置下，FT300 驱动日志应显示 `use_fake_mode -> 1`，不会出现独立夹爪控制栈；`ros2 control list_controllers -c /controller_manager` 应显示 `cartesian_impedance_controller` 在 Home 阶段为 inactive。只有确认手柄反馈与机器人状态正常后才启动第二终端。

## Home 后接管（仿真与真机共用）

Home 阶段使用轨迹控制器，笛卡尔阻抗控制器保持 inactive。teleop 启动后，在关节/TF 反馈有效且已到 Home 时自动切换阻抗，不需要按 RB，也不依赖手柄消息触发。切换失败或超时锁定遥操作，需检查后重启。

若阻抗控制器已经 active，teleop 在反馈有效后自动从当前实测位姿接管，不必再次回 Home。启动时松开 RB；就绪后只有按住 RB 并操作运动输入才产生位姿增量或夹爪命令。RB 不再负责切换控制器；启动时已按住 RB 或发生故障后，仍需先松开再按下，防止意外运动。

## 按键与参考系

| 输入          | 功能                        |
| ------------- | --------------------------- |
| 左摇杆上 / 左 | +X / +Y 平移                |
| RT / LT       | +Z / -Z 平移                |
| 右摇杆上 / 左 | 绕 +X / +Y 旋转             |
| 十字键左 / 右 | 绕 +Z / -Z 旋转             |
| RB            | 按住允许更新目标，松开仍跟踪末次目标 |
| LB            | 按住精细速度，默认 25%      |
| A             | 启用夹爪且 RB 有效时切换夹爪，忙时忽略 |
| X             | 切换 base/TCP 参考系        |

base 模式沿 `base_link` 的轴运动；TCP 模式沿实测 `tool0` 当前局部轴运动。平移和旋转均遵循所选参考系，切换当帧不叠加手柄增量；目标超前限制仍会生效。

仅在 RB 按住且运动输入非零时，手柄增量会累加到当前锁存目标；因此持续保持非零输入会以 `control_hz` 持续推进并发布新的目标位姿。首次更新以前会先用实测 `tool0` 位姿初始化目标。归一化输入直接乘 `xbot.max_translation_delta_m`（米）和 `xbot.max_rotation_delta_rad`（弧度），不乘周期时长；旋转增量用四元数合成。

松开 RB、摇杆回中或无输入地再次按下 RB，均不再累加手柄增量，控制器继续跟踪锁存目标。每周期仍会根据实测位姿约束目标超前量；若实测位姿变化导致超限，锁存目标会向实测位姿收回。遥操作节点会持续发布约束后的目标。

当前配置的最大增量为每周期 0.4 mm / 0.002 rad；50 Hz 下持续满杆对应目标每秒推进 20 mm / 0.10 rad，低于控制器当前 0.05 m/s / 0.2 rad/s 的内部参考速度上限。此前 10 mm / 0.08 rad 每周期的配置会在约两个控制周期内触及 20 mm / 0.10 rad 超前上限，造成目标反复被限幅。每周期增量不乘周期时长。目标相对实测 `tool0` 的超前上限分别为 `max_target_position_error_m: 0.02` 和 `max_target_orientation_error_rad: 0.10`：机械臂跟随时目标可持续前进，实测位姿停滞时目标停止继续超前。这两个上限不限制累计行程；实际运动仍受阻抗控制器的参考速度、力矩和安全限制约束。旧速度参数不再接受。

平移超前限幅优先收回最近操作的轴。例如仅操作 X 时，即使实测 Z 出现少量漂移，目标 Z 也保持不变；只有非操作轴的误差单独超过 20 mm 时才收回该轴。目标与实测位姿的三维总距离始终限制在 20 mm 内。

内部实测 TF 和录制动作统一使用 `base_link → tool0`。发布到 `/cartesian_impedance_controller/target_pose` 前，读取本模式安装的控制器 YAML 中的 `tf_prefix`、`base_frame`、`tip_frame`，通过 TF 同时转换目标位置和姿态：默认仿真为 `base_link`，真机为 `base`，受控末端始终为 `tool0`。缺少有效基座变换时禁止使能及发布目标。

`Ignoring target_pose outside base frame or with non-finite position` 表示控制器拒收目标，也可能由工作空间越界触发。检查消息坐标系是否与控制器一致；不能仅修改 `frame_id` 而不转换位姿。修改控制器坐标系配置后需在安全停止后重新启动 Home 和 teleop，确保两端加载相同配置。

录制按键、数据格式及数据集位置见[数据采集](data_recorder.md)。

## 保护与限制

- 默认 50 Hz 更新；平移、旋转增量分别按向量模长限制，LB 将两者缩小至 25%。这不是机械臂实际速度限制。
- 松开 RB 只禁止新的手柄增量，正常情况下继续跟踪最后目标；若目标相对实测位姿超限，限幅仍会修正目标。Joy/TF/关节反馈超时、软件急停或定时器卡顿仍退出使能并锁存故障时实测位姿作为保持目标；默认超时 0.25 秒，恢复后须重新按 RB。
- 每周期的手柄增量分别按向量模长限制；目标相对实测位姿的平移距离和最短姿态角分别限幅。控制器自身的工作空间、力和力矩限制保持不变。
- `/list_controllers` 每 0.5 秒异步查询一次；单次查询延迟或失败不打断 50 Hz 目标更新。明确查到阻抗控制器非 active、切换失败，或连续 10 秒无法确认控制器状态时锁定遥操作，检查后重启。
- 故障保持位姿在故障开始时锁定；TF 丢失只能使用最后有效实测位姿。夹爪停止使用 action cancel，实际效果取决于夹爪控制器。
- `/teleop/e_stop` 是软件停止请求，不是安全认证急停。遥操作进程崩溃后，阻抗控制器仍可能追踪最后目标；必须保留物理急停和人工监护。
- `/teleop/xbot_status` 显示参考系及运动状态，`/teleop/xbot_ready` 提供录制就绪心跳。

## 终端诊断

`xbot.diagnostic_hz` 默认以 5 Hz 输出一行“遥操作诊断”，包括使能/故障原因、RB/LB、归一化平移与旋转输入、目标是否已发布、控制器状态、Joy/TF/关节/控制器查询数据龄，以及当前和目标的完整 `xyz+xyzw` 位姿与超前误差。两组位姿都用 `base_link` 表示，便于直接比较；真机发送给阻抗控制器时会转换到 `base`。`目标已发布=1` 只表示 Xbot 已调用发布接口，需结合控制器终端的“已接收目标 pose”确认接收。改变日志频率不会改变 50 Hz 控制频率。

启动瞬间偶发的“控制器基座 TF 无效”若随后消失，且诊断显示 `目标已发布=1`，表示 TF 已就绪；持续出现时检查 `base` 与 `base_link` 的变换。日志中的目标是 Xbot 锁存目标，不是阻抗控制器滤波后的内部参考。

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
