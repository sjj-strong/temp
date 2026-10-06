# Xbot 手柄遥操作

使用 `cartesian_impedance_controller` 控制 `tool0` 法兰位姿，复用 `ur10e_robotiq_ft_description` 的组合 URDF。夹爪末端 `gripper_tcp` 与 `tool0` 不重合。Xbot 不启动 Alicia 或 Ruckig。

Xbot 不在手柄配置中定义阻抗刚度、阻尼、wrench 或力矩；这些参数保存在控制器 YAML 中。`xbot.controller_config_file` 指定该文件，`xbot_cell.launch.py` 将同一完整路径传给 spawner 的 `--param-file`。手柄节点和录制器也从该文件读取控制器参考 link，避免目标与 action 使用另一套坐标系。启动 `home.launch.py` 时终端会打印所传路径；Home 阶段控制器虽为 inactive，参数文件已传入。

```yaml
xbot:
  controller_config_file: /ros2_ws/src/cartesian_impedance_controller/config/ur10e_ft300_cartesian_impedance.yaml
```

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
同时将 `xbot.controller_config_file` 指向 `cartesian_impedance_controller/config/ur10e_xbot_sim_cartesian_impedance.yaml`，其 `base_frame` 为 `base_link`。

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
`xbot.controller_config_file` 可指向 `cartesian_impedance_controller/config/ur10e_ft300_cartesian_impedance.yaml` 或同格式的自定义真机参数文件；默认文件的 `base_frame` 为 `base`。

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

| 输入          | 功能                                   |
| ------------- | -------------------------------------- |
| 左摇杆上 / 左 | +X / +Y 平移                           |
| RT / LT       | +Z / -Z 平移                           |
| 右摇杆上 / 左 | 绕 +X / +Y 旋转                        |
| 十字键左 / 右 | 绕 +Z / -Z 旋转                        |
| RB            | 按住允许更新目标，松开仍跟踪末次目标   |
| LB            | 按住精细速度，默认 25%                 |
| A             | 启用夹爪且 RB 有效时切换夹爪，忙时忽略 |
| X             | 切换 base/TCP 参考系                   |

base 模式沿 `base_link` 轴运动，TCP 模式沿实测 `tool0` 局部轴运动；平移和旋转都遵循所选模式。X 键切换当帧不叠加增量。左、右摇杆默认分别保留幅值较大的轴；`left_stick_xy_free: true` 只取消左摇杆的主轴过滤。死区后的单轴满量程为 ±1，平移和旋转命令各自按向量模长归一化；LB 再将命令乘 `precision_scale`。

有运动输入时，`Δp = action_p × max_linear_speed_m_s / control_hz`，`Δr = action_r × max_angular_speed_rad_s / control_hz`。默认 50 Hz、1.0 m/s 和 5.0 rad/s 对应满量程单周期 20 mm 和 0.1 rad。实际计时只用于识别超过 0.1 秒的卡顿，不放大该周期增量。

目标按自由度更新：有平移输入的轴取“本周期实测 TCP 位置 + 该轴增量”，没有平移输入的轴保持上次发布的目标值；有旋转输入时，以本周期实测姿态加旋转增量，只有平移输入时则保持上次目标姿态。TCP 模式先把局部增量旋转到 `base_link`，再更新其在 `base_link` 中产生非零增量的轴。例如 base 模式只推 Y 时，目标 Y 随实测 Y 推进，而目标 Z 保持原值；实测 Z 偏移不会被下一条目标吸收。被操作轴仍不从上次目标累加。摇杆回中或松开 RB 时保持完整末次目标；反馈或控制器故障时的原有目标重置与重新按 RB 门控继续生效。

手柄节点接管阻抗控制器后，首条 `/cartesian_impedance_controller/current_pose` 锁定本次运行的工作空间原点。目标在控制器参考坐标系按 `workspace_half_extent_m` 对 XYZ 裁剪，默认相对原点各 ±0.20 m；本次节点运行期间不重新锁定。此处只限制目标位置，不限制姿态；控制器内部仍有位姿误差、wrench 和关节力矩限幅。旧的每周期固定增量与目标超前参数已停用。

内部实测位姿来自 `base_link → tool0`；发布前完整转换到控制器要求的参考 link，真机为 `base`，仿真为 `base_link`。工作空间裁剪在转换后执行，绝对和相对录制动作也在此参考 link 中编码。缺少有效 TF 或工作空间原点时不允许手柄更新目标。

录制按键、数据格式及数据集位置见[数据采集](data_recorder.md)。

## 保护与限制

- RB 只控制手柄输入是否更新目标；Joy、TF、关节反馈或控制器状态失效时退出使能，恢复后需松开再按 RB。
- 工作空间按启动时原点裁剪最终目标。移除手柄侧目标超前限制后，控制器仍按 `max_pose_error`、`max_wrench` 和 `max_torque` 限幅。
- `/list_controllers` 异步查询控制器状态；明确失活、切换失败或长期无法确认时锁定遥操作，检查后重启。
- `/teleop/e_stop` 为软件停止请求。真机操作仍需物理急停和现场监护。

## 终端诊断

`xbot.diagnostic_hz` 默认以 5 Hz 输出一行“遥操作诊断”，包括使能/故障原因、RB/LB、归一化平移与旋转输入、目标是否已发布、控制器状态、Joy/TF/关节/控制器查询数据龄，以及当前和目标的完整 `xyz+xyzw` 位姿与两者的位姿差。两组位姿都用 `base_link` 表示，便于直接比较；真机发送给阻抗控制器时会转换到 `base`。`目标已发布=1` 只表示 Xbot 已调用发布接口，需结合控制器终端的“已接收目标 pose”确认接收。改变日志频率不会改变 50 Hz 控制频率。

启动瞬间偶发的“控制器基座 TF 无效”若随后消失，且诊断显示 `目标已发布=1`，表示 TF 已就绪；持续出现时检查 `base` 与 `base_link` 的变换。日志中的目标是 Xbot 根据实测位姿计算并保持的目标，不是阻抗控制器滤波后的内部参考。

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
