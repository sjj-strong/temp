# Xbot 配置参数说明

本页逐项说明 [`config/xbot_teleop.yaml`](../config/xbot_teleop.yaml) 的参数作用。表中数值为配置示例，不是统一的代码默认值；设备、速度、任务和目录应以实际文件为准；实际运行以传给 `config_file` 的文件和启动参数为准。该文件是 Xbot 的独立配置，不继承 `alicia_teleop.yaml`。修改后需重启相应的 Home、遥操作或录制进程；标定文件的按键和轴编号见 `config/xbot_joy.yaml`。

## 运行模式与设备

| 参数 | 示例值 | 作用 |
| --- | --- | --- |
| `mode` | `record` | `teleop.launch.py` 的默认模式：`record` 同时启动手柄遥操作和数据录制器，`teleop` 只启动遥操作。启动时的 `mode:=teleop` 或 `mode:=record` 可覆盖此值；Home 不使用它。 |
| `sim` | `false` | `true` 使用 mock UR 与虚拟 FT300；`false` 连接真机 UR。Xbot Home 直接读取此字段，不受 `home.launch.py sim:=...` 覆盖。 |
| `cell.ur_type` | `ur10e` | 型号记录字段；**当前 Xbot 启动链未读取或转发此值**，组合模型仍按 UR10e 启动。修改此字段不能切换机器人型号。 |
| `cell.robot_ip` | `169.254.138.15` | 真机 UR 控制柜 IP；`sim: true` 时不使用。 |
| `cell.gripper_port` | `/dev/ttyUSB0` | 真机 Robotiq 夹爪串口，仅`sim: false` 且 `gripper.enabled: true` 时连接。 |
| `cell.ftdi_id` | `ttyUSB2` | FT300 设备名，不带`/dev/`；仅 `sim: false` 且 `cell.ft300_enabled: true` 时访问该串口。 |
| `cell.ft300_enabled` | `false` | 真机 FT300 硬件开关。`false` 使用虚拟传感器接口并保留组合模型与 TF，不打开 FT300 串口；虚拟读数不能代表实测力。仿真始终使用虚拟接口。 |
| `cell.launch_rviz` | `true` | Home 启动单元时是否打开 RViz；无图形界面可设为`false`。 |

## 回 Home

| 参数 | 示例值 | 作用 |
| --- | --- | --- |
| `home.slave` | 六个关节角 | UR 的 Home 目标，单位 rad，顺序为`shoulder_pan`、`shoulder_lift`、`elbow`、`wrist_1`、`wrist_2`、`wrist_3`。启动 Home 会发送轨迹运动目标。 |
| `home.at_home_tolerance_rad` | `0.1` | 每个关节与 Home 目标的最大允许误差，单位 rad；也用于遥操作接管前判断是否已到 Home。 |
| `home.move_timeout_s` | `60.0` | 等待 Home 轨迹完成和到位的超时时间，单位秒。 |
| `home.move_duration_s` | `8.0` | 发送给轨迹控制器的 Home 目标到达时间，单位秒；并非实际运动必定耗时。 |
| `home.verify_duration_s` | `2.0` | 到达容差范围后必须连续保持的时间，单位秒，满足后才报告 Home 成功。 |

## 控制方式与手柄

| 参数 | 示例值 | 作用 |
| --- | --- | --- |
| `teleop.control_source` | `xbot` | 选择 Xbot 手柄分支；此配置应保持`xbot`。 |
| `teleop.controller` | `cartesian_impedance` | Xbot 使用笛卡尔阻抗控制器；配置校验要求该值。 |
| `xbot.device_id` | `0` | `teleop.launch.py` 启动 `joy_node` 时使用的手柄设备编号。 |
| `xbot.calibration_file` | `/ros2_ws/src/ur_teleop/config/xbot_joy.yaml` | Joy 按键编号、轴编号、静止值和正方向端点的标定文件；由`xbot_calibrate` 生成。遥操作启动时必须存在。 |
| `xbot.controller_config_file` | `/ros2_ws/src/cartesian_impedance_controller/config/ur10e_ft300_cartesian_impedance.yaml` | 笛卡尔阻抗参数文件；空阻尼会自动计算后加载。支持绝对路径，或相对本 Xbot 配置文件所在目录的路径；手柄与录制器也从中读取 `base_frame`、`tf_prefix` 和 `tip_frame`。仿真时改为 `ur10e_xbot_sim_cartesian_impedance.yaml` 或等效配置。文件不存在、缺少控制器参数或受控末端不是 `tool0` 时启动报错。 |
| `xbot.control_hz` | `50.0` | 遥操作定时器频率，单位 Hz；每周期读取最近一次 Joy 状态及实测`tool0` 位姿并发布目标。 |
| `xbot.diagnostic_hz` | `5.0` | 仅 `debug: true` 时的“遥操作诊断”频率，单位 Hz，不改变控制频率。 |
| `xbot.joy_timeout_s` | `0.25` | 最近一次有效`/joy` 消息允许的数据龄，单位秒；超时退出运动使能，恢复后需松开再按 RB。 |
| `xbot.tcp_timeout_s` | `0.25` | `tool0` TF 和 UR 关节反馈允许的数据龄，单位秒；夹爪反馈新鲜度也使用此值。 |
| `xbot.max_linear_speed_m_s` | `1.0` | 满量程平移速度，单位 m/s；单周期增量为此值除以 `control_hz`。 |
| `xbot.max_angular_speed_rad_s` | `5.0` | 满量程旋转速度，单位 rad/s；单周期旋转向量为此值除以 `control_hz`。 |
| `xbot.workspace_half_extent_m` | `[0.20, 0.20, 0.20]` | 相对控制器首条当前位姿的 XYZ 半范围，单位米；最终目标在参考 link 中逐轴裁剪。 |
| `xbot.precision_scale` | `0.25` | 按住 LB 时平移和旋转增量乘以此系数；`0.25` 表示正常值的四分之一。 |
| `xbot.deadzone` | `0.08` | Joy 轴归一化后的死区；小于阈值的输入视为零，其余输入重新缩放。 |
| `xbot.left_stick_xy_free` | `false` | `false` 时左摇杆 X/Y 只保留幅值较大的主轴；`true` 允许同时输出 XY。右摇杆始终做主轴过滤。 |
| `xbot.view_hold_s` | `0.5` | View 键持续按住达到此秒数时触发录制结束事件，同时锁定本次遥操作。 |

手柄映射由标定文件决定。当前控制逻辑中，RB 按住允许运动，LB 降低增量，A 请求切换夹爪，X 切换 base/TCP 参考系；运动输入和录制按键见[手柄操作](xbot_control.md#按键与参考系)。摇杆回中或松开 RB 时继续发布末次目标；Joy、TF 或关节反馈失效时按故障保护处理。

## 夹爪

| 参数 | 示例值 | 作用 |
| --- | --- | --- |
| `gripper.enabled` | `false` | 同时控制真机夹爪驱动、仿真夹爪控制器及 A 键夹爪命令。关闭时仍保留组合模型的夹爪外形和 TF。 |
| `gripper.action_server` | `/robotiq_gripper_controller/gripper_cmd` | 启用夹爪时发送`ParallelGripperCommand` 的 action 服务名称。 |
| `gripper.close_threshold_m` | `0.0125` | Alicia 夹爪迟滞控制使用的闭合阈值，单位米；**Xbot 不使用，可省略**。 |
| `gripper.open_threshold_m` | `0.005` | Alicia 夹爪迟滞控制使用的张开阈值，单位米；**Xbot 不使用，可省略**。 |
| `gripper.open_pos_rad` | `0.0` | Xbot 发送夹爪打开目标时的关节角，单位 rad。 |
| `gripper.close_pos_rad` | `0.4` | Xbot 发送夹爪闭合目标时的关节角，单位 rad。 |
| `gripper.max_effort` | `50.0` | Xbot 夹爪 action 的最大 effort 命令值；实际单位与限制由夹爪控制器定义。 |

## 数据录制

| 参数 | 示例值 | 作用 |
| --- | --- | --- |
| `recorder.repo_id` | `my_user/ur10e_xbot` | LeRobot 数据集标识；此字段本身不触发上传。 |
| `recorder.root` | `/ros2_ws/dataset/xbot` | 本地数据集目录；已存在的数据集会改用带时间戳的新目录。 |
| `recorder.fps` | `20` | 录制器目标采样频率，单位帧/秒；缺失必要反馈的周期会跳过。 |
| `recorder.robot_type` | `ur10e_xbot_teleop` | 写入数据集元数据的机器人类型字符串。 |
| `recorder.action_space` | `cartesian_pose` | Xbot 固定使用笛卡尔位姿动作，配置校验不接受其他值。 |
| `recorder.action_mode` | `abs` | `abs` 保存最终发布的 xyz+xyzw；`rel` 保存最终目标相对同周期实测 TCP 的 xyz+旋转向量。均另存 `action.reference_link`。 |
| `recorder.record_action_joints` | `true` | Xbot 位姿动作始终保存；该旧字段在 Xbot 中固定为 true。 |
| `recorder.record_action_gripper` | `false` | 是否在 action 末尾保存夹爪二值指令：打开 `0`、闭合 `1`。 |
| `recorder.record_joint_position` / `record_joint_velocity` / `record_joint_effort` | `true` | 分别保存 6 维关节位置、速度、effort；UR 的 effort 可能是电机电流，不当作实测关节力矩。 |
| `recorder.data_timeout_s` | `0.5` | 启用字段及就绪心跳、动作允许的最大数据龄；缺失或超时则跳过该帧。 |
| `recorder.ee_pose_parent_frame` | `base_link` | Alicia 的 TF 父 link；Xbot 自动使用控制器参考 link，当前高档真机和仿真配置均为 `base_link`，其他配置以 `base_frame` 为准。 |
| `recorder.ee_pose_child_frame` | `tool0` | observation 的 TCP 子 link；保存时同时记录该 link。 |
| `recorder.use_videos` | `true` | 非空相机配置下，`true` 将图像字段保存为视频特征，`false` 保存为图像特征；当前 `cameras: {}` 时没有图像字段。 |
| `recorder.cameras` | `{}` | 按 camera.yaml 顶层名称选择相机，仅配置 enabled 和可选 image_key；话题及保存尺寸从相机文件读取。 |
| `recorder.task` | `xbot_teleoperation` | 写入每帧的任务标签。 |
| `recorder.min_frames_per_episode` | `2` | 保存 episode 所需的最少有效帧数；不足时自动丢弃。 |

仅通过 `recorder.record_action_gripper` 控制是否保存夹爪开合指令（打开 `0`、闭合 `1`）。各观测开关及 FT300／UR 内置力数据来源见[数据采集](data_recorder.md)。

`recorder.record_tcp_pose` 控制是否保存 TCP 位姿；`record_wrench` 控制是否保存力／力矩。夹爪只保存指令，不保存实测开合 observation，也不依赖夹爪反馈进行录制。

完整的 `recorder.num_episodes`、视频编码和数据集创建参数见[Record 配置参数](recorder_config.md)。

`recorder.camera_config_file` 可指定自定义相机文件，默认安装目录 config/camera.yaml；相对路径以本次实际配置目录解析。`view_hold_s` 代码缺省为 2 秒，当前 YAML 显式为 0.5 秒。表中的观测开关分别生效，位置缺省 true、速度/effort 缺省 false、TCP 缺省 true、wrench 缺省 false。
