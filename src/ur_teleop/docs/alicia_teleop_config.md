# Alicia 配置参数

配置文件为 [`config/alicia_teleop.yaml`](../config/alicia_teleop.yaml)。推荐在 Home 和遥操作命令中显式传入源码配置路径；不传时使用安装目录的配置。仿真、真机和采集命令见[完整流程](workflow.md)。

## 加载与继承

`base_config` 指向父配置，相对路径以当前配置所在目录为基准；字典递归合并，列表和其他值由子配置替换，循环继承会报错。`mode`、`sim`、`home`、`teleop` 为共同必需段；Alicia 还要求 `mapping`、`safety` 和双臂 Home。Xbot 的控制器参数路径会按最终配置文件所在目录解析，其他字段并非都自动相对父配置解析。

代码默认值与 YAML 中显式值不同。下面分别列出，不把用户当前设备、Home 或数据目录写成通用默认值。

## 顶层与设备

`mode` 为 teleop/record，遥操作启动参数 `mode` 非空时覆盖。`sim` 控制模拟/真实单元，但 Alicia Home 的启动默认读取安装 Alicia 文件，指定自定义 config_file 不会自动重算所有 CLI 默认；必要时显式传 sim 和 controller。Xbot 的不同处理见[启动说明](launch.md)。

`debug` 缺省 false，只接受 YAML 布尔值。它控制诊断，错误、操作、配置频率和首次控制接口日志仍可见。

| `cell` 字段 | 作用 |
| --- | --- |
| `ur_type` | cell 的 UR 型号；Home 没有同名声明，默认型号从安装 Alicia 配置取得 |
| `robot_ip` | 真机 UR 地址；launch 缺失字段时兜底 0.0.0.0，实际必须填写可连接地址 |
| `gripper_port` | 真机独立 Robotiq 驱动串口 |
| `ftdi_id` | Alicia 真机独立 FT300 驱动标识 |
| `ft300_enabled` | 配合 Home 的 enable_ft300 控制实际设备启动 |
| `launch_rviz` | Alicia cell 中仅 sim 分支打开 RViz |
| `launch_alicia`、`alicia_port` | Alicia 驱动启动与串口；UR mock 仍可连接真实主臂 |

Home 可用的完整参数见[启动说明](launch.md)。

## Home 与静止检测

| `home` 字段 | 代码缺省 | 作用 |
| --- | --- | --- |
| `master`、`slave` | 必填六项 | 目标位置，rad |
| `master_gripper_value` | 1000 | Alicia Gripper 指令，1000=开 |
| `at_home_tolerance_rad` | 0.05 | 最大关节误差；当前 YAML 为 0.1 rad |
| `move_timeout_s` | 30 | Home 的分阶段等待窗口；当前 YAML 为 60 秒 |
| `move_duration_s` | 8 | Home 轨迹点到达时间，秒 |
| `verify_duration_s` | 2 | Home 连续到位验证，秒 |
| `settle_time_s` | 2 | teleop 捕获 offset 前静止时间，秒 |
| `settle_motion_threshold_rad` | 0.01 | teleop 逐周期静止阈值，rad |

Home 与实际捕获值的区别见[Home 节点](home_node.md)和[会话偏移](session_offset.md)。

## 映射与遥操作

`mapping.alicia_joint_order`、`ur_joint_order`、`sign`、`scale` 均为六项，sign 和 scale 按索引作用于主臂相对捕获位置的变化；当前 YAML scale 为 0.8。`safety.limits` 必须包含六个 UR 关节，`clamp_margin_rad` 代码缺省 0.1 rad。名称列表并不使节点自动重排关节，具体约束见[关节映射](joint_mapper.md)。

| `teleop` 字段 | 代码缺省 | 当前 YAML / 含义 |
| --- | --- | --- |
| `control_source` | alicia | 缺省选择 Alicia |
| `controller` | forward_position | YAML 选择 joint_impedance |
| `command_rate_hz` | 50 | YAML 为 500 Hz；遥操定时器频率 |
| `watchdog_timeout_s` | 0.5 | 主臂反馈超时阈值，秒 |
| `restore_controller_on_exit` | true | 退出尝试切回轨迹控制器 |

`ruckig.enabled` 缺省 true，仅对关节阻抗路径生效；前向位置始终经过 Ruckig。`ruckig.control_hz` 由 teleop launch 读取，缺省 500 Hz；独立 Ruckig 节点的 ROS 参数缺省为 100 Hz。六维 max_velocity/max_acceleration/max_jerk 的单位分别为 rad/s、rad/s²、rad/s³，详见[Ruckig](ruckig_node.md)。不要把 launch 的 use_ruckig 当作覆盖 teleop 的 YAML 开关，当前两者未同步。

## 夹爪与录制

夹爪配置和请求门控见[夹爪控制](gripper_controller.md)。`gripper.enabled` 控制执行；`recorder.record_action_gripper` 仅控制数据字段。两者可以独立关闭。

全部 recorder 字段及创建参数见[录制参数](recorder_config.md)。`recorder.cameras` 仅按名称选择相机并可指定 image_key；发布话题、采集尺寸、resize 和保存宽高来自独立 `camera.yaml`。自定义相机文件用 `recorder.camera_config_file`，相对路径以实际遥操作配置所在目录为基准。
