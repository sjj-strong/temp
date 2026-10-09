# ur_teleop

ROS 2 Jazzy 遥操作与数据采集功能包：使用 **Alicia-D 主臂**或 **Xbot 手柄**控制 UR 从臂，并通过 LeRobot 保存关节、末端位姿、力和相机数据。

两种输入源各使用独立配置：Alicia 使用 `config/alicia_teleop.yaml`，Xbot 使用 `config/xbot_teleop.yaml`。支持 `teleop` 测试与 `record` 采集；相机发布和调参独立于机械臂启动。Home 阶段使用轨迹控制器，遥操作阶段按配置切换至关节或笛卡尔阻抗控制器，详见[控制器说明](docs/controllers.md)。

使用顺序：**硬件与配置准备 → 相机调试 → Home → teleop 验证 → record 采集**。正确的终端顺序和可复制命令见[完整使用流程](docs/workflow.md)，具体参数见下列文档。

## 文档导航

| 文档                                             | 作用                                                           |
| ------------------------------------------------ | -------------------------------------------------------------- |
| [docs/README.md](docs/README.md)                  | 按使用阶段组织的文档入口                                       |
| [完整使用流程](docs/workflow.md)                  | 从准备到 teleop 测试、record 采集的最短操作顺序和命令          |
| [硬件与环境准备](docs/hardware.md)                | UR 网络、Alicia 串口、Xbot 标定、相机和可选夹爪/FT300 接入     |
| [控制器说明](docs/controllers.md)                 | 两种输入源使用的控制器、配置文件、切换顺序及对应控制器包文档   |
| [启动与相机](docs/launch.md)                      | Home/teleop 两阶段启动、参数覆盖规则和独立相机发布             |
| [相机检查与调参](docs/camera_inspector.md)        | PySide6 + PyQtGraph 实时预览、原生参数调节及相机 YAML 字段生成 |
| [读取从臂初始位置](docs/capture_slave_home.md)    | 读取 UR 当前关节角，备份并更新两个配置的`home.slave`         |
| [Xbot 手柄遥操作](docs/xbot_control.md)           | 手柄标定、仿真/真机启动、按键、坐标系和故障处理                |
| [Xbot 独立手柄测试](docs/xbot_joy_test.md)        | 不连接机器人，检查按键、运动方向和录制操作映射                 |
| [Xbot 配置参数](docs/xbot_teleop_config.md)       | `xbot_teleop.yaml` 参数含义、单位和生效条件                  |
| [Record 配置参数](docs/recorder_config.md)        | 目标 episode 数、全部 LeRobot 创建选项及数据字段开关           |
| [数据采集](docs/data_recorder.md)                 | 开始/保存/丢弃 episode、数据字段、相机录制和 debug/tqdm 日志   |
| [Alicia 配置与校验](docs/alicia_teleop_config.md) | Alicia 配置、字段默认值、继承与加载规则                        |
| [数据流与架构](docs/pipeline.md)                  | Alicia 数据流及模块之间的关系                                  |
| [Home 节点](docs/home_node.md)                    | Home 等待、External Control 回车确认、轨迹发送与到位验证       |
| [遥操作节点](docs/teleop_node.md)                 | Alicia 状态机、使能、反馈超时与退出流程                        |
| [控制接口日志](docs/control_interface_logging.md) | 首次发布控制目标时的接口、消息类型与控制器事件说明             |
| [控制器切换](docs/controller_switcher.md)         | controller_manager 异步查询、加载和切换接口                    |
| [关节映射](docs/joint_mapper.md)                  | Alicia 到 UR 的关节顺序、符号、缩放和限位                      |
| [会话偏移](docs/session_offset.md)                | 静止检测与主从臂 offset 捕获                                   |
| [Ruckig 平滑](docs/ruckig_node.md)                | 关节目标的速度、加速度、jerk 限制和发布接口                    |
| [夹爪控制](docs/gripper_controller.md)            | 夹爪执行开关、迟滞控制和 action 接口                           |
| [键盘读取](docs/keyboard.md)                      | `ros2 launch` 下通过控制终端读取非阻塞按键                   |
