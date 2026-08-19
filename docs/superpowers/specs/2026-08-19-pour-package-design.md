# pour 功能包设计 — 电子秤 PD 倾倒控制

日期:2026-08-19
状态:待审阅

## 背景与目标

参考 `/ros2_ws/src/robot_utils/liquid_pouring/liquid_pouring`(controllers.py / serial_manager.py / robot_driver.py),在 `/ros2_ws/src/` 下新建独立 `pour` 功能包:

- 使用 Universal_Robots_ROS2_Driver 的**速度控制器** `forward_velocity_controller`(velocity_controllers/JointGroupVelocityController),发布 6 关节速度数组,只动倾倒关节(wrist3,默认索引 5)
- 控制算法为**核心 PD 逻辑**(保留自 `MultiModalPDController.calculate()`),接收电子秤串口发布的 weight
- 串口读取独立成节点,持续读串口、按配置频率发布,使用 ROS 日志系统
- **全部配置**(含串口端口)通过 `config/` 目录的 YAML 文件设置,支持 launch 传参覆盖
- 启动即跑,**|误差| ≤ 误差允许范围(tolerance)时自动停止**(发零速)并结束循环

## 架构

两个节点 + 一个 launch,同属 `pour` 包(ament_python):

```
scale_serial_node ── /scale/weight (std_msgs/Float64) ──> pour_control_node ──> /forward_velocity_controller/commands
                                                              │
                                                              └── 订阅 /joint_states (记录关节角,日志用)
```

- `scale_serial_node`:`串口读取 → 解析 → 按 publish_rate 发布`,持续读取,不依赖控制节点
- `pour_control_node`:`订阅 weight → PD 计算 → 按 control_rate 发布速度指令`,达标自停

## 包结构

```
src/pour/
├── package.xml
├── setup.py
├── setup.cfg                # 遵循本工作区 colcon 脚本安装规范($base/lib/<pkg>)
├── resource/pour
├── config/pour_params.yaml  # 全部配置
├── launch/pour.launch.py    # 同时启动两节点,param_file=config/pour_params.yaml
├── pour/
│   ├── __init__.py
│   ├── scale_serial_node.py   # 串口读取节点(可执行)
│   ├── pour_control_node.py   # PD 控制节点(可执行)
│   ├── pd_controller.py       # 纯 PD 逻辑类(无 ROS 依赖,可单测)
│   └── scale_protocol.py      # 串口行解析(参考 serial_manager 正则,可单测)
└── tests/
    ├── test_pd_controller.py
    └── test_scale_protocol.py
```

## 配置(config/pour_params.yaml,全部参数)

```yaml
scale_node:
  ros__parameters:
    port: /dev/ttyUSB0        # 串口端口(必须可配)
    baudrate: 9600
    timeout: 0.1              # 串口读超时(秒)
    publish_rate: 10.0        # 重量发布频率(Hz)
    weight_topic: /scale/weight

pour_control_node:
  ros__parameters:
    weight_topic: /scale/weight
    cmd_topic: /forward_velocity_controller/commands
    joint_state_topic: /joint_states
    wrist_index: 5            # 倾倒关节 = wrist3(0 基序号)
    velocity_sign: -1.0       # 速度方向(参考实现取负)
    control_rate: 9.0         # 控制频率(Hz)
    kp: 0.0045
    kd: 0.06
    joint_range: [-2.0, 2.0]  # 输出速度限幅 (rad/s)
    target_weight: 70.0       # 目标重量 (g)
    tolerance: 1.0            # 误差允许范围 (g):|error| ≤ tolerance 即达标停止
    near_target_kd_enable: false      # 近目标 kd 缩小 trick 开关(默认关)
    near_target_kd_threshold: 10.0    # 触发阈值 (g)
    near_target_kd: 0.025             # 近目标时的 kd 值
    weight_timeout: 2.0       # 重量数据超时(秒):超时发零速安全停
```

launch 支持 `ros2 launch pour pour.launch.py port:=/dev/ttyUSB1` 覆盖。

## 模块设计

### pd_controller.py — 核心 PD(纯逻辑)

参考 `controllers.py` `MultiModalPDController.calculate()` 保留:

```python
class PDController:
    def __init__(self, kp, kd, joint_range, near_target_kd_enable=False,
                 near_target_kd_threshold=10.0, near_target_kd=0.025):
        self.prev_error = 0.0

    def calculate(self, target_weight, current_weight) -> (output, error):
        error = target_weight - current_weight
        derivative = error - self.prev_error          # 每控制周期误差变化
        kd_eff = kd
        if near_target_kd_enable and abs(error) <= near_target_kd_threshold:
            kd_eff = near_target_kd                    # 近目标降微分增益
        output = kp * error + kd_eff * derivative
        output = clip(output, joint_range)
        self.prev_error = error
        return output, error

    def reached(self, error) -> bool:   # |error| ≤ tolerance 由节点传入判断
```

**不保留**(YAGNI):动态 tanh 调参、特征工程、神经网络推理、超调预测、防卡死 +0.01、30 周期确认循环、实验数据记录。

### scale_protocol.py — 串口解析

参考 `serial_manager.py` 的正则,支持 `ST,GS,  0.15 g`、`0.15 g`、`+ 0.15`、`-0.15` 等原始模式:

```python
WEIGHT_PATTERN = re.compile(rb'([+-]?)\s*(\d+(?:\.\d+)?)\s*g?', re.IGNORECASE)
def parse_weight_line(raw: bytes) -> float | None
```

### scale_serial_node.py

- 参数:port / baudrate / timeout / publish_rate / weight_topic
- 打开串口:**先置 DTR/RTS 为 False 再 open**(参考实现经验),失败则 error 日志 + 按重试间隔持续重试,不崩节点
- 后台线程持续 `readline()` + 解析;主循环按 `publish_rate` 定时发布**最新一帧**重量(`std_msgs/Float64`)
- 解析失败:计数,按节流(如每 50 次)warn 日志;连接/断开/发布启动:info 日志;异常:error 日志
- 支持 `SerialException` 断开重连

### pour_control_node.py

- 订阅 `/scale/weight`(reliable QoS,深度 10,与发布端一致),存最新值 + 时间戳
- 订阅 `/joint_states`(BEST_EFFORT,参考实现同款 QoS)记录 wrist 关节角
- 定时循环(`control_rate`):
  1. 取最新 weight;若距离上次收到超过 `weight_timeout` → 发零速、error 日志、退出循环(安全停)
  2. `output, error = pd.calculate(target_weight, current_weight)`
  3. 构造 6 关节数组(其余 0,`wrist_index` 处 = `velocity_sign * output`)发布到 `cmd_topic`
  4. `|error| ≤ tolerance` → 发零速、info 日志、结束循环
  5. 周期日志:按 1Hz 节流打印一条 info(error / output / 关节角),启动时打印控制频率与 PD 参数
- 节点销毁时确保发零速(参考 RobotMover.destroy_node 的 stop 行为)

### launch/pour.launch.py

- 两个 Node 动作,参数都指向 `config/pour_params.yaml`
- 支持 launch 参数覆盖:`port`、`target_weight`、`control_rate`、`kp`、`kd`、`tolerance` 等
- README 注明前置:`ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_velocity_controller`(与参考实现一致,速度控制器需要从默认轨迹控制器切换)

## 测试

- `test_pd_controller.py`:误差方向、导数项、kp/kd 输出正确性、clip 限幅、近目标 kd 开关生效、prev_error 状态更新
- `test_scale_protocol.py`:各格式行解析(正/负/带 g/带 ST,GS 前缀)、无效行返回 None

## 非目标

- 不做模型推理/多控制模式(仅 PD)
- 不做数据录制、不做多关节同时控制(仅倾倒关节)
- 不自动执行控制器切换(文档说明前置命令)

## 验收标准

1. `colcon build` 通过;`ros2 launch pour pour.launch.py` 可启动两节点
2. 串口节点按 `publish_rate` 发布 `/scale/weight`,日志齐全
3. 控制节点按 `control_rate` 运行,达标(|error|≤tolerance)自动发零速停止
4. 所有参数(端口、频率、kp/kd、tolerance、目标重量、话题名)均可通过 config YAML 设置
