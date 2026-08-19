# pour — 电子秤 PD 倾倒控制

UR10e 倾倒任务:电子秤串口读取 + 核心 PD 控制,驱动 UR 驱动的速度控制器(`forward_velocity_controller`)。

参考实现:`src/robot_utils/liquid_pouring/`(controllers.py / serial_manager.py / robot_driver.py),本包只保留核心 PD 逻辑。

## 节点

| 节点 | 功能 |
|---|---|
| `scale_node` | 后台线程持续读电子秤串口,按 `publish_rate` 发布 `/scale/weight`(`std_msgs/Float64`,克);断线自动重连,解析失败节流 warn |
| `pour_control_node` | 订阅 `/scale/weight`,按 `control_rate` 跑 PD,发布速度到 `/forward_velocity_controller/commands`(6 关节,只动 wrist3);`\|error\| ≤ tolerance` 达标自动停 |

## 前置条件

1. 启动 UR 控制(cell 启动后):
   ```bash
   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=<robot_ip>
   ```
2. **切换速度控制器**(默认激活的是轨迹控制器):
   ```bash
   ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller \
       --activate forward_velocity_controller
   ```
3. 电子秤串口可见:`/dev/ttyUSB0`(容器内需 `-v /dev:/dev` 挂载)。

## 启动

```bash
ros2 launch pour pour.launch.py
# 覆盖参数示例:
ros2 launch pour pour.launch.py port:=/dev/ttyUSB1 target_weight:=100.0 \
    control_rate:=9.0 kp:=0.0045 kd:=0.06 tolerance:=1.0
```

行为:启动即跑;`|error| <= tolerance` 时发零速并结束控制循环(节点保持运行);
`weight_timeout` 秒未收到新重量 → 发零速安全停。

## 配置(`config/pour_params.yaml`,全部参数)

### scale_node

| 参数 | 默认 | 说明 |
|---|---|---|
| `port` | `/dev/ttyUSB0` | 电子秤串口端口 |
| `baudrate` | `9600` | 波特率 |
| `timeout` | `0.1` | 串口读超时(秒) |
| `publish_rate` | `10.0` | 重量发布频率(Hz) |
| `weight_topic` | `/scale/weight` | 重量话题 |

### pour_control_node

| 参数 | 默认 | 说明 |
|---|---|---|
| `weight_topic` | `/scale/weight` | 订阅的重量话题 |
| `cmd_topic` | `/forward_velocity_controller/commands` | 速度指令话题 |
| `joint_state_topic` | `/joint_states` | 关节状态话题 |
| `wrist_index` | `5` | 倾倒关节(0 基,UR10e wrist3) |
| `velocity_sign` | `-1.0` | 速度方向(安装方向不同可改) |
| `control_rate` | `9.0` | 控制频率(Hz) |
| `kp` / `kd` | `0.0045` / `0.06` | PD 增益 |
| `joint_range` | `[-2.0, 2.0]` | 输出速度限幅(rad/s) |
| `target_weight` | `70.0` | 目标倾倒重量(g) |
| `tolerance` | `1.0` | 误差允许范围(g),`\|error\|≤tolerance` 达标停止 |
| `near_target_kd_enable` | `false` | 近目标 kd 缩小 trick 开关(默认关) |
| `near_target_kd_threshold` | `10.0` | trick 触发阈值(g) |
| `near_target_kd` | `0.025` | 近目标时的 kd |
| `weight_timeout` | `2.0` | 重量超时(秒),超时安全停 |

## 串口数据格式

电子秤原始模式输出,单位 g(可省略),示例:
```
ST,GS,    0.15 g
ST,GS,   -0.15 g
0.15
+ 0.15 g
```

## 测试

```bash
cd src/pour && /usr/bin/python3 -m pytest tests -v   # 系统 python3(勿用 venv 的 pytest)
```

## 目录结构

```
pour/
├── config/pour_params.yaml   # 全部配置
├── launch/pour.launch.py
├── pour/
│   ├── scale_serial_node.py  # 串口读取节点
│   ├── pour_control_node.py  # PD 控制节点
│   ├── pd_controller.py      # 核心 PD(纯逻辑)
│   └── scale_protocol.py     # 串口行解析(纯逻辑)
└── tests/                    # pytest 单测
```
