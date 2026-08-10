# ur_teleop 开发文档索引

> 路径：`/ros2_ws/src/ur_teleop/docs/README.md` — 本包开发文档的总索引：包结构总览、文档地图与推荐阅读顺序。

本文档面向**开发者**。包根 `README.md` 是用户手册（安装、使用、键位、配置表），`docs/` 是开发文档（架构、机制、实现细节、测试覆盖），两者互补、互不照抄。本包所有文档均以 HEAD 代码为准（spec 描述与代码冲突时以代码为准，如 `enable_pending` 锁存为 spec 之后的最终修复）。

## 概述

UR10e 从臂 + Alicia-D 主臂遥操作功能包（ROS 2 Jazzy，ament_python）。主臂 Alicia 100 Hz 发布 `/joint_states`，遥操核心映射后 50 Hz 发布到 UR 前向位置控制；两阶段启动（cell 持久 + home/teleop 分阶段）；teleop / record 双模式；sim（mock UR + rviz）/ real（真机 + 夹爪 + FT300）双形态，**主臂始终是真实 Alicia**。

## 包结构总览

```
ur_teleop/
├── ur_teleop/                  # 源码：3 个节点 + 纯逻辑模块
│   ├── home_node.py            # 阶段 1 节点（一次性）：cell 就绪 → 移双臂 home → 验证 → 退出
│   ├── teleop_node.py          # 阶段 2 核心节点（常驻）：8 态 FSM + 50 Hz 映射 + 夹爪 FSM
│   ├── data_recorder.py        # record 模式节点（常驻）：键盘 + episode 生命周期 + lerobot 录制
│   ├── joint_mapper.py         # 纯逻辑：映射公式 + clamp（无 ROS 导入）
│   ├── gripper_controller.py   # 纯逻辑：夹爪迟滞 FSM（死区阈值）
│   ├── offset.py               # 纯逻辑：会话内 offset 捕获/应用（不写盘）
│   ├── controller_switcher.py  # controller_manager 服务客户端（纯异步，从不 spin）
│   ├── frame_builder.py        # 纯逻辑：record 帧组装（state 14 维 + action 7 维 + 相机）
│   ├── config.py               # 纯逻辑：ur_teleop.yaml 解析 + 校验 + Gripper 单位换算
│   └── keyboard.py             # 纯逻辑：非阻塞 stdin 单键读取（select）
├── launch/                     # 三 launch：cell（持久）/ home（阶段 1）/ teleop（阶段 2）
├── config/
│   ├── ur_teleop.yaml          # 单一配置入口（launch 参数优先、yaml 兜底）
│   └── rviz/ur_teleop.rviz     # sim 模式 rviz 配置
├── tests/                      # 单测 + 集成测试（fake_master.py 仅测试用、不安装）
├── docs/                       # 本文档目录
├── resource/ur_teleop          # ament 资源标记
├── setup.py / package.xml      # 构建与依赖声明
└── README.md                   # 用户手册
```

各层职责：

| 层 | 内容 | 职责与约束 |
|---|---|---|
| 节点层 | `home_node` / `teleop_node` / `data_recorder` | 进程边界：cell 之外仅此三进程；teleop 与 recorder 进程隔离（recorder 崩溃不影响遥操，已保存 episodes 保留）；全部单线程 executor，定时器驱动，回调不阻塞 |
| 纯逻辑层 | `joint_mapper` / `gripper_controller` / `offset` / `config` / `keyboard` / `frame_builder` | 无 rclpy 导入，可独立单测；输入输出均为纯数据 |
| 协议层 | 话题 / 服务 / 动作（见 pipeline.md 清单） | teleop→recorder 仅 3 个话题（commands/status/enable），无请求-应答 |
| 配置层 | `config/ur_teleop.yaml` + `config.py` | 单一入口；缺失必选键解析时 `ConfigError` 报错；launch 参数优先、yaml 兜底 |
| launch 层 | `launch/cell.launch.py` `home.launch.py` `teleop.launch.py` | 两阶段编排与 sim/real 形态选择（详见 launch.md） |
| 测试层 | `tests/` | 35 单测（无 ROS）+ 25 集成（全栈冒烟，约 5 分钟） |

## 文档列表

| 文档 | 一句话摘要 |
|---|---|
| [README.md](README.md) | 本文档：索引、包结构、阅读顺序 |
| [pipeline.md](pipeline.md) | **核心**：两阶段启动、节点拓扑、话题/服务/动作清单、数据流、8 态状态机总览、错误处理总览 |
| [launch.md](launch.md) | 三个 launch 的职责、包含关系、参数表与「launch 参数 > yaml 默认」优先级机制 |
| [config.md](config.md) | `config.py` 解析/校验逻辑、`ur_teleop.yaml` 全部键、Gripper 米 ↔ 0-1000 单位换算（对应 `config.py`） |
| [joint_mapper.md](joint_mapper.md) | 映射公式 `ur_cmd = slave_home + sign·scale·(master − master_home)`、clamp 与 safety（对应 `joint_mapper.py`） |
| [offset.md](offset.md) | 会话内 offset 捕获与应用、不写盘设计（对应 `offset.py`） |
| [gripper_controller.md](gripper_controller.md) | 夹爪迟滞 FSM、死区阈值、开/合目标与指令信号（对应 `gripper_controller.py`） |
| [keyboard.md](keyboard.md) | 非阻塞 stdin 单键读取（select）与 `'enter'` 归一化（对应 `keyboard.py`） |
| [controller_switcher.md](controller_switcher.md) | controller_manager 三服务封装：list/load/switch、异步 Future 链、STRICT 切换（对应 `controller_switcher.py`） |
| [frame_builder.md](frame_builder.md) | record 帧组装：state 14 维 / action 7 维 / 相机、EE NaN 策略（对应 `frame_builder.py`） |
| [home_node.md](home_node.md) | 阶段 1 编排：cell_ready 等待、UR home 轨迹接受重试、Alicia `/joint_commands` 持续命令、到位验证（对应 `home_node.py`） |
| [teleop_node.md](teleop_node.md) | 状态机实现细节：8 态迁移条件、enable 门控/锁存、watchdog、e_stop 冻结、退出恢复（对应 `teleop_node.py`） |
| [data_recorder.md](data_recorder.md) | record 模式键盘（Enter/S/D/Q）、episode 生命周期、lerobot 数据集写入（对应 `data_recorder.py`） |

## 推荐阅读顺序

新人按此顺序阅读：

```
README.md（用户手册，先建立使用直觉）
  → docs/pipeline.md（架构全貌：两阶段、拓扑、数据流、状态机总览）
  → docs/teleop_node.md（最复杂节点：8 态 FSM 的每条迁移与修复）
  → docs/home_node.md（阶段 1：cell 就绪 → home 轨迹 → 验证退出）
  → docs/launch.md（把以上串起来的启动编排）
  → 按需深入：joint_mapper / offset / gripper_controller / frame_builder / data_recorder
```

## 测试覆盖

- **单元测试 35 个**（无 ROS 依赖）：`test_joint_mapper.py`、`test_gripper_controller.py`、`test_offset.py`、`test_config.py`、`test_frame_builder.py`、`test_keyboard.py`。
- **集成测试 25 个**：`test_integration.py`（`-m integration`），假主臂 + mock UR 全栈冒烟（home + teleop + record），约 5 分钟；须设独立 `ROS_DOMAIN_ID`（如 77）避免与本机常驻栈的 `/joint_states` 交织。
- 验证命令与测试细节见包根 `README.md` §6，此处不重复。
