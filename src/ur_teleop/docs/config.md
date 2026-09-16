# 配置加载与校验（config.py）

> 路径：`ur_teleop/config.py` —— 纯逻辑（无 rclpy 依赖），负责 `ur_teleop.yaml` 的加载/校验、路径解析与夹爪单位换算。

## 概述

config.py 是全包配置的唯一入口：`load_config()` 被 teleop_node、home_node、data_recorder 等所有节点调用，返回原始 dict（不做默认值填充，默认值由各消费方在读取时用 `.get(key, default)` 自行处理）。文件头定义了两套关节名常量与夹爪单位换算函数，供下游模块（joint_mapper、frame_builder、teleop_node）复用。

## 公开接口

### 常量

```python
UR_JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
ALICIA_JOINT_NAMES = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
GRIPPER_JOINT = "Gripper"                    # Alicia 侧夹爪关节名
UR_GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"   # UR 侧夹爪关节名
```

### `load_config(path: str | Path) -> dict[str, Any]`

校验顺序（任一失败抛 `ConfigError(ValueError)`，config.py:29-58）：

1. 文件不存在 → `ConfigError("Config file not found: {p}")`。
2. `yaml.safe_load` 结果空 → 视为 `{}`。
3. 必需顶层键缺失：`_REQUIRED_TOP = ["mode", "sim", "home", "mapping", "safety", "teleop"]`（`cell` / `gripper` / `recorder` 为可选）。
4. `mode` 必须为 `"teleop"` 或 `"record"`。
5. 可选的 `teleop.controller` 必须为 `"forward_position"` 或 `"joint_impedance"`；未写时兼容旧配置并使用 `forward_position`。
6. `mapping` 必需键：`_REQUIRED_MAPPING = ["alicia_joint_order", "ur_joint_order", "sign", "scale"]`；且 `ur_joint_order` 必须恰好 6 项。
7. `home` 必需键：`_REQUIRED_HOME = ["master", "slave"]`，各自必须恰好 6 项（长度不足时报 `home.master must have 6 values` 之类）。
8. `safety.limits` 必须包含全部 6 个 `UR_JOINT_NAMES` 关节名（缺失报 `safety.limits missing joint '{joint}'`）。

### `default_config_path() -> str`（config.py:61）

经 `ament_index_python` 返回 `get_package_share_directory("ur_teleop")/config/ur_teleop.yaml`；包未安装（例如源码直跑）时返回空串。teleop_node 用它作为 `config_file` 参数默认值。

### 夹爪单位换算（Alicia 位置 ↔ 指令值）

```python
gripper_position_to_value(position_m: float, gripper_type: str = "50mm") -> float
gripper_value_to_position(value: float, gripper_type: str = "50mm") -> float
```

- 语义镜像自 `alicia_d_driver`：`value = 1000 - clamp(pos, 0, stroke)/stroke * 1000`（config.py:71-85）。
- 行程：`stroke = 0.05`（`gripper_type == "100mm"`）否则 `0.025`（50mm）。即 50mm 默认、100mm 显式指定。
- 方向约定：**位置 0 = 张开**，对应指令值 1000；位置 = 全行程 = 闭合，对应指令值 0。注意与 Robotiq 侧单位（弧度，0=开）是两套体系，换算只在本包与 alicia_d_driver 之间生效。
- 输入输出均做 `[0, 1000]` clamp。

## 配置键参考（config/ur_teleop.yaml）

按 yaml 分组列出；"默认值"为消费方 `.get()` 兜底值（与 yaml 内联值一致），"必填"仅指 `load_config` 强校验的键。

### mode / sim（顶层，必填）

| 键       | 类型 | 默认值     | 含义                                           | 消费方                                                       |
| -------- | ---- | ---------- | ---------------------------------------------- | ------------------------------------------------------------ |
| `mode` | str  | —（必填） | `teleop` / `record`，非二者报错            | teleop_node`_mode`（launch 参数 `mode` 优先，yaml 兜底） |
| `sim`  | bool | —（必填） | cell 端 sim（mock+rviz）/ real（真机+夹爪+FT） | teleop_node 日志；cell.launch.py 分支                        |

### cell（可选）

| 键                         | 类型 | 默认值           | 含义                                                                                                                                                                                                         | 消费方                                                                                       |
| -------------------------- | ---- | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------- |
| `ur_type`                | str  | `ur10e`        | 机械臂型号                                                                                                                                                                                                   | 仅 cell.launch.py（`_yaml_default` 读，fallback `ur10e`；home.launch.py 不声明也不读取） |
| `robot_ip`               | str  | `192.168.1.1`  | UR 控制器 IP                                                                                                                                                                                                 | cell.launch.py（fallback`0.0.0.0`）/ home.launch.py                                        |
| `gripper_port`           | str  | `/dev/ttyUSB1` | 夹爪串口，real 模式 include robotiq_control 用（作`com_port` 传入）                                                                                                                                        | cell.launch.py（fallback`/dev/ttyUSB1`）                                                   |
| `ftdi_id`                | str  | `""`           | FT 传感器串口标识；real 模式 include rq_fts 驱动（`ft_sensor_standalone.launch.py`）用。**launch 参数名即 `ftdi_id`**（默认值 cell.launch.py:47，声明 59，传 ft_sensor_standalone 105）            | cell.launch.py                                                                               |
| `launch_rviz`            | bool | `true`         | cell 端是否起 rviz（sim 模式且为 true 时）                                                                                                                                                                   | cell.launch.py                                                                               |
| `description_launchfile` | str  | 组合模型 rsp     | rviz/controller_manager 的 URDF 来源——**launch 专属参数，无 yaml 键**；默认 `_description_launchfile()`（ur10e_robotiq_ft_description 组合模型，未安装回退官方 ur_rsp），见 launch.md「rviz 模型」 | cell.launch.py / home.launch.py                                                              |

### home（必填段）

| 键                              | 类型        | 默认值              | 含义                                      | 消费方                                                                                                          |
| ------------------------------- | ----------- | ------------------- | ----------------------------------------- | --------------------------------------------------------------------------------------------------------------- |
| `master` / `slave`          | list[float] | —（必填，各 6 项） | 目标 home（用户提前设置，本包不解释含义） | home_node（规划到位）、teleop_node`_verify_home` 校验                                                         |
| `master_gripper_value`        | float       | `1000`            | 主臂夹爪 home 指令值（0-1000，0=闭）      | home_node.py:39（`_gripper_value`）                                                                           |
| `at_home_tolerance_rad`       | float       | `0.05`            | 判定"已到位"的逐关节误差容限              | home_node、teleop_node`_verify_home`；`force_home:=true` 时 teleop_node 覆写为 `inf`（teleop_node.py:65） |
| `settle_time_s`               | float       | `2.0`             | 到位后静止时长，静止才捕获 offset         | teleop_node`_settling`                                                                                        |
| `settle_motion_threshold_rad` | float       | `0.01`            | 静止判定的最大逐关节运动量                | teleop_node`_settling`                                                                                        |
| `move_timeout_s`              | float       | `30.0`            | 机械臂到位的总体超时                      | home_node.py:41,114                                                                                             |
| `move_duration_s`             | float       | `8.0`             | 轨迹运动时长                              | home_node.py:42,97（`time_from_start`）                                                                       |
| `verify_duration_s`           | float       | `2.0`             | 到位后保持验证时长                        | home_node.py:43,168                                                                                             |

### mapping（必填段）

| 键                     | 类型        | 默认值                | 含义                                 | 消费方                                                            |
| ---------------------- | ----------- | --------------------- | ------------------------------------ | ----------------------------------------------------------------- |
| `alicia_joint_order` | list[str]   | —（必填，6 项）      | 主臂关节顺序                         | JointMapper 构造（经`build_mapping_config`，teleop_node.py:46） |
| `ur_joint_order`     | list[str]   | —（必填，必须 6 项） | 从臂关节顺序                         | 同上；load_config 校验长度                                        |
| `sign`               | list[int]   | —（必填，6 项）      | 每关节方向 ±1                       | 同上（JointMapper`master_to_slave` 公式）                       |
| `scale`              | list[float] | —（必填，6 项）      | 每关节缩放系数（rad 比例，默认 1.0） | 同上                                                              |

### safety（必填段）

| 键                   | 类型                | 默认值                 | 含义                           | 消费方                                      |
| -------------------- | ------------------- | ---------------------- | ------------------------------ | ------------------------------------------- |
| `clamp_margin_rad` | float               | `0.1`                | clamp 时上下界向内收缩的余量   | JointMapper`_clamp`                       |
| `limits`           | dict[str, [lo, hi]] | —（必填，6 关节齐全） | 每关节弧度限位，键为 UR 关节名 | load_config 校验齐全；JointMapper`_clamp` |

### teleop（必填段）

| 键                             | 类型  | 默认值   | 含义                             | 消费方                                           |
| ------------------------------ | ----- | -------- | -------------------------------- | ------------------------------------------------ |
| `controller`                 | str   | `forward_position` | 从臂遥操控制器：`forward_position` 或 `joint_impedance`。前者发布 6 维位置数组；后者发布带六个关节名的 `JointState`，并在 home 阶段由 cell 预加载为 inactive | cell.launch、teleop_node、ruckig_node |
| `command_rate_hz`            | float | `50`   | 控制/发布频率（50 Hz timer）     | teleop_node.py:68,110                            |
| `watchdog_timeout_s`         | float | `0.5`  | 主臂数据超时 → INACTIVE         | teleop_node.py:69,300                            |
| `restore_controller_on_exit` | bool  | `true` | 退出时切回 trajectory controller | teleop_node`shutdown`（teleop_node.py:70,363） |

### gripper（可选段）

| 键                    | 类型  | 默认值                                      | 含义                                           | 消费方                                                                  |
| --------------------- | ----- | ------------------------------------------- | ---------------------------------------------- | ----------------------------------------------------------------------- |
| `enabled`           | bool  | `false`                                   | 是否启用夹爪 FSM；sim 默认 false，real 设 true | GripperController、teleop_node（决定是否建 ActionClient 与 10 Hz tick） |
| `action_server`     | str   | `/robotiq_gripper_controller/gripper_cmd` | 夹爪 action server 名                          | teleop_node`_gripper_action`（ParallelGripperCommand）                |
| `close_threshold_m` | float | `0.0125`                                  | 判定 CLOSED 的位置阈值（m）                    | GripperController`update`                                             |
| `open_threshold_m`  | float | `0.005`                                   | 判定 OPEN 的位置阈值（m）                      | 同上                                                                    |
| `open_pos_rad`      | float | `0.0`                                     | 张开时 knuckle 指令（rad）                     | GripperController`get_knuckle_command`                                |
| `close_pos_rad`     | float | `0.79`                                    | 闭合时 knuckle 指令（rad）                     | 同上                                                                    |
| `max_effort`        | float | `50.0`                                    | 夹爪目标 effort                                | GripperController`max_effort` → goal                                 |
| `fsm_rate_hz`       | float | `10.0`                                    | 夹爪 FSM tick 频率（yaml 未写，代码默认）      | teleop_node.py:113                                                      |

### recorder（可选段）

| 键                         | 类型  | 默认值                  | 含义                                             | 消费方                                         |
| -------------------------- | ----- | ----------------------- | ------------------------------------------------ | ---------------------------------------------- |
| `repo_id`                | str   | `my_user/ur_teleop`   | LeRobot 数据集 repo 名                           | data_recorder.py:131（已存在时追加时间戳新建） |
| `root`                   | str   | `""`                  | 数据集根目录；空 =`HF_LEROBOT_HOME`            | data_recorder.py:132（空则传 None）            |
| `fps`                    | int   | `50`                  | 录制帧率                                         | data_recorder.py:34                            |
| `robot_type`             | str   | `ur10e_alicia_teleop` | LeRobot 机器人类型                               | data_recorder.py:135                           |
| `use_videos`             | bool  | `true`                | 图像特征 dtype：`video`/`image`              | FrameBuilder`features`                       |
| `ee_pose_source`         | str   | `tf`                  | `tf` / `topic(/tcp_pose)` / `none`         | data_recorder.py:54                            |
| `ee_pose_topic`          | str   | `/tcp_pose`           | topic 模式订阅位姿话题                           | data_recorder.py:62                            |
| `ee_pose_parent_frame`   | str   | `base_link`           | tf 查询父系                                      | data_recorder.py:117                           |
| `ee_pose_child_frame`    | str   | `gripper_tcp`         | tf 查询子系                                      | data_recorder.py:118                           |
| `cameras`                | dict  | `{}`                  | `{"wrist": {topic, image_key, height, width}}` | FrameBuilder、data_recorder 图像订阅           |
| `task`                   | str   | `teleoperation`       | 每帧写入的 task 字段                             | FrameBuilder`build`                          |
| `min_frames_per_episode` | int   | `2`                   | 低于则自动丢弃 episode                           | data_recorder.py:35,173                        |
| `state_threshold_rad`    | float | `0.4`                 | UR 夹爪弧度 → 0/1 state 的阈值                  | FrameBuilder`build`                          |

## 错误处理 / 已知边界

- 所有校验失败均抛 `ConfigError`（继承 `ValueError`），节点构造时即失败退出——配置错误不延迟到运行期。
- 可选键不做默认填充：`load_config` 返回原始 dict，各消费方 `.get(key, default)` 自行兜底，默认值散落在各模块（见上表），不在 config.py 集中。
- 校验只查"键存在/长度"，不校验数值范围（如 `sign` 是否为 ±1、`limits[0] < limits[1]`）——数值合法性由 JointMapper/GripperController 的 clamp 与下游行为兜底。
- 使用 `joint_impedance` 时，启动 `home.launch.py`（或 `cell.launch.py`）会以 `--inactive` 预加载 `joint_impedance_controller` 的参数文件；遥操启用时才严格停用轨迹/前向位置控制器并激活它。真实机械臂仍须遵守工作区的 `wrist_3_joint` 单关节测试边界。

## 测试覆盖（tests/test_config.py）

- 最小合法配置加载（`test_load_minimal_valid_config`）。
- 错误路径：缺顶层键（match `mode`）、文件不存在（`not found`）、非法 mode（`mode must be`）、home 长度不足（`home`）、缺 `mapping.alicia_joint_order`、缺 `safety.limits` 某关节（match 关节名）。
- 单位换算：50mm 双向 4 点（0→1000、全行程→0、半行程→500）与 100mm 行程 0.05。
