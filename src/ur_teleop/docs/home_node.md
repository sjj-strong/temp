# home_node（阶段 1：双臂回 home）

> 路径：ur_teleop/home_node.py
> 职责：真机提示示教器启动外部控制并等回车 → 等 cell 就绪 → 发 UR home 轨迹 → 发布 Alicia home 指令 → 验证双臂到位 → 打印 HOME REACHED 并退出（一次性节点，exit 0/1）。

## 概述

home_node 是两阶段启动的阶段 1 编排节点。Alicia 和 Xbot 真机模式共用 `wait_for_external_control()`：先提示用户在示教器启动外部控制，等待启动终端的回车，打印“已收到回车确认”后才进入后续流程。使用 `/dev/tty` 读取回车，支持 `ros2 launch`；仿真跳过此确认。

随后完成三件事：

1. **等 cell 就绪**：`/joint_states`（双臂数据）与 `/scaled_joint_trajectory_controller/follow_joint_trajectory` action server 同时可用；
2. **移动双臂到 home**：UR 走 trajectory action（`scaled_joint_trajectory_controller`），Alicia 走 `/joint_commands` 持续指令（含夹爪 0–1000 值）；
3. **验证到位**：双臂在容差内保持 `verify_duration_s` 后打印 `HOME REACHED`，退出 0；任一环节失败退出 1。

节点是**单线程**的：所有等待都靠主循环里 `executor.spin_once` 轮询完成——没有后台 spin 线程，没有嵌套 `spin_until_future_complete`（重构问题 7/10 的修复）。与 teleop_node 的"定时器驱动"不同，home_node 是一次性脚本式的顺序流程，每步一个轮询窗口。

## 两阶段启动与 launch 结构

```text
home.launch.py（阶段 1）
 ├── include cell.launch.py          ← 持续运行，阶段 2 共享同一个 controller_manager
 │     sim : ur_robot_driver ur_control.launch.py（use_mock_hardware=true）+ rviz（可选）
 │     real: ur_control.launch.py（真机）+ robotiq_control + rq_fts 驱动
 └── Node home_node                 ← 本节点，完成后退出，cell 不关
teleop.launch.py（阶段 2）            ← 连接已运行的 cell，不再重启 UR 栈
```

cell 由阶段 1 启动并**保持运行**（home.launch.py 的 `IncludeLaunchDescription` + home 节点先等 cell 就绪的时序），两阶段共享同一个 controller_manager，避免 UR 栈重启丢失位姿。home.launch.py 声明 `config_file` / `sim` / `robot_ip` / `gripper_port` / `ftdi_id` / `launch_rviz` / `description_launchfile` 七个参数并透传给 cell；除 `description_launchfile` 默认取组合模型 rsp（`_description_launchfile()`，见 launch.md「rviz 模型」）外，其余全部 **launch 参数优先、yaml 兜底**（`_yaml_default` 从 `alicia_teleop.yaml` 逐级取值，取不到用 fallback）。

cell.launch.py 侧再补 `ur_type`（默认 `ur10e`）：sim 分支走 `ur_robot_driver` 的 `ur_control.launch.py`（`use_mock_hardware=true`）+ rviz（`launch_rviz` 控制）；real 分支真机 + robotiq_control（`gripper_port`）+ rq_fts 驱动（`ftdi_id`）。sim 判定用 `PythonExpression` 比较字符串 `"true"`，规避 `"false"` 字符串真值陷阱。home 节点自身的唯一参数是 `config_file`。

## 生命周期（main 的四步顺序流程）

```python
main()
 ├─ 真机提示启动 External Control 并等回车（Alicia/Xbot 共用，仿真跳过）
 ├─ 等 cell 就绪    ：30 s 轮询 cell_ready()（spin_once 0.5 s）→ 失败 exit 1
 ├─ send_ur_home_trajectory(executor)
 │     └─ 10 s 内等到 goal 被接受 → 等 result（move_timeout_s）→ 失败 exit 1
 ├─ 循环（至 move_timeout_s）：每 50 ms spin_once + 发布 alicia home + 检查 at_home
 │     └─ 连续 verify_duration_s 到位 → ok_verified
 │     └─ 超时未验证 → exit 1
 └─ 打印 HOME REACHED → exit 0
```

代码结构：`HomeNode`（rclpy.node.Node）只承载订阅/发布/查询逻辑，`main()` 承载顺序编排（home_node.py:142-192）。`finally` 中统一 `executor.shutdown()` → `destroy_node()` → `rclpy.shutdown()`，正常与失败路径都不漏清理。

`main()` 的返回码由 `sys.exit(main())` 传导（home_node.py:195-196）：0 = HOME REACHED，1 = 任一失败环节；集成测试 E 组以 rc==0 作为冒烟判据。

成功路径的日志序列：

```text
[INFO] 等待 UR cell 就绪（/joint_states + trajectory action server）...
[INFO] UR home trajectory -> [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
[WARN] UR home goal 未接受（控制器可能尚未激活），重试...   ← cell 快速启动时第一发必被拒
[INFO] Alicia home -> [0.0, -1.2, 0.5, 0, 0, 0]（夹爪开）
==================================================
HOME REACHED — 双臂已到位。现在运行 teleop.launch（阶段 2）。
==================================================
```

## 关键机制

### executor.spin_once 轮询（无嵌套 spin）

所有 future/状态等待都写成"外层 while 轮询 + `executor.spin_once(timeout_sec=...)`"（home_node.py:103-104、115-116、151-152、163-164），从不调用阻塞式 `spin_until_future_complete`。轮询间隔按等待粒度选择：等 cell 用 0.5 s，验证循环用 0.05 s。

### cell_ready 的语义：server 发现 ≠ 控制器激活

`cell_ready()`（home_node.py:73-74）= `slave_q() is not None and _traj_client.server_is_ready()`。action server 在控制器 **configure** 时即被发现（`server_is_ready()` 为 True），但 trajectory 控制器在 **activate** 前仍会 REJECT goal——这正是下一节"accept 重试窗口"存在的根本原因：等 cell 通过只是"服务已出现"，真正能接受轨迹还要再等 activation 窗口（约 1–2 s）。另注意 Jazzy 的 action client 接口是 `server_is_ready()`，**不存在** `server_is_available()`（见测试覆盖 E 组的踩坑记录）。

### send_ur_home_trajectory：goal-accept 10 s 有界重试 + result 等待

两个独立阶段（home_node.py:82-124）：

1. **accept 阶段**（10 s 窗口）：action server 在控制器 configure 时就被发现，而 trajectory 控制器在 **activate 前会 REJECT goal**（cell 启动的 activation 窗口约 1–2 s）→ 未接受即重试：每次 `send_goal_async` 后轮询最多 5 s（`deadline = min(now + 5, accept_deadline)`），未接受打 warn 后 sleep 0.5 s 再发。**第一发必被拒是正常路径不是异常**。窗口耗尽 → `_log_home_failure("UR home trajectory rejected/timed out")`，返回 False。
2. **result 阶段**（`move_timeout_s`）：goal 接受后 `get_result_async()`，轮询 `error_code`；`FollowJointTrajectory.Result.SUCCESSFUL` 才返回 True，其他 error_code 或超时分别打 `"UR home trajectory failed, error_code=..."` / `"UR home trajectory timed out"`。

### D3：双侧 /joint_states 按侧合并

UR cell 与 alicia_d_driver 各自发布 `/joint_states`，**交织在同一话题**（两个发布者）。`_joint_cb`（home_node.py:53-60）用关节名子集判别归属并**逐侧更新各自保留**（`_ur_states` / `_alicia_states`），避免"整包覆盖"导致某侧瞬时缺失（同 teleop_node 的 `_joint_cb`）。读取侧用 `_q()` 按名字取值（home_node.py:62-65），缺任何关节名返回 None——`slave_q()` / `master_q()` 两个访问器供 cell_ready / at_home / 失败日志复用。

### Duration.to_msg()（Jazzy pybind 陷阱）

```python
pt.time_from_start = Duration(seconds=self._move_duration).to_msg()   # home_node.py:97
```

Jazzy 的 `rclpy.duration.Duration` 是 pybind11 实现，不会隐式转换为 `builtin_interfaces/msg/Duration`；直接赋给 `JointTrajectoryPoint.time_from_start` 会序列化失败。必须显式 `.to_msg()`。

### at_home 验证与 hold 逻辑

`at_home()`（home_node.py:133-139）：两侧各自 `max(|实测 − home|) ≤ at_home_tolerance_rad`，任一侧缺数据视为未到位。验证循环（home_node.py:163-172）里 **每 50 ms 持续发布 alicia home 指令**（直到到位，覆盖 driver 串口重连等情况）；到位后记录 `hold_since` 时间戳，保持连续 `verify_duration_s` 才算通过，中途离开 home 立即重置 `hold_since = None`。

## 话题 / 接口

| 方向   | 名称                                                            | 类型                                          | 说明                                 |
| ------ | --------------------------------------------------------------- | --------------------------------------------- | ------------------------------------ |
| 订阅   | `/joint_states`                                               | `sensor_msgs/msg/JointState`                | 双臂数据，按侧合并                   |
| 发布   | `/joint_commands`                                             | `sensor_msgs/msg/JointState`                | Alicia home：6 关节 + Gripper 夹爪值 |
| Action | `/scaled_joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | UR home 轨迹                         |

`publish_alicia_home()`（home_node.py:76-80）的消息结构：

```python
msg.name     = ALICIA_JOINT_NAMES + [GRIPPER_JOINT]     # Joint1..6 + "Gripper"
msg.position = home.master + [master_gripper_value]     # 0-1000，1000 = 开
```

三个接口均默认 QoS（depth 10）：`/joint_commands` 与 `/joint_states` 是可靠传输，`/joint_commands` 单帧丢失无所谓——验证循环每 50 ms 重发即是容错手段。

## 配置键（home.*，alicia_teleop.yaml）

| 键                             | 默认值 | 含义                                |
| ------------------------------ | ------ | ----------------------------------- |
| `home.master`                | 必填   | Alicia 目标 home（6 维弧度）        |
| `home.slave`                 | 必填   | UR 目标 home（6 维弧度）            |
| `home.master_gripper_value`  | 1000.0 | Alicia 夹爪命令值（0 关–1000 开）  |
| `home.at_home_tolerance_rad` | 0.05   | 双臂到位容差（弧度，逐关节 max）    |
| `home.move_timeout_s`        | 30.0   | 轨迹 result 与到位验证的总时限      |
| `home.move_duration_s`       | 8.0    | 轨迹插值时长（`time_from_start`） |
| `home.verify_duration_s`     | 2.0    | 到位后需保持的时间                  |

启动方式：

```bash
ros2 launch ur_teleop home.launch.py           # 含 cell + home 节点
ros2 run ur_teleop home_node --ros-args -p config_file:=/path/to/alicia_teleop.yaml
```

## 错误处理

| 场景                         | 行为（日志原文）                                                                                                                                                     |
| ---------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| cell 未就绪（30 s）          | `error("cell 未就绪。请先运行 home.launch（含 cell）。")`，exit 1                                                                                                  |
| 轨迹被拒/超时（accept 窗口） | warn`"UR home goal 未接受（控制器可能尚未激活），重试..."`；耗尽后 `_log_home_failure("UR home trajectory rejected/timed out")`，exit 1                          |
| 轨迹执行失败/超时            | `_log_home_failure("UR home trajectory failed, error_code=...")` 或 `"UR home trajectory timed out"`，exit 1                                                     |
| 到位超时                     | `error("到位超时（未在 move_timeout 内验证双臂位于 home）。master 目标=…, 当前=…; UR 目标=…, 当前=…. 可用 teleop.launch force_home:=true 跳过验证。")`，exit 1 |

`_log_home_failure`（home_node.py:126-131）统一格式：`原因. 目标=<slave_home>, 当前=<slave_q 或 "无 /joint_states">. 请检查机器人与 cell；cell 保持运行。` —— 出错后 cell **不关**：可直接修复后重跑 home_node，或带 `force_home:=true` 启动阶段 2 跳过到位验证（teleop_node 的 `force_home` 参数把 `home.at_home_tolerance_rad` 置 `inf`，home 节点自身不消费该参数）。重跑时无需重启 home.launch：cell 已在跑，只重等 action server 发现 + 轨迹执行即可。

## 测试覆盖（tests/test_integration.py）

- **白盒 C 组**（`home_node` fixture，in-process 直驱，test_integration.py:925-957）：
  - `test_home_at_home_tolerance_boundary`：恰好 tolerance → `at_home()` True；逐关节 `tol+ε` → False（两侧各 6 关节全扫）；`_ur_states` / `_alicia_states` 置 None（双侧缺数据）→ False；
  - `test_home_publish_alicia_home_opens_gripper`：`publish_alicia_home()` 的消息名为 `ALICIA_JOINT_NAMES + [GRIPPER_JOINT]`、位置为 `[0.0]*6 + [1000.0]`（夹爪开）。
- **子进程冒烟 E 组**：`test_home_node_subprocess_smoke`（test_integration.py:1017-1042）—— 最终评审补上的一环：此前 home_node 只有白盒覆盖，`cell_ready()` 里 `server_is_available` 的 AttributeError（**Jazzy 没有该方法**，代码用 `server_is_ready()`）直到真跑子进程才暴露。本测试起 mock cell + fake_master（**保持 home 静止直到收到 `/teleop/status=true`**）+ `home_node` 完整 `main()`：等 cell → 轨迹（`home.slave` 即 mock UR 初始位姿，轨迹即刻到位）→ alicia home → 验证 → 断言 exit 0 且日志含 `HOME REACHED`（90 s 超时）。

运行方式（集成测试默认被 pytest.ini 的 `addopts = -m "not integration"` 排除，需显式启用）：

```bash
colcon test --packages-select ur_teleop --pytest-args "-m integration"
```
