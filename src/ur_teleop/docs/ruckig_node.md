# Ruckig 在线平滑

实现：`ur_teleop/ruckig_node.py`。订阅 `/ruckig/target_joint_positions` 六维关节目标，使用 UR `/joint_states` 初始化在线轨迹生成器，再按 `teleop.controller` 发布：

| 控制器选择 | 话题 | 类型 |
| --- | --- | --- |
| `forward_position` | `/forward_position_controller/commands` | Float64MultiArray |
| `joint_impedance` | `/joint_impedance_controller/target_joint_state` | JointState，六关节名称和 position |

ROS 参数 `control_hz` 的节点缺省是 **100 Hz**。`teleop.launch.py` 则显式传入 `ruckig_control_hz`，为空时读取 YAML `ruckig.control_hz`，该 launch 兜底为 **500 Hz**；当前 Alicia 配置也为 500 Hz。独立 `ros2 run` 不会自动采用 YAML 中的 control_hz，需显式传参数。

初始化前须六个 UR 关节都收到过反馈；位置和速度来自实测值，初始目标设为当前位置。初始化后每 tick 都执行 `otg.update()`，发布输出再 `out.pass_to_input(inp)`，不等待 teleop ACTIVE 信号。实测初速度可能非零，不能保证启动后绝对没有运动。后续反馈缓存更新，但正常 OTG 的当前状态沿用前一步输出，而非每步重新覆盖实测值。

目标长度不为 6 或含 NaN/Inf 时拒绝；Ruckig 结果不为 Working/Finished 时记录 error 并跳过本周期。六维限速数组从 YAML 读取，缺失或长度不为 6 时回退 `[0.30]×6` rad/s、`[0.80]×6` rad/s² 和 `[4.0]×6` rad/s³。配置加载失败也回退前向位置和代码限速值；不可把 fallback 当作成功加载原配置。

正常使用是在 Home 完成后启动 teleop，步骤见[完整流程](workflow.md)。独立运行的参数写法为：

```bash
ros2 run ur_teleop ruckig_node --ros-args \
  -p config_file:=/ros2_ws/src/ur_teleop/config/alicia_teleop.yaml \
  -p control_hz:=500.0
```

同一输出话题只运行一个发布器。关节阻抗的 `ruckig.enabled: false` 让 teleop 直接发布，此时不要另起 Ruckig。前向位置在 teleop 代码中始终走 Ruckig。`cell.launch.py` 没有 `ruckig` 启动参数。

节点不订阅 `/teleop/e_stop`，上游暂停不会取消既有目标。依赖为 Python `ruckig` 库；离线逻辑测试位于 `tests/test_ruckig_node.py`。
