# Xbot 手柄遥操作

## 模型与启动

复用 `ur10e_robotiq_ft_description/urdf/ur10e_robotiq_ft.urdf.xacro`，不维护第二份 URDF。先按 [启动说明](xbot_startup.md) 启动 Home，再启动遥操作。真机必须先核对 `gripper_tcp` 与工具标定，并逐轴低速验收。

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run joy joy_node --ros-args -p autorepeat_rate:=50.0 -p deadzone:=0.0
# 另一个终端：只读手柄，不连接机器人
ros2 run ur_teleop xbot_calibrate
```

校准完成后停止上述 `joy_node`，避免两个发布者；正式 launch 会启动它。映射基于 ROS `/joy`，不能直接复制 `xbot_control/scripts` 中 pygame 的轴号。工具分别记录扳机松开零位、按下方向，以及十字键的轴/按钮表示。已有映射不会覆盖；用 `--output` 选择新文件，并更新 `xbot.calibration_file`。

```bash
ros2 launch ur_teleop home.launch.py config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml
# Home 到位后，另一个终端
ros2 launch ur_teleop teleop.launch.py config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml mode:=teleop
```

Xbot 不启动 Alicia、Ruckig 或键盘遥操作。配置继承当前 `ur_teleop.yaml` 的 `sim` 与 `home.slave`，运行前必须检查；首次验收只用 `sim: true`。默认 `mode: record`，单独验证运动时显式传 `mode:=teleop`。

## 操作

首次松开所有按键，按 RB 切换控制器；成功后再次松开/按下 RB 才运动。启动时若不在 Home，拒绝切换。不支持 `force_home` 绕过。重启遥操作前应重新执行 Home。

| 输入 | 功能 |
| --- | --- |
| 左摇杆上/左 | +X / +Y 平移 |
| RT / LT | +Z / -Z 平移 |
| 右摇杆上/左 | 绕 +X / +Y 旋转 |
| 十字键左/右 | 绕 +Z / -Z 旋转 |
| RB | 按住运动使能 |
| LB | 按住精细速度（默认 25%） |
| A | RB 有效时切换夹爪；忙时不重复发送 |
| X | base/TCP 参考系切换，切换当帧不积分 |
| Menu / Y / B | 开始 / 保存 / 丢弃 episode |
| View 长按 2 秒 | finalize 并锁定运动，重启节点后才可继续 |

base 模式沿 `base_link` 的轴平移和旋转；TCP 模式沿实测 `gripper_tcp` 当前局部轴运动。两种模式均转换为 `base_link` 中的目标，再发到 `/cartesian_impedance_controller/target_pose`。启动参数覆盖控制器的 base/tip，避免沿用另一任务的 `base → tool0` 配置。

`/teleop/commands` 为 `[vx, vy, vz, wx, wy, wz, gripper]`，前六维是本周期采用的 **base_link** 速度（m/s、rad/s），不是 TCP 位姿；第七维为夹爪已接受目标（0 开、1 闭）。停止时前六维为零。

## 保护边界

50 Hz 控制，默认线速度 0.02 m/s、角速度 0.1 rad/s；按向量模长限速。Joy/TF 超过 0.25 秒、关节状态超时、软件急停、定时器卡顿或目标领先实测超过 3 cm / 0.15 rad 都会停止积分，恢复后须重新按 RB。TF 丢失时只能发送最后有效实测位姿，无法保证仍是当前位姿。夹爪停止请求使用 action cancel，实际停止能力取决于夹爪控制器。

`/teleop/e_stop` (`std_msgs/Bool`) 是软件停止请求，不是安全认证急停。当前笛卡尔控制器没有命令超时回到实测位置的机制：遥操作进程崩溃后控制器仍可能追踪最后目标。物理急停、UR 安全配置及人工监护必须保留；不得将本节点作为安全系统。

`/teleop/xbot_status` 显示当前参考系和运动/保持状态；`/teleop/xbot_ready` 是录制开始的就绪心跳。真实手柄方向、真机运动与停止距离尚须人工验收。

故障保持位姿只在故障开始时锁定，不会随着后续反馈持续漂移。校准采样会继续收集一秒最大行程；请推到极限并保持。重复运动轴或复用功能键的映射被拒绝。夹爪 action 初值取启动时的实测开合状态。

实机入口显式传 `use_cartesian_impedance:=false` 给组合 `real_bringup`，防止其默认行为提前激活阻抗。Xbot 另行加载 inactive 阻抗，先用 active 轨迹控制器完成 Home，再由 RB 切换。
