# Xbot 手柄遥操作与数据采集方案

## 方案概要

在 `ur_teleop` 配置文件中增加 `teleop.control_source: alicia | xbot`，默认保持 `alicia`。两种方式启动时择一运行。Xbot 模式使用现有 `cartesian_impedance_controller`，通过手柄控制 `gripper_tcp` 的六维运动，并采集数据。

## 实现内容

- **启动与回 home：** Xbot 模式不启动 Alicia 或 Ruckig。UR 单独回到配置的 `home.slave` 并验证到位；笛卡尔阻抗控制器先加载为 inactive，手柄使能后再与轨迹控制器严格切换。仿真和实机描述均提供一致的 `gripper_tcp`。
- **手柄输入：** 使用 ROS `joy_node` 和运行前校准生成的按键映射。左摇杆控制 XY 平移、双扳机差值控制 Z、右摇杆控制绕 X/Y 旋转、十字键左右控制绕 Z 旋转；RB 按住使能，LB 按住精细速度，A 切换夹爪。
- **base/TCP 参考坐标系切换：** 按 X 在 `base` 和 `tcp` 两种模式间切换，平移与旋转均随模式改变。`base` 模式沿基座轴运动；`tcp` 模式沿末端当前朝向的局部轴运动。切换时不产生运动或目标跳变，日志显示当前模式。无论选择哪种输入参考系，发给控制器的 `PoseStamped` 均转换到 `base_link`，目标末端始终是 `gripper_tcp`。
- **保护：** 以实际 TCP 位姿建立目标。松开 RB、手柄断连、TCP 反馈失效或 `/teleop/e_stop` 触发时停止积分并保持当前位姿；恢复后须重新按下 RB，并以实测位姿重建目标。默认控制频率 50 Hz，最大平移速度 0.02 m/s、角速度 0.1 rad/s，手柄超时 0.25 s。
- **数据采集：** 可先用手柄预摆位，再按 Menu 开始 episode；Y 保存、B 丢弃、View 长按 2 秒结束。Xbot 数据集的七维 action 为实际采用的 `[vx, vy, vz, wx, wy, wz, gripper]`；TCP 模式的指令先转换到 base 坐标系再记录，使两种模式的数据含义一致。Alicia 原有的关节 action 格式和采集流程保持不变。
- **文档与提交：** 更新 `ur_teleop/docs/` 中的启动、校准、按键、坐标系和采集说明。按独立功能在当前分支分别验证并提交，仅纳入本任务改动。

## 验证

单元测试覆盖两种参考系下的平移与旋转方向、切换瞬间目标连续性、按键边沿、超时保持和 action 坐标转换。mock 集成测试覆盖 UR 单独回 home、控制器切换、夹爪与 episode 操作；实机验收先核对 `gripper_tcp` 标定，再低速逐轴检查 base/TCP 两种模式。

## 假设

Alicia 与 Xbot 由配置文件选择，不同时运行；实际 Joy 轴号需在首次实机使用前校准。当前工作区没有现成的流式笛卡尔速度控制器，因此本方案沿用指定的笛卡尔阻抗控制器。参见 [UR 官方控制器说明](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_robot_driver/ur_robot_driver/doc/usage/position_velocity_control.html)。

## 实施记录（2026-09-26）

- 已复用 `ur10e_robotiq_ft_description`，只增加默认关闭的 effort mock 开关，不维护独立 URDF。
- 已实现 Xbot Home 分支、Joy 校准、RB 使能与严格控制器切换、base/TCP 积分、夹爪和故障恢复保护。
- 已实现手柄 episode 事件、七维基座速度 action 和独立数据集配置，Alicia 流程保留。
- 使用说明：[启动](xbot_startup.md)、[校准与控制](xbot_control.md)、[录制](xbot_recording.md)。
- 真机动作、物理手柄校准、相机视频编码及 LeRobot 实际落盘尚未验收；mock 不能替代实机安全验收。
- 当前阻抗控制器永久锁存目标，没有进程失联超时保护。节点只能在仍运行时处理输入/反馈超时；物理急停和人工监护不可省略。
