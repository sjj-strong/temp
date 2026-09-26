# Xbot 数据采集

完成 [手柄校准及运动验证](xbot_control.md) 后，使用 `teleop.launch.py config_file:=.../xbot_teleop.yaml mode:=record`。相机仍通过原有 camera launch 单独启动。

手柄运动使能与 episode 独立：可以先按 RB 预摆位，不会自动录制。Menu 开始，Y 保存并停止本 episode，B 丢弃并停止，View 长按 2 秒保存当前有效 episode 并 finalize，随后遥操作锁定，需重新启动。录制不读取键盘，也不发送 Alicia `/teleop/enable`。

开始前必须具备新鲜的就绪心跳、关节状态、动作、TCP 位姿及所有配置相机。录制中数据失效的周期跳过，不重复记录旧动作/图像；恢复后继续当前 episode，因此时间戳按数据集帧率排列，不代表故障期间真实经过的墙钟时间。需要连续时间数据时应丢弃受到断连影响的 episode。帧数不足 `min_frames_per_episode` 时保存会自动丢弃。

Xbot 使用独立的 `repo_id: my_user/ur10e_xbot` 和 `root: /ros2_ws/dataset/xbot`；已存在时创建带时间戳的新目录，不向原有 Alicia 数据集追加。

- observation 保持原有配置：UR 六关节、TCP 位姿、夹爪状态和相机。
- action 固定七维：`vx, vy, vz, wx, wy, wz, cmd_gripper`。
- 前六维是在 `base_link` 中实际用于目标积分的速度（m/s、rad/s），不是测得速度，也不是关节位置；TCP 模式先旋转到 base 再记录。
- 夹爪为最后接受的二值目标，0 开、1 闭；实际夹爪状态在 observation 中。
- Alicia 原有关节 action 名称和键盘流程保持不变。

`/teleop/record_event` 是 `std_msgs/String`：`start/save/discard/finalize`。回调只入队，保存和编码在录制主线程执行；过期两秒的非 finalize 操作忽略，避免耗时保存结束后执行旧的开始命令。

自动测试使用内存数据集验证开始、保存、丢弃、finalize 和超时门控；真实相机、视频编码及 LeRobot 落盘需要在目标采集环境验收。
