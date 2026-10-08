# 控制流程

两种输入源都先启动 Home，再启动遥操作；相机独立启动。仿真和真机命令见[完整流程](workflow.md)。

| 阶段 | Alicia | Xbot |
| --- | --- | --- |
| Home | 主从双臂回配置位置 | UR 回配置位置 |
| 准备 | 验证 Home、等待静止、捕获偏移 | 检查反馈、自动切换笛卡尔阻抗 |
| 操作 | 按 Enter 或开始采集后跟随主臂 | 就绪后按住 RB 更新目标 |
| 控制目标 | 六关节位置 | tool0 位姿 |
| 平滑 | 根据控制路径使用 Ruckig | 不使用 Ruckig |
| 夹爪 | 根据 Alicia 输入阈值开合 | A 键切换开合 |

## Alicia

主臂关节变化经过[映射与限位](joint_mapper.md)，再发送给所选从臂控制器。`forward_position` 始终经过 Ruckig；`joint_impedance` 可通过 `ruckig.enabled` 选择平滑或直接发布。

控制频率由 `teleop.command_rate_hz` 设置，Ruckig 输出频率由 `ruckig.control_hz` 设置。配置频率不等于实际执行频率。反馈超时和停止行为见[遥操作说明](teleop_node.md)。

## Xbot

手柄输入根据所选 base/TCP 参考系生成位姿增量，转换到控制器参考坐标系，裁剪工作空间后发布。松开 RB 或输入回中时保持末次目标。按键和故障处理见[手柄说明](xbot_control.md)。

## 数据采集

`mode:=record` 同时启动遥操作和录制器，将实际状态、目标动作和所选相机图像写入 LeRobot 数据集。录制字段和就绪条件见[数据采集](data_recorder.md)。结束采集不会自动关闭硬件终端。
