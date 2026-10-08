# Alicia 遥操作

先运行 Home，保持硬件终端，再启动遥操作。仿真/真机完整命令见[使用流程](workflow.md#3-回-home)。

```bash
ros2 launch ur_teleop teleop.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/alicia_teleop.yaml mode:=teleop
```

## 开始操作

程序依次等待主从臂反馈、验证 Home、等待静止并捕获会话偏移。出现就绪提示后按 Enter，控制器切换成功后开始跟随主臂。采集模式由首次开始一段录制触发使能，见[数据采集](data_recorder.md)。

| 参数 | 作用 |
| --- | --- |
| `teleop.controller` | forward_position 或 joint_impedance |
| `teleop.command_rate_hz` | 主臂映射与目标更新频率，Hz |
| `teleop.watchdog_timeout_s` | 主臂反馈超时阈值，秒 |
| `teleop.restore_controller_on_exit` | Alicia 退出时是否尝试恢复轨迹控制器 |

映射参数见[Alicia 配置](alicia_teleop_config.md)，平滑参数见[Ruckig](ruckig_node.md)。夹爪启用后独立跟随，不以按 Enter 为开合门控，见[夹爪控制](gripper_controller.md)。

## 停止与退出

主臂反馈超时后改发 UR 当前反馈位置作为保持目标，反馈恢复后自动继续跟随。

`/teleop/e_stop` 软件停止只暂停本节点的关节和夹爪处理，不取消已有夹爪请求，也不清除 Ruckig 的既有目标，不能替代硬件急停。

Ctrl-C 退出时，若启用了恢复开关且已进入控制阶段，会尝试切回轨迹控制器；恢复未成功会输出错误，应查看实际控制器状态。

## 常见问题

| 现象 | 检查 |
| --- | --- |
| 等不到硬件 | Home 终端、ROS 域、主从臂关节反馈和 controller_manager 服务 |
| 一直验证 Home | 实际关节角与 `home.master/slave`、Home 容差 |
| 一直等待静止 | 主从臂是否移动及静止阈值 |
| 回到等待使能 | 控制器加载/切换失败；排查后重新使能 |
| 没有跟随 | 当前控制器、Ruckig 配置与主臂反馈是否一致 |

不要用 `force_home` 跳过验证来掩盖 Home 故障。
