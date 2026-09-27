# Xbot 手柄遥操作

使用 `cartesian_impedance_controller` 控制 `tool0`（TCP），复用 `ur10e_robotiq_ft_description` 的组合 URDF。Xbot 不启动 Alicia 或 Ruckig。

## 配置与校准

配置入口为 `config/xbot_teleop.yaml`，该文件是完整的 XBot 独立配置，**不继承** `ur_teleop.yaml`。
其中 `home.slave` 是 UR 的六关节 Home 位姿，按 `shoulder_pan`、`shoulder_lift`、`elbow`、`wrist_1`、
`wrist_2`、`wrist_3` 顺序填写，单位为 rad。XBot 模式不需要 Alicia 的 `home.master`、关节映射或 Alicia
串口字段。首次验证设为 `sim: true`；真机前核对 TCP 标定。Xbot 的仿真/真机选择以配置文件为准，不使用 Home 的 `sim:=` 参数覆盖。

在两个终端分别加载 ROS 环境后运行：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

# 终端 1：仅发布手柄输入
ros2 run joy joy_node --ros-args -p autorepeat_rate:=50.0 -p deadzone:=0.0

# 终端 2：按提示校准，不发送机器人指令
ros2 run ur_teleop xbot_calibrate
```

每次操作推到极限并保持，步骤间松开按键、摇杆回中。结果默认写入 `/ros2_ws/src/ur_teleop/config/xbot_joy.yaml`，遥操作从同一路径读取；已有文件不覆盖，另存时用 `--output` 并更新 `xbot.calibration_file`。映射使用 ROS Joy 编号，不能直接复制 pygame 轴号。

校准完成后停止手动启动的 `joy_node`，正式启动会自动运行它。

## 启动

```bash
# 终端 1：UR 回 Home，到位后保持此终端运行
ros2 launch ur_teleop home.launch.py config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml

# 终端 2：仅遥操作；录制时改为 mode:=record
ros2 launch ur_teleop teleop.launch.py config_file:=/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml mode:=teleop
```

Home 阶段使用轨迹控制器，笛卡尔阻抗控制器保持 inactive。首次松开所有按键，按 RB 切换控制器；成功后再次松开并按下 RB 才运动。不在 Home 时拒绝切换，不支持 `force_home` 绕过；重启遥操作前重新执行 Home。

仿真使用组合模型的 `xbot_effort_mock` 开关。实机复用 `real_bringup.launch.py`，显式关闭其自动激活阻抗功能，由 Xbot 负责后续切换；FT300 不另开串口驱动。

## 按键与参考系

| 输入 | 功能 |
| --- | --- |
| 左摇杆上 / 左 | +X / +Y 平移 |
| RT / LT | +Z / -Z 平移 |
| 右摇杆上 / 左 | 绕 +X / +Y 旋转 |
| 十字键左 / 右 | 绕 +Z / -Z 旋转 |
| RB | 按住运动使能 |
| LB | 按住精细速度，默认 25% |
| A | RB 有效时切换夹爪，忙时忽略 |
| X | 切换 base/TCP 参考系 |

base 模式沿 `base_link` 的轴运动；TCP 模式沿实测 `tool0` 当前局部轴运动。平移和旋转均遵循所选参考系，切换当帧目标不变。

每周期以实测 `tool0` 位姿叠加手柄增量生成目标，不累加上一周期目标。平移增量为速度乘周期时长，旋转增量用四元数合成；RB 按住且输入为零时，目标等于本周期实测位姿。

目标统一转换为 `base_link` 下的 `PoseStamped`，发布到 `/cartesian_impedance_controller/target_pose`。实测 TF、控制器 base/tip 和录制末端均使用 `base_link → tool0`。

录制按键、数据格式及数据集位置见[数据采集](data_recorder.md)。

## 保护与限制

- 默认 50 Hz，线速度上限 0.02 m/s、角速度上限 0.1 rad/s，按向量模长限速。
- 松开 RB、Joy/TF/关节反馈超时、软件急停、定时器卡顿或目标领先实测超过 3 cm / 0.15 rad 时停止积分。Joy/反馈默认超时 0.25 秒，恢复后须重新按 RB。
- 故障保持位姿在故障开始时锁定；TF 丢失只能使用最后有效实测位姿。夹爪停止使用 action cancel，实际效果取决于夹爪控制器。
- `/teleop/e_stop` 是软件停止请求，不是安全认证急停。遥操作进程崩溃后，阻抗控制器仍可能追踪最后目标；必须保留物理急停和人工监护。
- `/teleop/xbot_status` 显示参考系及运动状态，`/teleop/xbot_ready` 提供录制就绪心跳。

## 测试

仅在 `sim: true` 的隔离域中运行。两个终端均设置 `ROS_DOMAIN_ID=225` 和相同的 `ROS_HOME`：

```bash
export ROS_DOMAIN_ID=225
export ROS_HOME=/tmp/ur_xbot_validation
```

终端 1 按上文启动 Home；到位后，终端 2 执行：

```bash
UR_XBOT_MOCK_TEST=1 PYTHONPATH=/ros2_ws/src/ur_teleop:$PYTHONPATH \
  /usr/bin/python3 -m pytest /ros2_ws/src/ur_teleop/tests -q
```

测试会切换控制器和夹爪，重复执行前恢复轨迹 active、阻抗 inactive、夹爪打开，或重新启动 mock Home。

mock 已覆盖控制器切换、参考系、夹爪、断连恢复和录制事件。曾出现夹爪时序测试间歇性失败，恢复初态后三次复测未重现。物理手柄、真机动作、相机视频编码及 LeRobot 实际落盘尚未验收。
