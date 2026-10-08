# 将 UR 当前关节位置保存为从臂初始位置

脚本启动或复用 UR 控制器，等待 `joint_state_broadcaster` 激活，再从 `/joint_states` 读取完整六关节角，更新以下两个源码配置的 `home.slave`：

- `/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml`
- `/ros2_ws/src/ur_teleop/config/alicia_teleop.yaml`

两份配置按输入设备命名：Alicia 使用 `alicia_teleop.yaml`，Xbot 使用 `xbot_teleop.yaml`。原 `ur_teleop.yaml` 已重命名；自定义启动命令需要同步修改路径。

## 使用

先将机械臂停在希望记录的位置，然后执行：

```bash
/ros2_ws/src/ur_teleop/scripts/capture_slave_home.sh
```

脚本自动加载 ROS Jazzy 和工作区环境，使用系统 Python，无需重新构建本包。默认从两个配置的 `cell.robot_ip`、`cell.ur_type` 读取连接参数，读取超时为 60 秒；此工具读取真机，不使用配置中的 `sim` 值。

```bash
# 显式指定连接参数：
/ros2_ws/src/ur_teleop/scripts/capture_slave_home.sh --robot-ip 169.254.138.15 --ur-type ur10e
# 控制器已启动时，只连接现有控制器：
/ros2_ws/src/ur_teleop/scripts/capture_slave_home.sh --no-start
# 指定其他配置文件：
/ros2_ws/src/ur_teleop/scripts/capture_slave_home.sh --alicia-config /路径/alicia_teleop.yaml --xbot-config /路径/xbot_teleop.yaml
# 自定义状态话题：
/ros2_ws/src/ur_teleop/scripts/capture_slave_home.sh --no-start --topic /joint_states
# 修改等待时长：
/ros2_ws/src/ur_teleop/scripts/capture_slave_home.sh --timeout 90
```

运行环境的 `ROS_DOMAIN_ID` 和其他 ROS 网络设置应与目标机械臂一致；同一域中应只有目标 UR 的 `/controller_manager` 和 `/joint_states`。复用已有控制器时，脚本读取该域内已有状态，不会验证已有驱动的 IP。

## 读取与写入行为

保存顺序为 `shoulder_pan_joint`、`shoulder_lift_joint`、`elbow_joint`、`wrist_1_joint`、`wrist_2_joint`、`wrist_3_joint`，单位为 rad。按消息中的名称排序，不依赖消息数组顺序，忽略夹爪等额外关节。不完整、重复名称或含非有限值的样本不会用于写入。

只替换两个文件的 `home.slave` 单行数组，保留其余字段、主臂位置、注释及格式。两个配置均验证通过后再写入，原文件备份为同目录的 `文件名.<时间戳>.bak`；第二个文件写入失败时恢复已写入的第一个文件。超时或连接失败不会修改配置。

脚本不会调用 `home.launch.py`，不会发送关节、轨迹或位姿指令。新启动驱动时设置 `activate_joint_controller:=false`、`headless_mode:=false` 和 `launch_dashboard_client:=false`，运动控制器保持未激活。复用已有控制器时不切换其状态。

这是一次性读取脚本：完成或失败后停止自己启动的驱动，保留用户已经运行的控制器。保存值会影响后续 Home 的目标，但脚本本身不会执行 Home；工作区真机测试仍仅允许控制 `wrist_3_joint`。

如果后续启动使用安装目录中的配置，需要重新构建 `ur_teleop` 并加载环境，或在启动时显式传入源码配置路径。

## 验证

纯逻辑测试覆盖关节排序与过滤、异常样本、只替换目标字段、双配置备份、预验证失败不写入、第二个文件写入失败回滚，以及驱动启动时禁止激活运动控制器的参数。测试不连接真实机械臂。

验证结果：6 项逻辑测试通过；另在 `ROS_DOMAIN_ID=231` 的模拟控制器服务和模拟 JointState 发布器上完成了端到端验证，确认按关节名排序、两个配置同步替换、注释保留与两个备份生成。真实连接与驱动启动需在用户执行脚本时验证。
