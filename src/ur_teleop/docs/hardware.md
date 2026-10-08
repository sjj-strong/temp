# 硬件与环境准备

本包使用 UR 从臂，输入源二选一：Alicia-D 主臂或 Xbot 手柄。相机、Robotiq 夹爪和 FT300 按配置启用。

## 环境与构建

依赖需先安装或构建：UR 驱动/描述包、`joy`、`joint_impedance_controller`、`cartesian_impedance_controller`、组合描述包 `ur10e_robotiq_ft_description`，以及 Alicia 分支使用的 `alicia_d_driver`。启用相机发布还需 `data_collection`；启用 RealSense 需 `realsense2_camera`。夹爪和 FT300 开启时需相应驱动包。

以下命令构建本包，前提是这些依赖已安装到当前工作区：

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --packages-select ur_teleop --symlink-install
source /ros2_ws/install/setup.bash
```

每个 ROS 终端加载前述两个 setup，并使用相同 `ROS_DOMAIN_ID`。同一域只运行一套目标 UR 控制栈。完整 Home 会自行启动驱动，因此不要同时运行另一个独立 UR 驱动。

```bash
ros2 pkg prefix ur_robot_driver
ros2 pkg prefix joint_impedance_controller
ros2 pkg prefix cartesian_impedance_controller
# 使用相机发布时确认此依赖可找到：
ros2 pkg prefix data_collection
```

`src/lerobot`、`src/robot_utils` 当前可被 `COLCON_IGNORE` 排除；存在源码不等于 ROS 包已安装。`data_collection` 位于 robot_utils 中，相机 launch 需要其安装产物。`ros2 pkg prefix data_collection` 失败时应先安装该包，不能直接跳到相机发布。LeRobot 是 Python 依赖，不靠 colcon 构建；采集器可自动转入 `/opt/lerobot_venv`，该环境需已有 LeRobot、tqdm 及所需图像/视频依赖。

## UR 与初始位置

在选用的配置中检查 `cell.ur_type`、`cell.robot_ip` 和 `home.slave`；UR 与主机网络应可互通。

```bash
# 地址必须与所选配置一致；此处为当前配置地址。
ping -c 3 169.254.138.15
# 只读取当前关节角，更新两个配置的 home.slave：
/ros2_ws/src/ur_teleop/scripts/capture_slave_home.sh
```

读取脚本会启动或复用控制器，不发送运动指令，并备份配置，详见[读取从臂初始位置](capture_slave_home.md)。Alicia 的 `home.master` 需独立设置，不由该脚本更新。

真机 Home 会提示在示教器启动 External Control 并等回车。工作区真机测试仅允许控制 `wrist_3_joint`；完整六轴 Home/遥操作测试使用 mock，不将下文 mock 示例改为真机测试命令。

## Alicia

连接 Alicia USB 串口，在 `ur_teleop.yaml` 中核对 `cell.alicia_port`、`cell.launch_alicia`、`home.master` 和关节映射。UR mock 不会把 Alicia 变成模拟设备；Alicia 分支默认仍需真实主臂。

```bash
ls -l /dev/serial/by-id/
```

原始驱动说明见 [Alicia-D-ROS2](../../Alicia-D-ROS2/README.md)，映射规则见[关节映射](joint_mapper.md)。

## Xbot

连接手柄，核对 `xbot.device_id` 和 `xbot.calibration_file`。先独立测试，不启动机械臂：

```bash
source /opt/ros/jazzy/setup.bash
python3 /ros2_ws/src/ur_teleop/ur_teleop/xbot_joy_test.py
```

映射不正确时，按[Xbot 标定步骤](xbot_control.md#配置与校准)运行 `joy_node` 和 `xbot_calibrate`。独立测试采用宽松布局解析；正式遥操作采用严格布局校验，因此采集前必须确认标定文件与实际手柄一致。

## 相机、夹爪与 FT300

相机需能从运行环境访问视频设备；容器还需映射相应设备和稳定路径。型号、端口与实时调参见[相机检查](camera_inspector.md)。

`gripper.enabled` 控制夹爪执行，`cell.ft300_enabled` 控制 FT300 真实设备接入。未接设备时关闭对应开关。需要接入时核对 `cell.gripper_port` 与 `cell.ftdi_id`；Xbot 的 FT300 接入使用组合 ros2_control 硬件接口，不再启动争用同一串口的独立驱动。详见[夹爪控制](gripper_controller.md)、[组合硬件说明](../../ur10e_robotiq_ft_description/docs/real_bringup.md)及[控制器说明](controllers.md)。
