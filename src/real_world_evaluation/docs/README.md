# 真机远程推理使用指南

服务器执行 LeRobot 模型推理，主机采集原始观测并将相对位姿动作发送给现有笛卡尔阻抗控制器。运行顺序：**准备配置 → 启动推理服务与 SSH 隧道 → 启动硬件和相机 → 只读检查 → 执行动作 → 停止**。

> 当前工作区禁止真机笛卡尔运动测试。本文说明完整部署及操作方式；动作执行步骤须在允许该操作的运行环境中使用。只读模式不会发送目标，但硬件启动入口本身会激活阻抗控制器。

## 1. 准备环境与模型

两端均部署本项目，以下约定目录为 `/ros2_ws/src/real_world_evaluation`。所有 Python 依赖通过 `uv` 安装到已有的 `/opt/lerobot_venv`。

**服务器：**安装与训练一致的 LeRobot、对应 policy 依赖，以及支持 RTX 4090 的 PyTorch CUDA 环境。

```bash
cd /ros2_ws/src/real_world_evaluation
uv pip install --python /opt/lerobot_venv/bin/python -r requirements-server.txt
/opt/lerobot_venv/bin/python -c 'import torch; print(torch.cuda.is_available())'
```

最后一条应输出 `True`。将实际训练得到的完整 `pretrained_model/` 目录和训练数据的完整 `meta/` 目录复制到服务器；无需复制训练图像和视频。权重目录必须包含模型配置、权重、前后处理配置及其统计状态文件。

**主机：**已安装并构建 UR 驱动、`cartesian_impedance_controller`、组合机器人描述包、相机驱动，以及需要使用的夹爪驱动。

```bash
cd /ros2_ws/src/real_world_evaluation
uv pip install --python /opt/lerobot_venv/bin/python -r requirements-client.txt
```

所有操作 ROS 的主机终端先执行：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

主机各终端使用同一个 ROS 域。下面的 IP、SSH 用户、串口、相机名称和模型路径均需按现场替换。

## 2. 配置服务器和主机

### 服务器：`configs/server.yaml`

```yaml
checkpoint_path: /data/checkpoints/task/pretrained_model
dataset_root: /data/datasets/task  # 此目录下包含完整 meta/。
device: cuda
reference_frame: base
tcp_link: tool0
wrench_frame: null
host: 127.0.0.1
port: 8000
```

- Policy 类型及维度从 checkpoint 自动识别；状态字段顺序来自训练元信息。
- `reference_frame`、`tcp_link` 必须同时符合训练定义和控制器设置。本文独立硬件入口默认使用 `base/tool0`，不能将 `base_link` 与 `base` 混用。
- 模型使用力观测时，将 `wrench_frame` 改为训练所用、且与力话题 `header.frame_id` 一致的坐标系；不使用力观测时保留 `null`。
- 当前支持六维 rel pose，或附带夹爪的七维动作。单观测、无时序集成且支持直接完整 chunk 推理的策略才能启动；依赖观测历史或内部队列的策略会被拒绝，当前仓库的 Diffusion 属于后者。

### 主机：`configs/client.yaml`

保留现有参数，先修改以下内容：

```yaml
mode: ros
read_only: true
server_url: http://127.0.0.1:8000
instruction: 实际任务指令
cameras:
  observation.images.front: /camera/usb_front/color/image_raw
gripper:
  enabled: true  # 七维动作启用；六维动作可关闭。
  max_effort: 50.0
```

| 参数 | 设置依据 |
| --- | --- |
| `action_hz` | 与训练动作的时间尺度一致；不是底层阻抗控制频率 |
| `execute_steps` | 每次执行 chunk 的前多少步，再重新采集和推理 |
| `max_steps` | 本次最多执行的动作总数 |
| `request_timeout_s` | HTTP 连接、读写等阶段的超时，单位秒 |
| `data_timeout_s` | ROS 消息允许的最大年龄，单位秒 |
| `startup_timeout_s` | 初始化后等待必要观测就绪的时限，单位秒 |
| `controller` | 默认 `/cartesian_impedance_controller` |
| `joint_topic` | 默认 `/joint_states`，仅在模型需要关节观测时使用 |
| `wrench_topic` | 默认 UR 内置力话题 `/force_torque_sensor_broadcaster/ft_data` |

`cameras` 的键必须与模型图像特征名一致，值为本地原始图像话题。实际图像尺寸必须与模型输入一致；`ur_teleop/config/camera.yaml` 的 `resize` 只控制录制保存尺寸，不会改变原始图像话题。训练时若使用了保存缩放，不能直接发送不同尺寸的原始图像；当前服务不会自动猜测缩放方式。

夹爪接口固定为 `/robotiq_gripper_controller/gripper_cmd`，驱动关节固定为 `robotiq_85_left_knuckle_joint`，打开/闭合目标固定为 `0.0/0.4 rad`，不提供配置覆盖。

## 3. 启动推理服务和隧道

**服务器终端：**

```bash
cd /ros2_ws/src/real_world_evaluation
/opt/lerobot_venv/bin/python -m server --config configs/server.yaml
```

等待模型加载完成，出现监听 `127.0.0.1:8000` 的提示，保持终端运行。

**主机隧道终端：**

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 8000:127.0.0.1:8000 用户名@10.5.174.93
```

保持此终端运行。在另一主机终端检查：

```bash
curl --fail http://127.0.0.1:8000/health
```

应返回 `ready: true`。核对 `state_names`、`cameras`、`action_names`、参考坐标系、TCP 和训练 `fps`，再完成主机配置。

## 4. 启动机械臂、夹爪和相机

停止其他 UR 驱动实例、遥操作节点、键盘控制工具及目标发布程序，避免重复启动和同时控制。示教器中的 External Control 程序应配置为连接本地主机，按现场流程完成机器人上电和驱动连接。

**主机硬件终端：**以下使用 UR 内置力反馈，不连接 FT300。

```bash
ros2 launch cartesian_impedance_controller standalone_real.launch.py \
  robot_ip:=169.254.138.15 \
  use_gripper:=true gripper_com_port:=/dev/ttyUSB0 \
  use_ft300:=false launch_rviz:=false
```

不使用夹爪时改为 `use_gripper:=false`。按照驱动提示在示教器运行 External Control，保持硬件终端运行。

此入口复用 `ur10e_robotiq_ft_description/real_bringup.launch.py`，启动 UR 驱动和可选夹爪，并激活笛卡尔阻抗控制器；初始目标为当前实测位姿，不执行 Home。若训练使用 FT300，需启用真实传感器并单独确认对应 broadcaster 已发布力话题，再修改 `wrench_topic/wrench_frame`；仅打开 `use_ft300` 不代表力话题已就绪。

**主机相机终端：**先在 `ur_teleop/config/camera.yaml` 中配置实际设备、分辨率和帧率，只启用需要的相机，然后运行：

```bash
ros2 launch ur_teleop camera.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/camera.yaml
```

此入口复用 USB 的 `data_collection/opencv_camera_node` 或 RealSense 的 `realsense2_camera_node`。保持相机终端运行；不要另开占用同一设备的采集程序。

## 5. 检查硬件与只读观测

**主机检查终端：**

```bash
ros2 control list_controllers -c /controller_manager
ros2 topic echo /cartesian_impedance_controller/current_pose --once
ros2 topic hz /camera/usb_front/color/image_raw
```

确认 `cartesian_impedance_controller` 为 `active`，位姿坐标系符合配置，相机持续发布；`topic hz` 检查后按 Ctrl-C 退出。启用夹爪时再检查：

```bash
ros2 control list_controllers -c /robotiq_controller_manager
ros2 action list -t
```

应有 active 的夹爪控制器及 `/robotiq_gripper_controller/gripper_cmd`。模型需要力观测时执行：

```bash
ros2 topic echo /force_torque_sensor_broadcaster/ft_data --once
```

核对力消息坐标系，不满足训练定义时先处理配置或传感器问题。

保持 `mode: ros`、`read_only: true`，执行：

```bash
cd /ros2_ws/src/real_world_evaluation
/opt/lerobot_venv/bin/python -m rollout --config configs/client.yaml
```

显示“只读观测成功”后正常退出，表示所需观测已就绪。此模式会检查 `/health`，但不会调用 `/infer`、发布位姿或操作夹爪。

## 6. 执行推理与停止

在允许真机动作执行的运行环境中，将 `client.yaml` 的 `read_only` 改为 `false`，确认任务指令、动作频率、执行步数和夹爪开关后，重新运行上述 rollout 命令。

程序每轮采集观测，由服务器完成预处理、推理和反归一化，返回完整 chunk。主机执行前 `execute_steps` 步，其余丢弃；每一步按最新实测 TCP 合成绝对目标，再重新采集。同步推理等待会降低整体平均动作频率。

达到 `max_steps` 自动退出；按 Ctrl-C 可提前结束。退出时尝试以新鲜实测位姿保持并取消夹爪请求。确认机器人处于可控状态后，按现场停机流程停止硬件，再关闭相机、SSH 隧道和服务器。

**退出 rollout 或停止发布目标不等于硬件急停。** 控制器可能继续保持上一目标；反馈失效时保持请求也可能失败。需要立即停止机器人时使用现场硬件停止措施。

## 常见问题

| 现象 | 检查项 |
| --- | --- |
| `/health` 无法访问 | 服务是否已完成加载、SSH 隧道是否成功、本地 8000 端口是否被占用 |
| 模型启动失败 | checkpoint 与处理器是否完整、LeRobot 版本和依赖是否匹配、CUDA 是否可用、策略是否满足当前接口要求 |
| 相机键或尺寸不符 | 比较 `/health`、`client.yaml` 与实际原始图像；录制 resize 不会缩放发布话题 |
| 状态缺失或数据过期 | 控制器是否 active、消息是否持续发布、话题和消息时间戳是否正确 |
| 基座、TCP 或力坐标系不符 | 核对训练定义、服务器配置与实际控制器/传感器，勿仅改字符串绕过检查 |
| 七维动作无法初始化 | 同时启用硬件夹爪和 `gripper.enabled`，确认固定 Action 可用 |
| 运行超时或推理失败 | 程序会停止后续动作且不重试；排除故障并确认机器人状态后重新启动 |
