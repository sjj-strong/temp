# 真实环境远程评估

使用 HTTP 传输原始观测和完整物理量 Action Chunk。服务器运行 LeRobot，本地主机连接现有 ROS 2 相机和阻抗控制器。真机笛卡尔运动测试禁止运行；运动验证使用 Mock 或仿真。

## Python 环境

所有 Python 依赖统一由 `uv` 管理在 `/opt/lerobot_venv`，不创建项目或临时虚拟环境：

```bash
uv pip install --python /opt/lerobot_venv/bin/python -r requirements-server.txt
/opt/lerobot_venv/bin/python -m pytest tests -q
```

主机仅安装 `requirements-client.txt` 即可；服务器额外使用与训练一致的 LeRobot 及 policy 依赖。ROS Python 包来自系统 ROS 安装，通过 source 环境使用。

## 通信协议

`Observation` 包含 `step_id`、具名原始数值 `values`、相机键到 PNG/Base64 的 `images`、`reference_frame`、`tcp_link`、可选 `wrench_frame` 和 `instruction`。图像保持采集分辨率及像素，声明 `rgb8` 或 `bgr8`。

`ActionChunk` 为 `step_id` 和非空 `[T,D]` 的 `actions`，仅接受六维 rel pose 或附带夹爪的七维动作。NaN/Inf、非矩形及其他维度拒绝执行。

本机 ROS 的 `launch_testing` 插件与共享环境 pytest 不兼容，本项目测试命令使用 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /opt/lerobot_venv/bin/python -m pytest tests -q`，不修改共享插件。协议测试当前 8 项通过。

## 服务端与权重验证

进入 `/ros2_ws/src/real_world_evaluation` 后，首次离线生成测试权重：

```bash
HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 /opt/lerobot_venv/bin/python tests/smoke_checkpoint.py
/opt/lerobot_venv/bin/python -m server --config configs/server.yaml
```

脚本创建 16 帧 RGB 图像、具名 TCP 状态、七维相对动作，训练 ACT 一步；禁用预训练视觉权重下载、Hub 上传及 WandB。产物写在被 Git 忽略的 `artifacts/`，已有训练输出不覆盖。数据已存在时复用；需要再次训练请先自行更换或备份输出目录。一步训练只验证链路，不具备控制质量。

实际生成的目录：

```text
artifacts/act/checkpoints/000001/
├── pretrained_model/
│   ├── config.json
│   ├── model.safetensors
│   ├── train_config.json
│   ├── policy_preprocessor.json
│   ├── policy_postprocessor.json
│   ├── policy_preprocessor_step_3_normalizer_processor.safetensors
│   └── policy_postprocessor_step_0_unnormalizer_processor.safetensors
└── training_state/
    ├── optimizer_param_groups.json
    ├── optimizer_state.safetensors
    ├── rng_state.safetensors
    └── training_step.json
```

服务器从 `config.json` 自动识别 policy，再调用 LeRobot 工厂及 checkpoint 处理器，不硬编码 ACT。`checkpoint_path` 指向 `pretrained_model`；`dataset_root` 指向训练数据目录，只需完整 `meta/`，不传输视频/帧文件。省略 `dataset_root` 时读取 `train_config.json` 中记录的路径。路径相对于启动工作目录；迁移服务器时需修改路径。

输入维度来自 checkpoint，状态排列和动作字段来自训练元信息。元信息不能只用维度替代。当前协议提供 `observation.state` 和具名 RGB 图像；图像必须与训练原始输入尺寸一致，服务器完成 RGB/CHW/float 转换，训练处理器与 policy 完成其余处理，不自行猜测缩放方式。

`GET /health` 返回就绪状态、policy 类型、字段、相机 CHW 尺寸、坐标系及训练 FPS。`POST /infer` 返回完整、恰好反归一化一次的 chunk。六维动作单位为米、弧度；第七维是夹爪开合值。服务器串行接受推理，繁忙返回 409，非法观测返回 422，模型错误返回 500。客户端不重试。

### Policy 兼容边界

自动识别类型不等于所有模型均支持同一种观测调用：本版仅接受单观测快照、无 temporal ensemble，且 `predict_action_chunk` 可直接接受已预处理 batch 的策略。启动时以模拟观测探测并复位策略和处理器，探测失败则不监听端口。多帧历史和依赖私有观测队列的策略不猜测、不伪造历史、不用重复 `select_action` 拼装动作。

已验证 ACT 完整 4 步输出，即使 checkpoint 的 `n_action_steps=2` 也不截断。第二策略 Diffusion 的配置识别和权重重载通过；本仓库其直接 chunk 接口读取内部观测队列，因此启动探测明确拒绝，不宣称支持 Diffusion 实际 rollout。

验证环境：LeRobot 源码提交 `1396b9f`、PyTorch `2.10.0+cpu`。服务器与模型测试 4 项通过，协议测试 8 项通过。测试全程使用 `/opt/lerobot_venv`；CPU 验证不代表 RTX 4090 性能测试。

## 主机与 UR 环境

另一个终端进入项目目录后运行：

```bash
/opt/lerobot_venv/bin/python -m rollout --config configs/client.yaml
```

默认 `mode: mock`，不导入 ROS，不访问机器人；使用上面生成的 ACT 样本，可完成 10 步、每次执行 2 步的 HTTP 闭环。`max_steps` 是总执行动作数，`execute_steps` 是每次执行的前缀长度。每个动作至少留出 `1/action_hz` 的时间；调用过慢时不追赶补发，最后一步也等待一个周期再采集。同步 HTTP 延迟会降低平均动作频率。

`ur_env.py` 将 ROS 消息、资源管理和执行作为同一个环境职责，提供 `observe/step/hold/close`。约 250 行，已检查职责，没有为减少行数额外引入 manager 或抽象基类。

### ROS 接口

先按现有工作流启动控制器和相机，source ROS 环境，再使用同一 `/opt/lerobot_venv/bin/python`：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
/opt/lerobot_venv/bin/python -m rollout --config configs/client.yaml
```

将配置改为 `mode: ros`，保留 `read_only: true` 时只获取一次观测并退出，不发布位姿或夹爪目标。相机映射的键必须与 `/health` 完全一致，值是原始图像话题。数值字段使用训练元信息的名称：`joint_position.<关节名>`、`joint_velocity.<关节名>`、`joint_effort.<关节名>`、`tcp_ee_x...tcp_ee_qw`、`force.x...torque.z`。不存在或过期的必要字段导致失败，不填零。

启动读取控制器 `base_frame`、`tip_frame`、`tf_prefix` 参数，与训练配置比较；每条位姿还检查 `header.frame_id`。力数据保持消息原始坐标系，检查 `wrench_frame`。接收时间和消息时间戳都必须新鲜，主机及 ROS 时钟必须一致。当前支持 `rgb8/bgr8` 原始图像，正确忽略行尾填充字节。

仅在仿真运动验证时设置 `read_only: false`。七维动作还需 `gripper.enabled: true` 和可用夹爪 action 服务；六维动作不操作夹爪。夹爪按 0.5 阈值开合，同状态不重复发送；开合位置与力度沿用配置。ROS executor 持续更新消息，HTTP 和动作前缀执行仍为同步流程。

末端目标按 `p_target=p_current+Δp` 和 `q_target=rotvec(Δr)×q_current` 计算，每一步使用最新实测 pose。旋转向量位于参考坐标系，不能逐项相加欧拉角，也不能累计上一目标。客户端不会 Home、切换控制器或启动硬件。

异常或正常结束都尝试保持并关闭资源。保持只依赖新鲜 TCP，相机失效不会阻止保持；TCP 过期时停止发布并报告保持失败。现有阻抗控制器没有命令超时自动停止功能，仍可能保持上一目标；这不等于硬件急停。并发操纵同一控制器的遥操作程序必须先停止。

## 远程部署

服务器准备与训练一致的 LeRobot 环境和本项目文件，全部 Python 依赖使用 `/opt/lerobot_venv`，通过 `uv pip install --python /opt/lerobot_venv/bin/python ...` 管理。复制 `pretrained_model` 及训练数据的 `meta`，配置服务器上的路径和 `device: cuda`。服务器仍只监听 `127.0.0.1:8000`。

主机手工建立隧道（替换 SSH 用户名）：

```bash
ssh -N -o ExitOnForwardFailure=yes -L 8000:127.0.0.1:8000 用户名@10.5.174.93
```

主机 `server_url` 保持 `http://127.0.0.1:8000`。隧道断开或请求超时即终止，不重试。服务器不会调用任何机器人接口。SSH 隧道和现有相机、控制器进程之外，新增业务进程只有 `server` 和 `rollout` 两个。

## 测试分层

```bash
# 无 ROS、无实际权重：协议与 Mock 执行。
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /opt/lerobot_venv/bin/python -m pytest tests/test_protocol.py tests/test_rollout.py -q
# 权重已生成：实际模型、反归一化和 HTTP。
HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /opt/lerobot_venv/bin/python -m pytest tests/test_server.py -q
# ROS 消息模拟器，DDS 域固定 231、话题固定 /rwe_test，不加载硬件。
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /opt/lerobot_venv/bin/python -m pytest tests/test_ur_env.py -q
```

ROS 测试检查图像颜色和步长、原始数值读取、逐步使用反馈、夹爪去重、相机故障时保持、过期反馈拒绝、只读模式与 TCP 不匹配拒绝。它验证消息接口，不替代真实阻抗控制动力学验证。本次未对真实机械臂下发任何命令。

## 本次验证记录（2026-10-09）

- 协议与 Mock rollout：21 项通过。
- 实际 checkpoint、反归一化、HTTP 与第二策略加载：4 项通过。
- 隔离 ROS 模拟接口：2 项通过。
- 两个真实 Python 入口分别启动，`GET /health` 和 5 次 `POST /infer` 均返回 200；Mock 共执行 10 步后正常退出，验证用服务器随后关闭。
- 没有访问 `10.5.174.93`，没有进行 RTX 4090 性能测试、完整阻抗动力学仿真或真机运动。远程主机 SSH 用户、实际模型路径和训练坐标系仍需部署时填写。

上述 Python 运行与依赖安装均使用 `/opt/lerobot_venv`。最初尝试创建的临时测试环境已停用，不作为本项目运行环境。
