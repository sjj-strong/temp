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
