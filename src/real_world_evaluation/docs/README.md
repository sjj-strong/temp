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
