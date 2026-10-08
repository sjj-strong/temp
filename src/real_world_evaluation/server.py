"""从 LeRobot checkpoint 自动加载策略并提供同步 HTTP 推理。"""
import argparse
import json
from pathlib import Path
from threading import Lock

import numpy as np
import yaml
from fastapi import FastAPI, HTTPException

from protocol import ActionChunk, Health, ImageData, Observation


class PolicyServer:
    """复用 LeRobot 工厂和处理器；启动探测直接 chunk 接口契约。"""

    def __init__(self, config):
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.datasets.dataset_metadata import LeRobotDatasetMetadata
        from lerobot.policies.factory import make_policy, make_pre_post_processors

        self.torch = torch
        path = Path(config['checkpoint_path']).expanduser().resolve()
        if not (path / 'config.json').is_file():
            raise ValueError('checkpoint_path 必须指向包含 config.json 的 pretrained_model 目录')
        train_path = path / 'train_config.json'
        training = json.loads(train_path.read_text()) if train_path.exists() else {}
        dataset = training.get('dataset', {})
        root = config.get('dataset_root') or dataset.get('root')
        if not root or not (Path(root) / 'meta/info.json').is_file():
            raise ValueError('需要本地训练数据元信息；请设置 dataset_root（只需 meta 目录）')
        meta = LeRobotDatasetMetadata(dataset.get('repo_id', 'local/evaluation'), root=root)
        cfg = PreTrainedConfig.from_pretrained(path)
        cfg.pretrained_path = path
        cfg.device = config.get('device', 'cuda')
        if cfg.device.startswith('cuda') and not torch.cuda.is_available():
            raise ValueError('指定 CUDA 但当前环境不可用；模拟验证请配置 cpu')
        # 不能把隔若干执行步采集的一帧冒充连续历史，亦不能绕过训练的时序集成。
        if cfg.n_obs_steps != 1 or getattr(cfg, 'temporal_ensemble_coeff', None) is not None:
            raise ValueError('当前单快照协议不支持多帧观测历史或时序集成')
        original_outputs = dict(cfg.output_features)
        original_inputs = dict(cfg.input_features)
        self.policy = make_policy(cfg=cfg, ds_meta=meta).eval()
        if cfg.output_features != original_outputs:
            raise ValueError('训练元信息的动作维度与 checkpoint 不一致')
        self.pre, self.post = make_pre_post_processors(
            cfg, pretrained_path=str(path),
            preprocessor_overrides={'device_processor': {'device': cfg.device}},
        )
        unsupported = set(original_inputs) - {'observation.state'} - set(cfg.image_features)
        if unsupported:
            raise ValueError(f'原始观测协议尚未提供这些特征：{unsupported}')
        names = meta.features.get('observation.state', {}).get('names', [])
        state_shape = original_inputs.get('observation.state')
        if state_shape is None:
            names = []
        elif not names or len(names) != state_shape.shape[0] or len(set(names)) != len(names):
            raise ValueError('训练元信息缺少唯一、完整的 state 字段名')
        self.health = Health(
            policy_type=cfg.type, state_names=names,
            cameras={key: list(ft.shape) for key, ft in cfg.image_features.items()},
            action_names=meta.features['action'].get('names', []),
            reference_frame=config['reference_frame'], tcp_link=config['tcp_link'],
            wrench_frame=config.get('wrench_frame'), fps=meta.fps,
        )
        if any(name.startswith(('force.', 'torque.')) for name in names) and not self.health.wrench_frame:
            raise ValueError('使用力观测时必须声明训练时的 wrench_frame')
        if any(len(shape) != 3 or shape[0] != 3 for shape in self.health.cameras.values()):
            raise ValueError('只支持三通道 CHW 图像特征')
        self.policy.reset()
        probe = Observation(
            step_id=0, values={name: 0. for name in names},
            images={key: ImageData.encode(np.zeros((shape[1], shape[2], 3), np.uint8))
                    for key, shape in self.health.cameras.items()},
            reference_frame=self.health.reference_frame, tcp_link=self.health.tcp_link,
            wrench_frame=self.health.wrench_frame, instruction='接口探测',
        )
        try:
            self.infer(probe)
        except Exception as exc:
            raise ValueError(f'{cfg.type} 不满足当前完整 chunk 调用契约：{exc}') from exc
        finally:
            self.policy.reset()
            self.pre.reset()
            self.post.reset()

    def infer(self, observation):
        """返回完整物理量 chunk，不消费 select_action 的动作队列。"""
        torch = self.torch
        health = self.health
        for name in ('reference_frame', 'tcp_link', 'wrench_frame'):
            if getattr(observation, name) != getattr(health, name):
                raise ValueError(f'{name} 与训练约定不符')
        batch = {'task': observation.instruction}
        if health.state_names:
            try:
                batch['observation.state'] = torch.tensor(
                    [observation.values[name] for name in health.state_names], dtype=torch.float32)
            except KeyError as exc:
                raise ValueError(f'缺少原始观测字段：{exc}') from exc
        if set(observation.images) != set(health.cameras):
            raise ValueError('相机键与模型要求不符')
        for key, shape in health.cameras.items():
            pixels = observation.images[key].decode()
            if list(pixels.shape) != [shape[1], shape[2], 3]:
                raise ValueError(f'{key} 分辨率与训练输入不符，不猜测训练时的缩放方式')
            batch[key] = torch.from_numpy(pixels).permute(2, 0, 1).float() / 255.
        with torch.inference_mode():
            inputs = self.pre(batch)
            actions = self.policy.predict_action_chunk(inputs)
            if actions.ndim != 3 or actions.shape[0] != 1:
                raise ValueError('策略必须返回 [1,T,D]')
            actions = self.post(actions)
        if actions.ndim != 3 or actions.shape[0] != 1 or actions.shape[2] != len(health.action_names):
            raise ValueError('后处理输出维度不符')
        return ActionChunk(step_id=observation.step_id, actions=actions[0].detach().cpu().tolist())


def create_app(runtime):
    """一次只处理一个推理请求，不共享并发中的处理器状态。"""
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
    lock = Lock()

    @app.get('/health', response_model=Health)
    def health():
        return runtime.health

    @app.post('/infer', response_model=ActionChunk)
    def infer(observation: Observation):
        if not lock.acquire(blocking=False):
            raise HTTPException(409, '推理服务正忙；只允许单个 rollout 客户端')
        try:
            # 每个请求独立，超时请求不会留下影响下一轮的缓存。
            runtime.policy.reset()
            runtime.pre.reset()
            runtime.post.reset()
            return runtime.infer(observation)
        except ValueError as exc:
            raise HTTPException(422, str(exc)) from exc
        except Exception as exc:
            raise HTTPException(500, '模型推理失败') from exc
        finally:
            lock.release()

    return app


def main():
    import uvicorn
    parser = argparse.ArgumentParser(description='LeRobot 远程推理服务器')
    parser.add_argument('--config', default='configs/server.yaml')
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.config).read_text())
    runtime = PolicyServer(config)
    uvicorn.run(create_app(runtime), host=config.get('host', '127.0.0.1'), port=config.get('port', 8000))


if __name__ == '__main__':
    main()
