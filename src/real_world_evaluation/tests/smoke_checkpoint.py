"""离线生成模拟数据，ACT 训练一步并保存；不导入任何机器人驱动。"""
from pathlib import Path

import numpy as np
import torch
from lerobot.configs.default import DatasetConfig, WandBConfig
from lerobot.configs.train import TrainPipelineConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.scripts.lerobot_train import train


def main():
    torch.set_num_threads(2)
    root = Path('artifacts').resolve()
    names = ['tcp_ee_' + axis for axis in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')]
    features = {
        'observation.state': {'dtype': 'float32', 'shape': (7,), 'names': names},
        'observation.images.front': {'dtype': 'image', 'shape': (32, 32, 3),
                                     'names': ['height', 'width', 'channels']},
        'action': {'dtype': 'float32', 'shape': (7,),
                   'names': ['dx', 'dy', 'dz', 'drx', 'dry', 'drz', 'cmd_gripper']},
    }
    if not (root / "dataset/meta/info.json").exists():
        dataset = LeRobotDataset.create('local/evaluation', fps=20, features=features,
                                       root=root / 'dataset', use_videos=False)
        rng = np.random.default_rng(42)
        for index in range(16):
            state = np.r_[rng.normal(0, .01, 3), 0., 0., 0., 1.].astype(np.float32)
            action = np.r_[rng.normal(0, .001, 6), index % 2].astype(np.float32)
            dataset.add_frame({'observation.state': state, 'action': action,
                               'observation.images.front': rng.integers(0, 256, (32, 32, 3), dtype=np.uint8),
                               'task': '模拟相对位姿测试'})
        dataset.save_episode()
        dataset.finalize()
    cfg = TrainPipelineConfig(
        dataset=DatasetConfig(repo_id='local/evaluation', root=str(root / 'dataset')),
        policy=ACTConfig(device='cpu', push_to_hub=False, pretrained_backbone_weights=None,
                         chunk_size=4, n_action_steps=2, dim_model=32, n_heads=4,
                         dim_feedforward=64, n_encoder_layers=1, n_decoder_layers=1,
                         n_vae_encoder_layers=1, latent_dim=8),
        output_dir=root / 'act', steps=1, save_freq=1, log_freq=1,
        batch_size=2, num_workers=0, eval_freq=0, wandb=WandBConfig(enable=False),
    )
    train(cfg)
    print('一步训练完成：', root / 'act/checkpoints/000001/pretrained_model')


if __name__ == '__main__':
    main()
