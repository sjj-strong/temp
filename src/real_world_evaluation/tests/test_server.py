"""实际 checkpoint、HTTP 校验与模型工厂验证，绝不连接 ROS。"""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from fastapi.testclient import TestClient

from protocol import ActionChunk, Health, ImageData, Observation, POSE_ACTION
from server import PolicyServer, create_app

ROOT = Path(__file__).resolve().parents[1]


def observation(health, step_id=0):
    return Observation(step_id=step_id, values={name: 0. for name in reversed(health.state_names)},
                       images={key: ImageData.encode(np.zeros((shape[1], shape[2], 3), np.uint8))
                               for key, shape in health.cameras.items()},
                       reference_frame=health.reference_frame, tcp_link=health.tcp_link,
                       wrench_frame=health.wrench_frame)


@pytest.fixture(scope='module')
def runtime():
    import torch
    torch.set_num_threads(2)
    checkpoint = ROOT / 'artifacts/act/checkpoints/000001/pretrained_model'
    if not checkpoint.exists():
        pytest.skip('先运行 tests/smoke_checkpoint.py 生成一步训练样本')
    return PolicyServer(dict(checkpoint_path=str(checkpoint), device='cpu',
                             dataset_root=str(ROOT / 'artifacts/dataset'),
                             reference_frame='base_link', tcp_link='tool0'))


def test_full_chunk_and_reload(runtime):
    request = observation(runtime.health)
    request.values = {name: float(i) / 10 for i, name in enumerate(runtime.health.state_names)}
    actual = runtime.infer(request)
    assert len(actual.actions) == 4  # checkpoint 的 n_action_steps=2，不应截断。
    reordered = request.model_copy(update={'values': dict(reversed(list(request.values.items())))})
    assert np.allclose(actual.actions, runtime.infer(reordered).actions)
    # 用训练配套处理器独立计算，验证返回的是物理量，且恰好反归一化一次。
    torch = runtime.torch
    batch = {'observation.state': torch.tensor(list(request.values.values()), dtype=torch.float32),
             'observation.images.front': torch.zeros(3, 32, 32), 'task': ''}
    with torch.inference_mode():
        raw = runtime.policy.predict_action_chunk(runtime.pre(batch))
        expected = runtime.post(raw)[0].numpy()
    assert np.allclose(actual.actions, expected)
    assert not np.allclose(actual.actions, raw[0].numpy())


def test_http_and_bad_observations(runtime):
    with TestClient(create_app(runtime)) as client:
        assert client.get('/health').json()['policy_type'] == 'act'
        request = observation(runtime.health, 4).model_dump()
        response = client.post('/infer', json=request)
        assert response.status_code == 200 and response.json()['step_id'] == 4
        request['reference_frame'] = '错误坐标系'
        assert client.post('/infer', json=request).status_code == 422
        request['reference_frame'] = 'base_link'
        request['values'] = {}
        assert client.post('/infer', json=request).status_code == 422
        request = observation(runtime.health).model_dump()
        request['images']['observation.images.front']['png'] = 'broken'
        assert client.post('/infer', json=request).status_code == 422


def test_second_policy_automatic_load_and_contract(tmp_path, runtime):
    """Diffusion 可自动加载；其直接 chunk 接口需队列，必须清楚拒绝。"""
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.configs.types import PolicyFeature, FeatureType
    from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
    from lerobot.policies.factory import get_policy_class, make_pre_post_processors
    cfg = DiffusionConfig(device='cpu', push_to_hub=False, n_obs_steps=1, horizon=4,
                          n_action_steps=4, down_dims=(16, 32), n_groups=4,
                          diffusion_step_embed_dim=16, num_inference_steps=1,
                          spatial_softmax_num_keypoints=4, crop_shape=None,
                          input_features=runtime.policy.config.input_features,
                          output_features={'action': PolicyFeature(type=FeatureType.ACTION, shape=(7,))})
    policy = get_policy_class(cfg.type)(cfg)
    path = tmp_path / 'diffusion'
    policy.save_pretrained(path)
    stats = json.loads((ROOT / 'artifacts/dataset/meta/stats.json').read_text())
    pre, post = make_pre_post_processors(cfg, dataset_stats=stats)
    pre.save_pretrained(path)
    post.save_pretrained(path)
    loaded_config = PreTrainedConfig.from_pretrained(path)
    restored = get_policy_class(loaded_config.type).from_pretrained(path, config=loaded_config)
    assert restored.config.type == 'diffusion'
    with pytest.raises(ValueError, match='diffusion 不满足当前完整 chunk 调用契约'):
        PolicyServer(dict(checkpoint_path=str(path), dataset_root=str(ROOT / 'artifacts/dataset'),
                          device='cpu', reference_frame='base_link', tcp_link='tool0'))


def test_server_without_checkpoint_is_explicit(tmp_path):
    with pytest.raises(ValueError, match='checkpoint_path'):
        PolicyServer({'checkpoint_path': str(tmp_path)})
