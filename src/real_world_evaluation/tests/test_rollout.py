"""Mock HTTP 和执行计数、退出行为；不运行 ROS。"""
import httpx
import numpy as np
import pytest
from protocol import Health, POSE_ACTION
from rollout import run
from ur_env import MockEnv, compose_pose
from scipy.spatial.transform import Rotation


def setup_run(handler=None):
    health = Health(policy_type='mock', state_names=[], cameras={}, action_names=POSE_ACTION,
                    reference_frame='base_link', tcp_link='tool0', fps=20)
    requests, environments, waits = [], [], []
    def respond(request):
        if request.url.path == '/health':
            return httpx.Response(200, json=health.model_dump())
        import json
        obs = json.loads(request.content)
        requests.append(obs)
        if handler:
            return handler(obs)
        return httpx.Response(200, json={'step_id': obs['step_id'],
            'actions': [[.001, 0, 0, 0, 0, 0], [.002, 0, 0, 0, 0, 0], [99, 0, 0, 0, 0, 0]]})
    def factory(config, h):
        env = MockEnv(config, h)
        environments.append(env)
        return env
    config = dict(server_url='http://test', action_hz=20., request_timeout_s=1., execute_steps=2, max_steps=5)
    return config, factory, httpx.Client(base_url='http://test', transport=httpx.MockTransport(respond)), requests, environments, waits


def test_prefix_and_reinfer():
    config, factory, client, requests, envs, waits = setup_run()
    with client:
        assert run(config, factory, client, clock=lambda: 0., sleep=waits.append) == 5
    assert [r['step_id'] for r in requests] == [0, 1, 2]
    assert np.isclose(envs[0].pose[0], .007)
    assert waits == [.05] * 5
    assert envs[0].held and envs[0].closed


@pytest.mark.parametrize('handler', [
    lambda o: httpx.Response(200, json={'step_id': o['step_id'] + 1, 'actions': [[0.] * 6]}),
    lambda o: httpx.Response(200, json={'step_id': o['step_id'], 'actions': []}),
    lambda o: httpx.Response(200, json={'step_id': o['step_id'], 'actions': [[0.] * 7]}),
    lambda o: httpx.Response(500),
])
def test_errors_hold_and_close(handler):
    config, factory, client, _, envs, _ = setup_run(handler)
    with client, pytest.raises(Exception):
        run(config, factory, client, sleep=lambda _: None)
    assert not envs[0].executed and envs[0].held and envs[0].closed


@pytest.mark.parametrize('exception', [httpx.ReadTimeout('超时'), KeyboardInterrupt(), RuntimeError('观测失败')])
def test_interruption(exception):
    def fail(_):
        raise exception
    config, factory, client, _, envs, _ = setup_run(fail)
    with client, pytest.raises(type(exception)):
        run(config, factory, client)
    assert envs[0].held and envs[0].closed and not envs[0].executed


def test_rotation_and_latest_pose():
    pose = np.r_[.1, .2, .3, Rotation.from_euler('x', .8).as_quat()]
    action = [.01, -.02, .03, 0., 0., .6]
    target = compose_pose(pose, action)
    delta = Rotation.from_quat(target[3:]) * Rotation.from_quat(pose[3:]).inv()
    assert np.allclose(delta.as_rotvec(), action[3:])
    assert np.allclose(target[:3] - pose[:3], action[:3])
    next_pose = compose_pose(target, action)
    assert not np.allclose(next_pose, target)


def test_gripper_and_nonfinite():
    health = Health(policy_type='mock', state_names=[], cameras={}, action_names=POSE_ACTION + ['cmd_gripper'],
                    reference_frame='base_link', tcp_link='tool0', fps=20)
    env = MockEnv({}, health)
    env.step([0.] * 6 + [1.])
    assert env.gripper
    env.step([0.] * 7)
    assert not env.gripper
    with pytest.raises(ValueError):
        compose_pose(env.pose, [float('inf')] * 6)


def test_invalid_tail_prevents_entire_chunk():
    def response(obs):
        return httpx.Response(200, content='{"step_id":0,"actions":[[0,0,0,0,0,0],[NaN,0,0,0,0,0]]}',
                              headers={'content-type': 'application/json'})
    config, factory, client, _, envs, _ = setup_run(response)
    with client, pytest.raises(ValueError):
        run(config, factory, client)
    assert envs[0].executed == [] and envs[0].closed


def test_observation_failure_releases_resources():
    config, factory, client, requests, envs, _ = setup_run()
    def broken_factory(c, h):
        env = factory(c, h)
        def fail(_):
            raise RuntimeError('相机读取失败')
        env.observe = fail
        return env
    with client, pytest.raises(RuntimeError, match='相机'):
        run(config, broken_factory, client)
    assert not requests and envs[0].held and envs[0].closed


def test_slow_execution_does_not_catch_up():
    config, factory, client, _, envs, waits = setup_run()
    now = iter([0., .2, .2, .4, .4, .6, .6, .8, .8, 1.])
    with client:
        run(config, factory, client, clock=lambda: next(now), sleep=waits.append)
    assert waits == [0.] * 5
