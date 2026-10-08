"""主机同步采集、HTTP 推理和动作前缀执行。"""
import argparse
import logging
import math
import time
from pathlib import Path

import httpx
import yaml

from protocol import ActionChunk, Health
from ur_env import MockEnv, UREnv


def run(config, env_factory=None, client=None, clock=time.monotonic, sleep=time.sleep):
    """每轮仅执行指定前缀；超时及所有异常都保持并关闭环境。"""
    for name in ('execute_steps', 'max_steps'):
        value = config[name]
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise ValueError(f'{name} 必须为正整数')
    for name in ('action_hz', 'request_timeout_s'):
        if not math.isfinite(config[name]) or config[name] <= 0:
            raise ValueError(f'{name} 必须为有限正数')
    owned = client is None
    client = client or httpx.Client(base_url=config['server_url'], timeout=config['request_timeout_s'], trust_env=False)
    env = None
    executed = 0
    try:
        response = client.get('/health')
        response.raise_for_status()
        health = Health.model_validate(response.json())
        mode = config.get('mode', 'mock')
        if mode not in ('mock', 'ros'):
            raise ValueError('mode 必须为 mock 或 ros')
        factory = env_factory or (MockEnv if mode == 'mock' else UREnv)
        env = factory(config, health)
        period = 1. / config['action_hz']
        step_id = 0
        while executed < config['max_steps']:
            observation = env.observe(step_id)
            if mode == 'ros' and config.get('read_only', True):
                # 真机只读验收不向控制器或夹爪发送任何命令。
                logging.info('只读观测成功：%s 个字段、%s 台相机', len(observation.values), len(observation.images))
                return 0
            response = client.post('/infer', json=observation.model_dump())
            response.raise_for_status()
            chunk = ActionChunk.model_validate(response.json())
            if chunk.step_id != step_id or len(chunk.actions[0]) != len(health.action_names):
                raise ValueError('推理响应编号或动作维度不符')
            count = min(config['execute_steps'], len(chunk.actions), config['max_steps'] - executed)
            for action in chunk.actions[:count]:
                started = clock()
                env.step(action)
                executed += 1
                # 慢调用不补发；最后一步也留出一个动作周期再读取观测。
                sleep(max(0., period - (clock() - started)))
            step_id += 1
            logging.info('已执行 %s 步，完成 %s 次推理', executed, step_id)
        return executed
    finally:
        try:
            if env is not None:
                try:
                    env.hold()
                except Exception:
                    logging.exception('保持失败；控制器仍可能保持上一目标')
                finally:
                    env.close()
        finally:
            if owned:
                client.close()


def main():
    parser = argparse.ArgumentParser(description='UR 本地远程策略评估')
    parser.add_argument('--config', default='configs/client.yaml')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
    try:
        run(yaml.safe_load(Path(args.config).read_text()))
    except KeyboardInterrupt:
        logging.info('用户中断，已结束评估')
    except Exception:
        logging.exception('评估失败，已停止后续动作')
        raise SystemExit(1)


if __name__ == '__main__':
    main()
