"""外部控制确认的离线测试，不下发机械臂指令。"""
from types import SimpleNamespace

import pytest

from ur_teleop.home_node import HomeNode
import ur_teleop.home_node as module


@pytest.mark.parametrize('source', ['alicia', 'xbot'])
def test_real_home_waits_for_enter(monkeypatch, source):
    node = object.__new__(HomeNode)
    node._source, node._sim = source, False
    logs, spins, restored = [], [], []
    keys = iter([None, 'a', 'enter'])
    monkeypatch.setattr(module, 'KeyboardReader', lambda: SimpleNamespace(
        read_key=lambda: next(keys), _restore_terminal=lambda: restored.append(True)))
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    node.get_logger = lambda: SimpleNamespace(info=logs.append)
    executor = SimpleNamespace(spin_once=lambda **kwargs: spins.append(True))
    assert node.wait_for_external_control(executor)
    assert len(spins) == 3
    assert '外部控制' in logs[0] and '回车' in logs[0]
    assert restored == [True]


def test_sim_skips_confirmation():
    node = object.__new__(HomeNode)
    node._sim = True
    assert node.wait_for_external_control(None)
