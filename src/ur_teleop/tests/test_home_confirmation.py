"""外部控制确认的离线测试，不下发机械臂指令。"""
from types import SimpleNamespace

import pytest

from ur_teleop.home_node import HomeNode
import ur_teleop.home_node as module


@pytest.mark.parametrize('source', ['alicia', 'xbot'])
def test_real_home_waits_for_enter(monkeypatch, source):
    node = object.__new__(HomeNode)
    node._source, node._sim = source, False
    node._program_running = None
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


def test_connection_status_and_periodic_hint(monkeypatch):
    """模拟驱动连接和时间推进，提示重现且无回车时不开始 Home。"""
    node = object.__new__(HomeNode)
    node._sim, node._program_running = False, None
    logs, restored = [], []
    node.get_logger = lambda: SimpleNamespace(info=logs.append)
    keys = iter([None, None, None, 'enter'])
    monkeypatch.setattr(module, 'KeyboardReader', lambda: SimpleNamespace(
        read_key=lambda: next(keys), _restore_terminal=lambda: restored.append(True)))
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    times = iter([0, 1, 2, 7, 8])
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(times))
    spins = []
    def spin_once(**kwargs):
        spins.append(True)
        if len(spins) == 2:
            node._program_cb(SimpleNamespace(data=True))
    assert node.wait_for_external_control(SimpleNamespace(spin_once=spin_once))
    hints = [line for line in logs if 'Home 等待确认' in line]
    assert len(hints) == 3
    assert '等待 UR 外部控制连接' in hints[0]
    assert all('Ready to receive control commands' in line for line in hints[1:])
    assert len(spins) == 4 and restored == [True]


def test_disconnection_does_not_show_connected_hint():
    node = object.__new__(HomeNode)
    logs = []
    node.get_logger = lambda: SimpleNamespace(info=logs.append)
    node._program_cb(SimpleNamespace(data=False))
    node._confirmation_hint()
    assert '等待 UR 外部控制连接' in logs[0]
    assert 'Ready to receive control commands' not in logs[0]
