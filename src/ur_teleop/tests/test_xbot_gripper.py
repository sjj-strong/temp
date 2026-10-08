"""夹爪开合离线回归，不连接真实硬件。"""
import time
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from ur_teleop.xbot_teleop_node import XbotTeleopNode


@pytest.mark.parametrize('accepted', [True, False])
def test_toggle_uses_previous_target_when_feedback_below_point_four(accepted):
    node = object.__new__(XbotTeleopNode)
    node.cfg = {'gripper': {'enabled': True}}
    node.x = {'tcp_timeout_s': 1.}
    node.gripper_state = .1
    node.gripper_at = time.monotonic()
    node.gripper_pending = False
    node.gripper_command = 1.
    node.core = SimpleNamespace(enabled=True)
    node.estop = node.finished = False
    node.get_logger = lambda: SimpleNamespace(info=lambda *a: None, warn=lambda *a: None,
                                             error=lambda *a: None)
    goals = []
    result = Future()

    def send(goal):
        goals.append(goal)
        future = Future()
        future.set_result(SimpleNamespace(accepted=accepted, get_result_async=lambda: result))
        return future

    node.gripper = SimpleNamespace(server_is_ready=lambda: True, send_goal_async=send)
    node.toggle_gripper()
    assert list(goals[0].command.position) == [0.]
    assert node.gripper_command == (0. if accepted else 1.)
    if accepted:
        result.set_result(None)
        node.toggle_gripper()
        assert list(goals[1].command.position) == [.4]
        assert node.gripper_command == 1.
