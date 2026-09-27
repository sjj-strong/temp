"""键盘单步目标始终来自最新实测位置，并锁存平移测试的目标姿态。"""

import importlib.util
import copy
from pathlib import Path

import pytest
from geometry_msgs.msg import PoseStamped


module_path = Path(__file__).resolve().parents[1] / 'src' / 'keyboard_target_pose.py'
spec = importlib.util.spec_from_file_location('keyboard_target_pose', module_path)
keyboard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(keyboard)


def test_repeated_key_uses_latest_measured_pose():
    current = PoseStamped()
    current.header.frame_id = 'base'
    current.pose.position.x = 0.1
    current.pose.position.y = -0.2
    current.pose.orientation.w = 1.0
    held_orientation = copy.deepcopy(current.pose.orientation)

    first = keyboard.relative_target(current, held_orientation, 'x', 1, 0.005)
    again = keyboard.relative_target(current, held_orientation, 'x', 1, 0.005)
    assert first.pose.position.x == pytest.approx(0.105)
    assert again.pose.position.x == pytest.approx(0.105)
    assert current.pose.position.x == 0.1

    current.pose.position.x = 0.103
    current.pose.orientation.z = 0.1
    current.pose.orientation.w = 0.995
    following = keyboard.relative_target(current, held_orientation, 'x', 1, 0.005)
    assert following.pose.position.x == pytest.approx(0.108)
    assert following.pose.position.y == -0.2
    assert following.pose.orientation.z == 0.0
    assert following.pose.orientation.w == 1.0
