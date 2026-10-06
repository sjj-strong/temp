"""校准输出与遥操作读取路径保持一致。"""
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from ur_teleop.config import load_config
from ur_teleop.xbot_core import JoyMapping


def test_default_calibration_path_matches_teleop():
    pytest.importorskip('rclpy')
    from ur_teleop.xbot_calibrate import parse_args
    config_dir = Path(__file__).resolve().parents[1] / 'config'
    cfg = load_config(config_dir / 'xbot_teleop.yaml')
    expected = '/ros2_ws/src/ur_teleop/config/xbot_joy.yaml'
    assert parse_args([]).output == cfg['xbot']['calibration_file'] == expected
    assert parse_args(['--output', '/tmp/custom_joy.yaml']).output == '/tmp/custom_joy.yaml'


def test_saved_mapping_is_valid():
    path = Path(__file__).resolve().parents[1] / 'config/xbot_joy.yaml'
    if not path.exists():
        pytest.skip('尚未生成手柄标定文件')
    JoyMapping(yaml.safe_load(path.read_text()))


def test_neutral_waits_for_release_and_one_second():
    """按键松开后连续回中一秒才进入下一步。"""
    pytest.importorskip('rclpy')
    from ur_teleop.xbot_calibrate import wait_neutral
    now = [0.]
    baseline = SimpleNamespace(axes=[0.], buttons=[0])

    def sample():
        now[0] += .1
        return SimpleNamespace(axes=[0.], buttons=[1 if now[0] < .5 else 0])

    result = wait_neutral(sample, baseline, clock=lambda: now[0])
    assert result.buttons == [0]
    assert now[0] >= 1.5


def test_axis_capture_requires_continuous_hold():
    """短暂触发后释放不会被误判为有效轴。"""
    pytest.importorskip('rclpy')
    from ur_teleop.xbot_calibrate import wait_capture
    now = [0.]
    baseline = SimpleNamespace(axes=[0.], buttons=[0])

    def sample():
        now[0] += .1
        value = 0. if .4 < now[0] < .6 else (1. if now[0] >= 1. else .8)
        return SimpleNamespace(axes=[value], buttons=[0])

    result = wait_capture(sample, '测试轴', baseline, 'axis', clock=lambda: now[0])
    assert result == {'kind': 'axis', 'index': 0, 'rest': 0., 'positive': 1.}
    assert now[0] >= 1.6


def test_button_capture_accepts_short_press():
    """A 等按键无需保持一秒，首次 Joy 按下即被记录。"""
    pytest.importorskip('rclpy')
    from ur_teleop.xbot_calibrate import wait_capture
    now = [0.]
    baseline = SimpleNamespace(axes=[0.], buttons=[0, 0])

    def sample():
        now[0] += .1
        return SimpleNamespace(axes=[0.], buttons=[1, 0])

    assert wait_capture(sample, '按 A', baseline, 'button', clock=lambda: now[0]) == {
        'kind': 'button', 'index': 0,
    }
    assert now[0] == pytest.approx(.1)
