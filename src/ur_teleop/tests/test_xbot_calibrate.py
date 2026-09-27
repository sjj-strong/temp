"""校准输出与遥操作读取路径保持一致。"""
from pathlib import Path

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
    JoyMapping(yaml.safe_load(path.read_text()))
