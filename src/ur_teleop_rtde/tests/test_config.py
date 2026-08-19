"""Config loading/validation tests."""

from pathlib import Path

import pytest
import yaml

from ur_teleop_rtde.config import ConfigError, load_config

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "ur_teleop_rtde.yaml"


def _write(tmp_path: Path, data: dict) -> Path:
    p = tmp_path / "test.yaml"
    p.write_text(yaml.safe_dump(data))
    return p


def test_loads_default_config(cfg):
    assert cfg["mode"] in ("teleop", "record")
    assert len(cfg["home"]["master"]) == 6
    assert len(cfg["home"]["slave"]) == 6
    assert len(cfg["mapping"]["ur_joint_order"]) == 6
    assert "shoulder_pan_joint" in cfg["mapping"]["safety"]["limits"]


def test_missing_file_raises(tmp_path):
    with pytest.raises(ConfigError):
        load_config(tmp_path / "nope.yaml")


def test_missing_top_key(tmp_path, cfg):
    data = dict(cfg)
    del data["rtde"]
    with pytest.raises(ConfigError, match="rtde"):
        load_config(_write(tmp_path, data))


def test_bad_mode(tmp_path, cfg):
    data = dict(cfg, mode="bogus")
    with pytest.raises(ConfigError, match="mode"):
        load_config(_write(tmp_path, data))


def test_wrong_home_length(tmp_path, cfg):
    data = dict(cfg)
    data["home"] = dict(data["home"], master=[0.0, 0.0])
    with pytest.raises(ConfigError, match="home.master"):
        load_config(_write(tmp_path, data))


def test_missing_safety_limit(tmp_path, cfg):
    data = dict(cfg)
    limits = dict(data["mapping"]["safety"]["limits"])
    del limits["wrist_3_joint"]
    data["mapping"] = dict(data["mapping"], safety=dict(data["mapping"]["safety"], limits=limits))
    with pytest.raises(ConfigError, match="wrist_3_joint"):
        load_config(_write(tmp_path, data))


def test_bad_servoj_gain(tmp_path, cfg):
    data = dict(cfg)
    data["rtde"] = dict(data["rtde"], servoj_gain=5000)
    with pytest.raises(ConfigError, match="servoj_gain"):
        load_config(_write(tmp_path, data))


def test_bad_lookahead(tmp_path, cfg):
    data = dict(cfg)
    data["rtde"] = dict(data["rtde"], servoj_lookahead=0.5)
    with pytest.raises(ConfigError, match="servoj_lookahead"):
        load_config(_write(tmp_path, data))


def test_empty_robot_ip(tmp_path, cfg):
    data = dict(cfg)
    data["robot"] = dict(data["robot"], robot_ip="")
    with pytest.raises(ConfigError, match="robot_ip"):
        load_config(_write(tmp_path, data))
