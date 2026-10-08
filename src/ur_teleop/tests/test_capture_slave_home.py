"""纯逻辑测试，不连接真实机械臂。"""
import math

import pytest
import yaml

from ur_teleop.capture_slave_home import JOINT_NAMES, driver_command, ordered_positions, replace_slave, write_configs

TEXT = '# 保留说明\ncell:\n  robot_ip: 1.2.3.4\nhome:\n  master: [1, 2, 3, 4, 5, 6]\n  slave:  [0, 0, 0, 0, 0, 0] # 保留行尾注释\n  settle_time_s: 2.0\n'


def test_joint_order_and_extra_gripper():
    names = list(reversed(JOINT_NAMES)) + ['gripper']
    positions = [5, 4, 3, 2, 1, 0, 42]
    assert ordered_positions(names, positions) == [0, 1, 2, 3, 4, 5]
    with pytest.raises(KeyError):
        ordered_positions(JOINT_NAMES[:-1], [0] * 5)
    with pytest.raises(ValueError):
        ordered_positions(JOINT_NAMES, [0] * 5 + [math.nan])
    with pytest.raises(ValueError):
        ordered_positions(JOINT_NAMES + [JOINT_NAMES[0]], [0] * 7)


def test_only_slave_changes():
    positions = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    updated = replace_slave(TEXT, positions)
    assert updated == TEXT.replace('[0, 0, 0, 0, 0, 0]', str(positions))
    assert yaml.safe_load(updated)['home']['slave'] == positions


def test_write_both_configs_and_backup(tmp_path):
    paths = [tmp_path / name for name in ['xbot.yaml', 'ur.yaml']]
    for path in paths:
        path.write_text(TEXT)
    positions = [0.1] * 6
    backups = write_configs(paths, positions)
    for path in paths:
        assert yaml.safe_load(path.read_text())['home']['slave'] == positions
        assert backups[path].read_text() == TEXT


def test_invalid_second_file_leaves_first_unchanged(tmp_path):
    first, second = tmp_path / 'first.yaml', tmp_path / 'second.yaml'
    first.write_text(TEXT)
    second.write_text('home: {}\n')
    with pytest.raises(ValueError):
        write_configs([first, second], [0.1] * 6)
    assert first.read_text() == TEXT
    assert not list(tmp_path.glob('*.bak'))


def test_write_failure_rolls_back_first_file(tmp_path, monkeypatch):
    import os
    first, second = tmp_path / 'first.yaml', tmp_path / 'second.yaml'
    for path in [first, second]:
        path.write_text(TEXT)
    original_replace = os.replace
    calls = 0

    def fail_second(source, target):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError('模拟第二个文件写入失败')
        return original_replace(source, target)

    monkeypatch.setattr(os, 'replace', fail_second)
    with pytest.raises(OSError):
        write_configs([first, second], [0.1] * 6)
    assert first.read_text() == second.read_text() == TEXT


def test_read_only_driver_startup_flags():
    command = driver_command('169.254.138.15', 'ur10e')
    assert command[:4] == ['ros2', 'launch', 'ur_robot_driver', 'ur_control.launch.py']
    assert 'activate_joint_controller:=false' in command
    assert 'headless_mode:=false' in command
    assert 'use_mock_hardware:=false' in command
    assert 'launch_dashboard_client:=false' in command
    assert 'home.launch.py' not in command
