"""独立手柄测试的短按、未映射输入、标定错误和断连恢复验证。"""

from pathlib import Path

from sensor_msgs.msg import Joy
import yaml

from ur_teleop.xbot_core import JoyMapping
from ur_teleop.xbot_joy_test import JoyDiagnostics, parse_args


def diagnostics():
    config = Path(__file__).parents[1] / 'config/xbot_joy.yaml'
    with config.open(encoding='utf-8') as stream:
        return JoyDiagnostics(JoyMapping(yaml.safe_load(stream)))


def message(monitor, pressed=()):
    cfg = monitor.mapping.cfg
    axes = [0.0] * cfg['axis_count']
    for spec in cfg['axes'].values():
        if spec.get('kind') != 'buttons':
            axes[spec['index']] = spec['rest']
    buttons = [0] * cfg['button_count']
    for index in pressed:
        buttons[index] = 1
    return Joy(axes=axes, buttons=buttons)


def test_short_press_emits_both_edges_before_snapshot():
    monitor = diagnostics()
    rb = monitor.mapping.cfg['buttons']['rb']
    monitor.receive(message(monitor), 0.0)
    assert any('RB（运动使能） 按下' in event
               for event in monitor.receive(message(monitor, [rb]), 0.01))
    assert any('RB（运动使能） 松开' in event
               for event in monitor.receive(message(monitor), 0.02))
    # 自动重复消息不会反复产生按键事件。
    assert monitor.receive(message(monitor), 0.03) == []
    assert '输入正常' in monitor.snapshot(0.04, 0.25)


def test_unmapped_button_and_axis_are_visible():
    monitor = JoyDiagnostics()
    events = monitor.receive(Joy(axes=[0.75], buttons=[0, 1]), 0.0)
    assert '按键[1] 未映射按键 按下' in events
    snapshot = monitor.snapshot(0.1, 0.25)
    assert '原始轴[0:+0.750]' in snapshot
    assert '原始按键[0:0,1:1]' in snapshot


def test_layout_mismatch_still_shows_raw_input():
    monitor = diagnostics()
    monitor.receive(Joy(axes=[-1.0], buttons=[1]), 0.0)
    snapshot = monitor.snapshot(0.1, 0.25)
    assert '标定不匹配' in snapshot
    assert '原始轴[0:-1.000]' in snapshot


def test_timeout_does_not_display_old_pressed_state_and_recovers():
    monitor = diagnostics()
    rb = monitor.mapping.cfg['buttons']['rb']
    assert '等待手柄输入' in monitor.snapshot(0.0, 0.25)
    monitor.receive(message(monitor, [rb]), 0.0)
    snapshot = monitor.snapshot(0.3, 0.25)
    assert '输入超时' in snapshot
    assert '功能键' not in snapshot
    events = monitor.receive(message(monitor, [rb]), 0.4)
    assert any('RB（运动使能） 按下' in event for event in events)
    assert '输入正常' in monitor.snapshot(0.41, 0.25)


def test_reconnect_between_snapshots_resets_baseline():
    monitor = diagnostics()
    rb = monitor.mapping.cfg['buttons']['rb']
    monitor.receive(message(monitor, [rb]), 0.0)
    events = monitor.receive(message(monitor, [rb]), 1.0)
    assert any('RB（运动使能） 按下' in event for event in events)


def test_calibrated_axis_direction_and_deadzone():
    monitor = diagnostics()
    msg = message(monitor)
    spec = monitor.mapping.cfg['axes']['lt']
    msg.axes[spec['index']] = spec['positive']
    monitor.receive(msg, 0.0)
    snapshot = monitor.snapshot(0.1, 0.25)
    assert 'LT:+1.000' in snapshot
    assert 'LX:+0.000' in snapshot


def test_default_topic_requires_explicit_external_driver_option():
    assert not parse_args([]).no_start_joy
    assert parse_args(['--no-start-joy']).no_start_joy
