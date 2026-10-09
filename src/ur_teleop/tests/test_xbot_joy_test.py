"""动作日志、局部映射、组合运动输入与断连恢复验证。"""

import math

import pytest
from sensor_msgs.msg import Joy

from ur_teleop.xbot_core import JoyMapping
from ur_teleop.xbot_joy_test import JoyDiagnostics, parse_args


def diagnostics(config=None, gripper_enabled=True):
    # 固定映射以覆盖用户实际遇到的 19 项标定与 11 项输入不一致。
    cfg = {
        'version': 1, 'axis_count': 8, 'button_count': 19,
        'buttons': {'rb': 7, 'lb': 6, 'a': 0, 'x': 3, 'menu': 11,
                    'y': 4, 'b': 1, 'view': 10},
        'axes': {name: {'kind': 'axis', 'index': index, 'rest': rest, 'positive': 1.0 if rest == 0.0 else -1.0}
                 for name, index, rest in [('lx', 0, 0.0), ('ly', 1, 0.0),
                                           ('rx', 2, 0.0), ('ry', 3, 0.0),
                                           ('rt', 4, 1.0), ('lt', 5, 1.0),
                                           ('yaw', 6, 0.0)]},
    }
    return JoyDiagnostics(JoyMapping(cfg), config=config, gripper_enabled=gripper_enabled)


def message(monitor, pressed=(), button_count=19, values=None):
    cfg = monitor.mapping.cfg
    axes = [0.0] * cfg['axis_count']
    for name, spec in cfg['axes'].items():
        if spec.get('kind') != 'buttons':
            axes[spec['index']] = (values or {}).get(name, spec['rest'])
    buttons = [0] * button_count
    for name in pressed:
        buttons[cfg['buttons'][name]] = 1
    return Joy(axes=axes, buttons=buttons)


def test_short_press_emits_actions_before_snapshot_and_no_raw_data():
    monitor = diagnostics()
    monitor.receive(message(monitor), 0.0)
    events = monitor.receive(message(monitor, ['rb']), 0.01)
    assert any('运动使能（RB）按下' in event for event in events)
    assert any('运动使能（RB）松开' in event
               for event in monitor.receive(message(monitor), 0.02))
    assert monitor.receive(message(monitor), 0.03) == []
    output = '\n'.join(events) + monitor.snapshot(0.04, 0.25)
    for unwanted in ['原始', '按键[', '消息数', '数据龄']:
        assert unwanted not in output


def test_eleven_buttons_keep_valid_actions_and_warn_menu_once():
    monitor = diagnostics()
    events = monitor.receive(message(monitor, ['rb'], button_count=11), 0.0)
    assert any('开始录制（MENU）不可用' in event for event in events)
    assert any('运动使能（RB）按下' in event for event in events)
    assert '运动已使能' in monitor.snapshot(0.1, 0.25)
    assert monitor.receive(message(monitor, ['rb'], button_count=11), 0.11) == []
    # 正式遥操作仍然拒绝不一致的布局。
    msg = message(monitor, button_count=11)
    with pytest.raises(ValueError, match='布局与校准不一致'):
        monitor.mapping.decode(msg.axes, msg.buttons)


def test_unmapped_button_is_silent():
    monitor = diagnostics()
    monitor.receive(message(monitor, button_count=11), 0.0)
    msg = message(monitor, button_count=11)
    msg.buttons[5] = 1
    assert monitor.receive(msg, 0.1) == []
    assert '当前按住=无' in monitor.snapshot(0.11, 0.25)


def test_missing_mapping_cannot_invent_actions():
    monitor = JoyDiagnostics()
    events = monitor.receive(Joy(axes=[0.75], buttons=[0, 1]), 0.0)
    assert any('无法识别动作' in event for event in events)
    assert '无法预览动作' in monitor.snapshot(0.1, 0.25)
    assert '原始' not in ''.join(events)


def test_missing_axes_and_illegal_axis_only_disable_affected_inputs():
    monitor = diagnostics()
    msg = Joy(axes=[math.nan, 1.0], buttons=[1])
    events = monitor.receive(msg, 0.0)
    assert any('LX 运动输入不可用：输入值非法' in event for event in events)
    assert any('RY 运动输入不可用：映射超出' in event for event in events)
    assert any('切换夹爪（A）按下' in event for event in events)
    assert '-X 平移' in monitor.snapshot(0.1, 0.25)
    assert 'nan' not in monitor.snapshot(0.1, 0.25)


def test_frame_toggle_on_rising_edge_only():
    monitor = diagnostics()
    events = monitor.receive(message(monitor, ['x']), 0.0)
    assert any('当前参考系=TCP' in event for event in events)
    assert monitor.receive(message(monitor, ['x']), 0.1) == []
    monitor.receive(message(monitor), 0.15)
    events = monitor.receive(message(monitor, ['x']), 0.2)
    assert any('当前参考系=BASE' in event for event in events)


def test_gripper_gating_is_reported_as_request_only():
    monitor = diagnostics(gripper_enabled=False)
    events = monitor.receive(message(monitor, ['a']), 0.0)
    assert any('配置已关闭夹爪、RB 未使能' in event for event in events)
    monitor = diagnostics()
    events = monitor.receive(message(monitor, ['a', 'rb']), 0.0)
    assert any('请求切换夹爪' in event for event in events)


@pytest.mark.parametrize('name,event_id', [('menu', 'start'), ('y', 'save'), ('b', 'discard')])
def test_record_action_names(name, event_id):
    monitor = diagnostics()
    events = monitor.receive(message(monitor, [name]), 0.0)
    assert any(f'action={event_id}' in event for event in events)


def test_view_hold_fires_once_and_rearms_after_release():
    monitor = diagnostics({'view_hold_s': 0.5})
    events = monitor.receive(message(monitor, ['view']), 0.0)
    assert not any('action=finalize' in event for event in events)
    for stamp in [0.1, 0.2, 0.3, 0.4]:
        monitor.receive(message(monitor, ['view']), stamp)
    assert 'action=finalize' in monitor.snapshot(0.5, 0.25)
    assert 'action=finalize' not in monitor.snapshot(0.51, 0.25)
    monitor.receive(message(monitor), 0.52)
    monitor.receive(message(monitor, ['view']), 0.53)
    for stamp in [0.6, 0.7, 0.8, 0.9, 1.0]:
        monitor.receive(message(monitor, ['view']), stamp)
    assert 'action=finalize' in monitor.snapshot(1.04, 0.25)


def test_disconnect_cancels_hold_and_reconnect_resets_edges():
    monitor = diagnostics({'view_hold_s': 0.5})
    assert '等待手柄输入' in monitor.snapshot(0.0, 0.25)
    monitor.receive(message(monitor, ['view', 'rb']), 0.0)
    snapshot = monitor.snapshot(0.4, 0.25)
    assert '输入超时' in snapshot and 'action=finalize' not in snapshot
    events = monitor.receive(message(monitor, ['view', 'rb']), 0.5)
    assert '手柄输入已恢复' in events
    assert any('运动使能（RB）按下' in event for event in events)
    assert 'action=finalize' not in monitor.snapshot(0.6, 0.25)
    # 即使定时日志未先发现超时，收到输入时也能重新建立基线。
    events = monitor.receive(message(monitor, ['rb']), 1.0)
    assert '手柄输入已恢复' in events


def test_motion_direction_deadzone_dominant_axis_precision_and_zero_event():
    monitor = diagnostics()
    monitor.receive(message(monitor, values={'lx': 0.5, 'ly': 1.0}), 0.0)
    snapshot = monitor.snapshot(0.1, 0.25)
    assert '运动未使能' in snapshot and '-X 平移 强度=1.000' in snapshot
    assert '-Y 平移' not in snapshot
    monitor.receive(message(monitor, ['rb', 'lb'], values={'ly': 1.0, 'rx': -1.0}), 0.11)
    snapshot = monitor.snapshot(0.12, 0.25)
    assert '运动已使能' in snapshot and '精细模式' in snapshot
    assert '-X 平移 强度=0.250' in snapshot and '绕 -Y 旋转 强度=0.250' in snapshot
    events = monitor.receive(message(monitor, values={'ly': 0.05}), 0.13)
    assert '动作预览：运动输入归零' in events
    assert monitor.receive(message(monitor), 0.14) == []


def test_free_left_stick_vector_limit_and_trigger_directions():
    monitor = diagnostics({'left_stick_xy_free': True})
    monitor.receive(message(monitor, ['rb'], values={'lx': 1.0, 'ly': 1.0, 'lt': -1.0}), 0.0)
    snapshot = monitor.snapshot(0.1, 0.25)
    assert '-X 平移 强度=0.577' in snapshot
    assert '-Y 平移 强度=0.577' in snapshot
    assert '+Z 平移 强度=0.577' in snapshot
    monitor.receive(message(monitor, ['rb'], values={'rt': -1.0, 'yaw': -1.0}), 0.11)
    snapshot = monitor.snapshot(0.12, 0.25)
    assert '-Z 平移 强度=1.000' in snapshot and '绕 -Z 旋转' in snapshot


def test_dpad_button_mapping():
    monitor = diagnostics()
    monitor.mapping.cfg['axes']['yaw'] = {'kind': 'buttons', 'positive': 8, 'negative': 9}
    msg = message(monitor)
    msg.buttons[9] = 1
    monitor.receive(msg, 0.0)
    assert '绕 -Z 旋转' in monitor.snapshot(0.1, 0.25)


def test_default_external_driver_option():
    assert not parse_args([]).no_start_joy
    assert parse_args(['--no-start-joy']).no_start_joy
