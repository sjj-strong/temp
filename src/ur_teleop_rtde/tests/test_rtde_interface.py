"""RTDEInterface tests with mocked ur-rtde modules."""

import time

import pytest

from ur_teleop_rtde.rtde_interface import RTDEInterface


def _cfg(**overrides):
    cfg = {
        "servoj_dt": 0.002, "servoj_velocity": 0.5, "servoj_acceleration": 0.5,
        "servoj_lookahead": 0.2, "servoj_gain": 100,
        "state_read_every_n": 5, "reconnect_attempts": 3, "reconnect_delay_s": 0.01,
    }
    cfg.update(overrides)
    return cfg


def test_connect_success(mock_rtde):
    rt = RTDEInterface("192.168.1.10", _cfg())
    assert rt.connect()
    assert rt.is_connected()
    assert rt.probe_connection()


def test_disconnect(mock_rtde):
    ctrl, recv = mock_rtde
    rt = RTDEInterface("192.168.1.10", _cfg())
    rt.connect()
    rt.disconnect()
    assert not rt.is_connected()
    assert ctrl.disconnected and recv.disconnected


def test_servo_loop_streams_targets(mock_rtde):
    ctrl, recv = mock_rtde
    rt = RTDEInterface("192.168.1.10", _cfg())
    rt.connect()
    rt.start_servo_loop()
    target = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    rt.set_target(target)
    # Give the loop a few cycles.
    deadline = time.time() + 1.0
    while ctrl.last_servoj != target and time.time() < deadline:
        time.sleep(0.01)
    assert ctrl.last_servoj == target
    rt.stop_servo_loop()


def test_servo_loop_seeds_initial_target(mock_rtde):
    """Without set_target, the loop seeds from actual joints (no jump)."""
    ctrl, recv = mock_rtde
    rt = RTDEInterface("192.168.1.10", _cfg())
    rt.connect()
    rt.start_servo_loop()
    deadline = time.time() + 1.0
    while ctrl.last_servoj is None and time.time() < deadline:
        time.sleep(0.01)
    assert ctrl.last_servoj == recv.q
    rt.stop_servo_loop()


def test_state_readback(mock_rtde):
    ctrl, recv = mock_rtde
    rt = RTDEInterface("192.168.1.10", _cfg(state_read_every_n=1))
    rt.connect()
    recv.q = [0.5, -1.0, 0.3, -1.2, 0.1, 0.2]
    recv.tcp = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    rt.start_servo_loop()
    deadline = time.time() + 1.0
    while rt.get_actual_joints() != recv.q and time.time() < deadline:
        time.sleep(0.01)
    assert rt.get_actual_joints() == recv.q
    assert rt.get_actual_tcp_pose() == recv.tcp
    rt.stop_servo_loop()


def test_loop_error_on_failure(mock_rtde):
    """If servoJ raises, the loop records the error and stops."""
    ctrl, recv = mock_rtde

    def boom(*a, **k):
        raise ConnectionError("connection lost")

    ctrl.servoJ = boom
    rt = RTDEInterface("192.168.1.10", _cfg())
    rt.connect()
    rt.start_servo_loop()
    deadline = time.time() + 2.0
    while rt.get_loop_error() is None and time.time() < deadline:
        time.sleep(0.01)
    assert isinstance(rt.get_loop_error(), ConnectionError)
    rt.stop_servo_loop()


def test_reconnect_after_failure(mock_rtde):
    ctrl, recv = mock_rtde
    rt = RTDEInterface("192.168.1.10", _cfg())
    rt.connect()
    rt.start_servo_loop()
    time.sleep(0.05)

    def boom(*a, **k):
        raise ConnectionError("connection lost")

    ctrl.servoJ = boom
    deadline = time.time() + 1.0
    while rt.get_loop_error() is None and time.time() < deadline:
        time.sleep(0.01)
    assert rt.get_loop_error() is not None

    # "Recover" by restoring servoJ and reconnecting.
    ctrl.servoJ = MockRTDE_servoJ_recovery
    rt.clear_loop_error()
    assert rt.reconnect()
    rt.start_servo_loop()
    time.sleep(0.05)
    assert rt.is_connected()
    rt.stop_servo_loop()
    rt.disconnect()


def MockRTDE_servoJ_recovery(*args, **kwargs):
    pass


def test_move_j(mock_rtde):
    ctrl, recv = mock_rtde
    rt = RTDEInterface("192.168.1.10", _cfg())
    rt.connect()
    target = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    assert rt.move_j(target, speed=1.0, acceleration=1.0)
    ctrl.moveJ_result = False
    assert not rt.move_j(target)


def test_start_loop_requires_connect(mock_rtde):
    rt = RTDEInterface("192.168.1.10", _cfg())
    with pytest.raises(RuntimeError):
        rt.start_servo_loop()
