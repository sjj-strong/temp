"""Shared fixtures: mock rtde modules, sample config, mock pyrobotiqgripper."""

import sys
import time
import types
from pathlib import Path

import pytest

from ur_teleop_rtde.config import load_config

CONFIG_PATH = (
    Path(__file__).resolve().parent.parent / "config" / "ur_teleop_rtde.yaml"
)


# ----------------------------------------------------------------------
# Mock ur-rtde (rtde_control / rtde_receive) — injected into sys.modules
# ----------------------------------------------------------------------

class MockRTDEControl:
    """Fake RTDEControlInterface: records servoJ calls, exposes loop hooks."""

    def __init__(self):
        self.last_servoj = None
        self.periods = 0
        self.moveJ_result = True
        self.disconnected = False
        self.servo_stopped = False

    def initPeriod(self):
        return time.time()

    def waitPeriod(self, t_start):
        self.periods += 1

    def servoJ(self, q, velocity, acceleration, dt, lookahead, gain):
        self.last_servoj = list(q)

    def moveJ(self, q, speed, accel):
        return self.moveJ_result

    def servoStop(self):
        self.servo_stopped = True

    def disconnect(self):
        self.disconnected = True


class MockRTDEReceive:
    """Fake RTDEReceiveInterface with configurable actual state."""

    def __init__(self):
        self.q = [0.1, -1.5, 0.2, -1.5, 0.0, 0.1]
        self.tcp = [0.3, 0.4, 0.5, 0.0, 0.0, 0.0]
        self.disconnected = False

    def getActualQ(self):
        return list(self.q)

    def getActualTCPPose(self):
        return list(self.tcp)

    def disconnect(self):
        self.disconnected = True


@pytest.fixture
def mock_rtde(monkeypatch):
    """Install mock rtde_control/receive modules; return the instances."""
    ctrl, recv = MockRTDEControl(), MockRTDEReceive()

    def _make_ctrl(ip):
        return ctrl

    def _make_recv(ip):
        return recv

    ctrl_mod = types.ModuleType("rtde_control")
    ctrl_mod.RTDEControlInterface = _make_ctrl
    recv_mod = types.ModuleType("rtde_receive")
    recv_mod.RTDEReceiveInterface = _make_recv
    monkeypatch.setitem(sys.modules, "rtde_control", ctrl_mod)
    monkeypatch.setitem(sys.modules, "rtde_receive", recv_mod)
    return ctrl, recv


# ----------------------------------------------------------------------
# Mock pyrobotiqgripper
# ----------------------------------------------------------------------

class MockGripperLib:
    """Fake pyrobotiqgripper module with the API we call."""

    class RobotiqGripper:
        def __init__(self, com_port):
            self._pos = 255  # 1.0 normalized = open
            self._activated = True
            self._connected = True

        def isActivated(self):
            return self._activated

        def activate(self):
            self._activated = True

        def open(self, speed=255, force=255, wait=True):
            self._pos = 255

        def close(self, speed=255, force=50, wait=True):
            self._pos = 0

        def position(self):
            return self._pos

        def disconnect(self):
            self._connected = False


@pytest.fixture
def mock_pyrobotiq(monkeypatch):
    monkeypatch.setitem(sys.modules, "pyrobotiqgripper", MockGripperLib())


# ----------------------------------------------------------------------
# Config fixtures
# ----------------------------------------------------------------------

@pytest.fixture
def cfg():
    return load_config(CONFIG_PATH)


@pytest.fixture
def joint_states_msg():
    """Alicia joint_states message factory."""
    def make(positions=None):
        from sensor_msgs.msg import JointState
        names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6",
                 "Gripper"]
        pos = positions or [0.0, -1.2, 0.5, 0.0, 0.0, 0.0, 0.02]
        msg = JointState()
        msg.name = names
        msg.position = pos
        return msg
    return make
