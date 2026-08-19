"""pyrobotiqgripper wrapper — direct serial gripper control.

Thin adapter around pyrobotiqgripper v3.2.6 so teleop_node stays clean and
tests can inject a mock. Import is lazy: the module imports fine without the
library installed; only connect() requires it.
"""

from typing import Optional


class GripperInterface:
    """Connect/open/close a Robotiq 2F-85 via USB-RS485 serial."""

    def __init__(self, com_port: str, speed: int = 255, force: int = 50):
        self._com_port = com_port
        self._speed = int(speed)
        self._force = int(force)
        self._gripper: Optional[object] = None

    @property
    def connected(self) -> bool:
        return self._gripper is not None

    def connect(self) -> bool:
        """Open the serial connection. Idempotent. Returns success."""
        if self._gripper is not None:
            return True
        try:
            import pyrobotiqgripper as rq
            self._gripper = rq.RobotiqGripper(com_port=self._com_port)
            if not self._gripper.isActivated():
                self._gripper.activate()
            return True
        except Exception:
            self._gripper = None
            return False

    def disconnect(self) -> None:
        if self._gripper is not None:
            try:
                self._gripper.disconnect()
            except Exception:
                pass
            self._gripper = None

    def open(self) -> bool:
        if self._gripper is None:
            return False
        try:
            self._gripper.open(speed=self._speed, force=255, wait=True)
            return True
        except Exception:
            return False

    def close(self) -> bool:
        if self._gripper is None:
            return False
        try:
            self._gripper.close(speed=self._speed, force=self._force, wait=True)
            return True
        except Exception:
            return False

    def position(self) -> Optional[float]:
        """Current gripper position normalized to [0,1]; 1=open, 0=closed."""
        if self._gripper is None:
            return None
        try:
            return self._gripper.position() / 255.0
        except Exception:
            return None
