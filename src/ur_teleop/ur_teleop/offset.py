"""Session-local offset captured at home.

Pure logic — no rclpy imports. Values are captured once per session after both
arms settle at home, and never persisted (spec §3: offset is session-scoped).
"""

from typing import Optional


class SessionOffset:
    """Holds the actual master/slave positions at home for the current session."""

    def __init__(self):
        self.master_home: Optional[list[float]] = None
        self.slave_home: Optional[list[float]] = None

    @property
    def captured(self) -> bool:
        return self.master_home is not None and self.slave_home is not None

    def capture(self, master_q: list[float], slave_q: list[float]) -> None:
        self.master_home = list(master_q)
        self.slave_home = list(slave_q)
