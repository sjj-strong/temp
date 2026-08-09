"""Non-blocking single-key stdin reader.

Pure logic (stdlib only). Nodes pass their stdin; timeout 0 makes read_key
cheap enough to call from a 50 Hz timer callback.
"""

import select
import sys


class KeyboardReader:
    def __init__(self, stream=None):
        self._stream = stream if stream is not None else sys.stdin

    def read_key(self, timeout: float = 0.0) -> str | None:
        """One key within `timeout` seconds, lowercased; 'enter' for newline; None otherwise."""
        try:
            ready, _, _ = select.select([self._stream], [], [], timeout)
        except (ValueError, OSError):
            return None
        if not ready:
            return None
        ch = self._stream.read(1)
        if not ch:
            return None
        if ch in ("\n", "\r"):
            return "enter"
        return ch.lower()
