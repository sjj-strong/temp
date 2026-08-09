"""Non-blocking single-key stdin reader.

Pure logic (stdlib only). Nodes pass their stdin; timeout 0 makes read_key
cheap enough to call from a 50 Hz timer callback.
"""

import os
import select
import sys


class KeyboardReader:
    def __init__(self, stream=None):
        self._stream = stream if stream is not None else sys.stdin

    def read_key(self, timeout: float = 0.0) -> str | None:
        """One key within `timeout` seconds, lowercased; 'enter' for newline; None otherwise."""
        try:
            ready, _, _ = select.select([self._stream], [], [], timeout)
            if not ready:
                return None
            # Read straight from the fd: a buffered stream read(1) would pull
            # the whole kernel chunk into its Python-side buffer and leave the
            # rest stuck behind select's not-ready check.
            ch = os.read(self._stream.fileno(), 1)
        except (ValueError, OSError):
            return None
        if not ch:
            return None
        text = ch.decode("utf-8", errors="ignore")
        if not text:
            return None
        if text in ("\n", "\r"):
            return "enter"
        return text.lower()
