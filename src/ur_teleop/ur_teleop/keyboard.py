"""Non-blocking single-key reader (controlling terminal /dev/tty by default).

Pure logic (stdlib only). By default opens /dev/tty so keyboard input works
under `ros2 launch` (which redirects the child's stdin to /dev/null); callers
may inject a stream (tests do). timeout 0 makes read_key cheap enough to call
from a 50 Hz timer callback.

/dev/tty 打开后切 cbreak 模式：终端默认 canonical 模式会把普通按键缓冲到
按下回车才整行交付，单键命令（s/d/q 等）会"按下无反应"；cbreak 逐键立即
交付且保留 Ctrl-C 信号。退出时（atexit）恢复原终端属性。
"""

import atexit
import os
import select
import sys
import termios
import tty


class KeyboardReader:
    def __init__(self, stream=None):
        self._old_attrs = None
        if stream is not None:
            self._stream = stream
        else:
            # ros2 launch 把子进程的 stdin 重定向到 /dev/null，sys.stdin 读不到键盘。
            # 直接打开控制终端 /dev/tty 绕过（launch 改写不了 /dev/tty）；无控制终端
            #（如 CI/无 tty）时 open 失败，回退 sys.stdin。
            try:
                self._stream = open("/dev/tty", "rb")
                self._set_cbreak(self._stream.fileno())
            except OSError:
                self._stream = sys.stdin

    def _set_cbreak(self, fd):
        try:
            self._old_attrs = termios.tcgetattr(fd)
            tty.setcbreak(fd)
            atexit.register(self._restore_terminal)
        except (termios.error, ValueError):
            self._old_attrs = None

    def _restore_terminal(self):
        if self._old_attrs is not None:
            try:
                termios.tcsetattr(self._stream.fileno(), termios.TCSADRAIN, self._old_attrs)
            except (termios.error, ValueError):
                pass

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
