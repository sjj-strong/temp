import os

from ur_teleop.keyboard import KeyboardReader


def test_reads_available_char():
    r, w = os.pipe()
    os.write(w, b"S")
    reader = KeyboardReader(os.fdopen(r, "r"))
    assert reader.read_key(0.2) == "s"      # 小写化
    os.close(w)


def test_enter_key_normalized():
    r, w = os.pipe()
    os.write(w, b"\n")
    reader = KeyboardReader(os.fdopen(r, "r"))
    assert reader.read_key(0.2) == "enter"
    os.close(w)


def test_no_input_returns_none():
    r, w = os.pipe()
    reader = KeyboardReader(os.fdopen(r, "r"))
    assert reader.read_key(0.05) is None
    os.close(w)


def test_eof_returns_none():
    r, w = os.pipe()
    os.close(w)                             # 立即 EOF
    reader = KeyboardReader(os.fdopen(r, "r"))
    assert reader.read_key(0.05) is None


def test_multiple_chars_pending_not_lost():
    r, w = os.pipe()
    os.write(w, b"ab")
    reader = KeyboardReader(os.fdopen(r, "r"))
    assert reader.read_key(0.2) == "a"
    assert reader.read_key(0.2) == "b"      # 缓冲流 read(1) 会把 "b" 滞留在 Python 缓冲
    os.close(w)
