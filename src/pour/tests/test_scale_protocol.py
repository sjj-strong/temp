from pour.scale_protocol import parse_weight_line


def test_plain_gram():
    assert parse_weight_line(b"0.15 g") == 0.15


def test_plain_number():
    assert parse_weight_line(b"0.15") == 0.15


def test_prefix_gram():
    assert parse_weight_line(b"ST,GS,    0.15 g") == 0.15


def test_negative():
    assert parse_weight_line(b"ST,GS,   -0.15 g") == -0.15


def test_positive_sign():
    assert parse_weight_line(b"+ 0.15 g") == 0.15


def test_ignore_case_g():
    assert parse_weight_line(b"12.5 G") == 12.5


def test_integer_without_decimal():
    assert parse_weight_line(b"ST,GS,  123 g") == 123.0


def test_invalid_line_returns_none():
    assert parse_weight_line(b"hello world") is None
    assert parse_weight_line(b"") is None
    assert parse_weight_line(b"\r\n") is None
