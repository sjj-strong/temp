"""电子秤串口行解析(纯逻辑)。

参考 robot_utils/liquid_pouring/serial_manager.py 的原始模式匹配,
支持以下格式(单位 g,可省略):
    ST,GS,    0.15 g
    ST,GS,   -0.15 g
    0.15 g
    0.15
    + 0.15 g
"""
import re

WEIGHT_PATTERN = re.compile(rb"([+-]?)\s*(\d+(?:\.\d+)?)\s*g?", re.IGNORECASE)


def parse_weight_line(raw):
    """解析一行串口数据,返回重量克数(float);无效行返回 None。"""
    if not raw:
        return None
    match = WEIGHT_PATTERN.search(raw)
    if not match:
        return None
    value = float(match.group(2))
    if match.group(1) == b"-":
        return -value
    return value
