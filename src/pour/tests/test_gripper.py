#!/usr/bin/env python3
"""Robotiq 2F-85 夹爪硬件测试(串口直连,不走 ROS)。

依赖:pyrobotiqgripper(已装在 /opt/lerobot_venv,包名 pyrobotiqgripper 3.2.6)。
运行:
    /opt/lerobot_venv/bin/python tests/test_gripper.py                 # 自动选端口
    /opt/lerobot_venv/bin/python tests/test_gripper.py --port /dev/ttyUSB1

pytest 模式下无硬件 / 无依赖时自动 skip:
    cd src/pour && /usr/bin/python3 -m pytest tests -v

测试序列:连接 → activate → open → close → move 128 → move 0(结束保持张开)。
位置语义:0 = 张开,255 = 完全闭合。

注意:依赖缺失时用 try/except 标记 skip,**不要**用模块级 pytest.importorskip——
它在导入阶段抛 Skipped 会打断同目录其他测试文件的收集(本机 ament/launch-testing
插件环境下表现为:整个 tests/ 目录只剩 1 skipped,其余文件 0 收集)。
"""
import os
import sys
from pathlib import Path

# scripts/ 在包目录外面(src/pour/scripts/),把两个目录都加进 sys.path
_POUR_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_POUR_ROOT))
sys.path.insert(0, str(_POUR_ROOT / "scripts"))

import pytest

try:
    import pyrobotiqgripper  # noqa: F401  仅系统 python 缺失;硬件测试用 /opt/lerobot_venv
    from gripper import RobotiqGripper, _autodetect_port
    _HAS_RQ = True
except ImportError:
    _HAS_RQ = False

# 夹爪端口:优先级 环境变量 > --port 参数 > 自动检测(FTDI FT232R → ttyUSB*)
DEFAULT_PORTS = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"]
POS_TOLERANCE = 15  # 位置回读容差(0~255 计数)


def _pick_port(cli_port=None):
    """返回要用的串口;自动检测与候选端口都不存在时返回 None。"""
    if cli_port:
        return cli_port
    env = os.environ.get("POUR_GRIPPER_PORT", "")
    if env:
        return env
    auto = _autodetect_port()
    if auto and auto != "auto":
        return auto
    for p in DEFAULT_PORTS:
        if os.path.exists(p):
            return p
    return None


def _hardware_available(cli_port=None):
    """无依赖或硬件(端口都不存在)时返回 False,用于 pytest skip。"""
    return _HAS_RQ and _pick_port(cli_port) is not None


needs_hardware = pytest.mark.skipif(
    not _hardware_available(),
    reason="缺少 pyrobotiqgripper 或未检测到夹爪串口(需要 /dev/ttyUSB* 或 FTDI by-id 链接)",
)


# ---------------------------------------------------------------------------
# pytest 模式
# ---------------------------------------------------------------------------
@needs_hardware
def test_gripper_full_sequence():
    """完整硬件序列:激活 → 开 → 合 → 半开 → 回到张开。"""
    port = _pick_port()
    gripper = RobotiqGripper(port=port)
    try:
        # 激活(已激活会自动跳过)
        gripper.activate()
        assert gripper.is_activated(), "activate 后 isActivated() 应为 True"

        # 张开:位置应接近 0
        gripper.open()
        pos_open = gripper.get_position()
        assert pos_open <= POS_TOLERANCE, f"open 后位置应接近 0,实际 {pos_open}"

        # 闭合:本机这台 2F-85 完全闭合时回读 ~230(请求 gPR=255,实际 gPO=230,
        # 属单元标定特性),所以断言放宽到"明显大于张开位置"
        gripper.close()
        pos_closed = gripper.get_position()
        assert pos_closed >= 200, f"close 后位置应远大于 open({pos_open}),实际 {pos_closed}"

        # 半开:move 128
        gripper.move(position=128)
        pos_mid = gripper.get_position()
        assert abs(pos_mid - 128) <= POS_TOLERANCE, (
            f"move 128 后位置应接近 128,实际 {pos_mid}")

        # 回到张开,结束保持松开状态
        gripper.move(position=0)
        pos_final = gripper.get_position()
        assert pos_final <= POS_TOLERANCE, f"move 0 后位置应接近 0,实际 {pos_final}"
    finally:
        gripper.disconnect()


# ---------------------------------------------------------------------------
# 脚本模式:完整序列 + 每步打印
# ---------------------------------------------------------------------------
def _run_sequence(port):
    print(f"=== Robotiq 2F-85 硬件测试 | 端口: {port} ===")
    gripper = RobotiqGripper(port=port)
    try:
        print("[1/5] 激活 ...")
        gripper.activate()
        print(f"      activated={gripper.is_activated()}")

        print("[2/5] 张开 ...")
        gripper.open()
        print(f"      position={gripper.get_position()}")

        print("[3/5] 闭合 ...")
        gripper.close()
        print(f"      position={gripper.get_position()}")

        print("[4/5] 半开 move 128 ...")
        gripper.move(position=128)
        print(f"      position={gripper.get_position()}")

        print("[5/5] 回到张开 move 0 ...")
        gripper.move(position=0)
        pos = gripper.get_position()
        print(f"      position={pos}")
        print(f"      status={gripper.get_status()}")

        ok = (
            gripper.is_activated()
            and pos <= POS_TOLERANCE
        )
        print(f"\n=== {'PASS ✅' if ok else 'FAIL ❌'} ===")
        return 0 if ok else 1
    finally:
        gripper.disconnect()
        print("已断开串口")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Robotiq 2F-85 夹爪硬件测试")
    parser.add_argument("--port", default=None,
                        help=f"夹爪串口(默认自动检测;候选: {', '.join(DEFAULT_PORTS)})")
    args = parser.parse_args()

    if not _HAS_RQ:
        print("❌ 缺少 pyrobotiqgripper(硬件测试请用 /opt/lerobot_venv/bin/python)",
              file=sys.stderr)
        sys.exit(1)

    port = _pick_port(args.port)
    if port is None:
        print("❌ 未检测到夹爪串口,请确认 USB-RS485 转接已连接", file=sys.stderr)
        sys.exit(1)
    sys.exit(_run_sequence(port))
