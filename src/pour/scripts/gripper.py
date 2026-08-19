"""
Robotiq 2F-85 gripper wrapper.

适配 pyrobotiqgripper 3.2.6（本机 anaconda / Python 3.12）。
注意：3.2.6 的方法名与旧版 1.0.1 不同（move/position/status/disconnect 等），
本 wrapper 按 3.2.6 API 实现，对外保持与旧 wrapper 相同的接口，
assemble_force.py / assemble.py 等已有调用代码无需修改。

3.2.6 与旧版方法名对照：
    goTo(pos)            -> move(position=pos)
    goTomm(mm)           -> move_mm(positionmm=mm)
    getPosition()        -> position()
    readAll()/paramDic   -> status()
    self.serial.close()  -> disconnect()
    （activate / open / close 名字不变）

Interface:
    gripper = RobotiqGripper()        # 无参也可，自动选 FTDI 串口

    gripper.activate()                # 上电后激活一次
    gripper.open()
    gripper.close()
    gripper.move(position)            # 0(开)~255(合)
    gripper.move_mm(mm)               # 需要先做 mm 标定

    gripper.get_position()
    gripper.get_status()
    gripper.is_activated()
    gripper.disconnect()
"""


import os
import glob

import pyrobotiqgripper as rq


def _autodetect_port() -> str:
    """
    自动挑选夹爪串口。

    Robotiq 的 USB-RS485 转接（API-1005-03）用的是 FTDI FT232R，
    所以优先返回 by-id 名字里含 FTDI/FT232 的端口；找不到则退回
    第一个存在的 /dev/ttyUSB*；都没有就返回 'auto' 让库自己扫。
    """
    for link in glob.glob("/dev/serial/by-id/*"):
        name = os.path.basename(link)
        if "FTDI" in name and "FT232" in name:
            return os.path.realpath(link)
    for i in range(4):
        p = f"/dev/ttyUSB{i}"
        if os.path.exists(p):
            return p
    return "auto"


class RobotiqGripper:
    """
    High-level wrapper for Robotiq 2F-85.

    Low-level communication:
        pyrobotiqgripper 3.2.6 (Modbus RTU)

    Default:
        FTDI FT232R 串口自动识别（Robotiq USB-RS485 标配芯片）
    """

    def __init__(self, port: str = None, device_id: int = 9):
        """
        Initialize Robotiq gripper 并自动连接。

        Parameters
        ----------
        port:
            指定串口（如 '/dev/ttyUSB1'）。None 时自动挑选 FTDI 口。
        device_id:
            Modbus 从站 ID，Robotiq 出厂默认 9。
        """

        com_port = port or _autodetect_port()

        # 3.2.6 的 RobotiqGripper(...) 构造时即自动连接该端口
        self._gripper = rq.RobotiqGripper(com_port=com_port, device_id=device_id)

        self._activated = False


    def activate(self):
        """
        Activate gripper.

        This should be called once after power on. 已激活则跳过，避免重复校准。
        """

        if not self.is_activated():
            self._gripper.activate()
            self._activated = True


    def open(
        self,
        speed: int = 255,
        force: int = 255,
    ):
        """
        Open gripper.

        Parameters
        ----------
        speed:
            Gripper speed 0~255
        force:
            Gripper force 0~255
        """

        self._gripper.open(
            speed=speed,
            force=force,
            wait=True,
        )


    def close(
        self,
        speed: int = 255,
        force: int = 50,
    ):
        """
        Close gripper.

        Parameters
        ----------
        speed:
            Gripper speed 0~255
        force:
            Gripper force 0~255（默认 50，安全夹取不易夹坏物体）
        """

        self._gripper.close(
            speed=speed,
            force=force,
            wait=True,
        )


    def move(
        self,
        position: int,
        speed: int = 255,
        force: int = 255,
    ):
        """
        Move gripper to target position.

        Parameters
        ----------
        position:
            0(open) ~ 255(close)
        speed:
            0~255
        force:
            0~255
        """

        self._gripper.move(
            position,
            speed=speed,
            force=force,
            wait=True,
        )


    def move_mm(
        self,
        position_mm: float,
        speed: int = 255,
        force: int = 255,
    ):
        """
        Move gripper using millimeter.

        Requires mm calibration.
        """

        self._gripper.move_mm(
            position_mm,
            speed=speed,
            force=force,
            wait=True,
        )


    def get_position(self):
        """
        Get current gripper position.

        Returns
        -------
        int:
            0~255
        """

        return self._gripper.position()


    def get_status(self):
        """
        Get current gripper status.
        """

        return self._gripper.status()


    def is_activated(self):
        """
        Check activation state.
        """

        return self._gripper.isActivated()


    def disconnect(self):
        """
        Close communication.
        """

        self._gripper.disconnect()

        self._activated = False


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Robotiq 2F-85 夹爪命令行控制",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
示例:
  python gripper/gripper.py activate        # 上电后激活一次
  python gripper/gripper.py open            # 张开
  python gripper/gripper.py close           # 闭合
  python gripper/gripper.py move 128        # 走到指定位置 0(开)~255(合)
  python gripper/gripper.py move 128 -f 50  # 指定力 50
  python gripper/gripper.py status          # 查看状态
""",
    )
    parser.add_argument("--port", default=None, help="串口，如 /dev/ttyUSB1（默认自动选 FTDI 口）")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("activate", help="激活夹爪（上电后调用一次）")
    sub.add_parser("open", help="张开夹爪")
    sub.add_parser("close", help="闭合夹爪")
    p_move = sub.add_parser("move", help="移动到指定位置 0(开)~255(合)")
    p_move.add_argument("position", type=int, help="目标位置 0~255")
    sub.add_parser("status", help="读取并打印夹爪状态")

    for name in ("activate", "open", "close", "move"):
        p = sub.choices[name]
        p.add_argument("--speed", "-s", type=int, default=255, help="速度 0~255 (默认 255)")
        p.add_argument("--force", "-f", type=int, default=None, help="力 0~255 (默认: open/move=255, close=50)")

    args = parser.parse_args()

    gripper = RobotiqGripper(port=args.port)

    def _force(name_default):
        return args.force if args.force is not None else name_default

    try:
        if args.command == "activate":
            gripper.activate()
            print("activated")
        elif args.command == "status":
            print("Gripper status:", gripper.get_status())
            print("position:", gripper.get_position())
            print("activated:", gripper.is_activated())
        else:
            # open/close/move 前必须先激活，否则手指不动
            if not gripper.is_activated():
                print("夹爪未激活，先 activate ...")
                gripper.activate()
            if args.command == "open":
                gripper.open(speed=args.speed, force=_force(255))
                print("opened")
            elif args.command == "close":
                gripper.close(speed=args.speed, force=_force(50))
                print("closed")
            elif args.command == "move":
                gripper.move(position=args.position, speed=args.speed, force=_force(255))
                print(f"moved to {args.position}")
    finally:
        gripper.disconnect()
