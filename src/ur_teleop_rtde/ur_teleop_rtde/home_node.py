"""Stage 1: move both arms to their configured home joints (one-shot).

UR side: RTDE moveJ (blocking call in a background thread with timeout).
Alicia side: continuous /joint_commands publishing until verified.
Verifies arrival, prints HOME REACHED, exits 0/1.

Single-threaded: all waiting is done by polling an executor.spin_once in the
main loop — no background spin thread, no nested spin_until_future_complete.
"""

import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ur_teleop_rtde.config import default_config_path, load_config
from ur_teleop_rtde.constants import ALICIA_JOINT_NAMES, GRIPPER_JOINT
from ur_teleop_rtde.rtde_interface import RTDEInterface


class HomeNode(Node):
    def __init__(self):
        super().__init__("home_node")
        self.declare_parameter("config_file", default_config_path())
        cfg = load_config(self.get_parameter("config_file").value)
        home = cfg["home"]
        self._master_home = list(home["master"])
        self._slave_home = list(home["slave"])
        self._gripper_value = float(home.get("master_gripper_value", 1000.0))
        self._tolerance = float(home.get("at_home_tolerance_rad", 0.05))
        self._move_timeout = float(home.get("move_timeout_s", 30.0))
        self._move_speed = float(cfg.get("home", {}).get("move_speed_rad_s", 1.0))
        self._move_accel = float(cfg.get("home", {}).get("move_accel_rad_s2", 1.0))
        self._verify_duration = float(home.get("verify_duration_s", 2.0))
        self._alicia_q: Optional[list[float]] = None
        self._alicia_stamp = 0.0

        self._joint_sub = self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self._cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)

        self._rtde = RTDEInterface(cfg["robot"]["robot_ip"], cfg.get("rtde", {}))

    def _joint_cb(self, msg: JointState):
        names = set(msg.name)
        if all(n in names for n in ALICIA_JOINT_NAMES):
            self._alicia_q = [msg.position[msg.name.index(n)] for n in ALICIA_JOINT_NAMES]
            self._alicia_stamp = time.time()

    def cell_ready(self) -> bool:
        """Alicia /joint_states 可见 + RTDE 可连接."""
        return self._alicia_q is not None and self._rtde.probe_connection()

    def publish_alicia_home(self):
        msg = JointState()
        msg.name = ALICIA_JOINT_NAMES + [GRIPPER_JOINT]
        msg.position = self._master_home + [self._gripper_value]   # 0-1000，1000=开
        self._cmd_pub.publish(msg)

    def send_ur_home(self) -> bool:
        """RTDE moveJ 到 slave home。moveJ 是阻塞调用 → 后台线程 + 超时。"""
        result: dict = {}

        def _move():
            try:
                result["ok"] = self._rtde.move_j(
                    list(self._slave_home), self._move_speed, self._move_accel
                )
            except Exception as e:
                result["err"] = e

        self.get_logger().info(f"UR moveJ home -> {self._slave_home}")
        if not self._rtde.connect():
            self.get_logger().error("RTDE 连接失败")
            return False
        t = threading.Thread(target=_move, daemon=True)
        t.start()
        t.join(timeout=self._move_timeout)
        if t.is_alive():
            self._log_home_failure(f"UR moveJ 超时（{self._move_timeout}s）")
            return False
        if result.get("err") is not None:
            self._log_home_failure(f"UR moveJ 异常: {result['err']}")
            return False
        if not result.get("ok", False):
            self._log_home_failure("UR moveJ 返回 False")
            return False
        return True

    def _log_home_failure(self, reason: str):
        current = self._rtde.read_actual_joints()
        self.get_logger().error(
            f"{reason}. 目标={self._slave_home}, "
            f"当前={current if current is not None else '无 RTDE 读数'}. 请检查机器人。"
        )

    def at_home(self) -> bool:
        m = self._alicia_q
        s = self._rtde.read_actual_joints()
        if m is None or s is None:
            return False
        m_err = max(abs(a - b) for a, b in zip(m, self._master_home))
        s_err = max(abs(a - b) for a, b in zip(s, self._slave_home))
        return m_err <= self._tolerance and s_err <= self._tolerance

    def shutdown(self):
        self._rtde.disconnect()


def main():
    rclpy.init()
    node = HomeNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    rc = 0
    try:
        node.get_logger().info("等待 Alicia /joint_states + RTDE 连接就绪...")
        deadline = time.time() + 30.0
        while rclpy.ok() and not node.cell_ready() and time.time() < deadline:
            executor.spin_once(timeout_sec=0.5)
        if not node.cell_ready():
            node.get_logger().error("未就绪：Alicia /joint_states 或 RTDE 连接不可用。")
            rc = 1
        elif not node.send_ur_home():
            rc = 1
        else:
            node.get_logger().info(f"Alicia home -> {node._master_home}（夹爪开）")
            deadline = time.time() + node._move_timeout
            ok_verified = False
            hold_since = None
            while rclpy.ok() and time.time() < deadline:
                executor.spin_once(timeout_sec=0.05)
                node.publish_alicia_home()          # 持续命令，直到到位
                if node.at_home():
                    hold_since = hold_since if hold_since is not None else time.time()
                    if time.time() - hold_since >= node._verify_duration:
                        ok_verified = True
                        break
                else:
                    hold_since = None
            if not ok_verified:
                m = node._alicia_q
                s = node._rtde.read_actual_joints()
                node.get_logger().error(
                    f"到位超时（未在 move_timeout 内验证双臂位于 home）。"
                    f"master 目标={node._master_home}, "
                    f"当前={m if m is not None else '无 /joint_states'}; "
                    f"UR 目标={node._slave_home}, "
                    f"当前={s if s is not None else '无 RTDE 读数'}. "
                    "可用 teleop.launch force_home:=true 跳过验证。"
                )
                rc = 1
            else:
                node.get_logger().info("=" * 50)
                node.get_logger().info("HOME REACHED — 双臂已到位。现在运行 teleop.launch（阶段 2）。")
                node.get_logger().info("=" * 50)
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
