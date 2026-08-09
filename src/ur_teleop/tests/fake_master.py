"""Test-only fake Alicia master (spec §10) — NOT installed, lives in tests/.

Publishes /joint_states at 50 Hz: home positions (zeros) until teleop_node
reports /teleop/status==true (ACTIVE), then sine waves; --off-home publishes
[0.5]*6 forever (VERIFY_HOME rejection tests).

Deviation from the brief's "home for 12 s then sine": the mock cell needs
~20+ s to become ready, so a fixed 12 s home phase expired before
teleop_node's VERIFY_HOME ran. Triggering the sine on /teleop/status==true
keeps the master at home through VERIFY_HOME → SETTLING → CAPTURE_OFFSET
(which all require a static master) and starts motion only once teleop is
ACTIVE — deterministic regardless of cell startup time.
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
AMPL = [0.3, 0.2, 0.4, 0.3, 0.5, 0.2]
FREQ = [0.5, 0.4, 0.6, 0.7, 0.8, 0.9]


class FakeMaster(Node):
    def __init__(self, off_home: bool):
        super().__init__("fake_master")
        self._off_home = off_home
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self._moving = off_home
        self._moving_t0 = time.time()
        self.create_subscription(Bool, "/teleop/status", self._status_cb, 10)
        self.create_timer(0.02, self._tick)  # 50 Hz

    def _status_cb(self, msg: Bool):
        if msg.data and not self._moving:
            self._moving = True
            self._moving_t0 = time.time()

    def _tick(self):
        if self._off_home:
            q = [0.5] * 6
        elif not self._moving:
            q = HOME
        else:
            t = time.time() - self._moving_t0
            q = [HOME[i] + AMPL[i] * math.sin(2 * math.pi * FREQ[i] * t)
                 for i in range(6)]
        msg = JointState()
        msg.name = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6", "Gripper"]
        gripper_m = 0.025 if int(time.time() / 5) % 2 == 1 else 0.0
        msg.position = list(q) + [gripper_m]
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = FakeMaster(off_home="--off-home" in sys.argv)
    rclpy.spin(node)
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
