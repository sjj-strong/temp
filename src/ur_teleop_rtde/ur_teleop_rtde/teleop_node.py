"""Teleop core (RTDE): home-verify → settle → offset → Enter → servoJ mirror.

Differs from ur_teleop/teleop_node.py in two ways:
  1. Control goes directly through RTDE servoJ (no controller_manager switching).
  2. Gripper actuation goes through pyrobotiqgripper (serial), not an action server.

Reusable logic (JointMapper / SessionOffset / GripperController / KeyboardReader /
FrameBuilder + joint-name constants) is vendored into this package — no ur_teleop
imports (requirement).

6-state FSM: INIT → VERIFY_HOME → SETTLING → CAPTURE_OFFSET → ARMED → ACTIVE
(+ SAFETY_HOLD on watchdog timeout or RTDE loop failure; auto-recovers).

Single-threaded executor; timers do the work; the 500 Hz servoJ loop runs in a
dedicated daemon thread inside RTDEInterface.
"""

import math
import sys
import threading
import time
from enum import Enum, auto
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray

from ur_teleop_rtde.config import default_config_path, load_config
from ur_teleop_rtde.constants import (
    ALICIA_JOINT_NAMES, GRIPPER_JOINT, UR_GRIPPER_JOINT, UR_JOINT_NAMES,
)
from ur_teleop_rtde.gripper_controller import GripperController, GripperTarget
from ur_teleop_rtde.gripper_interface import GripperInterface
from ur_teleop_rtde.joint_mapper import JointMapper
from ur_teleop_rtde.keyboard import KeyboardReader
from ur_teleop_rtde.offset import SessionOffset
from ur_teleop_rtde.rtde_interface import RTDEInterface


class State(Enum):
    INIT = auto()
    VERIFY_HOME = auto()
    SETTLING = auto()
    CAPTURE_OFFSET = auto()
    ARMED = auto()
    ACTIVE = auto()
    SAFETY_HOLD = auto()


def build_mapping_config(cfg: dict) -> dict:
    """mapping + safety merged into the shape JointMapper expects."""
    mapping = dict(cfg["mapping"])
    if "safety" not in mapping:
        mapping["safety"] = cfg["mapping"]["safety"]
    return mapping


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")
        self.declare_parameter("config_file", default_config_path())
        self.declare_parameter("mode", "")
        self.declare_parameter("force_home", False)
        cfg = load_config(self.get_parameter("config_file").value)
        if self.get_parameter("force_home").value:
            cfg = dict(cfg, home=dict(cfg["home"], at_home_tolerance_rad=float("inf")))
        self._cfg = cfg
        self._mode = self.get_parameter("mode").value or cfg["mode"]
        self._command_rate = float(cfg["teleop"].get("command_rate_hz", 100))
        self._state_rate = float(cfg["teleop"].get("state_publish_rate_hz", 50))
        self._watchdog_timeout = float(cfg["teleop"].get("watchdog_timeout_s", 0.5))

        self._state = State.INIT
        self._start_time = time.time()
        self._fatal_error = False
        self._e_stop = False
        self._enable_pending = False
        self._master_engaged = False
        self._last_master_stamp = 0.0
        self._master_q: Optional[list[float]] = None
        self._master_gripper_m = 0.0
        self._lock = threading.Lock()

        self._mapper: Optional[JointMapper] = None
        self._offset = SessionOffset()
        self._gripper_fsm = GripperController(cfg.get("gripper", {}))
        self._gripper_hw = GripperInterface(
            cfg["gripper"].get("com_port", "/dev/ttyUSB1"),
            speed=cfg["gripper"].get("speed", 255),
            force=cfg["gripper"].get("force", 50),
        )
        self._gripper_connected = False
        self._gripper_target_pending: Optional[GripperTarget] = None
        self._kb = KeyboardReader()

        self._rtde = RTDEInterface(cfg["robot"]["robot_ip"], cfg.get("rtde", {}))
        self._rtde_hold_target: Optional[list[float]] = None

        self._joint_sub = self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self._enable_sub = self.create_subscription(Bool, "/teleop/enable", self._enable_cb, 10)
        self._estop_sub = self.create_subscription(Bool, "/teleop/e_stop", self._estop_cb, 10)

        self._ur_joints_pub = self.create_publisher(JointState, "/ur_teleop_rtde/ur_joints", 10)
        self._tcp_pose_pub = self.create_publisher(PoseStamped, "/ur_teleop_rtde/tcp_pose", 10)
        self._gripper_state_pub = self.create_publisher(Bool, "/ur_teleop_rtde/gripper_state", 10)
        self._status_pub = self.create_publisher(Bool, "/ur_teleop_rtde/status", 10)
        self._target_pub = self.create_publisher(
            Float64MultiArray, "/ur_teleop_rtde/target_joints", 10)
        self._cmd_pub = self.create_publisher(Float64MultiArray, "/teleop/commands", 10)
        self._demo_pub = self.create_publisher(Bool, "/demonstration", 10)
        # 实时 rviz：夹爪实际位置写回 /joint_states（仅含 robotiq_85_* 关节，
        # 与 Alicia / UR driver 的发布互不冲突，rsp 按关节名合并）
        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)

        self._timer = self.create_timer(1.0 / self._command_rate, self._tick)
        self._state_timer = self.create_timer(1.0 / self._state_rate, self._publish_ur_state)
        self._gripper_timer = self.create_timer(1.0 / 10.0, self._gripper_tick)

        self._settle_start = 0.0
        self._last_pose: Optional[list[float]] = None
        self._status_was_active = False

    @property
    def fatal_error(self) -> bool:
        return self._fatal_error

    # ---------- callbacks ----------

    def _joint_cb(self, msg: JointState):
        # 只处理 Alicia（主臂）数据；UR 状态全部走 RTDE，不依赖 UR ROS2 driver。
        names = set(msg.name)
        if all(n in names for n in ALICIA_JOINT_NAMES):
            with self._lock:
                self._master_q = [msg.position[msg.name.index(n)] for n in ALICIA_JOINT_NAMES]
                if GRIPPER_JOINT in names:
                    self._master_gripper_m = msg.position[msg.name.index(GRIPPER_JOINT)]
                self._master_engaged = True
                self._last_master_stamp = time.time()

    def _enable_cb(self, msg: Bool):
        if msg.data and self._state == State.ARMED:
            self.get_logger().info("[teleop] enable 收到 — 开始控制")
            self._begin_active()
        elif msg.data:
            self._enable_pending = True

    def _estop_cb(self, msg: Bool):
        self._e_stop = msg.data
        self.get_logger().warn(f"[teleop] e_stop={'ON' if msg.data else 'OFF'}")

    # ---------- state machine ----------

    def _tick(self):
        if self._e_stop:
            return  # 冻结：关节指令与夹爪 FSM 均暂停
        if self._state == State.INIT:
            self._init_state()
        elif self._state == State.VERIFY_HOME:
            self._verify_home()
        elif self._state == State.SETTLING:
            self._settling()
        elif self._state == State.CAPTURE_OFFSET:
            self._capture_offset()
        elif self._state == State.ARMED:
            self._armed()
        elif self._state == State.ACTIVE:
            self._active()
        elif self._state == State.SAFETY_HOLD:
            self._safety_hold()

    def _log_state(self, new: State):
        self._state = new
        self.get_logger().info(f"[teleop] 状态: {new.name}")

    def _slave_state(self) -> Optional[list[float]]:
        """UR 实际关节（RTDE getActualQ，不依赖 /joint_states）。"""
        return self._rtde.read_actual_joints()

    def _init_state(self):
        if time.time() - self._start_time > 30.0:
            self._fatal_error = True
            self.get_logger().error(
                "30 s 内未检测到数据（Alicia /joint_states 或 RTDE 连接）。请先启动 home.launch 与 Alicia 驱动。"
            )
            rclpy.try_shutdown()
            return
        with self._lock:
            master_ok = self._master_q is not None
        if master_ok and self._rtde.probe_connection():
            self.get_logger().info(f"[teleop] RTDE 连通 ({self._cfg['robot']['robot_ip']})")
            self._log_state(State.VERIFY_HOME)

    def _verify_home(self):
        home = self._cfg["home"]
        tol = float(home.get("at_home_tolerance_rad", 0.05))
        slave_q = self._slave_state()
        if slave_q is None:
            return  # RTDE 尚未连接/读取
        with self._lock:
            m_err = max(abs(a - b) for a, b in zip(self._master_q, home["master"]))
        s_err = max(abs(a - b) for a, b in zip(slave_q, home["slave"]))
        if m_err <= tol and s_err <= tol:
            self._settle_start = time.time()
            self._last_pose = None
            self._log_state(State.SETTLING)
        else:
            self.get_logger().warn(
                f"[teleop] 双臂不在 home（master err={m_err:.3f} rad, slave err={s_err:.3f} rad），"
                f"请先运行 home.launch；确认已到位可用 force_home:=true 跳过"
            )

    def _settling(self):
        settle = float(self._cfg["home"].get("settle_time_s", 2.0))
        thresh = float(self._cfg["home"].get("settle_motion_threshold_rad", 0.01))
        slave_q = self._slave_state()
        with self._lock:
            if self._master_q is None:
                return
            pose = list(self._master_q) + list(slave_q or [0.0] * 6)
        if self._last_pose is None:
            self._last_pose = pose
            return
        motion = max(abs(a - b) for a, b in zip(pose, self._last_pose))
        self._last_pose = pose
        if motion > thresh:
            self._settle_start = time.time()
        elif time.time() - self._settle_start >= settle:
            self._log_state(State.CAPTURE_OFFSET)

    def _capture_offset(self):
        slave_q = self._slave_state()
        if slave_q is None:
            return  # RTDE 未就绪，重试
        with self._lock:
            self._offset.capture(list(self._master_q), slave_q)
        self._mapper = JointMapper(
            build_mapping_config(self._cfg), self._offset.master_home, self._offset.slave_home
        )
        # Real RTDE connection (the probe disconnected at INIT).
        if not self._rtde.connect():
            self.get_logger().error("[teleop] RTDE 正式连接失败，无法继续")
            self._fatal_error = True
            rclpy.try_shutdown()
            return
        # Connect gripper (optional — teleop works without it).
        if self._gripper_fsm.enabled:
            self._gripper_connected = self._gripper_hw.connect()
            if not self._gripper_connected:
                self.get_logger().warn("[teleop] 夹爪连接失败，夹爪 FSM 禁用")
                self._gripper_fsm.enabled = False
        self.get_logger().info(
            f"[teleop] offset 捕获完成 master={self._offset.master_home} "
            f"slave={self._offset.slave_home}"
        )
        if self._mode == "record":
            self.get_logger().info("[teleop] 等待 recorder 的 Enter（/teleop/enable）...")
        else:
            self.get_logger().info("[teleop] 按 Enter 开始控制")
        self._log_state(State.ARMED)

    def _armed(self):
        if self._enable_pending:
            self._enable_pending = False
            self.get_logger().info("[teleop] enable 已在 ARMED 前收到 — 开始控制")
            self._begin_active()
            return
        if self._mode == "teleop" and self._kb.read_key(0.0) == "enter":
            self.get_logger().info("[teleop] Enter 按下 — 开始")
            self._begin_active()

    def _begin_active(self):
        # Seed servo target with current position to avoid a jump.
        cur = self._rtde.read_actual_joints() or [0.0] * 6
        self._rtde_hold_target = list(cur)
        self._rtde.set_target(cur)
        self._rtde.start_servo_loop()
        self._publish_demo(True)
        self._publish_status(True)
        self._log_state(State.ACTIVE)

    def _active(self):
        if self._rtde.get_loop_error() is not None:
            self.get_logger().error("[teleop] RTDE 控制线程异常 → SAFETY_HOLD")
            self._enter_safety_hold()
            return
        if time.time() - self._last_master_stamp > self._watchdog_timeout:
            self.get_logger().warn("[teleop] 主臂数据超时 → SAFETY_HOLD（保持当前位置）")
            self._enter_safety_hold()
            return
        with self._lock:
            if self._master_q is None:
                return
            cmd = self._mapper.master_to_slave(self._master_q)
        self._rtde.set_target(cmd)
        self._publish_commands(cmd)

    def _enter_safety_hold(self):
        self._rtde_hold_target = self._rtde.get_target() or self._rtde.read_actual_joints()
        self._publish_status(False)
        self._log_state(State.SAFETY_HOLD)

    def _safety_hold(self):
        if self._rtde.get_loop_error() is not None:
            # Try to recover the connection; stay in hold until it succeeds.
            ok = self._rtde.reconnect()
            if ok:
                self._rtde.clear_loop_error()
                self._rtde.start_servo_loop()
                self.get_logger().info("[teleop] RTDE 重连成功，继续保持")
            else:
                self.get_logger().error("[teleop] RTDE 重连失败，仍在 SAFETY_HOLD")
                return
        hold = self._rtde_hold_target
        if hold is not None:
            self._rtde.set_target(hold)  # 保持最后有效位置
        if time.time() - self._last_master_stamp <= self._watchdog_timeout:
            self.get_logger().info("[teleop] 主臂数据恢复 → ACTIVE")
            self._publish_status(True)
            self._log_state(State.ACTIVE)

    # ---------- gripper FSM ----------

    def _gripper_tick(self):
        if self._e_stop or not self._gripper_fsm.enabled:
            return
        if not self._gripper_connected:
            return
        with self._lock:
            target = self._gripper_fsm.update(self._master_gripper_m)
        if target == GripperTarget.UNKNOWN:
            return
        # Don't send a new command while the previous one is still moving.
        if self._gripper_target_pending is not None:
            return
        self._gripper_target_pending = target
        if target == GripperTarget.CLOSED:
            self._gripper_hw.close()
        else:
            self._gripper_hw.open()
        self._gripper_target_pending = None

    # ---------- state publishing ----------

    def _publish_ur_state(self):
        joints = self._rtde.get_actual_joints()
        tcp = self._rtde.get_actual_tcp_pose()
        target = self._rtde.get_target()
        gripper_pos = self._gripper_hw.position() if self._gripper_connected else None

        if joints is not None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(UR_JOINT_NAMES)
            msg.position = list(joints)
            self._ur_joints_pub.publish(msg)

        if tcp is not None:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "base_link"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = tcp[:3]
            # RPY → quaternion (UR reports pose as [x,y,z,rx,ry,rz]).
            rx, ry, rz = tcp[3:]
            cx, cy, cz = (math.cos(rx / 2), math.cos(ry / 2), math.cos(rz / 2))
            sx, sy, sz = (math.sin(rx / 2), math.sin(ry / 2), math.sin(rz / 2))
            pose.pose.orientation.w = cx * cy * cz + sx * sy * sz
            pose.pose.orientation.x = sx * cy * cz - cx * sy * sz
            pose.pose.orientation.y = cx * sy * cz + sx * cy * sz
            pose.pose.orientation.z = cx * cy * sz - sx * sy * cz
            self._tcp_pose_pub.publish(pose)

        # Gripper state: 1=open, 0=closed (from actual gripper position if
        # available, else from the hysteresis FSM target).
        if gripper_pos is not None:
            open_state = gripper_pos > 0.5
        else:
            open_state = self._gripper_fsm.current_target != GripperTarget.CLOSED
        self._gripper_state_pub.publish(Bool(data=open_state))

        # 实时 rviz 夹爪动画：position() 归一化 0-1（1=开）→ 2F-85 knuckle rad
        # （0.0=全开, 0.7929=全闭，与工作区动作服务器同一定义）。
        if gripper_pos is not None:
            knuckle = 0.7929 * (1.0 - gripper_pos)
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [UR_GRIPPER_JOINT]
            js.position = [knuckle]
            self._js_pub.publish(js)

        if target is not None:
            t = Float64MultiArray()
            t.data = list(target)
            self._target_pub.publish(t)

    # ---------- helpers ----------

    def _publish_commands(self, cmd: list[float]):
        tcmd = Float64MultiArray()
        tcmd.data = list(cmd) + [
            self._gripper_fsm.get_gripper_command_signal(self._gripper_fsm.current_target)
        ]
        self._cmd_pub.publish(tcmd)

    def _publish_status(self, active: bool):
        self._status_pub.publish(Bool(data=active))

    def _publish_demo(self, on: bool):
        self._demo_pub.publish(Bool(data=on))

    def shutdown(self):
        """Ctrl-C 退出流程：停 servo → 断连 → 恢复 Alicia 力矩。"""
        self._publish_demo(False)
        self._publish_status(False)
        self._rtde.servo_stop()
        self._rtde.disconnect()
        if self._gripper_connected:
            self._gripper_hw.disconnect()
            self._gripper_connected = False


def main():
    rclpy.init()
    node = TeleopNode()
    node.get_logger().info("=" * 60)
    node.get_logger().info(
        f"ur_teleop_rtde 就绪 — mode={node._mode}, ip={node._cfg['robot']['robot_ip']}"
    )
    node.get_logger().info("  等待双臂到位 → 静止 → offset → Enter 开始控制")
    node.get_logger().info("=" * 60)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if node.fatal_error else 0


if __name__ == "__main__":
    sys.exit(main())
