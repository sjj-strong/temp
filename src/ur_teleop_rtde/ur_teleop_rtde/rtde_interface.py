"""RTDE interface: connection lifecycle + 500 Hz servoJ control thread.

Wraps ur-rtde's RTDEControlInterface (commands) and RTDEReceiveInterface
(state readback) into a thread-safe object. The servo loop runs in a daemon
thread at servoJ rate (default 500 Hz); targets are updated from the ROS main
thread via set_target(); actual joints/TCP pose are read back periodically and
exposed via get_actual_joints()/get_actual_tcp_pose().

Gripper actuation (pyrobotiqgripper) is intentionally NOT part of this class —
teleop_node owns it. This class is pure RTDE.

Pattern follows gello_software/gello/robots/ur.py:
    t_start = initPeriod(); servoJ(q, vel, acc, dt, lookahead, gain); waitPeriod(t_start)
"""

import threading
from typing import Optional


class RTDEInterface:
    """Manages RTDE connections and the servoJ control-loop thread."""

    def __init__(self, robot_ip: str, config: dict):
        self._ip = robot_ip
        self._dt = float(config.get("servoj_dt", 0.002))
        self._velocity = float(config.get("servoj_velocity", 0.5))
        self._acceleration = float(config.get("servoj_acceleration", 0.5))
        self._lookahead = float(config.get("servoj_lookahead", 0.2))
        self._gain = float(config.get("servoj_gain", 100))
        self._state_read_every_n = int(config.get("state_read_every_n", 5))
        self._reconnect_attempts = int(config.get("reconnect_attempts", 3))
        self._reconnect_delay_s = float(config.get("reconnect_delay_s", 1.0))

        # ur-rtde imports are deferred — module must be importable without
        # rtde installed (unit tests, config-only tools).
        self._ctrl: Optional[object] = None      # RTDEControlInterface
        self._recv: Optional[object] = None      # RTDEReceiveInterface
        self._connected = False

        self._target_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._target_joints: Optional[list[float]] = None
        self._actual_joints: Optional[list[float]] = None
        self._actual_tcp_pose: Optional[list[float]] = None

        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._loop_error: Optional[Exception] = None

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Establish RTDE control + receive connections. Idempotent."""
        if self._connected:
            return True
        import rtde_control
        import rtde_receive
        try:
            self._ctrl = rtde_control.RTDEControlInterface(self._ip)
            self._recv = rtde_receive.RTDEReceiveInterface(self._ip)
        except Exception:
            self._ctrl = None
            self._recv = None
            self._connected = False
            return False
        self._connected = True
        return True

    def disconnect(self) -> None:
        """Stop servo loop and close RTDE connections."""
        self.stop_servo_loop()
        if self._ctrl is not None:
            try:
                self._ctrl.disconnect()
            except Exception:
                pass
        if self._recv is not None:
            try:
                self._recv.disconnect()
            except Exception:
                pass
        self._ctrl = None
        self._recv = None
        self._connected = False

    def reconnect(self) -> bool:
        """Disconnect then reconnect. Returns success."""
        self.disconnect()
        ok = self.connect()
        if ok:
            # Re-seed the target so the loop doesn't jump from a stale target.
            q = self.get_actual_joints()
            if q is not None:
                self.set_target(q)
        return ok

    def is_connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------
    # Target / state exchange (thread-safe)
    # ------------------------------------------------------------------

    def set_target(self, joints: list[float]) -> None:
        """Set the next servoJ target (called from ROS main thread)."""
        with self._target_lock:
            self._target_joints = list(joints)

    def get_target(self) -> Optional[list[float]]:
        with self._target_lock:
            return list(self._target_joints) if self._target_joints is not None else None

    def get_actual_joints(self) -> Optional[list[float]]:
        with self._state_lock:
            return list(self._actual_joints) if self._actual_joints is not None else None

    def get_actual_tcp_pose(self) -> Optional[list[float]]:
        with self._state_lock:
            return list(self._actual_tcp_pose) if self._actual_tcp_pose is not None else None

    def get_loop_error(self) -> Optional[Exception]:
        """Exception that killed the servo loop, if any (for watchdog)."""
        return self._loop_error

    def clear_loop_error(self) -> None:
        """Clear the recorded loop error after a successful recovery."""
        self._loop_error = None

    # ------------------------------------------------------------------
    # Servo loop
    # ------------------------------------------------------------------

    def start_servo_loop(self) -> None:
        """Spawn the daemon servoJ thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        if not self._connected:
            raise RuntimeError("RTDEInterface.connect() must succeed before start_servo_loop()")
        self._running = True
        self._loop_error = None
        self._thread = threading.Thread(target=self._servo_loop, daemon=True,
                                        name="ur_rtde_servoj")
        self._thread.start()

    def stop_servo_loop(self, timeout_s: float = 3.0) -> None:
        """Signal the loop to stop and join the thread."""
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=timeout_s)
        self._thread = None

    def move_j(self, joints: list[float], speed: float = 1.0, acceleration: float = 1.0) -> bool:
        """Blocking moveJ to a joint configuration (home phase). Returns success."""
        if self._ctrl is None:
            return False
        try:
            return bool(self._ctrl.moveJ(list(joints), speed, acceleration))
        except Exception:
            return False

    def servo_stop(self) -> None:
        """Gracefully stop servo motion on the UR controller."""
        if self._ctrl is not None:
            try:
                self._ctrl.servoStop()
            except Exception:
                pass

    def _servo_loop(self) -> None:
        """Run in dedicated daemon thread at servoJ rate."""
        try:
            # Seed target with current actual position to avoid a jump.
            q = self._recv.getActualQ()
            with self._target_lock:
                self._target_joints = list(q)
            with self._state_lock:
                self._actual_joints = list(q)

            cycle = 0
            while self._running:
                with self._target_lock:
                    target = list(self._target_joints) if self._target_joints else list(q)
                t_start = self._ctrl.initPeriod()
                self._ctrl.servoJ(
                    target, self._velocity, self._acceleration,
                    self._dt, self._lookahead, self._gain,
                )
                self._ctrl.waitPeriod(t_start)

                cycle += 1
                if cycle % self._state_read_every_n == 0:
                    try:
                        q = self._recv.getActualQ()
                        tcp = self._recv.getActualTCPPose()
                        with self._state_lock:
                            self._actual_joints = list(q)
                            self._actual_tcp_pose = list(tcp)
                    except Exception:
                        pass  # transient read failure — don't kill the loop
        except Exception as e:  # connection loss / interface errors
            self._loop_error = e
            self._running = False

    # ------------------------------------------------------------------
    # Direct one-shot queries (used by home_node / offset capture)
    # ------------------------------------------------------------------

    def read_actual_joints(self) -> Optional[list[float]]:
        """Blocking read of actual joints via the receive interface."""
        if self._recv is None:
            return None
        try:
            return list(self._recv.getActualQ())
        except Exception:
            return None

    def read_actual_tcp_pose(self) -> Optional[list[float]]:
        """Blocking read of actual TCP pose (list: x,y,z,rx,ry,rz)."""
        if self._recv is None:
            return None
        try:
            return list(self._recv.getActualTCPPose())
        except Exception:
            return None

    def probe_connection(self) -> bool:
        """Connect + disconnect without starting the loop (reachability check)."""
        if not self.connect():
            return False
        self.disconnect()
        return True
