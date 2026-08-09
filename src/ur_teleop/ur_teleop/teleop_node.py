"""Teleop core: state machine home-verify → settle → offset → Enter → mirror.

Single-threaded executor; timers do the work; nothing blocks; the controller
switch is an async chain polled by the state machine (spec §5, 问题 7/10).
"""

import sys
import threading
import time
from enum import Enum, auto

import rclpy
from control_msgs.action import ParallelGripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray

from ur_teleop.config import (
    ALICIA_JOINT_NAMES,
    GRIPPER_JOINT,
    UR_GRIPPER_JOINT,
    UR_JOINT_NAMES,
    default_config_path,
    load_config,
)
from ur_teleop.controller_switcher import ControllerSwitcher
from ur_teleop.gripper_controller import GripperController, GripperTarget
from ur_teleop.joint_mapper import JointMapper
from ur_teleop.keyboard import KeyboardReader
from ur_teleop.offset import SessionOffset


class State(Enum):
    WAITING_CELL = auto()
    VERIFY_HOME = auto()
    SETTLING = auto()
    CAPTURE_OFFSET = auto()
    ARMED = auto()
    SWITCHING = auto()
    ACTIVE = auto()
    INACTIVE = auto()


def build_mapping_config(cfg: dict) -> dict:
    """mapping + safety merged into the shape JointMapper expects."""
    return dict(
        cfg["mapping"],
        safety={
            "clamp_margin_rad": cfg["safety"]["clamp_margin_rad"],
            "limits": cfg["safety"]["limits"],
        },
    )


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
        self._mode = self.get_parameter("mode").value or cfg["mode"]   # launch 参数优先 yaml 兜底
        self._command_rate = float(cfg["teleop"].get("command_rate_hz", 50))
        self._watchdog_timeout = float(cfg["teleop"].get("watchdog_timeout_s", 0.5))
        self._restore_on_exit = bool(cfg["teleop"].get("restore_controller_on_exit", True))
        self._traj_ctrl = "scaled_joint_trajectory_controller"
        self._fwd_ctrl = "forward_position_controller"

        self._state = State.WAITING_CELL
        self._start_time = time.time()
        self._fatal_error = False
        self._e_stop = False
        self._master_engaged = False
        self._last_master_stamp = 0.0
        self._master_q: list[float] | None = None
        self._master_gripper_m = 0.0
        self._slave_q: list[float] | None = None
        self._slave_gripper_rad = 0.0
        self._lock = threading.Lock()

        self._mapper: JointMapper | None = None
        self._offset = SessionOffset()
        self._gripper = GripperController(cfg.get("gripper", {}))
        self._gripper_probed = False
        self._gripper_future = None
        self._switcher = ControllerSwitcher(self)
        self._switch_future = None
        self._switch_phase = ""
        self._switch_attempt = 0
        self._kb = KeyboardReader()

        self._joint_sub = self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self._enable_sub = self.create_subscription(Bool, "/teleop/enable", self._enable_cb, 10)
        self._estop_sub = self.create_subscription(Bool, "/teleop/e_stop", self._estop_cb, 10)
        self._fwd_pub = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
        self._cmd_pub = self.create_publisher(Float64MultiArray, "/teleop/commands", 10)
        self._status_pub = self.create_publisher(Bool, "/teleop/status", 10)
        self._demo_pub = self.create_publisher(Bool, "/demonstration", 10)
        self._gripper_action = (
            ActionClient(self, ParallelGripperCommand, self._gripper.action_server)
            if self._gripper.enabled else None
        )

        self._timer = self.create_timer(1.0 / self._command_rate, self._tick)
        if self._gripper.enabled:
            self._gripper_timer = self.create_timer(
                1.0 / float(cfg.get("gripper", {}).get("fsm_rate_hz", 10.0)), self._gripper_tick
            )
        self._settle_start = 0.0
        self._last_pose: list[float] | None = None

    @property
    def fatal_error(self) -> bool:
        return self._fatal_error

    # ---------- callbacks ----------

    def _joint_cb(self, msg: JointState):
        names = set(msg.name)
        if all(n in names for n in ALICIA_JOINT_NAMES):
            with self._lock:
                self._master_q = [msg.position[msg.name.index(n)] for n in ALICIA_JOINT_NAMES]
                if GRIPPER_JOINT in names:
                    self._master_gripper_m = msg.position[msg.name.index(GRIPPER_JOINT)]
                self._master_engaged = True
                self._last_master_stamp = time.time()
        if all(n in names for n in UR_JOINT_NAMES):
            with self._lock:
                self._slave_q = [msg.position[msg.name.index(n)] for n in UR_JOINT_NAMES]
                if UR_GRIPPER_JOINT in names:
                    self._slave_gripper_rad = msg.position[msg.name.index(UR_GRIPPER_JOINT)]

    def _enable_cb(self, msg: Bool):
        if msg.data and self._state == State.ARMED:
            self.get_logger().info("[teleop] enable 收到 — 开始控制")
            self._begin_switch()
        elif msg.data:
            self.get_logger().info(f"[teleop] enable 收到但状态为 {self._state.name}，忽略")

    def _estop_cb(self, msg: Bool):
        self._e_stop = msg.data
        self.get_logger().warn(f"[teleop] e_stop={'ON' if msg.data else 'OFF'}")

    # ---------- state machine ----------

    def _tick(self):
        if self._e_stop:
            return                                            # 冻结：关节指令与夹爪 FSM 均暂停
        if self._state == State.WAITING_CELL:
            if time.time() - self._start_time > 30.0:
                self._fatal_error = True
                self.get_logger().error(
                    "30 s 内未检测到 cell（/joint_states + controller_manager）。"
                    "请先运行 home.launch。"
                )
                rclpy.try_shutdown()
                return
            with self._lock:
                cell_ok = self._slave_q is not None and self._master_q is not None
            if cell_ok and self._switcher.services_ready():
                self._log_state(State.VERIFY_HOME)
        elif self._state == State.VERIFY_HOME:
            self._verify_home()
        elif self._state == State.SETTLING:
            self._settling()
        elif self._state == State.CAPTURE_OFFSET:
            self._capture_offset()
        elif self._state == State.ARMED:
            self._armed()
        elif self._state == State.SWITCHING:
            self._switching()
        elif self._state == State.ACTIVE:
            self._active()
        elif self._state == State.INACTIVE:
            self._inactive()

    def _log_state(self, new: State):
        self._state = new
        self.get_logger().info(f"[teleop] 状态: {new.name}")

    def _verify_home(self):
        home = self._cfg["home"]
        tol = float(home.get("at_home_tolerance_rad", 0.05))
        with self._lock:
            m_err = max(abs(a - b) for a, b in zip(self._master_q, home["master"]))
            s_err = max(abs(a - b) for a, b in zip(self._slave_q, home["slave"]))
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
        with self._lock:
            pose = list(self._master_q) + list(self._slave_q)
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
        with self._lock:
            self._offset.capture(list(self._master_q), list(self._slave_q))
        self._mapper = JointMapper(
            build_mapping_config(self._cfg), self._offset.master_home, self._offset.slave_home
        )
        self.get_logger().info(
            f"[teleop] offset 捕获完成 master={self._offset.master_home} slave={self._offset.slave_home}"
        )
        if self._mode == "record":
            self.get_logger().info("[teleop] 等待 recorder 的 Enter（/teleop/enable）...")
        else:
            self.get_logger().info("[teleop] 按 Enter 开始控制")
        self._log_state(State.ARMED)

    def _armed(self):
        if self._mode == "teleop" and self._kb.read_key(0.0) == "enter":
            self.get_logger().info("[teleop] Enter 按下 — 开始")
            self._begin_switch()

    def _begin_switch(self):
        self._switch_attempt = 0
        self._switch_phase = "list"
        self._switch_future = self._switcher.list_controllers()
        self._log_state(State.SWITCHING)

    def _switching(self):
        fut = self._switch_future
        if fut is None:
            self.get_logger().error("controller_manager 服务未就绪，无法切换")
            self._log_state(State.ARMED)
            return
        if not fut.done():
            return
        if self._switch_phase == "list":
            controllers = ControllerSwitcher.list_result(fut)
            if self._fwd_ctrl not in controllers:
                self._switch_phase = "load"
                self._switch_future = self._switcher.load_controller(self._fwd_ctrl)
            else:
                self._switch_phase = "switch"
                self._switch_future = self._switcher.switch([self._fwd_ctrl], [self._traj_ctrl])
        elif self._switch_phase == "load":
            if fut.result() is not None and fut.result().ok:
                self._switch_phase = "switch"
                self._switch_future = self._switcher.switch([self._fwd_ctrl], [self._traj_ctrl])
            else:
                self.get_logger().error("加载 forward_position_controller 失败")
                self._log_state(State.ARMED)
        elif self._switch_phase == "switch":
            if ControllerSwitcher.switch_ok(fut):
                self.get_logger().info("[teleop] 控制器切换完成 → ACTIVE")
                self._publish_demo(True)
                self._publish_status(True)
                self._log_state(State.ACTIVE)
            else:
                self._switch_attempt += 1
                if self._switch_attempt >= 5:
                    self.get_logger().error("控制器切换 5 次失败，回到 ARMED；检查 controller_manager")
                    self._log_state(State.ARMED)
                else:
                    self.get_logger().warn(f"[teleop] 切换失败（第 {self._switch_attempt} 次），重试...")
                    self._switch_future = self._switcher.switch([self._fwd_ctrl], [self._traj_ctrl])

    def _active(self):
        if time.time() - self._last_master_stamp > self._watchdog_timeout:
            self.get_logger().warn("[teleop] 主臂数据超时 → INACTIVE（改发当前位置）")
            self._publish_status(False)
            self._log_state(State.INACTIVE)
            return
        with self._lock:
            cmd = self._mapper.master_to_slave(self._master_q)
        self._publish_commands(cmd)

    def _inactive(self):
        with self._lock:
            if time.time() - self._last_master_stamp <= self._watchdog_timeout:
                self.get_logger().info("[teleop] 主臂恢复 → ACTIVE")
                self._publish_status(True)
                self._log_state(State.ACTIVE)
                return
            hold = list(self._slave_q) if self._slave_q else [0.0] * 6
        self._publish_commands(hold)                        # 发当前位置避免跳变

    # ---------- gripper FSM ----------

    def _gripper_tick(self):
        if self._e_stop or not self._gripper.enabled:
            return
        if not self._gripper_probed:
            self._gripper_probed = True
            if self._gripper_action is None or not self._gripper_action.server_is_available():
                self.get_logger().warn("[teleop] 夹爪 action server 不存在，禁用夹爪 FSM")
                self._gripper.enabled = False
                return
        if self._gripper_future is not None and not self._gripper_future.done():
            return                                          # 上一个 goal 未完成，跳过本 tick
        with self._lock:
            target = self._gripper.update(self._master_gripper_m)
        if target != GripperTarget.UNKNOWN:
            self._send_gripper_goal(target)

    def _send_gripper_goal(self, target: GripperTarget):
        goal = ParallelGripperCommand.Goal()
        goal.command.name = [UR_GRIPPER_JOINT]
        goal.command.position = [self._gripper.get_knuckle_command(target)]
        goal.command.effort = [self._gripper.max_effort]
        self._gripper_future = self._gripper_action.send_goal_async(goal)

    # ---------- helpers ----------

    def _publish_commands(self, cmd: list[float]):
        fwd = Float64MultiArray()
        fwd.data = list(cmd)
        self._fwd_pub.publish(fwd)
        tcmd = Float64MultiArray()
        tcmd.data = list(cmd) + [self._gripper.get_gripper_command_signal(self._gripper.current_target)]
        self._cmd_pub.publish(tcmd)

    def _publish_status(self, active: bool):
        self._status_pub.publish(Bool(data=active))

    def _publish_demo(self, on: bool):
        self._demo_pub.publish(Bool(data=on))

    def shutdown(self):
        """Ctrl-C 退出流程：恢复力矩 → 切回 trajectory controller（spec §5）。"""
        self._publish_demo(False)
        if self._restore_on_exit and self._state in (State.ACTIVE, State.INACTIVE, State.SWITCHING):
            fut = self._switcher.switch([self._traj_ctrl], [self._fwd_ctrl])
            if fut is not None:
                deadline = time.time() + 5.0
                while not ControllerSwitcher.switch_ok(fut) and time.time() < deadline:
                    time.sleep(0.1)


def main():
    rclpy.init()
    node = TeleopNode()
    node.get_logger().info("=" * 60)
    node.get_logger().info(f"ur_teleop 就绪 — mode={node._mode}, sim={node._cfg['sim']}")
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
