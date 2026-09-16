"""Stage 1: move both arms to their configured home joints (one-shot).

Waits for the UR cell (started by home.launch), sends a UR home trajectory
via scaled_joint_trajectory_controller, commands the Alicia to home via
/joint_commands, verifies arrival, prints READY, exits 0/1.

Single-threaded: all waiting is done by polling an executor.spin_once in the
main loop — no background spin thread, no nested spin_until_future_complete.
"""

import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ur_teleop.config import (
    ALICIA_JOINT_NAMES,
    GRIPPER_JOINT,
    UR_JOINT_NAMES,
    default_config_path,
    load_config,
)


class HomeNode(rclpy.node.Node):
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
        self._move_duration = float(home.get("move_duration_s", 8.0))
        self._verify_duration = float(home.get("verify_duration_s", 2.0))

        self._ur_states: JointState | None = None
        self._alicia_states: JointState | None = None
        self._joint_sub = self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self._cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self._traj_client = ActionClient(
            self, FollowJointTrajectory, "/scaled_joint_trajectory_controller/follow_joint_trajectory"
        )
        self._cell_ready_since: float | None = None

    def _joint_cb(self, msg: JointState):
        # 双臂各自发布 /joint_states（UR cell 与 alicia_d_driver 交织在同一话题），
        # 逐侧更新各自保留，避免整包覆盖导致另一侧瞬时缺失（同 teleop_node）。
        names = set(msg.name)
        if all(n in names for n in UR_JOINT_NAMES):
            self._ur_states = msg
        if all(n in names for n in ALICIA_JOINT_NAMES):
            self._alicia_states = msg

    def _q(self, src: JointState | None, names: list[str]) -> list[float] | None:
        if src is None or not all(n in src.name for n in names):
            return None
        return [src.position[src.name.index(n)] for n in names]

    def slave_q(self) -> list[float] | None:
        return self._q(self._ur_states, UR_JOINT_NAMES)

    def master_q(self) -> list[float] | None:
        return self._q(self._alicia_states, ALICIA_JOINT_NAMES)

    def cell_ready(self) -> bool:
        """等待轨迹 action 与状态通道稳定后才允许发送 home goal。

        action server 可能早于控制器激活而出现。首次同时收到 UR 状态与 action
        server 后保留 2 秒稳定窗口，避免启动阶段接受的 goal 在控制器激活时被立即结束。
        这里不调用 controller_manager 的 list_controllers 服务：Jazzy 在 spawner 超时
        取消请求时可能因响应已失效的客户端而终止 controller_manager。
        """
        ready = self.slave_q() is not None and self._traj_client.server_is_ready()
        if not ready:
            self._cell_ready_since = None
            return False
        if self._cell_ready_since is None:
            self._cell_ready_since = time.monotonic()
            return False
        return time.monotonic() - self._cell_ready_since >= 2.0

    def publish_alicia_home(self):
        msg = JointState()
        msg.name = ALICIA_JOINT_NAMES + [GRIPPER_JOINT]
        msg.position = self._master_home + [self._gripper_value]   # 0-1000，1000=开
        self._cmd_pub.publish(msg)

    def send_ur_home_trajectory(self, executor) -> bool:
        """Send home trajectory; poll async action via executor; log contrast on failure.

        Action server 在控制器 configure 时即被发现，而 trajectory 控制器在
        activate 前会 REJECT goal（cell 启动的 activation 窗口 ~1-2 s）→
        未接受即重试，直到接受或总窗口超时（cell 快速启动时实测第一发必被拒）。
        """
        accept_deadline = time.time() + 10.0
        accepted = False
        while rclpy.ok() and time.time() < accept_deadline:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = JointTrajectory()
            goal.trajectory.joint_names = UR_JOINT_NAMES
            pt = JointTrajectoryPoint()
            pt.positions = self._slave_home
            pt.time_from_start = Duration(seconds=self._move_duration).to_msg()
            goal.trajectory.points = [pt]

            self.get_logger().info(f"UR home trajectory -> {self._slave_home}")
            future = self._traj_client.send_goal_async(goal)
            deadline = min(time.time() + 5.0, accept_deadline)
            while rclpy.ok() and not future.done() and time.time() < deadline:
                executor.spin_once(timeout_sec=0.1)
            if future.done() and future.result() is not None and future.result().accepted:
                accepted = True
                break
            self.get_logger().warn("UR home goal 未接受（控制器可能尚未激活），重试...")
            time.sleep(0.5)
        if not accepted:
            self._log_home_failure("UR home trajectory rejected/timed out")
            return False
        result_future = future.result().get_result_async()
        deadline = time.time() + self._move_timeout
        while rclpy.ok() and not result_future.done() and time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)
        if result_future.done() and result_future.result() is not None:
            code = result_future.result().result.error_code
            if code == FollowJointTrajectory.Result.SUCCESSFUL:
                return True
            self._log_home_failure(f"UR home trajectory failed, error_code={code}")
            return False
        self._log_home_failure("UR home trajectory timed out")
        return False

    def _log_home_failure(self, reason: str):
        current = self.slave_q()
        self.get_logger().error(
            f"{reason}. 目标={self._slave_home}, 当前={current if current is not None else '无 /joint_states'}. "
            f"请检查机器人与 cell；cell 保持运行。"
        )

    def at_home(self) -> bool:
        m, s = self.master_q(), self.slave_q()
        if m is None or s is None:
            return False
        m_err = max(abs(a - b) for a, b in zip(m, self._master_home))
        s_err = max(abs(a - b) for a, b in zip(s, self._slave_home))
        return m_err <= self._tolerance and s_err <= self._tolerance


def main():
    rclpy.init()
    node = HomeNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    rc = 0
    try:
        node.get_logger().info(
            "等待 UR cell 就绪（/joint_states + trajectory action server 稳定）..."
        )
        deadline = time.time() + 30.0
        while rclpy.ok() and not node.cell_ready() and time.time() < deadline:
            executor.spin_once(timeout_sec=0.5)
        if not node.cell_ready():
            node.get_logger().error("cell 未就绪。请先运行 home.launch（含 cell）。")
            rc = 1
        elif not node.send_ur_home_trajectory(executor):
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
                m, s = node.master_q(), node.slave_q()
                node.get_logger().error(
                    f"到位超时（未在 move_timeout 内验证双臂位于 home）。"
                    f"master 目标={node._master_home}, "
                    f"当前={m if m is not None else '无 /joint_states'}; "
                    f"UR 目标={node._slave_home}, "
                    f"当前={s if s is not None else '无 /joint_states'}. "
                    "可用 teleop.launch force_home:=true 跳过验证。"
                )
                rc = 1
            else:
                node.get_logger().info("=" * 50)
                node.get_logger().info("HOME REACHED — 双臂已到位。现在运行 teleop.launch（阶段 2）。")
                node.get_logger().info("=" * 50)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
