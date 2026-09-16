#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from ruckig import Ruckig, InputParameter, OutputParameter, Result

from ur_teleop.config import default_config_path, load_config


DOF = 6

# ruckig 段缺失/损坏时使用的保守默认（第一阶段真机测试参数）
_DEFAULT_MAX_VELOCITY = [0.30] * DOF
_DEFAULT_MAX_ACCELERATION = [0.80] * DOF
_DEFAULT_MAX_JERK = [4.0] * DOF

# 从臂控制器共同使用的关节顺序。
UR_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

UR_JOINT_INDEX = {
    name: index
    for index, name in enumerate(UR_JOINT_NAMES)
}


class RuckigNode(Node):

    def __init__(self):
        super().__init__("ruckig_node")

        # ============================================================
        # Control configuration
        # ============================================================

        self.declare_parameter("control_hz", 100.0)
        self.declare_parameter("config_file", default_config_path())

        self.control_hz = float(
            self.get_parameter("control_hz").value
        )

        self.dt = 1.0 / self.control_hz

        # ------------------------------------------------------------
        # 运动参数从 ur_teleop.yaml 的 ruckig 段读取（可选段，launch
        # 参数优先、yaml 兜底、缺省用下方保守默认）；后续再根据
        # UR10e 和遥操需要调整
        # ------------------------------------------------------------

        try:
            config = load_config(self.get_parameter("config_file").value)
            ruckig_cfg = config.get("ruckig", {})
            self.controller_kind = config["teleop"].get("controller", "forward_position")
        except Exception:
            ruckig_cfg = {}
            self.controller_kind = "forward_position"

        def _vec6(values, default):
            vec = [float(v) for v in values] if values is not None else default
            return vec if len(vec) == DOF else default

        self.max_velocity = _vec6(
            ruckig_cfg.get("max_velocity"), _DEFAULT_MAX_VELOCITY
        )
        self.max_acceleration = _vec6(
            ruckig_cfg.get("max_acceleration"), _DEFAULT_MAX_ACCELERATION
        )
        self.max_jerk = _vec6(
            ruckig_cfg.get("max_jerk"), _DEFAULT_MAX_JERK
        )

        # ============================================================
        # UR actual state
        # ============================================================

        self.robot_q = np.zeros(DOF)
        self.robot_dq = np.zeros(DOF)

        # 因为 /joint_states 中可能混有主臂和 UR，
        # 需要确认 UR 六个 joint 都至少收到一次
        self.robot_joint_valid = np.zeros(
            DOF,
            dtype=bool,
        )

        # ============================================================
        # Target
        # ============================================================

        self.target_q = None

        # ============================================================
        # Ruckig
        # ============================================================

        self.otg = Ruckig(
            DOF,
            self.dt,
        )

        self.inp = InputParameter(DOF)
        self.out = OutputParameter(DOF)

        self.inp.max_velocity = self.max_velocity
        self.inp.max_acceleration = self.max_acceleration
        self.inp.max_jerk = self.max_jerk

        self.inp.target_velocity = [0.0] * DOF
        self.inp.target_acceleration = [0.0] * DOF

        self.initialized = False

        # ============================================================
        # ROS subscribers
        # ============================================================

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            50,
        )

        # 独立测试输入
        #
        # 固定顺序：
        #
        # shoulder_pan
        # shoulder_lift
        # elbow
        # wrist_1
        # wrist_2
        # wrist_3
        #
        self.target_sub = self.create_subscription(
            Float64MultiArray,
            "/ruckig/target_joint_positions",
            self.target_callback,
            10,
        )

        # ============================================================
        # ROS publisher
        # ============================================================

        if self.controller_kind == "joint_impedance":
            self.command_pub = self.create_publisher(
                JointState,
                "/joint_impedance_controller/target_joint_state",
                10,
            )
        else:
            self.command_pub = self.create_publisher(
                Float64MultiArray,
                "/forward_position_controller/commands",
                10,
            )

        # ============================================================
        # Control timer
        # ============================================================

        self.timer = self.create_timer(
            self.dt,
            self.control_loop,
        )

        self.get_logger().info(
            "Ruckig node started"
        )

        self.get_logger().info(
            f"control frequency: {self.control_hz:.1f} Hz"
        )

        self.get_logger().info(
            f"control period: {self.dt:.6f} s"
        )
        self.get_logger().info(f"target controller: {self.controller_kind}")

        self.get_logger().info(
            "waiting for UR /joint_states ..."
        )

    # ================================================================
    # UR actual joint state
    # ================================================================

    def joint_state_callback(
        self,
        msg: JointState,
    ) -> None:

        # 不能直接使用 msg.position
        #
        # 因为你的 /joint_states 顺序例如：
        #
        # elbow
        # shoulder_lift
        # shoulder_pan
        # wrist_1
        # wrist_2
        # wrist_3
        #
        # 必须根据 joint name 重排

        for msg_index, joint_name in enumerate(msg.name):

            if joint_name not in UR_JOINT_INDEX:
                continue

            ur_index = UR_JOINT_INDEX[joint_name]

            if msg_index < len(msg.position):

                self.robot_q[ur_index] = (
                    msg.position[msg_index]
                )

                self.robot_joint_valid[ur_index] = True

            if msg_index < len(msg.velocity):

                self.robot_dq[ur_index] = (
                    msg.velocity[msg_index]
                )

    # ================================================================
    # Target callback
    # ================================================================

    def target_callback(
        self,
        msg: Float64MultiArray,
    ) -> None:

        if len(msg.data) != DOF:

            self.get_logger().error(
                "Target must contain exactly "
                f"{DOF} joint positions, "
                f"received {len(msg.data)}"
            )

            return

        new_target = np.asarray(
            msg.data,
            dtype=float,
        )

        if not np.all(np.isfinite(new_target)):

            self.get_logger().error(
                "Target contains NaN or Inf"
            )

            return

        self.target_q = new_target

    # ================================================================
    # Initialize Ruckig
    # ================================================================

    def initialize_ruckig(self) -> bool:

        if not np.all(self.robot_joint_valid):
            return False

        # ------------------------------------------------------------
        # Ruckig 初始状态来自 UR 实际状态
        # ------------------------------------------------------------

        self.inp.current_position = (
            self.robot_q.tolist()
        )

        self.inp.current_velocity = (
            self.robot_dq.tolist()
        )

        self.inp.current_acceleration = (
            [0.0] * DOF
        )

        # ------------------------------------------------------------
        # 启动时目标 = 当前 UR 状态
        #
        # 因此启动节点本身不会导致机器人运动
        # ------------------------------------------------------------

        self.target_q = self.robot_q.copy()

        self.inp.target_position = (
            self.target_q.tolist()
        )

        self.inp.target_velocity = (
            [0.0] * DOF
        )

        self.inp.target_acceleration = (
            [0.0] * DOF
        )

        self.initialized = True

        self.get_logger().info(
            "Ruckig initialized from UR state"
        )

        self.get_logger().info(
            "initial q: "
            + np.array2string(
                self.robot_q,
                precision=6,
            )
        )

        return True

    # ================================================================
    # Main Ruckig control loop
    # ================================================================

    def control_loop(self) -> None:

        # ------------------------------------------------------------
        # 1. Initialize
        # ------------------------------------------------------------

        if not self.initialized:

            self.initialize_ruckig()

            return

        # ------------------------------------------------------------
        # 2. Update latest target
        # ------------------------------------------------------------

        self.inp.target_position = (
            self.target_q.tolist()
        )

        self.inp.target_velocity = (
            [0.0] * DOF
        )

        self.inp.target_acceleration = (
            [0.0] * DOF
        )

        # ------------------------------------------------------------
        # 3. Ruckig online trajectory generation
        # ------------------------------------------------------------

        result = self.otg.update(
            self.inp,
            self.out,
        )

        if result not in (
            Result.Working,
            Result.Finished,
        ):

            self.get_logger().error(
                f"Ruckig update failed: {result}"
            )

            return

        # ------------------------------------------------------------
        # 4. Publish the next position setpoint
        # ------------------------------------------------------------

        positions = [float(x) for x in self.out.new_position]
        if self.controller_kind == "joint_impedance":
            command = JointState()
            command.name = UR_JOINT_NAMES
            command.position = positions
        else:
            command = Float64MultiArray()
            command.data = positions

        self.command_pub.publish(command)

        # ------------------------------------------------------------
        # 5. Critical:
        #
        # output:
        #   q(k+1)
        #   dq(k+1)
        #   ddq(k+1)
        #
        # becomes next cycle current state.
        # ------------------------------------------------------------

        self.out.pass_to_input(
            self.inp
        )


def main(args=None):

    rclpy.init(args=args)

    node = RuckigNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
