#!/usr/bin/env python3
"""PD 倾倒控制节点:订阅电子秤重量,按 control_rate 跑核心 PD,
发布速度到 /forward_velocity_controller/commands(wrist_index 关节)。

接口约定参考 robot_utils/liquid_pouring/robot_driver.py:
    - 速度指令 Float64MultiArray(6 关节),只动倾倒关节(wrist_index,默认 5)
    - /joint_states 订阅需 BEST_EFFORT QoS
    - /scale/weight 用默认 reliable QoS(与发布端一致)

行为:
    - 启动即跑;|error| <= tolerance(误差允许范围)时发零速并结束循环
    - weight_timeout 秒未收到新重量 → 发零速安全停
    - 节点销毁时确保发零速

前置(见 README):需先启动 ur_control 并切换速度控制器:
    ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller \
        --activate forward_velocity_controller
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from .pd_controller import PDController

LOG_THROTTLE = 1.0  # 周期日志节流(秒)
NUM_JOINTS = 6      # UR10e 六关节


class PourControlNode(Node):
    """电子秤 PD 倾倒控制节点。"""

    def __init__(self):
        super().__init__("pour_control_node")
        # ---- 参数(全部来自 config/pour_params.yaml)----
        self.declare_parameter("weight_topic", "/scale/weight")
        self.declare_parameter("cmd_topic", "/forward_velocity_controller/commands")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("wrist_index", 5)
        self.declare_parameter("velocity_sign", -1.0)
        self.declare_parameter("control_rate", 9.0)
        self.declare_parameter("kp", 0.0045)
        self.declare_parameter("kd", 0.06)
        self.declare_parameter("joint_range", [-2.0, 2.0])
        self.declare_parameter("target_weight", 70.0)
        self.declare_parameter("tolerance", 1.0)
        self.declare_parameter("near_target_kd_enable", False)
        self.declare_parameter("near_target_kd_threshold", 10.0)
        self.declare_parameter("near_target_kd", 0.025)
        self.declare_parameter("weight_timeout", 2.0)

        self.weight_topic = self.get_parameter("weight_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.wrist_index = int(self.get_parameter("wrist_index").value)
        self.velocity_sign = float(self.get_parameter("velocity_sign").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.target_weight = float(self.get_parameter("target_weight").value)
        self.tolerance = float(self.get_parameter("tolerance").value)
        self.weight_timeout = float(self.get_parameter("weight_timeout").value)
        kp = float(self.get_parameter("kp").value)
        kd = float(self.get_parameter("kd").value)
        joint_range = tuple(float(v) for v in self.get_parameter("joint_range").value)

        self.pd = PDController(
            kp=kp, kd=kd, joint_range=joint_range,
            near_target_kd_enable=self.get_parameter("near_target_kd_enable").value,
            near_target_kd_threshold=float(
                self.get_parameter("near_target_kd_threshold").value),
            near_target_kd=float(self.get_parameter("near_target_kd").value),
        )

        # ---- 发布 / 订阅 ----
        self.cmd_pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10,
        )
        self.js_sub = self.create_subscription(
            JointState, self.joint_state_topic, self._joint_state_cb, qos_sensor)
        self.weight_sub = self.create_subscription(
            Float64, self.weight_topic, self._weight_cb, 10)

        # ---- 状态 ----
        self._latest_weight = None
        self._weight_stamp = None       # rclpy Time
        self._joint_angle = 0.0
        self._finished = False
        self._log_next = 0.0
        self._start_time = time.monotonic()
        self._cycles = 0

        self.get_logger().info(
            f"pour_control_node ready: rate={self.control_rate}Hz "
            f"kp={kp} kd={kd} target={self.target_weight}g "
            f"tolerance={self.tolerance}g wrist_idx={self.wrist_index} "
            f"sign={self.velocity_sign} cmd_topic={self.cmd_topic}")
        self._timer = self.create_timer(1.0 / self.control_rate, self._control_tick)

    # ------------------------------------------------------------ 回调
    def _weight_cb(self, msg):
        self._latest_weight = msg.data
        self._weight_stamp = self.get_clock().now()

    def _joint_state_cb(self, msg):
        if msg.position and self.wrist_index < len(msg.position):
            self._joint_angle = float(msg.position[self.wrist_index])

    # -------------------------------------------------------- 控制循环
    def _control_tick(self):
        if self._finished:
            return
        self._cycles += 1

        weight = self._latest_weight
        if weight is None:
            return  # 尚无数据,等下一周期

        # 重量超时安全停
        if self._weight_stamp is not None:
            age = (self.get_clock().now() - self._weight_stamp).nanoseconds * 1e-9
            if age > self.weight_timeout:
                self._stop_and_finish(
                    f"重量数据超时({age:.1f}s > {self.weight_timeout}s),安全停止",
                    level="error")
                return

        output, error = self.pd.calculate(self.target_weight, weight)

        # 发布 6 关节速度(只动 wrist_index)
        self._publish_velocity(self.velocity_sign * output)

        # 节流日志
        now = time.monotonic()
        if now >= self._log_next:
            self._log_next = now + LOG_THROTTLE
            self.get_logger().info(
                f"weight={weight:.2f}g error={error:+.2f}g "
                f"output={output:+.4f}rad/s joint={self._joint_angle:.3f}")

        # 达标自动停
        if self.pd.is_reached(error, self.tolerance):
            elapsed = time.monotonic() - self._start_time
            self._stop_and_finish(
                f"达标: |error|={abs(error):.2f}g <= tolerance={self.tolerance}g, "
                f"用时 {elapsed:.1f}s, 实际频率 {self._cycles / elapsed:.1f}Hz",
                level="info")
            return

    # ------------------------------------------------------------ 工具
    def _publish_velocity(self, wrist_velocity):
        """发布 6 关节速度,wrist_index 关节 = wrist_velocity,其余 0。"""
        msg = Float64MultiArray()
        data = [0.0] * NUM_JOINTS
        data[self.wrist_index] = wrist_velocity
        msg.data = data
        self.cmd_pub.publish(msg)

    def _stop_and_finish(self, message, level="error"):
        """发零速并结束控制循环。"""
        self._publish_velocity(0.0)
        self._finished = True
        if level == "info":
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)

    def destroy_node(self):
        if not self._finished:
            try:
                self._stop_and_finish("节点销毁,发送零速")
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PourControlNode()
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
