"""Xbot 遥操作：只向基座坐标系中的 gripper_tcp 目标发送指令。"""

import time
from pathlib import Path

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import ParallelGripperCommand
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool, String, Float64MultiArray
from tf2_ros import Buffer, TransformListener, TransformException

from ur_teleop.config import load_config, default_config_path, UR_JOINT_NAMES, UR_GRIPPER_JOINT
from ur_teleop.controller_switcher import ControllerSwitcher
from ur_teleop.xbot_core import AXES, BUTTONS, JoyMapping, ButtonEvents, PoseIntegrator


class XbotTeleopNode(Node):
    def __init__(self):
        super().__init__('xbot_teleop')
        self.declare_parameter('config_file', default_config_path())
        self.cfg = load_config(self.get_parameter('config_file').value)
        if self.cfg['teleop'].get('control_source') != 'xbot':
            raise ValueError('节点只接受 control_source: xbot')
        self.x = self.cfg['xbot']
        for key in ('control_hz', 'joy_timeout_s', 'tcp_timeout_s', 'max_linear_speed_m_s',
                    'max_angular_speed_rad_s', 'precision_scale', 'target_lead_m', 'target_lead_rad'):
            if not np.isfinite(self.x[key]) or self.x[key] <= 0:
                raise ValueError(f'Xbot 参数必须为正数: {key}')
        with Path(self.x['calibration_file']).expanduser().open() as stream:
            self.mapping = JoyMapping(yaml.safe_load(stream), self.x['deadzone'])
        self.core = PoseIntegrator(self.x)
        self.events = ButtonEvents(self.x.get('view_hold_s', 2.))
        self.axes = dict.fromkeys(AXES, 0.)
        self.buttons = dict.fromkeys(BUTTONS, False)
        self.edges = set()
        self.joy_at = self.joints_at = -float('inf')
        self.joints = None
        self.gripper_state = None
        self.gripper_command = 0.
        self.gripper_pending = False
        self.gripper_goal = None
        self.estop = False
        self.finished = False
        self.controller_active = False
        self.switch_future = None
        self.switch_at = 0.
        self.list_future = None
        self.list_at = -float('inf')
        self.controllers = {}
        self.controllers_at = -float('inf')
        self.previous_rb = True
        self.previous_time = time.monotonic()
        self.previous_safe = False
        self.last_status = ''
        self.last_actual = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.switcher = ControllerSwitcher(self)
        self.pose_pub = self.create_publisher(PoseStamped, '/cartesian_impedance_controller/target_pose', 1)
        self.command_pub = self.create_publisher(Float64MultiArray, '/teleop/commands', 1)
        self.ready_pub = self.create_publisher(Bool, '/teleop/xbot_ready', 1)
        self.status_pub = self.create_publisher(String, '/teleop/xbot_status', 1)
        self.event_pub = self.create_publisher(String, '/teleop/record_event', 10)
        self.create_subscription(Joy, '/joy', self.on_joy, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Bool, '/teleop/e_stop', self.on_estop, 10)
        self.create_subscription(Bool, '/teleop/record_finished', self.on_finished, 10)
        grip = self.cfg.get('gripper', {})
        self.gripper = ActionClient(self, ParallelGripperCommand,
                                    grip.get('action_server', '/robotiq_gripper_controller/gripper_cmd'))
        self.create_timer(1. / self.x['control_hz'], self.tick)
        self.get_logger().info('Xbot 就绪：先完成 Home；松开 RB 后首次按下切换控制器，再松开/按下开始运动')

    def on_joy(self, msg):
        try:
            axes, buttons = self.mapping.decode(msg.axes, msg.buttons)
        except (ValueError, IndexError) as exc:
            self.joy_at = -float('inf')
            self.get_logger().error(str(exc), throttle_duration_sec=2.)
            return
        now = time.monotonic()
        if now - self.joy_at > self.x['joy_timeout_s']:
            self.events.reset()
            self.edges.clear()
            self.core.stop()
            self.previous_rb = True
        self.axes, self.buttons = axes, buttons
        self.edges.update(self.events.update(buttons, now))
        self.joy_at = now

    def on_joints(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(n in values and np.isfinite(values[n]) for n in UR_JOINT_NAMES):
            self.joints = np.array([values[n] for n in UR_JOINT_NAMES])
            self.joints_at = time.monotonic()
        if UR_GRIPPER_JOINT in values and np.isfinite(values[UR_GRIPPER_JOINT]):
            self.gripper_state = values[UR_GRIPPER_JOINT]

    def on_estop(self, msg):
        self.estop = msg.data
        if msg.data:
            self.core.stop()
            self.cancel_gripper()

    def on_finished(self, msg):
        if msg.data:
            self.finished = True
            self.core.stop()
            self.cancel_gripper()

    def actual_pose(self):
        try:
            transform = self.buffer.lookup_transform('base_link', 'gripper_tcp', Time())
            age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
            if not 0 <= age <= self.x['tcp_timeout_s']:
                return None
            p, q = transform.transform.translation, transform.transform.rotation
            pose = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            if not np.isfinite(pose).all() or np.linalg.norm(pose[3:]) < .5:
                return None
            pose[3:] /= np.linalg.norm(pose[3:])
            return pose
        except TransformException:
            return None

    def cancel_gripper(self):
        if self.gripper_goal is not None:
            self.gripper_goal.cancel_goal_async()

    def toggle_gripper(self):
        grip = self.cfg.get('gripper', {})
        if (not grip.get('enabled', False) or self.gripper_pending or
                self.gripper_state is None or not self.gripper.server_is_ready()):
            return
        desired = 0. if self.gripper_state > .4 else 1.
        goal = ParallelGripperCommand.Goal()
        goal.command.name = [UR_GRIPPER_JOINT]
        goal.command.position = [float(grip.get('close_pos_rad', .79) if desired else grip.get('open_pos_rad', 0.))]
        goal.command.effort = [float(grip.get('max_effort', 50.))]
        self.gripper_pending = True
        future = self.gripper.send_goal_async(goal)

        def accepted(f):
            try:
                handle = f.result()
                if not handle.accepted:
                    self.gripper_pending = False
                    return
                self.gripper_goal = handle
                self.gripper_command = desired
                if not self.core.enabled or self.estop or self.finished:
                    handle.cancel_goal_async()
                handle.get_result_async().add_done_callback(done)
            except Exception as exc:
                self.gripper_pending = False
                self.get_logger().error(f'夹爪请求失败: {exc}')

        def done(f):
            self.gripper_pending = False
            self.gripper_goal = None

        future.add_done_callback(accepted)

    def tick(self):
        now = time.monotonic()
        dt, self.previous_time = now - self.previous_time, now
        actual = self.actual_pose()
        if actual is not None:
            self.last_actual = actual.copy()
        fresh = (now - self.joy_at <= self.x['joy_timeout_s'] and
                 now - self.joints_at <= self.x['tcp_timeout_s'] and actual is not None)
        edges, self.edges = self.edges, set()
        rb_edge = self.buttons['rb'] and not self.previous_rb
        self.previous_rb = self.buttons['rb'] if fresh else True
        if self.list_future is not None and self.list_future.done():
            try:
                self.controllers = self.switcher.list_result(self.list_future)
                self.controllers_at = now
            except Exception:
                self.controllers = {}
            self.list_future = None
        if now - self.list_at >= .5 and self.list_future is None:
            self.list_future = self.switcher.list_controllers()
            self.list_at = now
        if self.switch_future is not None and self.switch_future.done():
            try:
                self.controller_active = self.switcher.switch_ok(self.switch_future)
            except Exception:
                self.controller_active = False
            self.switch_future = None
            self.core.stop(actual)
            self.get_logger().info(f'笛卡尔控制器切换结果: {self.controller_active}；请重新按 RB')
            # 下一次查询确认 active 前禁止运动。
            self.controllers = {}
        home_ok = (self.joints is not None and np.max(np.abs(self.joints - self.cfg['home']['slave']))
                   <= self.cfg['home'].get('at_home_tolerance_rad', .05))
        available = fresh and not self.estop and not self.finished
        if (rb_edge and available and not self.controller_active and self.switch_future is None and home_ok
                and self.controllers.get('cartesian_impedance_controller') == 'inactive'
                and self.controllers.get('scaled_joint_trajectory_controller') == 'active'):
            self.switch_future = self.switcher.switch(['cartesian_impedance_controller'],
                                                      ['scaled_joint_trajectory_controller'])
            self.switch_at = now
        if self.switch_future is not None and now - self.switch_at > 10.:
            self.finished = True
            self.get_logger().error('切换超时，锁定遥操作；检查控制器状态后重启', throttle_duration_sec=5.)
        safe = (available and not self.finished and self.controller_active and
                now - self.controllers_at < 2. and
                self.controllers.get('cartesian_impedance_controller') == 'active')
        was_enabled = self.core.enabled
        action = self.core.step(actual, self.axes, self.buttons, dt, safe, toggle=fresh and 'x' in edges)
        if actual is None:
            self.core.stop(self.last_actual)
        if was_enabled and not self.core.enabled:
            self.cancel_gripper()
        if safe and 'a' in edges and self.core.enabled:
            self.toggle_gripper()
        if fresh:
            for button, event in (('menu', 'start'), ('y', 'save'), ('b', 'discard'), ('finalize', 'finalize')):
                if button in edges and (event != 'start' or safe):
                    self.event_pub.publish(String(data=event))
                    if event == 'finalize':
                        self.finished = True
                        self.core.stop(actual)
                        self.cancel_gripper()
                        action[:] = 0.
        # TF 失效只能保持最后有效实测位姿；不能假设控制器提供命令超时保护。
        if self.controller_active and self.core.target is not None:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            p = self.core.target
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p[:3])
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w) = map(float, p[3:])
            self.pose_pub.publish(msg)
        self.command_pub.publish(Float64MultiArray(data=[*map(float, action), self.gripper_command]))
        self.ready_pub.publish(Bool(data=bool(safe and not self.finished)))
        status = f'{self.core.frame}: ' + ('运动' if self.core.enabled else '保持/等待重新使能')
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status
        self.status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = XbotTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
