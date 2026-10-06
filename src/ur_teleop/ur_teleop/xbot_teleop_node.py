"""Xbot 遥操作：每周期以实测 tool0 位姿和最新手柄输入生成目标。"""

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
from std_msgs.msg import Bool, String, Float64MultiArray, MultiArrayDimension
from tf2_ros import Buffer, TransformListener, TransformException

from ur_teleop.config import load_config, default_config_path, UR_JOINT_NAMES, UR_GRIPPER_JOINT
from ur_teleop.controller_switcher import ControllerSwitcher
from ur_teleop.xbot_core import AXES, BUTTONS, JoyMapping, ButtonEvents, PoseIntegrator, orientation_distance
from ur_teleop.cartesian_action import encode_action, action_label
from ur_teleop.controller_frame import (controller_base_frame, transform_pose,
                                         inverse_transform_pose, clip_workspace_target)


class XbotTeleopNode(Node):
    def __init__(self):
        super().__init__('xbot_teleop')
        self.declare_parameter('config_file', default_config_path())
        self.cfg = load_config(self.get_parameter('config_file').value)
        if self.cfg['teleop'].get('control_source') != 'xbot':
            raise ValueError('节点只接受 control_source: xbot')
        self.x = self.cfg['xbot']
        self.diagnostic_hz = float(self.x.get('diagnostic_hz', 5.))
        self.controller_frame = controller_base_frame(self.cfg['sim'])
        self.action_mode = self.cfg.get('recorder', {}).get('action_mode', 'abs')
        if any(key in self.x for key in ('max_translation_delta_m', 'max_rotation_delta_rad',
                                         'max_target_position_error_m', 'max_target_orientation_error_rad')):
            raise ValueError('固定周期增量和目标超前参数已停用')
        for key in ('control_hz', 'joy_timeout_s', 'tcp_timeout_s', 'max_linear_speed_m_s',
                    'max_angular_speed_rad_s', 'precision_scale'):
            if not np.isfinite(self.x[key]) or self.x[key] <= 0:
                raise ValueError(f'Xbot 参数必须为正数: {key}')
        if self.x['control_hz'] < 10 or self.x['precision_scale'] > 1:
            raise ValueError('control_hz 必须至少 10 Hz，precision_scale 不得超过 1')
        if not np.isfinite(self.diagnostic_hz) or self.diagnostic_hz <= 0:
            raise ValueError('Xbot 诊断日志频率必须为正数')
        self.workspace_half_extent = np.asarray(self.x.get('workspace_half_extent_m', [.2] * 3), dtype=float)
        if (self.workspace_half_extent.shape != (3,) or
                not np.isfinite(self.workspace_half_extent).all() or
                np.any(self.workspace_half_extent <= 0)):
            raise ValueError('workspace_half_extent_m 必须为三个正数')
        self.workspace_origin = None
        self.command_dt = 1. / self.x['control_hz']
        if not isinstance(self.x.get('left_stick_xy_free', False), bool):
            raise ValueError('left_stick_xy_free 必须为布尔值')
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
        self.gripper_at = -float('inf')
        self.gripper_command = 0.
        self.gripper_initialized = False
        self.gripper_pending = False
        self.gripper_goal = None
        self.estop = False
        self.finished = False
        self.controller_active = False
        self.awaiting_controller_confirmation = False
        self.switch_future = None
        self.switch_at = 0.
        self.list_future = None
        self.list_at = -float('inf')
        self.controllers = {}
        self.controllers_at = -float('inf')
        self.previous_rb = True
        self.previous_time = time.monotonic()
        self.last_diagnostic_at = -float('inf')
        self.last_status = ''
        self.last_actual = None
        self.tcp_age_s = float('inf')
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
        self.create_subscription(PoseStamped, '/cartesian_impedance_controller/current_pose',
                                 self.on_controller_pose, 10)
        grip = self.cfg.get('gripper', {})
        self.gripper = ActionClient(self, ParallelGripperCommand,
                                    grip.get('action_server', '/robotiq_gripper_controller/gripper_cmd'))
        self.create_timer(1. / self.x['control_hz'], self.tick)
        self.get_logger().info('Xbot 启动：反馈就绪且已到 Home 后自动切换阻抗；RB 仅用于运动使能')

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
            self.gripper_at = time.monotonic()
            if not self.gripper_initialized:
                self.gripper_command = float(self.gripper_state > .4)
                self.gripper_initialized = True

    def on_estop(self, msg):
        self.estop = msg.data
        if msg.data:
            self.core.stop()
            self.cancel_gripper()

    def on_controller_pose(self, msg):
        """首条控制器当前位姿是本次节点运行的工作空间原点。"""
        if self.workspace_origin is not None or msg.header.frame_id != self.controller_frame:
            return
        p = msg.pose.position
        origin = np.array([p.x, p.y, p.z], dtype=float)
        if np.isfinite(origin).all():
            self.workspace_origin = origin
            self.get_logger().info(f'工作空间原点已锁定：{origin.tolist()}（{self.controller_frame}）')

    def on_finished(self, msg):
        if msg.data:
            self.finished = True
            self.core.stop()
            self.cancel_gripper()

    def actual_pose(self):
        self.tcp_age_s = float('inf')
        try:
            transform = self.buffer.lookup_transform('base_link', 'tool0', Time())
            age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
            self.tcp_age_s = age
            if not 0 <= age <= self.x['tcp_timeout_s']:
                return None
            p, q = transform.transform.translation, transform.transform.rotation
            pose = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            if not np.isfinite(pose).all() or not .5 <= np.linalg.norm(pose[3:]) <= 1.5:
                return None
            pose[3:] /= np.linalg.norm(pose[3:])
            return pose
        except TransformException:
            return None

    def controller_transform(self):
        """内部始终使用 base_link，发布时转换到控制器要求的坐标系。"""
        if self.controller_frame == 'base_link':
            return np.array([0., 0., 0., 0., 0., 0., 1.])
        try:
            tf = self.buffer.lookup_transform(self.controller_frame, 'base_link', Time())
            p, q = tf.transform.translation, tf.transform.rotation
            transform = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            # base 与 base_link 为模型中的固定变换；时间戳为零的静态 TF 有效。
            transform_pose([0., 0., 0., 0., 0., 0., 1.], transform)
            return transform
        except (TransformException, ValueError):
            self.get_logger().error('控制器基座 TF 无效，禁止发布目标', throttle_duration_sec=2.)
            return None

    def cancel_gripper(self):
        if self.gripper_goal is not None:
            self.gripper_goal.cancel_goal_async()

    def toggle_gripper(self):
        grip = self.cfg.get('gripper', {})
        if (not grip.get('enabled', False) or self.gripper_pending or
                self.gripper_state is None or time.monotonic() - self.gripper_at > self.x['tcp_timeout_s'] or
                not self.gripper.server_is_ready()):
            return
        desired = 0. if self.gripper_state > .4 else 1.
        self.get_logger().info(f'夹爪请求: 实测={self.gripper_state:.3f}, 目标={desired}')
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
                    self.get_logger().warn('夹爪 action 拒绝目标，请检查命令字段及控制器接口')
                    return
                self.gripper_goal = handle
                self.gripper_command = desired
                self.get_logger().info(f'夹爪目标已接受: {desired}')
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

    def ensure_impedance(self, actual, feedback_ready, now):
        """自动切换或接管；与 RB/Joy 输入无关，不在回调中阻塞等待。"""
        if (not feedback_ready or self.estop or self.finished or self.controller_active
                or self.switch_future is not None or now - self.controllers_at >= 2.):
            return
        state = self.controllers.get('cartesian_impedance_controller')
        if state == 'active':
            # 已激活时从当前实测位姿接管，不沿用旧进程的目标。
            self.core.stop(actual)
            self.controller_active = True
            self.awaiting_controller_confirmation = False
            self.get_logger().info('已接管激活的阻抗控制器；松开 RB 后按住 RB 操作')
            return
        home_ok = (self.joints is not None and np.max(np.abs(self.joints - self.cfg['home']['slave']))
                   <= self.cfg['home'].get('at_home_tolerance_rad', .05))
        if (home_ok and state == 'inactive'
                and self.controllers.get('scaled_joint_trajectory_controller') == 'active'):
            self.switch_future = self.switcher.switch(['cartesian_impedance_controller'],
                                                      ['scaled_joint_trajectory_controller'])
            self.switch_at = now

    def monitor_controllers(self, now, actual):
        """低频确认控制器状态；短暂查询延迟不打断手柄积分。"""
        if self.list_future is not None and self.list_future.done():
            try:
                if self.list_future.result() is None:
                    raise RuntimeError('未收到控制器状态响应')
                self.controllers = self.switcher.list_result(self.list_future)
                self.controllers_at = now
                if self.controller_active and self.controllers.get('cartesian_impedance_controller') == 'active':
                    self.awaiting_controller_confirmation = False
                elif self.controller_active:
                    self.controller_active = False
                    self.finished = True
                    self.core.stop(actual)
                    self.get_logger().error('阻抗控制器已失活，锁定遥操作；检查后重启')
            except Exception as exc:
                self.get_logger().warn(f'控制器状态查询失败，保留上次状态: {exc}', throttle_duration_sec=2.)
            self.list_future = None
        if now - self.list_at >= .5 and self.list_future is None:
            self.list_future = self.switcher.list_controllers()
            self.list_at = now
        if self.controller_active and now - self.controllers_at >= 10.:
            self.controller_active = False
            self.finished = True
            self.core.stop(actual)
            self.get_logger().error('控制器状态连续 10 秒未确认，锁定遥操作；检查后重启')

    def log_diagnostic(self, now, actual, published, target_transform):
        """定频输出同一 base_link 坐标系中的实测与目标位姿。"""
        if now - self.last_diagnostic_at < 1. / self.diagnostic_hz:
            return
        self.last_diagnostic_at = now

        def pose_text(pose):
            if pose is None:
                return '无'
            return ('xyz=(' + ','.join(f'{value:.4f}' for value in pose[:3]) + ') '
                    'xyzw=(' + ','.join(f'{value:.4f}' for value in pose[3:]) + ')')

        def age_text(age):
            return f'{age:.2f}s' if np.isfinite(age) and age >= 0 else '无'

        target = self.core.target
        if actual is not None and target is not None:
            lead_mm = 1000. * np.linalg.norm(target[:3] - actual[:3])
            lead_rad = orientation_distance(target[3:], actual[3:])
            error = f'{lead_mm:.1f}mm/{lead_rad:.3f}rad'
        else:
            error = '无'
        if self.finished:
            state = '已锁定'
        elif self.estop:
            state = '急停'
        elif now - self.joy_at > self.x['joy_timeout_s']:
            state = 'Joy超时'
        elif actual is None:
            state = 'tool0反馈无效'
        elif target_transform is None:
            state = '控制器基座TF无效'
        elif now - self.joints_at > self.x['tcp_timeout_s']:
            state = '关节反馈超时'
        elif not self.controller_active:
            state = '等待阻抗控制器'
        elif self.awaiting_controller_confirmation:
            state = '等待控制器确认'
        elif self.core.enabled:
            state = '运动使能'
        else:
            state = '等待RB'
        translation = (self.axes['ly'], self.axes['lx'], self.axes['rt'] - self.axes['lt'])
        rotation = (self.axes['ry'], self.axes['rx'], self.axes['yaw'])
        self.get_logger().info(
            f'遥操作诊断 [{state}] 坐标系=base_link 模式={self.core.frame} '
            f'RB={int(self.buttons["rb"])} LB={int(self.buttons["lb"])} '
            f'输入平移={tuple(round(value, 2) for value in translation)} '
            f'输入旋转={tuple(round(value, 2) for value in rotation)} '
            f'目标已发布={int(published)} 控制器={self.controllers.get("cartesian_impedance_controller", "未知")} '
            f'数据龄 Joy={age_text(now - self.joy_at)} TF={age_text(self.tcp_age_s)} '
            f'关节={age_text(now - self.joints_at)} 状态查询={age_text(now - self.controllers_at)} '
            f'当前={pose_text(actual)} 目标={pose_text(target)} 位姿差={error}')

    def tick(self):
        now = time.monotonic()
        elapsed_dt, self.previous_time = now - self.previous_time, now
        actual = self.actual_pose()
        target_transform = self.controller_transform()
        if actual is not None:
            self.last_actual = actual.copy()
        feedback_ready = (now - self.joints_at <= self.x['tcp_timeout_s'] and actual is not None
                          and target_transform is not None)
        fresh = now - self.joy_at <= self.x['joy_timeout_s'] and feedback_ready
        edges, self.edges = self.edges, set()
        self.monitor_controllers(now, actual)
        if self.switch_future is not None and self.switch_future.done():
            try:
                self.controller_active = self.switcher.switch_ok(self.switch_future)
            except Exception:
                self.controller_active = False
            self.switch_future = None
            self.core.stop(actual)
            self.awaiting_controller_confirmation = self.controller_active
            self.get_logger().info(f'笛卡尔控制器自动切换结果: {self.controller_active}')
            if not self.controller_active:
                self.finished = True
                self.get_logger().error('自动切换失败，锁定遥操作；检查控制器后重启')
            # 下一次查询确认 active 前禁止运动。
            self.controllers = {}
        available = fresh and not self.estop and not self.finished
        self.ensure_impedance(actual, feedback_ready, now)
        if self.switch_future is not None and now - self.switch_at > 10.:
            self.finished = True
            self.get_logger().error('切换超时，锁定遥操作；检查控制器状态后重启', throttle_duration_sec=5.)
        safe = (available and not self.finished and self.controller_active
                and not self.awaiting_controller_confirmation and self.workspace_origin is not None
                and 0 < elapsed_dt <= .1)
        was_enabled = self.core.enabled
        self.core.step(actual, self.axes, self.buttons, self.command_dt, safe,
                       toggle=fresh and 'x' in edges)
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
        # TF 失效只能保持最后有效实测位姿；不能假设控制器提供命令超时保护。
        published_target = False
        if (self.controller_active and self.core.target is not None and
                target_transform is not None and self.workspace_origin is not None):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.controller_frame
            p = transform_pose(self.core.target, target_transform)
            p = clip_workspace_target(p, self.workspace_origin, self.workspace_half_extent)
            # 保存的保持目标也必须与真正下发的目标一致。
            self.core.target = inverse_transform_pose(p, target_transform)
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p[:3])
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w) = map(float, p[3:])
            self.pose_pub.publish(msg)
            published_target = True
            if actual is not None:
                actual_controller = transform_pose(actual, target_transform)
                values = encode_action(p, actual_controller, self.gripper_command, self.action_mode).tolist()
                command = Float64MultiArray(data=values)
                command.layout.dim = [MultiArrayDimension(
                    label=action_label(self.action_mode, self.controller_frame),
                    size=len(values), stride=len(values))]
                self.command_pub.publish(command)
        self.ready_pub.publish(Bool(data=bool(safe and not self.finished)))
        status = f'{self.core.frame}: ' + ('允许输入更新目标' if self.core.enabled else '目标更新禁用，末次目标仍可被跟踪')
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status
        self.status_pub.publish(String(data=status))
        self.log_diagnostic(now, actual, published_target, target_transform)


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
