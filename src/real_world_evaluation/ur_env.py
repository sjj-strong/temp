"""UR 观测采集与动作执行；不实现机器人底层控制算法。"""
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from protocol import ActionChunk, ImageData, Observation

# 与现有 Robotiq 控制器约定一致，不提供 YAML 覆盖。
GRIPPER_ACTION = '/robotiq_gripper_controller/gripper_cmd'
GRIPPER_JOINT = 'robotiq_85_left_knuckle_joint'
GRIPPER_OPEN_RAD = 0.0
GRIPPER_CLOSE_RAD = 0.4

TCP_NAMES = ['tcp_ee_' + axis for axis in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')]


def compose_pose(pose, action):
    """录制 rel pose 的逆运算：参考系平移相加、旋转增量左乘。"""
    pose, action = np.asarray(pose, dtype=float), np.asarray(action, dtype=float)
    if pose.shape != (7,) or action.shape not in ((6,), (7,)) or not np.isfinite(np.r_[pose, action]).all():
        raise ValueError('位姿或动作维度/数值无效')
    if np.linalg.norm(pose[3:]) < 1e-8:
        raise ValueError('实测姿态四元数无效')
    rotation = Rotation.from_rotvec(action[3:6]) * Rotation.from_quat(pose[3:])
    return np.r_[pose[:3] + action[:3], rotation.as_quat()]


class MockEnv:
    """用于 HTTP 和执行逻辑验证的理想响应环境，不导入 ROS。"""
    def __init__(self, config, health):
        self.health = health
        self.instruction = config.get('instruction', '')
        self.pose = np.array([0., 0., 0., 0., 0., 0., 1.])
        self.executed, self.held, self.closed = [], False, False
        self.gripper = None

    def observe(self, step_id):
        values = {name: 0. for name in self.health.state_names}
        values.update(dict(zip(TCP_NAMES, self.pose)))
        return Observation(step_id=step_id, values=values,
                           images={key: ImageData.encode(np.zeros((s[1], s[2], 3), np.uint8))
                                   for key, s in self.health.cameras.items()},
                           reference_frame=self.health.reference_frame, tcp_link=self.health.tcp_link,
                           wrench_frame=self.health.wrench_frame, instruction=self.instruction)

    def step(self, action):
        self.pose = compose_pose(self.pose, action)
        self.executed.append(list(action))
        if len(action) == 7:
            self.gripper = action[6] >= .5

    def hold(self):
        self.held = True

    def close(self):
        self.closed = True


class UREnv:
    """ROS 回调只保存最新消息；同步主循环调用 observe/step。"""
    def __init__(self, config, health):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from control_msgs.action import ParallelGripperCommand
        from geometry_msgs.msg import PoseStamped, WrenchStamped
        from sensor_msgs.msg import Image, JointState
        from rcl_interfaces.srv import GetParameters

        self.config, self.health, self.ros = config, health, rclpy
        self.read_only = config.get('read_only', True)
        self.timeout = float(config.get('data_timeout_s', .5))
        self.messages, self.lock = {}, threading.Lock()
        self.pose_type, self.gripper_type = PoseStamped, ParallelGripperCommand
        self.grip_goal, self.grip_state, self.grip_error = None, None, None
        self.stopping = False
        self.pending_grip = None
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context, domain_id=config.get('ros_domain_id'))
        self.node = rclpy.create_node('real_world_evaluation', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        try:
            controller = config.get('controller', '/cartesian_impedance_controller')
            self.publisher = self.node.create_publisher(PoseStamped, controller + '/target_pose', 1)
            topics = [('pose', PoseStamped, controller + '/current_pose')]
            if any(n.startswith('joint_') for n in health.state_names):
                topics.append(('joints', JointState, config.get('joint_topic', '/joint_states')))
            if any(n.startswith(('force.', 'torque.')) for n in health.state_names):
                topics.append(('wrench', WrenchStamped, config['wrench_topic']))
            cameras = config.get('cameras', {})
            if set(cameras) != set(health.cameras):
                raise ValueError('本地相机话题映射与服务器不一致')
            topics.extend((key, Image, topic) for key, topic in cameras.items())
            for key, message_type, topic in topics:
                self.node.create_subscription(message_type, topic,
                    lambda msg, key=key: self._receive(key, msg), qos_profile_sensor_data)
            parameters = self.node.create_client(GetParameters, controller + '/get_parameters')
            if not parameters.wait_for_service(timeout_sec=3.):
                raise RuntimeError('阻抗控制器参数服务不可用')
            result = self._wait(parameters.call_async(GetParameters.Request(
                names=['base_frame', 'tip_frame', 'tf_prefix'])))
            base, tip, prefix = [value.string_value for value in result.values]
            if (prefix + base, prefix + tip) != (health.reference_frame, health.tcp_link):
                raise ValueError('阻抗控制器基座或 TCP 与训练定义不一致')
            self.gripper = ActionClient(self.node, ParallelGripperCommand,
                GRIPPER_ACTION)
            if len(health.action_names) == 7 and not self.read_only:
                if not config.get('gripper', {}).get('enabled', False) or not self.gripper.wait_for_server(timeout_sec=3.):
                    raise RuntimeError('七维动作需要已启用的夹爪控制器')
            deadline = time.monotonic() + float(config.get('startup_timeout_s', 5.))
            while True:
                try:
                    self.observe(0)
                    break
                except (ValueError, RuntimeError):
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(.02)
        except BaseException:
            self.close()
            raise

    def _wait(self, future, timeout=3.):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise TimeoutError('本地 ROS 服务超时')
        return future.result()

    def _receive(self, key, message):
        with self.lock:
            self.messages[key] = (time.monotonic(), message)

    def _snapshot(self, keys=None):
        with self.lock:
            messages = dict(self.messages)
        now = time.monotonic()
        ros_now = self.node.get_clock().now().nanoseconds / 1e9
        result = {}
        for key, (received, message) in messages.items():
            if keys is not None and key not in keys:
                continue
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            if now - received > self.timeout or not 0 <= ros_now - stamp <= self.timeout:
                raise RuntimeError(f'{key} 数据过期或时钟不一致')
            result[key] = message
        return result

    def _pose(self, messages):
        message = messages.get('pose')
        if message is None or message.header.frame_id != self.health.reference_frame:
            raise RuntimeError('实测 TCP 缺失或坐标系不符')
        p, q = message.pose.position, message.pose.orientation
        pose = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        compose_pose(pose, np.zeros(6))
        return pose

    def observe(self, step_id):
        messages = self._snapshot()
        values = dict(zip(TCP_NAMES, self._pose(messages)))
        if 'joints' in messages:
            msg = messages['joints']
            for field, data in [('position', msg.position), ('velocity', msg.velocity), ('effort', msg.effort)]:
                values.update({f'joint_{field}.{name}': value for name, value in zip(msg.name, data)})
        if 'wrench' in messages:
            msg = messages['wrench']
            if msg.header.frame_id != self.health.wrench_frame:
                raise ValueError('力传感器坐标系不符')
            for field in ('force', 'torque'):
                values.update({f'{field}.{axis}': getattr(getattr(msg.wrench, field), axis) for axis in 'xyz'})
        if not set(self.health.state_names) <= values.keys():
            raise RuntimeError('模型所需的数值观测缺失')
        images = {}
        for key in self.health.cameras:
            msg = messages.get(key)
            if msg is None or msg.encoding not in ('rgb8', 'bgr8') or msg.step < msg.width * 3:
                raise RuntimeError(f'{key} 图像缺失或编码无效')
            pixels = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
            images[key] = ImageData.encode(pixels[:, :msg.width * 3].reshape(msg.height, msg.width, 3), msg.encoding)
        return Observation(step_id=step_id, values=values, images=images,
                           reference_frame=self.health.reference_frame, tcp_link=self.health.tcp_link,
                           wrench_frame=self.health.wrench_frame, instruction=self.config.get('instruction', ''))

    def _publish(self, pose):
        message = self.pose_type()
        message.header.frame_id = self.health.reference_frame
        message.header.stamp = self.node.get_clock().now().to_msg()
        p, q = message.pose.position, message.pose.orientation
        p.x, p.y, p.z = map(float, pose[:3])
        q.x, q.y, q.z, q.w = map(float, pose[3:])
        self.publisher.publish(message)

    def step(self, action):
        ActionChunk(step_id=0, actions=[action])
        if len(action) != len(self.health.action_names):
            raise ValueError('动作维度与模型声明不一致')
        if self.read_only:
            raise RuntimeError('只读环境禁止执行动作')
        if self.grip_error:
            raise RuntimeError(self.grip_error)
        pose = self._pose(self._snapshot({'pose'}))
        self._publish(compose_pose(pose, action))
        if len(action) == 7:
            self._command_gripper(action[6] >= .5)

    def _command_gripper(self, closed):
        if closed == self.grip_state:
            return
        if self.grip_goal is not None:
            self._wait(self.grip_goal.cancel_goal_async())
        config = self.config['gripper']
        goal = self.gripper_type.Goal()
        goal.command.name = [GRIPPER_JOINT]
        goal.command.position = [GRIPPER_CLOSE_RAD if closed else GRIPPER_OPEN_RAD]
        goal.command.effort = [float(config.get('max_effort', 50.))]
        self.pending_grip = self.gripper.send_goal_async(goal)
        def accepted(future):
            handle = future.result()
            if self.stopping and handle.accepted:
                handle.cancel_goal_async()
        self.pending_grip.add_done_callback(accepted)
        self.grip_goal = self._wait(self.pending_grip)
        if not self.grip_goal.accepted:
            raise RuntimeError('夹爪拒绝目标')
        self.grip_state = closed
        def finished(future):
            try:
                if future.result().status not in (4, 5):
                    self.grip_error = '夹爪执行失败'
            except Exception as exc:
                self.grip_error = str(exc)
        self.grip_goal.get_result_async().add_done_callback(finished)

    def hold(self):
        self.stopping = True
        try:
            if not self.read_only:
                self._publish(self._pose(self._snapshot({'pose'})))
        finally:
            if self.grip_goal is not None and self.grip_goal.accepted:
                self._wait(self.grip_goal.cancel_goal_async())

    def close(self):
        self.stopping = True
        self.executor.shutdown(timeout_sec=3.)
        self.thread.join(timeout=3.)
        self.node.destroy_node()
        self.context.shutdown()
