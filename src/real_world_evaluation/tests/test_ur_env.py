"""隔离 DDS 域和命名空间内的 ROS 模拟接口，不加载真实硬件。"""
import threading
import time

import numpy as np
import pytest

from protocol import Health, POSE_ACTION
from ur_env import TCP_NAMES, UREnv


def eventually(predicate, timeout=3.):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError('模拟接口未在期限内响应')
        time.sleep(.01)


@pytest.fixture
def simulated_controller():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.action import ActionServer
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped, WrenchStamped
    from sensor_msgs.msg import Image, JointState
    from control_msgs.action import ParallelGripperCommand

    context = rclpy.context.Context()
    rclpy.init(context=context, domain_id=231)
    node = rclpy.create_node('controller', namespace='/rwe_test', context=context)
    for name, value in [('base_frame', 'base_link'), ('tip_frame', 'tool0'), ('tf_prefix', '')]:
        node.declare_parameter(name, value)
    feedback = node.create_publisher(PoseStamped, '~/current_pose', qos_profile_sensor_data)
    images = node.create_publisher(Image, '/rwe_test/image', qos_profile_sensor_data)
    joints = node.create_publisher(JointState, '/rwe_test/joints', qos_profile_sensor_data)
    wrench = node.create_publisher(WrenchStamped, '/rwe_test/wrench', qos_profile_sensor_data)
    state = {'pose': [0., 0., 0., 0., 0., 0., 1.], 'targets': [], 'gripper': [], 'publish': True}
    def target(msg):
        state['targets'].append(msg)
        p, q = msg.pose.position, msg.pose.orientation
        state['pose'] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    node.create_subscription(PoseStamped, '~/target_pose', target, 1)
    def publish():
        if not state['publish']:
            return
        msg = PoseStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        p, q = msg.pose.position, msg.pose.orientation
        p.x, p.y, p.z = state['pose'][:3]
        q.x, q.y, q.z, q.w = state['pose'][3:]
        feedback.publish(msg)
        image = Image()
        image.header = msg.header
        image.height, image.width, image.step = 2, 3, 12  # 每行包含填充字节。
        image.encoding = 'bgr8'
        image.data = bytes([1, 2, 3] * 3 + [0, 0, 0]) * 2
        images.publish(image)
        joint = JointState()
        joint.header = msg.header
        joint.name, joint.position = ['wrist_3_joint'], [.2]
        joints.publish(joint)
        force = WrenchStamped()
        force.header = msg.header
        force.header.frame_id = 'sensor'
        force.wrench.force.x = 1.
        wrench.publish(force)
    node.create_timer(.01, publish)
    def execute(handle):
        state['gripper'].append(handle.request)
        handle.succeed()
        return ParallelGripperCommand.Result()
    action = ActionServer(node, ParallelGripperCommand, '/rwe_test/gripper', execute)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    health = Health(policy_type='mock', state_names=TCP_NAMES + ['joint_position.wrist_3_joint', 'force.x'],
                    cameras={'observation.images.front': [3, 2, 3]}, action_names=POSE_ACTION + ['cmd_gripper'],
                    reference_frame='base_link', tcp_link='tool0', wrench_frame='sensor', fps=20)
    config = dict(ros_domain_id=231, controller='/rwe_test/controller', read_only=False,
                  data_timeout_s=.3, startup_timeout_s=3., joint_topic='/rwe_test/joints',
                  wrench_topic='/rwe_test/wrench', cameras={'observation.images.front': '/rwe_test/image'},
                  gripper={'enabled': True, 'action_server': '/rwe_test/gripper'})
    try:
        yield config, health, state
    finally:
        executor.shutdown(timeout_sec=3.)
        thread.join(timeout=3.)
        action.destroy()
        node.destroy_node()
        context.shutdown()


def test_ros_observe_step_gripper_and_stale(simulated_controller):
    config, health, state = simulated_controller
    env = UREnv(config, health)
    try:
        obs = env.observe(1)
        assert obs.values['joint_position.wrist_3_joint'] == .2
        assert obs.values['force.x'] == 1.
        assert np.array_equal(obs.images['observation.images.front'].decode()[0, 0], [3, 2, 1])
        env.step([.001, 0., 0., 0., 0., .1, 1.])
        eventually(lambda: len(state['targets']) == 1 and len(state['gripper']) == 1)
        eventually(lambda: env.observe(2).values['tcp_ee_x'] == .001)
        env.step([.002, 0., 0., 0., 0., 0., 1.])
        eventually(lambda: len(state['targets']) == 2)
        assert np.isclose(state['pose'][0], .003)
        assert len(state['gripper']) == 1
        # 相机失败不妨碍用仍新鲜的 TCP 进入保持。
        with env.lock:
            _, image = env.messages['observation.images.front']
            env.messages['observation.images.front'] = (0., image)
        env.hold()
        eventually(lambda: len(state['targets']) == 3)
        state['publish'] = False
        time.sleep(.4)
        with pytest.raises(RuntimeError, match='过期'):
            env.step([0.] * 7)
        assert len(state['targets']) == 3
    finally:
        env.close()


def test_ros_read_only_and_frame_check(simulated_controller):
    config, health, state = simulated_controller
    config['read_only'] = True
    env = UREnv(config, health)
    try:
        env.observe(0)
        with pytest.raises(RuntimeError, match='只读'):
            env.step([0.] * 7)
        env.hold()
        assert not state['targets'] and not state['gripper']
    finally:
        env.close()
    with pytest.raises(ValueError, match='基座或 TCP'):
        UREnv(config, health.model_copy(update={'tcp_link': '错误TCP'}))
