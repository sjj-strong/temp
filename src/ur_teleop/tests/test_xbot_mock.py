"""需先在隔离域启动 Xbot mock Home；不允许在默认域执行。"""
import os
import time

import pytest
import yaml
import numpy as np

pytestmark = pytest.mark.skipif(
    os.environ.get('UR_XBOT_MOCK_TEST') != '1' or os.environ.get('ROS_DOMAIN_ID') != '225',
    reason='仅显式选择的 ROS_DOMAIN_ID=225 mock 集成测试运行')


def test_joy_switch_motion_and_fault(tmp_path):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool, Float64MultiArray, String
    from ur_teleop.xbot_core import AXES, BUTTONS
    from ur_teleop.xbot_teleop_node import XbotTeleopNode

    mapping = dict(version=1, axis_count=7, button_count=8,
                   buttons=dict(zip(BUTTONS, range(8))),
                   axes={n: dict(index=i, rest=0., positive=1.) for i, n in enumerate(AXES)})
    calibration = tmp_path / 'joy.yaml'
    calibration.write_text(yaml.safe_dump(mapping))
    config = tmp_path / 'config.yaml'
    config.write_text(yaml.safe_dump(dict(
        base_config='/ros2_ws/src/ur_teleop/config/xbot_teleop.yaml', sim=True,
        recorder=dict(action_mode='rel'),
        xbot=dict(calibration_file=str(calibration)))))
    rclpy.init(args=['--ros-args', '-p', f'config_file:={config}'])
    node = XbotTeleopNode()
    probe = Node('xbot_mock_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)
    joy = probe.create_publisher(Joy, '/joy', 1)
    estop = probe.create_publisher(Bool, '/teleop/e_stop', 1)
    commands = []
    events = []
    probe.create_subscription(Float64MultiArray, '/teleop/commands', lambda m: commands.append(list(m.data)), 10)
    probe.create_subscription(String, '/teleop/record_event', lambda m: events.append(m.data), 10)
    axes, buttons = [0.]*7, [0]*8

    def run(seconds, send=True):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            if send:
                joy.publish(Joy(axes=axes, buttons=buttons))
            # 排空高频 TF/关节反馈，避免测试自身把执行器限制为每秒 100 个回调。
            spin_until = time.monotonic() + .015
            while time.monotonic() < spin_until:
                executor.spin_once(timeout_sec=.001)

    try:
        run(2.)
        # DDS 发现和首次 TF 查询异步完成；不将固定等待时间当作就绪条件。
        deadline = time.monotonic() + 8.
        while time.monotonic() < deadline:
            if (node.controller_active and node.core.released and
                    node.actual_pose() is not None and
                    time.monotonic() - node.joints_at < .1):
                break
            run(.05)
        else:
            pytest.fail(f'mock 未就绪: {node.controllers}')
        buttons[0] = 1
        run(2.)
        assert node.controller_active, (node.controllers, node.actual_pose(), node.joints)
        buttons[0] = 0
        run(.3)
        buttons[0], axes[1] = 1, 1.
        commands.clear()
        run(.3)
        assert any(c[0] > 0 for c in commands)
        axes[1] = 0.
        desired_gripper = 0. if node.gripper_state > .4 else 1.
        buttons[2] = 1
        run(1.)
        assert node.gripper_command == desired_gripper, (node.gripper_state, node.gripper_pending,
                                            node.gripper.server_is_ready(), node.core.enabled)
        buttons[2] = 0
        buttons[3] = 1
        run(.1)
        assert node.core.frame == 'tcp'
        buttons[3] = 0
        run(.4, send=False)
        assert not node.core.enabled
        hold_target = node.core.target.copy()
        commands.clear()
        run(.15)
        np.testing.assert_array_equal(node.core.target, hold_target)
        assert all(len(c) == 7 for c in commands)
        buttons[0] = 0
        run(.2)
        buttons[0] = 1
        run(.2)
        assert node.core.enabled
        estop.publish(Bool(data=True))
        run(.15)
        assert not node.core.enabled
        estop.publish(Bool(data=False))
        run(.15)
        assert not node.core.enabled
        buttons[0] = 0
        run(.2)
        for index in (4, 5, 6):
            buttons[index] = 1
            run(.1)
            buttons[index] = 0
            run(.1)
        assert events[-3:] == ['start', 'save', 'discard']
        buttons[7] = 1
        run(2.2)
        assert events[-1] == 'finalize' and node.finished
    finally:
        buttons[:] = [0]*8
        run(.2)
        node.destroy_node()
        probe.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
