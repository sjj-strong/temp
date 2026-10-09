"""Xbot 每段采集结束后的异步回 Home 流程。"""
import time

import numpy as np
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool

from ur_teleop.config import UR_JOINT_NAMES
from ur_teleop.session_logging import log_event


class EpisodeHome:
    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, FollowJointTrajectory,
                                   '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.phase = None
        self.future = self.goal = None
        self.stable_since = None

    def request(self, msg):
        if not msg.data or self.phase is not None:
            return
        self.phase = 'start'
        self.node.core.stop()
        log_event(self.node, 'home_started', message='episode 结束，切轨迹控制器回 Home')
        self.node.ready_pub.publish(Bool(data=False))

    def fail(self, reason):
        self.phase = 'failed'
        self.node.finished = True
        if self.goal is not None and self.goal.accepted:
            try:
                self.goal.cancel_goal_async()
            except Exception as exc:
                reason += '；取消轨迹请求失败：' + str(exc)
        log_event(self.node, 'home_failed', level='error', message=reason)

    def wait(self, phase, future, timeout):
        if future is None:
            self.fail('控制服务未就绪')
            return
        self.phase, self.future = phase, future
        self.deadline = time.monotonic() + timeout

    def cancel_late_goal(self, future):
        if self.phase == 'failed':
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()

    def tick(self):
        if self.phase is None:
            return False
        n, now = self.node, time.monotonic()
        n.edges.clear()
        if self.phase == 'failed':
            return True
        try:
            if n.estop:
                self.fail('软件停止中断回 Home')
            elif self.phase == 'start':
                if n.switch_future is not None:
                    self.fail('存在未完成的控制器切换或查询，请重新启动')
                elif (n.joints is None or now - n.joints_at > n.x['tcp_timeout_s'] or
                      not np.isfinite(n.joints).all()):
                    self.fail('关节反馈无效，禁止回 Home')
                else:
                    n.list_future = None
                    n.cancel_gripper()
                    self.wait('trajectory', n.switcher.switch(
                        ['scaled_joint_trajectory_controller'], ['cartesian_impedance_controller']), 10.)
            elif now > self.deadline:
                self.fail('回 Home 流程超时：' + self.phase)
            elif self.phase in ('trajectory', 'impedance') and self.future.done():
                if not n.switcher.switch_ok(self.future):
                    self.fail('控制器切换失败：' + self.phase)
                elif self.phase == 'impedance':
                    actual = n.actual_pose()
                    if actual is None:
                        self.fail('切回阻抗后 TCP 反馈无效')
                    else:
                        n.core.target = actual.copy()
                        n.core.fault_pose_latched = False
                        n.core.stop(actual)
                        n.controller_active = True
                        n.awaiting_controller_confirmation = True
                        n.list_future = None
                        n.controllers_at = time.monotonic()
                        n.controllers = {'cartesian_impedance_controller': 'active',
                                         'scaled_joint_trajectory_controller': 'inactive'}
                        self.phase = None
                        log_event(n, 'home_reached', message='已回 Home 并切回阻抗；松开后重新按 RB')
                else:
                    n.controller_active = False
                    if not self.client.server_is_ready():
                        self.fail('Home 轨迹 Action 未就绪')
                    else:
                        goal = FollowJointTrajectory.Goal()
                        goal.trajectory.joint_names = list(UR_JOINT_NAMES)
                        point = JointTrajectoryPoint()
                        point.positions = list(n.cfg['home']['slave'])
                        duration = float(n.cfg['home'].get('move_duration_s', 8.))
                        point.time_from_start.sec = int(duration)
                        point.time_from_start.nanosec = int((duration % 1) * 1e9)
                        goal.trajectory.points = [point]
                        self.wait('accepted', self.client.send_goal_async(goal), 10.)
                        self.future.add_done_callback(self.cancel_late_goal)
            elif self.phase == 'accepted' and self.future.done():
                self.goal = self.future.result()
                if self.goal is None or not self.goal.accepted:
                    self.fail('Home 轨迹被拒绝')
                else:
                    self.wait('result', self.goal.get_result_async(),
                              float(n.cfg['home'].get('move_timeout_s', 60.)))
            elif self.phase == 'result' and self.future.done():
                from action_msgs.msg import GoalStatus
                result = self.future.result()
                if (result.status != GoalStatus.STATUS_SUCCEEDED or
                        result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                    self.fail('Home 轨迹执行失败')
                else:
                    self.phase = 'verify'
                    self.deadline = now + float(n.cfg['home'].get('move_timeout_s', 60.))
                    self.stable_since = None
            elif self.phase == 'verify':
                fresh = now - n.joints_at <= n.x['tcp_timeout_s']
                at_home = (fresh and n.joints is not None and np.isfinite(n.joints).all() and
                           np.max(np.abs(n.joints - n.cfg['home']['slave'])) <=
                           n.cfg['home'].get('at_home_tolerance_rad', .05))
                if not at_home:
                    self.stable_since = None
                elif self.stable_since is None:
                    self.stable_since = now
                elif now - self.stable_since >= n.cfg['home'].get('verify_duration_s', 2.):
                    self.wait('impedance', n.switcher.switch(
                        ['cartesian_impedance_controller'], ['scaled_joint_trajectory_controller']), 10.)
        except Exception as exc:
            self.fail(str(exc))
        return True
