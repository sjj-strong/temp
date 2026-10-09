"""离线验证回 Home 的顺序、失败锁定与反馈验证，不启动机器人。"""
from types import SimpleNamespace as S

import numpy as np
from ur_teleop.episode_home import EpisodeHome


class Future:
    def __init__(self, value):
        self.value = value
    def done(self):
        return True
    def result(self):
        return self.value
    def add_done_callback(self, callback):
        callback(self)


def make_flow(monkeypatch):
    from ur_teleop import episode_home as m
    clock = [0.]
    monkeypatch.setattr(m.time, 'monotonic', lambda: clock[0])
    calls = []
    logger = S(info=lambda *a: None, error=lambda *a: None)
    logger.get_child = lambda name: logger
    n = S(estop=False, finished=False, edges=set(), switch_future=None, list_future=None,
          ready_pub=S(publish=lambda msg: calls.append(('ready', msg.data))),
          core=S(stop=lambda *a: calls.append(('stop',)), target=None),
          get_logger=lambda: logger, cancel_gripper=lambda: calls.append(('cancel',)),
          cfg={'home': {'slave': [0.]*6, 'verify_duration_s': 2., 'move_timeout_s': 10.}},
          joints=np.zeros(6), joints_at=0., x={'tcp_timeout_s': .25},
          actual_pose=lambda: np.array([0., 0., 0., 0., 0., 0., 1.]))
    def switch(a, d):
        calls.append(('switch', a, d))
        return Future(S(ok=True))
    n.switcher = S(switch=switch, switch_ok=lambda f: f.result().ok)
    flow = object.__new__(EpisodeHome)
    flow.node, flow.phase, flow.future, flow.goal = n, None, None, None
    handle = S(accepted=True, cancel_goal_async=lambda: calls.append(('cancel_goal',)),
               get_result_async=lambda: Future(S(status=4, result=S(error_code=0))))
    flow.client = S(server_is_ready=lambda: True,
                    send_goal_async=lambda goal: (calls.append(('goal', goal)), Future(handle))[1])
    return flow, n, calls, clock


def test_return_then_restore_and_reset_target(monkeypatch):
    f, n, calls, clock = make_flow(monkeypatch)
    f.request(S(data=True))
    for _ in range(4):
        assert f.tick()
    assert f.phase == 'verify'
    n.joints = np.ones(6)
    f.tick()
    assert f.stable_since is None
    n.joints = np.zeros(6)
    f.tick()
    clock[0] = n.joints_at = 2.1
    f.tick()
    assert f.phase == 'impedance'
    f.tick()
    assert f.phase is None and n.controller_active and not n.finished
    switches = [c for c in calls if c[0] == 'switch']
    assert switches == [
        ('switch', ['scaled_joint_trajectory_controller'], ['cartesian_impedance_controller']),
        ('switch', ['cartesian_impedance_controller'], ['scaled_joint_trajectory_controller'])]
    goal = next(c[1] for c in calls if c[0] == 'goal')
    assert list(goal.trajectory.points[0].positions) == [0.]*6
    np.testing.assert_allclose(n.core.target, n.actual_pose())


def test_switch_rejection_locks_without_goal(monkeypatch):
    f, n, calls, clock = make_flow(monkeypatch)
    n.switcher.switch = lambda *a: Future(S(ok=False))
    f.request(S(data=True))
    f.tick(); f.tick()
    assert f.phase == 'failed' and n.finished
    assert not any(c[0] == 'goal' for c in calls)


def test_timeout_and_late_acceptance_cancel(monkeypatch):
    f, n, calls, clock = make_flow(monkeypatch)
    f.request(S(data=True))
    f.tick(); f.tick()
    clock[0] = 11.
    f.tick()
    assert f.phase == 'failed'
    f.cancel_late_goal(f.future)
    assert ('cancel_goal',) in calls


def test_duplicate_request_and_estop(monkeypatch):
    f, n, calls, clock = make_flow(monkeypatch)
    f.request(S(data=True)); f.request(S(data=True))
    assert calls.count(('ready', False)) == 1
    n.estop = True
    f.tick()
    assert f.phase == 'failed' and n.finished


def test_rejected_goal_does_not_restore_impedance(monkeypatch):
    f, n, calls, clock = make_flow(monkeypatch)
    f.client.send_goal_async = lambda goal: Future(S(accepted=False))
    f.request(S(data=True))
    f.tick(); f.tick(); f.tick()
    assert f.phase == 'failed' and n.finished
    assert len([c for c in calls if c[0] == 'switch']) == 1


def test_stale_feedback_does_not_switch(monkeypatch):
    f, n, calls, clock = make_flow(monkeypatch)
    n.joints_at = -1.
    f.request(S(data=True)); f.tick()
    assert f.phase == 'failed'
    assert not any(c[0] == 'switch' for c in calls)
