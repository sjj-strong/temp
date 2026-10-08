"""ruckig_node 纯逻辑单测（无 ROS 消息依赖：fake msg 用 SimpleNamespace）。

覆盖：
- UR_JOINT_NAMES / UR_JOINT_INDEX 常量
- joint_state_callback 按名重排（乱序/混入非 UR 关节/velocity 缺失防护）
- target_callback 校验（维度 / NaN / Inf）
- initialize_ruckig 边界（关节未全收到 / 初始状态 = 当前状态）
- control_loop 一步 OTG（发布 6 维有限指令、pass_to_input 状态延续）

不实例化真实 rclpy Node：`object.__new__(RuckigNode)` 绕过 __init__，
手动注入回调所需属性；Ruckig/InputParameter/OutputParameter 用真实库对象
（ruckig 0.19.4 已装，C++ 绑定无需 ROS）。
"""

from types import SimpleNamespace

import numpy as np
import pytest

from ruckig import InputParameter, OutputParameter, Ruckig

from ur_teleop.ruckig_node import DOF, UR_JOINT_INDEX, UR_JOINT_NAMES, RuckigNode


class FakePub:
    def __init__(self):
        self.sent = []
        self.raw_sent = []

    def publish(self, msg):
        self.raw_sent.append(msg)
        self.sent.append(list(msg.position if hasattr(msg, "position") else msg.data))


class FakeLogger:
    def get_child(self, name):
        return self

    def info(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


def _make_node():
    """绕过 __init__ 的最小可用节点（不含 ROS 上下文）。"""
    node = object.__new__(RuckigNode)
    node.robot_q = np.zeros(DOF)
    node.robot_dq = np.zeros(DOF)
    node.robot_joint_valid = np.zeros(DOF, dtype=bool)
    node.target_q = None
    node.otg = Ruckig(DOF, 0.01)
    node.inp = InputParameter(DOF)
    node.out = OutputParameter(DOF)
    node.inp.max_velocity = [0.30] * DOF
    node.inp.max_acceleration = [0.80] * DOF
    node.inp.max_jerk = [4.0] * DOF
    node.inp.target_velocity = [0.0] * DOF
    node.inp.target_acceleration = [0.0] * DOF
    node.initialized = False
    node.controller_kind = "forward_position"
    node.command_pub = FakePub()
    node.get_logger = lambda: FakeLogger()
    return node


def _joint_msg(names, positions, velocities):
    return SimpleNamespace(
        name=names,
        position=positions,
        velocity=velocities,
    )


# ------------------------------------------------------------
# 常量
# ------------------------------------------------------------

def test_joint_name_constants():
    assert UR_JOINT_NAMES == [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    assert len(UR_JOINT_INDEX) == DOF
    for i, name in enumerate(UR_JOINT_NAMES):
        assert UR_JOINT_INDEX[name] == i


# ------------------------------------------------------------
# joint_state_callback：按名重排
# ------------------------------------------------------------

def test_joint_state_callback_reorders_by_name():
    node = _make_node()
    # 乱序 + 位置/速度各自由度的典型实测顺序
    msg = _joint_msg(
        names=["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
        positions=[0.3, -1.2, 0.1, -1.0, 0.5, 0.2],
        velocities=[0.01, -0.02, 0.03, 0.04, -0.05, 0.06],
    )
    node.joint_state_callback(msg)

    assert np.allclose(node.robot_q, [0.1, -1.2, 0.3, -1.0, 0.5, 0.2])
    assert np.allclose(node.robot_dq, [0.03, -0.02, 0.01, 0.04, -0.05, 0.06])
    assert node.robot_joint_valid.all()


def test_joint_state_callback_skips_non_ur_joints():
    node = _make_node()
    # 主臂关节混入同一话题（真实场景：/joint_states 双发布者）
    msg = _joint_msg(
        names=["Joint1", "shoulder_pan_joint", "Gripper", "shoulder_lift_joint",
               "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
        positions=[0.9, 0.1, 0.5, -1.2, 0.3, -1.0, 0.5, 0.2],
        velocities=[0.0] * 8,
    )
    node.joint_state_callback(msg)

    assert np.allclose(node.robot_q, [0.1, -1.2, 0.3, -1.0, 0.5, 0.2])
    assert node.robot_joint_valid.all()


def test_joint_state_callback_handles_missing_velocity():
    node = _make_node()
    # velocity 数组比 name 短（或缺失）时不崩溃，position 仍更新
    msg = _joint_msg(
        names=UR_JOINT_NAMES,
        positions=[0.1, -1.2, 0.3, -1.0, 0.5, 0.2],
        velocities=[0.01, 0.02],  # 只给了前两个
    )
    node.joint_state_callback(msg)

    assert np.allclose(node.robot_q, [0.1, -1.2, 0.3, -1.0, 0.5, 0.2])
    assert node.robot_joint_valid.all()
    # 只更新了有 velocity 的项
    assert node.robot_dq[0] == pytest.approx(0.01)
    assert node.robot_dq[1] == pytest.approx(0.02)
    assert node.robot_dq[2] == pytest.approx(0.0)


def test_joint_state_callback_requires_all_joints_for_initialize():
    node = _make_node()
    # 只收到部分 UR 关节 → valid 不全 → initialize 拒绝
    msg = _joint_msg(
        names=["shoulder_pan_joint", "shoulder_lift_joint"],
        positions=[0.1, -1.2],
        velocities=[0.0, 0.0],
    )
    node.joint_state_callback(msg)
    assert node.robot_joint_valid.sum() == 2
    assert not node.initialize_ruckig()


# ------------------------------------------------------------
# target_callback：校验
# ------------------------------------------------------------

def test_target_callback_updates_target():
    node = _make_node()
    msg = SimpleNamespace(data=[0.5, -1.5, 0.2, -1.2, 0.3, 0.0])
    node.target_callback(msg)
    assert node.target_q is not None
    assert np.allclose(node.target_q, [0.5, -1.5, 0.2, -1.2, 0.3, 0.0])


def test_target_callback_rejects_wrong_length():
    node = _make_node()
    node.target_callback(SimpleNamespace(data=[0.5, -1.5, 0.2]))  # 3 维
    assert node.target_q is None


def test_target_callback_rejects_nan_inf():
    node = _make_node()
    node.target_callback(SimpleNamespace(data=[0.5, np.nan, 0.2, -1.2, 0.3, 0.0]))
    assert node.target_q is None
    node.target_callback(SimpleNamespace(data=[0.5, np.inf, 0.2, -1.2, 0.3, 0.0]))
    assert node.target_q is None


# ------------------------------------------------------------
# initialize_ruckig：初始状态 = 当前 UR 状态
# ------------------------------------------------------------

def test_initialize_ruckig_sets_current_and_target():
    node = _make_node()
    q = [0.1, -1.2, 0.3, -1.0, 0.5, 0.2]
    dq = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    node.joint_state_callback(_joint_msg(UR_JOINT_NAMES, q, dq))

    assert node.initialize_ruckig()
    assert node.initialized

    # 初始状态来自 UR 实测
    assert np.allclose(node.inp.current_position, q)
    assert np.allclose(node.inp.current_velocity, dq)
    assert node.inp.current_acceleration == [0.0] * DOF
    # 启动目标 = 当前位置 → 节点启动本身不产生运动
    assert np.allclose(node.inp.target_position, q)
    assert node.target_q is not None
    assert np.allclose(node.target_q, q)


# ------------------------------------------------------------
# control_loop：OTG 一步
# ------------------------------------------------------------

def test_control_loop_publishes_finite_command():
    node = _make_node()
    q = [0.0] * DOF
    node.joint_state_callback(_joint_msg(UR_JOINT_NAMES, q, [0.0] * DOF))
    assert node.initialize_ruckig()

    # 收到一个目标，走一步 OTG
    node.target_callback(SimpleNamespace(data=[0.5, -1.5, 0.2, -1.2, 0.3, 0.0]))
    node.control_loop()

    assert len(node.command_pub.sent) == 1
    cmd = node.command_pub.sent[0]
    assert len(cmd) == DOF
    assert all(np.isfinite(cmd))
    # 首步指令 = 当前位置（OTG 从当前状态出发，不会跳变）
    assert np.allclose(cmd, q, atol=1e-6)


def test_control_loop_passes_output_to_input():
    node = _make_node()
    node.joint_state_callback(
        _joint_msg(UR_JOINT_NAMES, [0.0] * DOF, [0.0] * DOF))
    assert node.initialize_ruckig()

    node.target_callback(SimpleNamespace(data=[0.5, -1.5, 0.2, -1.2, 0.3, 0.0]))
    node.control_loop()
    # pass_to_input：下一步的 current = 本步 output
    assert np.allclose(node.inp.current_position, node.out.new_position)
    assert np.allclose(node.inp.current_velocity, node.out.new_velocity)


def test_control_loop_tracks_target_over_steps():
    node = _make_node()
    node.joint_state_callback(
        _joint_msg(UR_JOINT_NAMES, [0.0] * DOF, [0.0] * DOF))
    assert node.initialize_ruckig()

    target = [0.5, -1.5, 0.2, -1.2, 0.3, 0.0]
    node.target_callback(SimpleNamespace(data=target))

    # 跑足够多步：指令单调趋近目标并最终到达（dt=0.01 下 1000 步 = 10 s）
    for _ in range(2000):
        node.control_loop()
        node.target_callback(SimpleNamespace(data=target))

    assert np.allclose(node.command_pub.sent[-1], target, atol=1e-3)


def test_control_loop_publishes_named_joint_state_for_impedance():
    node = _make_node()
    node.controller_kind = "joint_impedance"
    node.joint_state_callback(
        _joint_msg(UR_JOINT_NAMES, [0.0] * DOF, [0.0] * DOF))
    assert node.initialize_ruckig()
    node.target_callback(SimpleNamespace(data=[0.1] * DOF))
    node.control_loop()

    assert len(node.command_pub.sent) == 1
    command = node.command_pub.raw_sent[0]
    assert list(command.name) == UR_JOINT_NAMES
    assert len(command.position) == DOF
