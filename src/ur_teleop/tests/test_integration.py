"""Full-stack integration tests: cell(mock) + teleop_node + fake_master.

Excluded by default (pytest.ini addopts). Run with:
  colcon test --packages-select ur_teleop --pytest-args "-m integration"
Requires a sourced ROS environment (and the lerobot venv for the record test).
"""

import concurrent.futures
import os
import re
import select
import signal
import subprocess
import sys
import time
import types
from pathlib import Path

import pytest
import rclpy
from sensor_msgs.msg import JointState

from ur_teleop.config import ALICIA_JOINT_NAMES, GRIPPER_JOINT, UR_JOINT_NAMES
from ur_teleop.controller_switcher import ControllerSwitcher
from ur_teleop.home_node import HomeNode
from ur_teleop.teleop_node import TeleopNode, State

# stdbuf -oL 对 Python 的管道 stdout 无效（Python 在 io 层缓冲，LD_PRELOAD 不生效）；
# 必须在任何子进程 spawn 前设置，ros2 topic echo 的输出才能逐行到达测试侧。
os.environ["PYTHONUNBUFFERED"] = "1"

pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif("AMENT_PREFIX_PATH" not in os.environ,
                       reason="需要已 source 的 ROS 环境"),
]

SELF = Path(__file__).resolve().parent
REPO = SELF.parent
FAKE_MASTER = SELF / "fake_master.py"

# 与 CFG_BODY home.slave 一致（= mock UR10e 初始位姿 = config/alicia_teleop.yaml）
SLAVE_HOME = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

CFG_BODY = """\
mode: teleop
sim: true
cell:
  launch_rviz: false
home:
  master: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  # mock UR10e 初始位姿即 UR home pose（与 config/alicia_teleop.yaml 的 home.slave 一致）
  slave: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
  master_gripper_value: 1000
  at_home_tolerance_rad: 0.05
  settle_time_s: 2.0
  settle_motion_threshold_rad: 0.01
  move_timeout_s: 30.0
mapping:
  alicia_joint_order: [Joint1, Joint2, Joint3, Joint4, Joint5, Joint6]
  ur_joint_order: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
  sign: [1, 1, 1, 1, 1, 1]
  scale: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
safety:
  clamp_margin_rad: 0.1
  limits:
    shoulder_pan_joint: [-6.283, 6.283]
    shoulder_lift_joint: [-6.283, 6.283]
    elbow_joint: [-3.142, 3.142]
    wrist_1_joint: [-6.283, 6.283]
    wrist_2_joint: [-6.283, 6.283]
    wrist_3_joint: [-6.283, 6.283]
teleop:
  command_rate_hz: 50
  watchdog_timeout_s: 0.5
  restore_controller_on_exit: true
gripper:
  enabled: false
recorder:
  repo_id: test/ur_teleop_it
  root: ""
  fps: 50
  robot_type: ur10e_alicia_teleop
  use_videos: false
  cameras: {}
  min_frames_per_episode: 2
"""


def _start(*argv, cwd=REPO, stdin=None):
    p = subprocess.Popen(list(argv), cwd=str(cwd), stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, stdin=stdin,
                         start_new_session=True)
    # Capture the process group id at spawn: the group leader can exit before
    # teardown, after which os.getpgid(pid) fails and the rest of the group
    # (cell/controller_manager children) would be orphaned.
    p.pgid = os.getpgid(p.pid)
    return p


def _kill(procs):
    for p in procs:
        try:
            os.killpg(p.pgid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
    time.sleep(1.0)
    for p in procs:
        try:
            os.killpg(p.pgid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass


def _sweep_orphaned_control_nodes():
    """清扫孤儿 controller_manager：cell 的 CM 经 --params-file /tmp/launch_params_*
    启动（路径不含测试 tmp_path），pytest 被外部杀死（finally 不执行）时它可能随
    launch 组长死亡而脱管残留，与当前测试的 CM 同时服务 switch_controller →
    SWITCHING 不确定、ACTIVE 永远到不了（2026-08-10 实况：SIGABRT 残留的 CM
    使 record 测试连续 2 次 60 s enable 窗口全失败）。
    判别：cmdline 含 ros2_control_node 且其 pgid 组长已不存在 → 孤儿，整组击杀。
    用户 mock 栈（组长即存活 launch）与当前测试的 CM（组长存活）均不匹配，
    误杀风险为零。"""
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\x00", b" ").decode()
            pgid = os.getpgid(int(d))
        except (OSError, ValueError):
            continue
        if "ros2_control_node" not in cmd:
            continue
        if not os.path.exists(f"/proc/{pgid}"):
            try:
                os.killpg(pgid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass


def _sweep(tmp_path):
    """清扫本测试可能残留的子进程：pytest 被外部终止（超时/断电）时 finally
    不会执行，残留的 fake_master（锁存 sine 发布）会污染后续测试的 VERIFY_HOME。
    tmp_path 出现在 stack 各进程的 cmdline（config_file:=...），fake 按路径匹配。
    注意 pkill -f tests/fake_master.py 会命中同一仓库的并行测试运行（同路径），
    误杀风险并非绝对为零；孤儿 CM 清扫以组长存活判别，用户 mock 栈（组长存活）
    不受影响。"""
    subprocess.run(["pkill", "-9", "-f", str(tmp_path)], capture_output=True)
    subprocess.run(["pkill", "-9", "-f", "tests/fake_master.py"], capture_output=True)
    _sweep_orphaned_control_nodes()
    time.sleep(0.5)


def _topic_once(topic, field=None, timeout=6.0):
    """ros2 topic echo --once; returns (ok, payload). ok=False on timeout/no message."""
    cmd = ["ros2", "topic", "echo", topic, "--once"]
    if field:
        cmd += ["--field", field]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return (out.returncode == 0 and bool(out.stdout.strip())), out.stdout.strip()
    except subprocess.TimeoutExpired:
        return False, ""


def _pub_bool(topic, value):
    subprocess.run(["ros2", "topic", "pub", "--once", topic, "std_msgs/msg/Bool",
                    f"{{data: {str(value).lower()}}}"], capture_output=True, timeout=10.0)


def _pub_bool_window(topic, value, times=10, rate=2):
    """窗口化发布 Bool：--once 发布者常在 DDS 发现完成前退出导致消息丢失，
    窗口发布让发布者存活 ~5 s，发现完成后持续送达（enable/e_stop 等锁存信号用）。"""
    subprocess.run(
        ["ros2", "topic", "pub", "--times", str(times), "--rate", str(rate),
         topic, "std_msgs/msg/Bool", f"{{data: {str(value).lower()}}}"],
        capture_output=True, timeout=15.0)


class _OnceEcho:
    """长驻 `ros2 topic echo --once`：订阅一次，捕获下一条消息。

    status/demonstration 是单次信号（teleop 仅在状态迁移时发布一次），
    每次轮询新建 --once echo 会因 DDS 发现延迟错过；本类在信号前订阅，
    next() 阻塞读取到消息或超时。

    msg_type: 显式消息类型（如 std_msgs/msg/Bool）。不带类型时 echo 在
    spawn 瞬间做一次非阻塞图查询，而新参与者的 DDS 发现尚未完成 →
    常以 "Could not determine the type for the passed topic" 立即退出
    （单次信号必丢）；显式类型跳过该查询，订阅确定性建立。
    """

    def __init__(self, topic, field="data", msg_type=None):
        cmd = ["ros2", "topic", "echo", topic]
        if msg_type is not None:
            cmd.append(msg_type)
        cmd += ["--field", field, "--once"]
        self._p = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

    def next(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            r, _, _ = select.select([self._p.stdout], [], [],
                                    max(0.0, deadline - time.time()))
            if not r:
                return None
            line = self._p.stdout.readline()
            if not line:
                return None                   # EOF（echo 已退出）
            if line.startswith("WARNING:"):
                continue                      # ros2 echo 的发布者发现提示，非消息负载
            return line.strip()
        return None

    def close(self):
        self._p.kill()


class _LineEcho:
    """长驻逐行 echo（stdbuf -oL 行缓冲）：连续流消息（commands 等）。

    与 _OnceEcho 不同：不退出、不断开，可持续读取后续消息；next() 超时
    返回 None（静默检测——e_stop 下 teleop 的 _tick 提前返回、不再发布）。
    管道输出必须行缓冲，否则块缓冲会把消息积压在内存里读不到。
    """

    def __init__(self, topic, field="data"):
        self._p = subprocess.Popen(
            ["stdbuf", "-oL", "ros2", "topic", "echo", topic, "--field", field],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

    def next(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            r, _, _ = select.select([self._p.stdout], [], [],
                                    max(0.0, deadline - time.time()))
            if not r:
                return None
            line = self._p.stdout.readline()
            if line:
                return line.strip()
            return None                       # EOF（echo 已退出）
        return None

    def drain(self, timeout=0.5):
        """读取并丢弃已缓冲的行（清掉冻结前最后几条在途指令，再断言静默）。"""
        while self.next(timeout) is not None:
            pass

    def close(self):
        self._p.kill()


def _wait_status(value, timeout=25.0):
    """等待 /teleop/status 的下一条消息为给定值（长驻 --once echo，事后订阅会错过）。"""
    echo = _OnceEcho("/teleop/status", msg_type="std_msgs/msg/Bool")
    try:
        got = echo.next(timeout)
        return got is not None and str(value).lower() in got.lower()
    finally:
        echo.close()


def _enable_and_wait(timeout=60.0):
    """窗口化发布 /teleop/enable 直到 ACTIVE。

    ACTIVE 判定双通道：
    - /teleop/status=true（状态迁移单次发布，信号前订阅的长驻 echo 捕获）；
    - /ruckig/target_joint_positions 出现新数据（50 Hz 流）。record 测试
      中 enable 由 recorder 的 Enter 先行发出，status 的 ACTIVE 迁移可能在测试
      订阅完成前就已发生（稳定 ACTIVE 不再发布 status），靠指令流兜底。
    注：teleop 仅在 ACTIVE 发布映射指令（INACTIVE 的 hold 需要主臂超时，假主臂
    常驻时不会发生），故指令流可作为 ACTIVE 判据。"""
    echo = _OnceEcho("/teleop/status", msg_type="std_msgs/msg/Bool")
    cmd = _LineEcho("/ruckig/target_joint_positions")
    try:
        deadline = time.time() + timeout
        while time.time() < deadline:
            _pub_bool_window("/teleop/enable", True)
            got = echo.next(2.0)
            if got is not None and "true" in got.lower():
                return True
            if cmd.next(1.0) is not None:
                return True
        return False
    finally:
        echo.close()
        cmd.close()


def _launch_stack(tmp_path, master_off_home=False):
    cfg_path = tmp_path / "alicia_teleop.yaml"
    cfg_path.write_text(CFG_BODY)
    procs = [
        _start("ros2", "launch", "ur_teleop", "cell.launch.py",
               f"config_file:={cfg_path}", "sim:=true", "launch_rviz:=false",
               "launch_alicia:=false"),
        _start("ros2", "run", "ur_teleop", "teleop_node",
               "--ros-args", "-p", f"config_file:={cfg_path}"),
        _start(sys.executable, str(FAKE_MASTER),
               "--off-home" if master_off_home else ""),
    ]
    return procs


def test_enter_gate_then_mirror(tmp_path):
    """无 enable 不 ACTIVE；enable 后 50 Hz 映射发布、clamp 生效、跟随正弦。"""
    _sweep(tmp_path)
    procs = _launch_stack(tmp_path)
    try:
        time.sleep(20.0)                       # cell 启动 + settle 2 s + offset 捕获
        ok, val = _topic_once("/teleop/status", "data", timeout=5.0)
        assert not ok, f"Enter/enable 前不应 ACTIVE，却收到 status={val}"

        assert _enable_and_wait(), "enable 后未进入 ACTIVE（重试发布）"

        samples = []
        for _ in range(4):
            ok, val = _topic_once("/ruckig/target_joint_positions", "data", timeout=5.0)
            assert ok, "未收到 /ruckig/target_joint_positions"
            # 偶发的 DDS 重复投递会在行首拼入残留值 → 取末尾 6 维（teleop 只发 6 维）
            vals = [float(x) for x in re.findall(r"-?\d+\.?\d*", val)]
            samples.append(vals[-6:] if len(vals) >= 6 else vals)
            time.sleep(1.0)
        assert all(len(s) == 6 for s in samples), f"命令维数错误: {samples}"
        lo, hi = -6.283 + 0.1, 6.283 - 0.1
        assert all(lo <= v <= hi for s in samples for v in s), f"指令越出 safety 范围: {samples}"
        spread = [max(x) - min(x) for x in zip(*samples)]
        assert max(spread) > 0.01, f"指令未跟随主臂运动: {samples}"
    finally:
        _kill(procs)
        _sweep(tmp_path)


def test_verify_home_rejects_off_home_master(tmp_path):
    """主臂不在 home 时停在 VERIFY_HOME，不进入 ACTIVE（spec §9）。"""
    _sweep(tmp_path)
    procs = _launch_stack(tmp_path, master_off_home=True)
    try:
        time.sleep(18.0)
        ok, val = _topic_once("/teleop/status", "data", timeout=5.0)
        assert not ok, f"主臂不在 home 时不应 ACTIVE，却收到 status={val}"
    finally:
        _kill(procs)
        _sweep(tmp_path)


def test_record_one_episode(tmp_path):
    """record 冒烟：Enter → ACTIVE → 录帧 → D 丢弃 → Q finalize（spec §11.3）。"""
    try:
        import lerobot  # noqa: F401
    except ImportError:
        pytest.skip("lerobot 未安装（需要 source /opt/lerobot_venv/bin/activate）")

    root = tmp_path / "data"
    cfg_path = tmp_path / "alicia_teleop.yaml"
    cfg_path.write_text(CFG_BODY.replace("mode: teleop", "mode: record")
                                .replace('root: ""', f"root: {root}"))
    _sweep(tmp_path)                           # 必须先于 spawn：pkill 按 tmp_path 匹配 stack cmdline
    procs = [
        _start("ros2", "launch", "ur_teleop", "cell.launch.py",
               f"config_file:={cfg_path}", "sim:=true", "launch_rviz:=false",
               "launch_alicia:=false"),
        _start("ros2", "run", "ur_teleop", "teleop_node",
               "--ros-args", "-p", f"config_file:={cfg_path}"),
        _start(sys.executable, str(FAKE_MASTER)),
        # 注意：data_recorder 用 venv 解释器以 -m 启动（ros2 run 的入口脚本
        # shebang 是系统 python3，缺少 lerobot 会直接崩溃）
        _start(sys.executable, "-m", "ur_teleop.data_recorder",
               "--ros-args", "-p", f"config_file:={cfg_path}", stdin=subprocess.PIPE),
    ]
    recorder = procs[-1]
    try:
        time.sleep(20.0)
        # KeyboardReader 把 '\n' 映射为 enter → 只写换行字节即开始 episode。
        # 注意其余按键必须单字节写入、不带 '\n'：'d\n' 会被读成 d + enter，
        # 丢弃后立刻又开新 episode，Q 时 finalize 会把新 episode 保存（实测 48 帧）。
        recorder.stdin.write("\n")
        recorder.stdin.flush()                 # 开始 episode 1 + 发 /teleop/enable
        assert _enable_and_wait(), "Enter 后未进入 ACTIVE（重试发布 enable）"
        time.sleep(3.0)                        # 录 ~150 帧
        recorder.stdin.write("d")
        recorder.stdin.flush()                 # 丢弃
        time.sleep(1.0)
        recorder.stdin.write("q")
        recorder.stdin.flush()                 # 退出 + finalize
        recorder.wait(timeout=15.0)
        # 本地布局（lerobot 0.5.x 显式 root）：<root>/meta/info.json 在 create 时
        # 即落盘；repo_id 仅是元数据，不存在 <root>/test/ur_teleop_it 目录
        assert (root / "meta" / "info.json").exists(), "数据集 meta/info.json 未创建"
        # D 丢弃后 0 episode 保存：data/chunk-000 仅在 save_episode 时创建
        assert not (root / "data" / "chunk-000").exists(), "丢弃的 episode 不应落盘"
        log = recorder.stdout.read()
        assert "finalize" in log.lower(), f"recorder 日志未见 finalize:\n{log[-2000:]}"
    finally:
        _kill(procs)
        _sweep(tmp_path)


# ============================================================================
# 追加覆盖（Task 11 派发 A–D）：in-process FSM 白盒 + 子进程冒烟
# ============================================================================

def _kill_one(p):
    """Kill a single process group (used to drop fake_master mid-test)."""
    try:
        os.killpg(p.pgid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        pass
    time.sleep(1.0)
    try:
        os.killpg(p.pgid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass


class _FakeFuture:
    """可控的 rclpy Future 替身：done()/result()；finish() 标记完成（白盒 FSM 测试用）。"""

    def __init__(self, result=None, exception=None, cancelled=False, done=True):
        self._result = result
        self._exception = exception
        self._cancelled = cancelled
        self._done = done

    def done(self):
        return self._done

    def result(self, timeout=None):
        if not self._done:
            raise RuntimeError("future not done")
        if self._exception is not None:
            raise self._exception
        if self._cancelled:
            raise concurrent.futures.CancelledError()
        return self._result

    def finish(self, result=None, exception=None):
        self._done = True
        self._result = result
        self._exception = exception


def _make_pending(store):
    f = _FakeFuture(done=False)
    store.append(f)
    return f


def _ctl(name, state="active"):
    return types.SimpleNamespace(name=name, state=state)


def _list_resp(controllers):
    return types.SimpleNamespace(controller=controllers)


def _load_resp(ok=True):
    return types.SimpleNamespace(ok=ok)


def _switch_resp(ok=True):
    return types.SimpleNamespace(ok=ok)


def _inject_joints(node, master=None, slave=None, gripper=None):
    # Jazzy 的 JointState.position 是 array.array：先拼 list 再整体赋值。
    names, positions = [], []
    if master is not None:
        names += ALICIA_JOINT_NAMES
        positions += list(master)
    if slave is not None:
        names += UR_JOINT_NAMES
        positions += list(slave)
    if gripper is not None:
        names += [GRIPPER_JOINT]
        positions += [gripper]
    msg = JointState()
    msg.name = names
    msg.position = positions
    node._joint_cb(msg)


class _CapturePub:
    def __init__(self, store):
        self._store = store

    def publish(self, msg):
        self._store.append(msg)


def test_disabled_ruckig_directly_publishes_impedance_target(fsm_node):
    """关闭 Ruckig 后，遥操目标应直接进入阻抗控制器目标话题格式。"""
    node = fsm_node
    messages = []
    node._use_ruckig = False
    node._controller_kind = "joint_impedance"
    node._direct_target_pub = _CapturePub(messages)

    node._publish_commands([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

    assert len(messages) == 1
    assert messages[0].name == [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    ]
    assert list(messages[0].position) == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]


def test_forward_position_ignores_ruckig_enabled_flag(fsm_node):
    """Ruckig 开关不能改变前向位置控制器既有的平滑链路。"""
    assert fsm_node._controller_kind == "forward_position"
    assert fsm_node._use_ruckig is True


@pytest.fixture
def fsm_node(tmp_path, monkeypatch):
    """teleop_node in-process：settle_time_s=0.1 加速；不 spin，直接驱动。"""
    cfg_path = tmp_path / "alicia_teleop.yaml"
    cfg_path.write_text(
        CFG_BODY.replace("settle_time_s: 2.0", "settle_time_s: 0.1")
        + "\nruckig:\n  enabled: false\n"
    )
    monkeypatch.setattr("ur_teleop.teleop_node.default_config_path", lambda: str(cfg_path))
    rclpy.init()
    node = TeleopNode()
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


@pytest.fixture
def home_node(tmp_path, monkeypatch):
    cfg_path = tmp_path / "alicia_teleop.yaml"
    cfg_path.write_text(CFG_BODY)
    monkeypatch.setattr("ur_teleop.home_node.default_config_path", lambda: str(cfg_path))
    rclpy.init()
    node = HomeNode()
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


# ---------- A. teleop_node FSM ----------

def test_switching_chain_list_load_switch_active(fsm_node):
    """A1: list → (fwd 未加载) → load → switch → ACTIVE（状态与 phase 顺序）。"""
    node = fsm_node
    list_fut = _FakeFuture(done=False)
    load_fut = _FakeFuture(done=False)
    switch_fut = _FakeFuture(done=False)
    node._switcher.list_controllers = lambda: list_fut
    node._switcher.load_controller = lambda name: load_fut
    node._switcher.switch = lambda a, d: switch_fut
    node._begin_switch()
    assert node._state == State.SWITCHING and node._switch_phase == "list"
    node._tick()                               # in-flight：不阻塞不前进
    assert node._state == State.SWITCHING and node._switch_phase == "list"
    list_fut.finish(_list_resp([_ctl("scaled_joint_trajectory_controller")]))
    node._tick()                               # list 完成且无 fwd → load
    assert node._switch_phase == "load"
    load_fut.finish(_load_resp(ok=True))
    node._tick()                               # load ok → switch
    assert node._switch_phase == "switch"
    switch_fut.finish(_switch_resp(ok=True))
    node._tick()                               # switch ok → ACTIVE
    assert node._state == State.ACTIVE


def test_switching_load_failure_returns_to_armed(fsm_node):
    """A2: load 响应 ok=false → 回到 ARMED。"""
    node = fsm_node
    node._switcher.list_controllers = lambda: _FakeFuture(
        _list_resp([_ctl("scaled_joint_trajectory_controller")]))
    node._switcher.load_controller = lambda name: _FakeFuture(_load_resp(ok=False))
    node._begin_switch()
    node._tick()                               # list → load
    assert node._switch_phase == "load"
    node._tick()                               # load ok=false → ARMED
    assert node._state == State.ARMED
    assert node._switch_attempt == 0           # load 失败不计入 switch 重试


def test_switching_five_failures_caps_to_armed(fsm_node):
    """A3: switch 连续失败 5 次 → ARMED（attempt 计数 5）。"""
    node = fsm_node
    made = []
    node._switcher.list_controllers = lambda: _FakeFuture(
        _list_resp([_ctl("forward_position_controller")]))
    node._switcher.switch = lambda a, d: _make_pending(made)
    node._begin_switch()
    node._tick()                               # fwd 已加载 → 直接 switch
    assert node._switch_phase == "switch"
    for attempt in range(1, 6):
        made[-1].finish(_switch_resp(ok=False))
        node._tick()
        if attempt < 5:
            assert node._state == State.SWITCHING, f"第 {attempt} 次失败应重试"
        else:
            assert node._state == State.ARMED, "第 5 次失败应回到 ARMED"
    assert node._switch_attempt == 5


def test_switching_list_exception_falls_through_to_load(fsm_node):
    """A4a: list future.result() 抛异常 → 视为未加载，走 load 路径。"""
    node = fsm_node
    node._switcher.list_controllers = lambda: _FakeFuture(
        exception=RuntimeError("controller_manager died mid-call"))
    node._switcher.load_controller = lambda name: _FakeFuture(_load_resp(ok=True))
    node._switcher.switch = lambda a, d: _FakeFuture(_switch_resp(ok=True))
    node._begin_switch()
    node._tick()                               # list 异常 → load
    assert node._switch_phase == "load"
    node._tick()                               # load ok → switch
    assert node._switch_phase == "switch"
    node._tick()                               # switch ok → ACTIVE
    assert node._state == State.ACTIVE


def test_switching_switch_exception_retries_then_armed(fsm_node):
    """A4b: switch future 抛异常 → 视为失败重试；5 次后 ARMED，异常不逃逸 _tick。"""
    node = fsm_node
    made = []
    node._switcher.list_controllers = lambda: _FakeFuture(
        _list_resp([_ctl("forward_position_controller")]))
    node._switcher.switch = lambda a, d: _make_pending(made)
    node._begin_switch()
    node._tick()
    assert node._switch_phase == "switch"
    for attempt in range(1, 6):
        made[-1].finish(exception=RuntimeError("controller_manager died mid-call"))
        node._tick()                           # 不得抛异常
        if attempt < 5:
            assert node._state == State.SWITCHING and node._switch_attempt == attempt
        else:
            assert node._state == State.ARMED and node._switch_attempt == 5


def test_switching_none_future_returns_to_armed(fsm_node):
    """A5: future=None（客户端未就绪）→ 直接 ARMED。"""
    node = fsm_node
    node._state = State.SWITCHING
    node._switch_phase = "switch"
    node._switch_future = None
    node._tick()
    assert node._state == State.ARMED


def test_switching_inflight_future_does_not_block(fsm_node):
    """A6: in-flight switch future 时 _tick 立即返回，完成后才推进。"""
    node = fsm_node
    list_fut = _FakeFuture(done=False)
    node._switcher.list_controllers = lambda: list_fut
    node._begin_switch()
    t0 = time.time()
    node._tick()
    elapsed = time.time() - t0
    assert node._state == State.SWITCHING
    assert elapsed < 0.5, f"_tick 阻塞 {elapsed:.2f}s（in-flight future 应立即返回）"
    list_fut.finish(_list_resp([_ctl("scaled_joint_trajectory_controller")]))
    node._switcher.load_controller = lambda name: _FakeFuture(_load_resp(ok=True))
    node._switcher.switch = lambda a, d: _FakeFuture(_switch_resp(ok=True))
    node._tick()
    node._tick()
    node._tick()
    assert node._state == State.ACTIVE


def test_teleop_alone_exits_1_without_cell(tmp_path):
    """A7（子进程）: 无 cell 时 ~30 s 超时 → exit 1 + 超时错误日志。"""
    cfg_path = tmp_path / "alicia_teleop.yaml"
    cfg_path.write_text(CFG_BODY)
    proc = _start("ros2", "run", "ur_teleop", "teleop_node",
                  "--ros-args", "-p", f"config_file:={cfg_path}")
    try:
        rc = proc.wait(timeout=40.0)
        log = proc.stdout.read()
        assert rc == 1, f"无 cell 应 exit 1，实际 {rc}；log:\n{log[-2000:]}"
        assert "30 s 内未检测到 cell" in log, f"未见超时错误日志:\n{log[-2000:]}"
    finally:
        _kill([proc])


def test_verify_home_tolerance_boundary(fsm_node):
    """A8: 恰好等于 tolerance → 通过；逐关节 tol+ε → 拒绝；进入 SETTLING。"""
    node = fsm_node
    tol = 0.05
    _inject_joints(node, master=[0.0] * 6, slave=SLAVE_HOME)
    node._state = State.VERIFY_HOME
    node._tick()
    assert node._state == State.SETTLING, "恰好在 tolerance 内应通过"
    for i in range(6):
        for side in ("master", "slave"):
            m = [0.0] * 6
            s = list(SLAVE_HOME)
            (m if side == "master" else s)[i] += tol + 1e-6
            _inject_joints(node, master=m, slave=s)
            node._state = State.VERIFY_HOME
            node._tick()
            assert node._state == State.VERIFY_HOME, f"{side}[{i}] 超限应拒绝"
    # 恰好 tol → 通过
    m = [0.0] * 6
    m[2] = tol
    _inject_joints(node, master=m, slave=SLAVE_HOME)
    node._state = State.VERIFY_HOME
    node._tick()
    assert node._state == State.SETTLING


def test_verify_home_force_home_skips_check(tmp_path, monkeypatch):
    """A8: force_home:=true 参数 → tolerance 置 inf，远离 home 也通过。"""
    cfg_path = tmp_path / "alicia_teleop.yaml"
    cfg_path.write_text(CFG_BODY)
    monkeypatch.setattr("ur_teleop.teleop_node.default_config_path", lambda: str(cfg_path))
    rclpy.init(args=["--ros-args", "-p", "force_home:=true"])
    node = TeleopNode()
    try:
        assert node._cfg["home"]["at_home_tolerance_rad"] == float("inf")
        _inject_joints(node, master=[0.5] * 6, slave=[-0.3] * 6)
        node._state = State.VERIFY_HOME
        node._tick()
        assert node._state == State.SETTLING, "force_home 应跳过 VERIFY_HOME 验证"
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_settling_motion_resets_timer(fsm_node):
    """A9: SETTLING 中运动超过阈值重置计时；静止满 settle_time_s 才前进。"""
    node = fsm_node
    _inject_joints(node, master=[0.0] * 6, slave=[0.0] * 6)
    node._state = State.SETTLING
    node._settle_start = time.time()
    node._tick()                               # 首帧：记录 _last_pose
    assert node._state == State.SETTLING
    time.sleep(0.06)                           # 未满 settle 0.1 s
    node._tick()
    assert node._state == State.SETTLING, "settle 未到 0.1 s 不应离开"
    _inject_joints(node, master=[0.05] * 6, slave=[0.0] * 6)
    node._tick()                               # motion 0.05 > 0.01 → 计时重置
    assert node._state == State.SETTLING
    time.sleep(0.06)                           # 重置后仅 0.06 s（未重置则累计 0.12 s）
    node._tick()
    assert node._state == State.SETTLING, "motion 重置后计时应重新开始"
    time.sleep(0.15)                           # 静止 0.15 s ≥ 0.1 s
    node._tick()
    assert node._state == State.CAPTURE_OFFSET


def test_capture_offset_builds_working_mapper(fsm_node):
    """A10: CAPTURE_OFFSET 后构造 JointMapper，master home → ≈ slave home。"""
    node = fsm_node
    master = [0.1, -0.2, 0.3, -0.1, 0.05, 0.2]
    slave = [-0.05, -1.6, 0.1, -1.5, 0.02, 0.1]
    _inject_joints(node, master=master, slave=slave)
    node._state = State.CAPTURE_OFFSET
    node._tick()
    assert node._state == State.ARMED
    assert node._offset.captured
    assert node._mapper is not None
    out = node._mapper.master_to_slave(master)
    assert all(abs(a - b) < 1e-9 for a, b in zip(out, slave)), \
        f"master home 应映射到 slave home: {out} vs {slave}"
    moved = [m + 0.1 for m in master]
    out2 = node._mapper.master_to_slave(moved)
    assert all(abs(a - b - 0.1) < 1e-9 for a, b in zip(out2, out)), "偏移映射错误"


def _cmd_values(line):
    vals = [float(x) for x in re.findall(r"-?\d+\.?\d*", line or "")]
    return vals if len(vals) == 6 else None


def _wait_still_commands(timeout=6.0, settle=1.0):
    """等待指令流变为恒定值（INACTIVE 的 hold：50 Hz 持续发布但值不变）。

    status=false 是单次信号、可能被 DDS 发现延迟错过；指令恒定是 INACTIVE
    的连续可观测行为，作为兜底判据。"""
    echo = _LineEcho("/ruckig/target_joint_positions")
    try:
        deadline = time.time() + timeout
        base, t0 = None, None
        while time.time() < deadline:
            vals = _cmd_values(echo.next(2.0))
            if vals is None:
                continue
            if base is None:
                base, t0 = vals, time.time()
            elif max(abs(a - b) for a, b in zip(base, vals)) > 0.01:
                base, t0 = vals, time.time()
            elif time.time() - t0 >= settle:
                return True
        return False
    finally:
        echo.close()


def _wait_moving_commands(timeout=8.0, move=0.01):
    """等待指令流恢复变化（ACTIVE 的映射跟随主臂正弦；INACTIVE 的 hold 恒定）。

    status=true 是单次信号、可能被 DDS 发现延迟错过；指令变化是 ACTIVE 恢复
    的连续可观测行为，作为兜底判据。"""
    echo = _LineEcho("/ruckig/target_joint_positions")
    try:
        deadline = time.time() + timeout
        first = None
        while time.time() < deadline:
            vals = _cmd_values(echo.next(2.0))
            if vals is None:
                continue
            if first is None:
                first = vals
                continue
            if max(abs(a - b) for a, b in zip(first, vals)) > move:
                return True
        return False
    finally:
        echo.close()


def test_watchdog_inactive_then_recover(tmp_path):
    """A11+A13（子进程）: ACTIVE → 杀 fake_master → status=false；重启 → status=true；
    且 ACTIVE 时 /demonstration=true。"""
    _sweep(tmp_path)
    procs = _launch_stack(tmp_path)
    echoes = []
    try:
        # /demonstration 只在 ACTIVE 迁移时发布一次。echo 传显式类型（跳过
        # spawn 时非阻塞图查询——新参与者 DDS 发现未完成会直接退出）并在
        # t=0 订阅，留足匹配时间，等 ACTIVE 时捕获。
        demo = _OnceEcho("/demonstration", msg_type="std_msgs/msg/Bool")
        echoes.append(demo)
        time.sleep(20.0)                         # 等栈到 ARMED
        assert _enable_and_wait(), "enable 后未进入 ACTIVE（重试发布）"
        got_demo = demo.next(5.0)
        assert got_demo is not None and "true" in got_demo.lower(), \
            f"/demonstration 应为 true: {got_demo!r}"
        # status=false 也是单次信号 → 先订阅（给 DDS 匹配留时间），再杀 fake
        e_false = _OnceEcho("/teleop/status", msg_type="std_msgs/msg/Bool")
        echoes.append(e_false)
        time.sleep(1.5)
        _kill_one(procs[2])                    # 杀掉 fake_master
        got = e_false.next(6.0)
        if got is None or "false" not in got.lower():
            # status=false 单次信号可能被错过 → 指令恒定（hold）兜底
            assert _wait_still_commands(6.0), \
                "主臂断开后未发布 status=false，指令流也未冻结为 hold"
        # 恢复：订阅后再重启 fake
        e_true = _OnceEcho("/teleop/status", msg_type="std_msgs/msg/Bool")
        echoes.append(e_true)
        time.sleep(1.0)
        procs[2] = _start(sys.executable, str(FAKE_MASTER))
        got = e_true.next(6.0)
        if got is None or "true" not in got.lower():
            # status=true 单次信号可能被错过 → 指令恢复跟随（正弦）兜底
            assert _wait_moving_commands(8.0), \
                "主臂恢复后未回到 ACTIVE（status 单次信号被错过，指令流也未恢复跟随）"
    finally:
        for e in echoes:
            e.close()
        _kill(procs)
        _sweep(tmp_path)


def test_estop_freezes_then_resumes(tmp_path):
    """A12（子进程）: ACTIVE → e_stop ON → 指令静默（_tick 提前返回、不再发布）；
    OFF → 恢复跟随。"""
    _sweep(tmp_path)
    procs = _launch_stack(tmp_path)
    echo = None
    try:
        time.sleep(20.0)
        assert _enable_and_wait(), "enable 后未进入 ACTIVE（重试发布）"
        echo = _LineEcho("/ruckig/target_joint_positions")
        s1 = echo.next(8.0)
        assert s1 is not None, "ACTIVE 下未收到 commands"
        s2 = echo.next(2.0)
        assert s1 != s2, f"ACTIVE 下指令应随主臂运动: {s1} vs {s2}"
        _pub_bool_window("/teleop/e_stop", True)   # --once 会因 DDS 发现延迟丢失
        time.sleep(3.0)                            # 等锁存生效（含发现延迟）
        echo.drain()                               # 清掉冻结前最后几条在途指令
        frozen = echo.next(3.0)
        assert frozen is None, \
            f"e_stop 下指令应静默（_tick 提前返回），却收到: {frozen!r}"
        _pub_bool_window("/teleop/e_stop", False)
        r1 = echo.next(8.0)
        assert r1 is not None, "e_stop 解除后未恢复 commands"
        r2 = echo.next(2.0)
        assert r1 != r2, f"e_stop 解除后指令应恢复运动: {r1} vs {r2}"
    finally:
        if echo is not None:
            echo.close()
        _kill(procs)
        _sweep(tmp_path)


def test_gripper_probe_disables_fsm_without_action_server(tmp_path, monkeypatch):
    """A14: gripper.enabled=true 且无 action server → 首次 _gripper_tick 探测后禁用，不抛异常。"""
    cfg_path = tmp_path / "alicia_teleop.yaml"
    cfg_path.write_text(CFG_BODY.replace("enabled: false", "enabled: true"))
    monkeypatch.setattr("ur_teleop.teleop_node.default_config_path", lambda: str(cfg_path))
    rclpy.init()
    node = TeleopNode()
    try:
        assert node._gripper.enabled
        assert node._gripper_action is not None
        node._gripper_tick()                   # 首次探测：server 不可用 → 禁用
        assert not node._gripper.enabled, "无 action server 时应禁用夹爪 FSM"
        assert node._gripper_probed
        node._gripper_tick()                   # 后续 tick 直接返回，不抛异常
        assert not node._gripper.enabled
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_shutdown_restore_without_controller_manager_is_prompt(fsm_node):
    """A15: restore_controller_on_exit 且无 controller_manager（switch→None）→ 立即返回不抛。"""
    node = fsm_node
    node._state = State.ACTIVE
    t0 = time.time()
    node.shutdown()                            # 客户端未就绪 → switch() 返回 None
    assert time.time() - t0 < 2.0


# ---------- B. ControllerSwitcher statics ----------

def test_switcher_statics_contract_on_bad_futures():
    """B: list_result/switch_ok 对 cancelled/异常 future 抛错（调用侧已守卫）；其余不抛。"""
    cancelled = _FakeFuture(cancelled=True)
    failed = _FakeFuture(exception=RuntimeError("controller_manager died mid-call"))
    with pytest.raises(concurrent.futures.CancelledError):
        ControllerSwitcher.list_result(cancelled)
    with pytest.raises(RuntimeError):
        ControllerSwitcher.list_result(failed)
    with pytest.raises(concurrent.futures.CancelledError):
        ControllerSwitcher.switch_ok(cancelled)
    with pytest.raises(RuntimeError):
        ControllerSwitcher.switch_ok(failed)
    # 未完成 / None / 空结果：不抛
    assert ControllerSwitcher.list_result(None) == {}
    assert ControllerSwitcher.list_result(_FakeFuture(done=False)) == {}
    assert ControllerSwitcher.list_result(_FakeFuture(result=None)) == {}
    assert ControllerSwitcher.switch_ok(None) is False
    assert ControllerSwitcher.switch_ok(_FakeFuture(done=False)) is False
    assert ControllerSwitcher.switch_ok(_FakeFuture(result=None)) is False


# ---------- C. home_node ----------

def test_home_at_home_tolerance_boundary(home_node):
    """C: at_home 恰好 tolerance → True；逐关节 tol+ε → False；缺数据 → False。"""
    node = home_node
    tol = node._tolerance
    assert tol == 0.05
    _inject_joints(node, master=[0.0] * 6, slave=SLAVE_HOME)
    assert node.at_home()
    for i in range(6):
        for side in ("master", "slave"):
            m = [0.0] * 6
            s = list(SLAVE_HOME)
            (m if side == "master" else s)[i] += tol + 1e-6
            _inject_joints(node, master=m, slave=s)
            assert not node.at_home(), f"{side}[{i}] 超限应 False"
    m = [0.0] * 6
    m[3] = tol
    _inject_joints(node, master=m, slave=SLAVE_HOME)
    assert node.at_home(), "恰好 tol 应 True"
    node._ur_states = None
    node._alicia_states = None
    assert not node.at_home(), "缺关节数据应 False"


def test_home_publish_alicia_home_opens_gripper(home_node):
    """C: publish_alicia_home 在 6 关节后附带夹爪 1000（开）。"""
    node = home_node
    captured = []
    node._cmd_pub = _CapturePub(captured)
    node.publish_alicia_home()
    assert len(captured) == 1
    msg = captured[0]
    assert list(msg.name) == ALICIA_JOINT_NAMES + [GRIPPER_JOINT]
    assert list(msg.position) == [0.0] * 6 + [1000.0]


# ---------- D. lerobot record 保存冒烟 ----------

def test_record_save_episode(tmp_path):
    """D（子进程）: Enter → ACTIVE → 录帧 → S 保存 → Q finalize（1 episode 已保存）。"""
    try:
        import lerobot  # noqa: F401
    except ImportError:
        pytest.skip("lerobot 未安装（需要 source /opt/lerobot_venv/bin/activate）")

    root = tmp_path / "data"
    cfg_path = tmp_path / "alicia_teleop.yaml"
    cfg_path.write_text(CFG_BODY.replace("mode: teleop", "mode: record")
                                .replace('root: ""', f"root: {root}"))
    _sweep(tmp_path)                           # 必须先于 spawn：pkill 按 tmp_path 匹配 stack cmdline
    procs = [
        _start("ros2", "launch", "ur_teleop", "cell.launch.py",
               f"config_file:={cfg_path}", "sim:=true", "launch_rviz:=false",
               "launch_alicia:=false"),
        _start("ros2", "run", "ur_teleop", "teleop_node",
               "--ros-args", "-p", f"config_file:={cfg_path}"),
        _start(sys.executable, str(FAKE_MASTER)),
        # 注意：data_recorder 用 venv 解释器以 -m 启动（ros2 run 的入口脚本
        # shebang 是系统 python3，缺少 lerobot 会直接崩溃）
        _start(sys.executable, "-m", "ur_teleop.data_recorder",
               "--ros-args", "-p", f"config_file:={cfg_path}", stdin=subprocess.PIPE),
    ]
    recorder = procs[-1]
    try:
        time.sleep(20.0)
        # KeyboardReader 把 '\n' 映射为 enter → 只写换行字节即开始 episode；
        # s/q 单字节写入（'s\n' 会保存后又开新 episode，Q 时被 finalize 保存）
        recorder.stdin.write("\n")
        recorder.stdin.flush()                 # 开始 episode 1 + 发 /teleop/enable
        assert _enable_and_wait(), "Enter 后未进入 ACTIVE（重试发布 enable）"
        time.sleep(3.0)                        # 录 ≥ min_frames 帧
        recorder.stdin.write("s")
        recorder.stdin.flush()                 # 保存 episode
        time.sleep(1.0)
        recorder.stdin.write("q")
        recorder.stdin.flush()                 # 退出 + finalize
        recorder.wait(timeout=15.0)
        # 本地布局（lerobot 0.5.x 显式 root）：info.json 在 create 时落盘，
        # episode parquet 在 save_episode 时落盘到 data/chunk-000/
        assert (root / "meta" / "info.json").exists(), "数据集 meta/info.json 未创建"
        assert (root / "data" / "chunk-000" / "file-000.parquet").exists(), \
            "episode parquet 未保存"
        log = recorder.stdout.read()
        assert "已保存" in log, f"recorder 日志未见保存:\n{log[-2000:]}"
        assert "finalize" in log.lower(), f"recorder 日志未见 finalize:\n{log[-2000:]}"
    finally:
        _kill(procs)
        _sweep(tmp_path)


# ============================================================================
# 最终评审修复波（E/F）：home_node 子进程冒烟 + enable ARMED 前锁存回归
# ============================================================================

def test_home_node_subprocess_smoke(tmp_path):
    """E（子进程）: home_node 对 mock cell 全流程冒烟 — 轨迹 + 验证 → exit 0。

    此前 home_node 只能 in-process 白盒测 at_home/publish，cell_ready 的
    server_is_available AttributeError 直到真跑 subprocess 才暴露（Jazzy 无
    该方法）。本测试跑 home_node 完整 main()：等 cell 就绪 → 发 UR home
    轨迹 → 发布 alicia home → 验证到位 → 打印 HOME REACHED → exit 0。"""
    _sweep(tmp_path)
    cfg_path = tmp_path / "alicia_teleop.yaml"
    cfg_path.write_text(CFG_BODY)              # home.slave = mock UR 初始位姿 → 轨迹即达
    procs = [
        _start("ros2", "launch", "ur_teleop", "cell.launch.py",
               f"config_file:={cfg_path}", "sim:=true", "launch_rviz:=false",
               "launch_alicia:=false"),
        _start(sys.executable, str(FAKE_MASTER)),   # master 保持 home（zeros）直到 status=true
        _start("ros2", "run", "ur_teleop", "home_node",
               "--ros-args", "-p", f"config_file:={cfg_path}"),
    ]
    home = procs[-1]
    try:
        rc = home.wait(timeout=90.0)           # cell ~20 s + 轨迹 8 s + 验证 2 s
        log = home.stdout.read()
        assert rc == 0, f"home_node 应 exit 0，实际 {rc}；log:\n{log[-500:]}"
        assert "HOME REACHED" in log, f"未见 HOME REACHED 消息；log:\n{log[-500:]}"
    finally:
        _kill(procs)
        _sweep(tmp_path)


def test_enable_single_pub_before_armed_latches(tmp_path):
    """F（子进程）: ARMED 前单次 enable 不丢 — 锁存后 ARMED 自动切换 ACTIVE。

    生产场景：data_recorder 启动即发一次 /teleop/enable（teleop 尚在
    WAITING_CELL/VERIFY_HOME），无锁存时该信号被 _enable_cb 丢弃且 recorder
    不再重发 → record 会话无法进入 ACTIVE。本测试只发布一次 enable，
    之后不再重发，60 s 内必须到达 ACTIVE（status=true）。"""
    _sweep(tmp_path)
    procs = _launch_stack(tmp_path)
    echo = _OnceEcho("/teleop/status", msg_type="std_msgs/msg/Bool")
    try:
        time.sleep(5.0)                        # 尚未 ARMED（~20 s），正是 recorder 发 enable 的时机
        _pub_bool("/teleop/enable", True)      # 恰好一次，不重发——被测 bug 的关键
        deadline = time.time() + 60.0
        while time.time() < deadline:
            got = echo.next(2.0)
            if got is not None and "true" in got.lower():
                return
        assert False, "ARMED 前单次 enable 未生效：60 s 内未到 ACTIVE（status=true）"
    finally:
        echo.close()
        _kill(procs)
        _sweep(tmp_path)
