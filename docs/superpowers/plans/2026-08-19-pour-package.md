# pour 功能包实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 /ros2_ws/src/ 下创建 pour 功能包:串口电子秤节点 + 核心 PD 倾倒控制节点,驱动 UR 的 forward_velocity_controller。

**Architecture:** 双节点单包(ament_python):scale_serial_node 读串口按 publish_rate 发布 /scale/weight(std_msgs/Float64);pour_control_node 订阅 weight,按 control_rate 跑 PD,达标自动停,发 Float64MultiArray 到 /forward_velocity_controller/commands。PD 与串口解析做成纯逻辑模块,可单测。

**Tech Stack:** ROS 2 Jazzy (rclpy)、pyserial、pytest。python3-serial 由 rosdep 提供。

## Global Constraints

- 包根目录:`/ros2_ws/src/pour/`
- 全部配置只走 `config/pour_params.yaml`;launch 支持 `port` 等传参覆盖
- setup.cfg 必须用 `$base/lib/pour`(develop.script_dir / install.install_scripts),否则 ros2 run 报 No executable found
- 速度话题 `/forward_velocity_controller/commands` 用 `std_msgs/Float64MultiArray`(6 关节,只动 wrist_index 默认 5)
- `/joint_states` 订阅 QoS 必须 BEST_EFFORT(参考 robot_driver.py 注释),`/scale/weight` 用默认 reliable
- 达标停止条件:`abs(error) <= tolerance`
- 重量超时 `weight_timeout` 秒无新数据 → 发零速 + error 日志 + 结束循环
- 串口打开前先 dtr/rts=False(参考 serial_manager.py);解析失败按节流 warn,不崩节点
- PD 只保留核心:error、derivative、kp*error+kd*derivative、clip、prev_error;近目标 kd 开关默认关
- 测试用 pytest:`pytest.ini` testpaths=tests(照抄 ur_teleop_rtde)

---

### Task 1: 包脚手架 + PD 控制器(pure, TDD)

**Files:**
- Create: `src/pour/package.xml`、`setup.py`、`setup.cfg`、`resource/pour`、`pytest.ini`、`pour/__init__.py`
- Create: `src/pour/pour/pd_controller.py`
- Test: `src/pour/tests/test_pd_controller.py`

**Interfaces:**
- Produces: `pour.pd_controller.PDController`:
  - `__init__(kp: float, kd: float, joint_range: tuple[float,float], near_target_kd_enable: bool = False, near_target_kd_threshold: float = 10.0, near_target_kd: float = 0.025)`
  - `calculate(target_weight: float, current_weight: float) -> tuple[float, float]` 返回 `(output, error)`
  - `is_reached(error: float, tolerance: float) -> bool`

- [ ] **Step 1: 写脚手架文件**(package.xml 照 ur_teleop_rtde 模板改名字;setup.cfg 用 `$base/lib/pour`;setup.py data_files 含 config/launch/resource,console_scripts 两个入口暂填占位名)
- [ ] **Step 2: 写失败的测试**

```python
# tests/test_pd_controller.py
import pytest
from pour.pd_controller import PDController


def test_proportional_term():
    pd = PDController(kp=0.01, kd=0.0, joint_range=(-2, 2))
    out, err = pd.calculate(target_weight=100.0, current_weight=90.0)
    assert err == pytest.approx(10.0)
    assert out == pytest.approx(0.1)  # kp*error


def test_derivative_term():
    pd = PDController(kp=0.0, kd=0.5, joint_range=(-2, 2))
    pd.calculate(target_weight=100.0, current_weight=90.0)  # error=10, prev=0
    out, err = pd.calculate(target_weight=100.0, current_weight=95.0)  # error=5
    assert err == pytest.approx(5.0)
    assert out == pytest.approx(-2.5)  # kd*(5-10)


def test_clip_to_joint_range():
    pd = PDController(kp=1.0, kd=0.0, joint_range=(-0.5, 0.5))
    out, _ = pd.calculate(target_weight=100.0, current_weight=0.0)
    assert out == 0.5


def test_near_target_kd_disabled_by_default():
    pd = PDController(kp=0.0, kd=0.1, joint_range=(-2, 2))
    pd.calculate(target_weight=100.0, current_weight=95.0)   # error=5 <= threshold
    out, _ = pd.calculate(target_weight=100.0, current_weight=95.5)
    assert out == pytest.approx(0.1 * 0.5)  # 默认不触发, kd 不变


def test_near_target_kd_enabled():
    pd = PDController(kp=0.0, kd=0.1, joint_range=(-2, 2),
                      near_target_kd_enable=True, near_target_kd_threshold=10.0,
                      near_target_kd=0.025)
    pd.calculate(target_weight=100.0, current_weight=95.0)
    out, _ = pd.calculate(target_weight=100.0, current_weight=95.5)
    assert out == pytest.approx(0.025 * 0.5)  # kd 换成 near_target_kd


def test_is_reached():
    pd = PDController(kp=0.0, kd=0.0, joint_range=(-2, 2))
    assert pd.is_reached(error=0.5, tolerance=1.0)
    assert not pd.is_reached(error=1.5, tolerance=1.0)
```

- [ ] **Step 3: 跑测试确认失败**(`pytest tests/test_pd_controller.py -v`,ModuleNotFoundError)
- [ ] **Step 4: 写最小实现**

```python
# pour/pd_controller.py
"""核心 PD 控制器(纯逻辑,无 ROS 依赖)。参考 robot_utils/liquid_pouring
controllers.py MultiModalPDController.calculate,只保留核心 PD 逻辑。"""


class PDController:
    """电子秤重量 PD 倾倒控制。

    output = kp*error + kd_eff*derivative,clip 到 joint_range。
    derivative = 当前误差 - 上一周期误差(每控制周期误差变化量)。
    近目标 kd 缩小 trick(可选):|error| <= threshold 时 kd 换成 near_target_kd,
    默认关闭,须显式开 near_target_kd_enable。
    """

    def __init__(self, kp, kd, joint_range, near_target_kd_enable=False,
                 near_target_kd_threshold=10.0, near_target_kd=0.025):
        self.kp = kp
        self.kd = kd
        self.joint_range = joint_range
        self.near_target_kd_enable = near_target_kd_enable
        self.near_target_kd_threshold = near_target_kd_threshold
        self.near_target_kd = near_target_kd
        self.prev_error = 0.0

    def calculate(self, target_weight, current_weight):
        """返回 (output, error)。"""
        error = target_weight - current_weight
        derivative = error - self.prev_error
        kd_eff = self.kd
        if self.near_target_kd_enable and abs(error) <= self.near_target_kd_threshold:
            kd_eff = self.near_target_kd
        output = self.kp * error + kd_eff * derivative
        output = max(self.joint_range[0], min(self.joint_range[1], output))
        self.prev_error = error
        return output, error

    def is_reached(self, error, tolerance):
        """误差是否在允许范围内(达标)。"""
        return abs(error) <= tolerance
```

- [ ] **Step 5: 跑测试确认通过**
- [ ] **Step 6: Commit**(`feat(pour): scaffold package + core PD controller with tests`)

---

### Task 2: 串口行解析协议(pure, TDD)

**Files:**
- Create: `src/pour/pour/scale_protocol.py`
- Test: `src/pour/tests/test_scale_protocol.py`

**Interfaces:**
- Produces: `pour.scale_protocol.parse_weight_line(raw: bytes) -> float | None`
- Consumes: 无(独立)

- [ ] **Step 1: 写失败的测试**

```python
# tests/test_scale_protocol.py
from pour.scale_protocol import parse_weight_line


def test_plain_gram():
    assert parse_weight_line(b"0.15 g") == 0.15


def test_plain_number():
    assert parse_weight_line(b"0.15") == 0.15


def test_prefix_gram():
    assert parse_weight_line(b"ST,GS,    0.15 g") == 0.15


def test_negative():
    assert parse_weight_line(b"ST,GS,   -0.15 g") == -0.15


def test_positive_sign():
    assert parse_weight_line(b"+ 0.15 g") == 0.15


def test_ignore_case_g():
    assert parse_weight_line(b"12.5 G") == 12.5


def test_invalid_line_returns_none():
    assert parse_weight_line(b"hello world") is None
    assert parse_weight_line(b"") is None
    assert parse_weight_line(b"\r\n") is None
```

- [ ] **Step 2: 跑测试确认失败**
- [ ] **Step 3: 写最小实现**

```python
# pour/scale_protocol.py
"""电子秤串口行解析。参考 robot_utils/liquid_pouring serial_manager.py
的原始模式匹配(ST,GS,  0.15 g / 0.15 g / + 0.15 / -0.15)。"""
import re

WEIGHT_PATTERN = re.compile(rb"([+-]?)\s*(\d+(?:\.\d+)?)\s*g?", re.IGNORECASE)


def parse_weight_line(raw):
    """解析一行串口数据,返回重量克数(float);无效行返回 None。"""
    if not raw:
        return None
    match = WEIGHT_PATTERN.search(raw)
    if not match:
        return None
    value = float(match.group(2))
    if match.group(1) == b"-":
        return -value
    return value
```

- [ ] **Step 4: 跑测试确认通过**
- [ ] **Step 5: Commit**(`feat(pour): scale serial line parser with tests`)

---

### Task 3: 串口读取节点 scale_serial_node

**Files:**
- Create: `src/pour/pour/scale_serial_node.py`
- Modify: `src/pour/setup.py`(console_scripts 加 `scale_serial_node`)

**Interfaces:**
- Consumes: `pour.scale_protocol.parse_weight_line`
- Produces: 发布 `/scale/weight`(`std_msgs/Float64`,默认 reliable,深度 10),节点名 `scale_node`
- 参数:`port`(str, /dev/ttyUSB0)、`baudrate`(int, 9600)、`timeout`(float, 0.1)、`publish_rate`(float, 10.0)、`weight_topic`(str, /scale/weight)

- [ ] **Step 1: 写节点实现**(后台线程持续 readline+解析;主定时器按 publish_rate 发布最新帧;打开失败/断线:error 日志 + 2s 重试循环;解析失败计数每 50 次 warn 一次;连接成功 info 日志)

```python
# pour/scale_serial_node.py
#!/usr/bin/env python3
"""串口电子秤读取节点:持续读串口,按 publish_rate 发布最新重量。

参考 robot_utils/liquid_pouring/serial_manager.py:原始模式行读取 +
dtr/rts 先关再开 + 只保留最新帧。所有参数走 config/pour_params.yaml。
"""
import threading
import time

import serial
import serial.tools.list_ports  # noqa: F401  (import 副作用:注册串口枚举)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from .scale_protocol import parse_weight_line

RECONNECT_INTERVAL = 2.0
PARSE_WARN_THRESHOLD = 50  # 解析失败累计多少条才 warn 一次


class ScaleSerialNode(Node):
    def __init__(self):
        super().__init__("scale_node")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("timeout", 0.1)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("weight_topic", "/scale/weight")

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.weight_topic = self.get_parameter("weight_topic").value

        self.pub = self.create_publisher(Float64, self.weight_topic, 10)
        self._latest_weight = None
        self._latest_stamp = None
        self._latest_raw = None
        self._lock = threading.Lock()
        self._parse_failures = 0

        self._serial_conn = None
        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="scale_reader", daemon=True
        )
        self._reader_thread.start()

        self._timer = self.create_timer(1.0 / self.publish_rate, self._publish_tick)
        self.get_logger().info(
            f"scale_node ready: port={self.port} baud={self.baudrate} "
            f"publish_rate={self.publish_rate}Hz topic={self.weight_topic}"
        )

    # ------------------------------------------------------------------
    def _connect(self):
        """打开串口(dtr/rts 先置 False 再 open)。失败返回 False。"""
        try:
            conn = serial.Serial()
            conn.port = self.port
            conn.baudrate = self.baudrate
            conn.timeout = self.timeout
            conn.dtr = False
            conn.rts = False
            conn.open()
            conn.reset_input_buffer()
            self._serial_conn = conn
            self.get_logger().info(f"串口连接成功: {self.port} @ {self.baudrate}bps")
            return True
        except (serial.SerialException, OSError) as e:
            self._serial_conn = None
            self.get_logger().error(f"串口连接失败: {e}, {RECONNECT_INTERVAL}s 后重试")
            return False

    def _reader_loop(self):
        """后台线程:持续 readline + 解析,保留最新帧。"""
        while not self._stop_event.is_set():
            if self._serial_conn is None or not self._serial_conn.is_open:
                if not self._connect():
                    self._stop_event.wait(RECONNECT_INTERVAL)
                    continue
            try:
                data = self._serial_conn.readline()
                if not data:
                    continue
                raw = data.strip()
                value = parse_weight_line(raw)
                if value is None:
                    self._parse_failures += 1
                    if self._parse_failures % PARSE_WARN_THRESHOLD == 1:
                        self.get_logger().warn(
                            f"解析失败(累计 {self._parse_failures} 次), 原始行: {raw!r}"
                        )
                    continue
                self._parse_failures = 0
                with self._lock:
                    self._latest_weight = value
                    self._latest_stamp = time.monotonic()
                    self._latest_raw = raw
            except serial.SerialException as e:
                self.get_logger().error(f"串口读取错误: {e}")
                try:
                    self._serial_conn.close()
                except Exception:
                    pass
                self._serial_conn = None
                self._stop_event.wait(RECONNECT_INTERVAL)
            except Exception as e:
                self.get_logger().error(f"串口读取线程异常: {e}")
                self._stop_event.wait(0.05)

    def _publish_tick(self):
        """按 publish_rate 发布最新一帧重量。"""
        with self._lock:
            weight = self._latest_weight
        if weight is None:
            return  # 尚无有效数据,不发布
        msg = Float64()
        msg.data = float(weight)
        self.pub.publish(msg)

    def destroy_node(self):
        self._stop_event.set()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        if self._serial_conn is not None and self._serial_conn.is_open:
            try:
                self._serial_conn.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScaleSerialNode()
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
```

- [ ] **Step 2: setup.py 注册入口**`"scale_serial_node = pour.scale_serial_node:main"`
- [ ] **Step 3: 语法冒烟**(`python3 -c "import pour.scale_serial_node"` 无导入错误)
- [ ] **Step 4: Commit**(`feat(pour): scale serial reader node`)

---

### Task 4: PD 控制节点 pour_control_node

**Files:**
- Create: `src/pour/pour/pour_control_node.py`
- Modify: `src/pour/setup.py`(console_scripts 加 `pour_control_node`)

**Interfaces:**
- Consumes: `PDController`(Task 1)、`/scale/weight`(Task 3 话题)
- Produces: 发布 `/forward_velocity_controller/commands`(`Float64MultiArray`,6 关节),节点名 `pour_control_node`
- 参数:`weight_topic`、`cmd_topic`、`joint_state_topic`、`wrist_index`(5)、`velocity_sign`(-1.0)、`control_rate`(9.0)、`kp`(0.0045)、`kd`(0.06)、`joint_range`([-2.0,2.0])、`target_weight`(70.0)、`tolerance`(1.0)、`near_target_kd_enable`(False)、`near_target_kd_threshold`(10.0)、`near_target_kd`(0.025)、`weight_timeout`(2.0)

- [ ] **Step 1: 写节点实现**(定时循环:取最新 weight → 超时检查(weight_timeout 无新数据→零速+error+退出)→ PD 计算 → 发布 6 关节速度(wrist_index=velocity_sign*output)→ |error|≤tolerance→零速+info+退出;日志:启动打印参数,运行中 1Hz 节流 info(error/output/关节角),循环结束 info 打印实际频率)

```python
# pour/pour_control_node.py
#!/usr/bin/env python3
"""PD 倾倒控制节点:订阅电子秤重量,按 control_rate 跑核心 PD,
发布速度到 /forward_velocity_controller/commands(wrist_index 关节)。

参考 robot_utils/liquid_pouring/robot_driver.py 的接口约定:
    - 速度指令 Float64MultiArray(6 关节),只动倾倒关节
    - /joint_states 需 BEST_EFFORT QoS
前置(README): ros2 control switch_controllers --deactivate
    scaled_joint_trajectory_controller --activate forward_velocity_controller
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from .pd_controller import PDController

LOG_THROTTLE = 1.0  # 周期日志节流(秒)


class PourControlNode(Node):
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
        self._weight_stamp = None       # rclpy time (nsec)
        self._joint_angle = 0.0
        self._finished = False
        self._log_next = 0.0
        self._start_time = time.monotonic()
        self._cycles = 0

        self.get_logger().info(
            f"pour_control_node ready: rate={self.control_rate}Hz "
            f"kp={kp} kd={kd} target={self.target_weight}g "
            f"tolerance={self.tolerance}g wrist_idx={self.wrist_index} "
            f"cmd_topic={self.cmd_topic}")
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
                    f"重量数据超时({age:.1f}s > {self.weight_timeout}s),安全停止")
                return

        output, error = self.pd.calculate(self.target_weight, weight)

        # 发布 6 关节速度(只动 wrist_index)
        msg = Float64MultiArray()
        data = [0.0] * 6
        data[self.wrist_index] = self.velocity_sign * output
        msg.data = data
        self.cmd_pub.publish(msg)

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
                f"用时 {elapsed:.1f}s, 平均频率 {self._cycles/elapsed:.1f}Hz", level="info")
            return

    def _stop_and_finish(self, message, level="error"):
        """发零速并结束控制循环。"""
        msg = Float64MultiArray()
        msg.data = [0.0] * 6
        self.cmd_pub.publish(msg)
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
```

- [ ] **Step 2: setup.py 注册入口**`"pour_control_node = pour.pour_control_node:main"`
- [ ] **Step 3: 语法冒烟**(`python3 -c "import pour.pour_control_node"` 无导入错误)
- [ ] **Step 4: Commit**(`feat(pour): PD pour control node`)

---

### Task 5: 配置文件 + launch + README + 构建验证

**Files:**
- Create: `src/pour/config/pour_params.yaml`、`launch/pour.launch.py`、`README.md`

- [ ] **Step 1: config/pour_params.yaml**(完整参数,见 spec;两节点各自 ros__parameters)

```yaml
# pour 运行参数 —— 全部配置都在这里,launch 可传参覆盖
scale_node:
  ros__parameters:
    port: /dev/ttyUSB0
    baudrate: 9600
    timeout: 0.1
    publish_rate: 10.0
    weight_topic: /scale/weight

pour_control_node:
  ros__parameters:
    weight_topic: /scale/weight
    cmd_topic: /forward_velocity_controller/commands
    joint_state_topic: /joint_states
    wrist_index: 5
    velocity_sign: -1.0
    control_rate: 9.0
    kp: 0.0045
    kd: 0.06
    joint_range: [-2.0, 2.0]
    target_weight: 70.0
    tolerance: 1.0
    near_target_kd_enable: false
    near_target_kd_threshold: 10.0
    near_target_kd: 0.025
    weight_timeout: 2.0
```

- [ ] **Step 2: launch/pour.launch.py**(两 Node + 一个公共 params_file;支持覆盖参数 port/target_weight/control_rate/kp/kd/tolerance)

```python
# launch/pour.launch.py
"""启动 pour 包的两个节点:scale_serial_node + pour_control_node。

用法:
    ros2 launch pour pour.launch.py
    ros2 launch pour pour.launch.py port:=/dev/ttyUSB1 target_weight:=100.0
前置(需先启动 ur_control 并切换速度控制器):
    ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller \
        --activate forward_velocity_controller
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("pour")
    default_params = os.path.join(pkg_share, "config", "pour_params.yaml")

    declared_args = [
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="pour 参数文件路径"),
        DeclareLaunchArgument("port", default_value="",
                              description="覆盖串口端口(如 /dev/ttyUSB1)"),
        DeclareLaunchArgument("target_weight", default_value="",
                              description="覆盖目标重量(g)"),
        DeclareLaunchArgument("control_rate", default_value="",
                              description="覆盖控制频率(Hz)"),
        DeclareLaunchArgument("kp", default_value="", description="覆盖 kp"),
        DeclareLaunchArgument("kd", default_value="", description="覆盖 kd"),
        DeclareLaunchArgument("tolerance", default_value="",
                              description="覆盖误差允许范围(g)"),
    ]

    def _build_nodes(context):
        """把非空的 launch 覆盖值合进参数;空串(未传)则回退到 yaml。"""
        params_file = LaunchConfiguration("params_file").perform(context)
        scale_params = [params_file]
        pour_params = [params_file]
        overrides = {
            "port": ("scale", "port"),
            "target_weight": ("pour", "target_weight"),
            "control_rate": ("pour", "control_rate"),
            "kp": ("pour", "kp"),
            "kd": ("pour", "kd"),
            "tolerance": ("pour", "tolerance"),
        }
        for launch_arg, (node_kind, param_key) in overrides.items():
            value = LaunchConfiguration(launch_arg).perform(context)
            if value:  # 空串 = 未提供, 回退 yaml
                (scale_params if node_kind == "scale" else pour_params).append(
                    {param_key: value})

        return [
            Node(package="pour", executable="scale_serial_node",
                 name="scale_node", output="screen",
                 parameters=scale_params),
            Node(package="pour", executable="pour_control_node",
                 name="pour_control_node", output="screen",
                 parameters=pour_params),
        ]

    return LaunchDescription(
        declared_args + [OpaqueFunction(function=_build_nodes)])
```

- [ ] **Step 3: README.md**(功能、前置 switch_controllers 命令、启动方式、参数表、串口接线/格式说明)
- [ ] **Step 4: colcon build**(`cd /ros2_ws && colcon build --packages-select pour --symlink-install` 通过)
- [ ] **Step 5: 跑全部测试**(`source install/setup.bash && pytest src/pour/tests -v` 全过)
- [ ] **Step 6: 冒烟启动**(`timeout 5 ros2 launch pour pour.launch.py` 两节点起来、无 traceback;`ros2 node list` 可见 scale_node 与 pour_control_node)
- [ ] **Step 7: Commit**(`feat(pour): config, launch, README; package builds and tests pass`)

---

## Self-Review 记录

- spec 覆盖:全部配置走 config ✓(Task 5)、端口可配 ✓(Task 3 参数 + Task 5)、发布频率/控制频率 ✓、PD 核心 ✓(Task 1)、近目标 kd 开关默认关 ✓、误差允许范围 ✓(tolerance)、weight 超时安全停 ✓(Task 4)、达标自动停 ✓、ROS 日志 ✓(两节点均用 get_logger)、包在 src 下 ✓
- 占位符:无;每步含实际代码
- 类型一致性:PDController.calculate 返回 (output, error) 与节点使用一致;parse_weight_line 返回 float|None 与节点使用一致
- launch 覆盖参数:用 OpaqueFunction 只把非空覆盖值合入参数表,空串回退 yaml(避免空串参数覆盖报错)
