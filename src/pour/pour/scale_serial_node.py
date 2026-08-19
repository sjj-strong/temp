#!/usr/bin/env python3
"""串口电子秤读取节点:持续读串口,按 publish_rate 发布最新重量。

参考 robot_utils/liquid_pouring/serial_manager.py:
    1. 原始模式行读取 + 正则解析(ST,GS,  0.15 g 等格式)
    2. 打开串口前先置 dtr/rts 为 False,避免部分 USB 串口触发协议错误
    3. 只保留最新一帧,按发布频率发送

所有参数走 config/pour_params.yaml;端口打开失败/断线自动重连(2s 间隔),
解析失败按节流 warn,节点不会崩。
"""
import threading
import time

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from .scale_protocol import parse_weight_line

RECONNECT_INTERVAL = 2.0     # 串口重连间隔(秒)
PARSE_WARN_THRESHOLD = 50    # 解析失败累计多少条才 warn 一次


class ScaleSerialNode(Node):
    """串口电子秤节点:后台线程持续读取,主线程按 publish_rate 发布。"""

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

        # 最新一帧缓存
        self._latest_weight = None
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
            self.get_logger().info(
                f"串口连接成功: {self.port} @ {self.baudrate}bps")
            return True
        except (serial.SerialException, OSError) as e:
            self._serial_conn = None
            self.get_logger().error(
                f"串口连接失败: {e}, {RECONNECT_INTERVAL}s 后重试")
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
                            f"解析失败(累计 {self._parse_failures} 次), "
                            f"原始行: {raw!r}")
                    continue
                self._parse_failures = 0
                with self._lock:
                    self._latest_weight = value
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
