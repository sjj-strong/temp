"""将多个 ROS Image 话题合成为一张图，供单个 rqt_image_view 窗口显示。"""

import math
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraMosaicViewer(Node):
    """只读订阅多个图像话题，并发布一个 BGR8 拼接图像。"""

    _TILE_WIDTH = 640
    _TILE_HEIGHT = 480
    _OUTPUT_TOPIC = "/camera_mosaic/image_raw"

    def __init__(self):
        super().__init__("camera_mosaic_viewer")
        # Jazzy 对空列表的类型推断为 BYTE_ARRAY。使用空字符串明确声明为
        # STRING_ARRAY，随后再滤掉它，以便 launch 能覆盖为图像话题列表。
        self.declare_parameter("topics", [""])
        self._topics = [
            str(topic) for topic in self.get_parameter("topics").value if topic
        ]
        if not self._topics:
            raise RuntimeError("未配置用于拼接预览的图像话题")
        self._frames = {}
        self._lock = threading.Lock()
        self._publisher = self.create_publisher(Image, self._OUTPUT_TOPIC, 5)
        self._subscriptions = [
            self.create_subscription(
                Image, topic,
                lambda msg, current=topic: self._image_cb(current, msg), 5,
            )
            for topic in self._topics
        ]
        self._timer = self.create_timer(1.0 / 30.0, self._publish_mosaic)
        self.get_logger().info(
            f"相机拼接发布器已启动：{self._OUTPUT_TOPIC}（共 {len(self._topics)} 路）")

    def _image_cb(self, topic, msg):
        frame = self._to_bgr8(msg)
        if frame is None:
            return
        with self._lock:
            self._frames[topic] = frame

    def _to_bgr8(self, msg):
        """转换常用原始图像编码，避免 cv_bridge 与虚拟环境 NumPy ABI 冲突。"""
        channels_by_encoding = {
            "bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1,
        }
        encoding = msg.encoding.lower()
        channels = channels_by_encoding.get(encoding)
        if channels is None:
            self.get_logger().warning(f"不支持的图像编码 {msg.encoding}，已跳过")
            return None
        try:
            rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            image = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
        except ValueError as error:
            self.get_logger().warning(f"图像数据尺寸异常，已跳过：{error}")
            return None
        if encoding == "bgr8":
            return image.copy()
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    def _tile(self, topic, frame):
        tile = np.zeros((self._TILE_HEIGHT, self._TILE_WIDTH, 3), dtype=np.uint8)
        if frame is None:
            cv2.putText(tile, "等待图像", (20, 240), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (180, 180, 180), 2)
        else:
            height, width = frame.shape[:2]
            scale = min(self._TILE_WIDTH / width, self._TILE_HEIGHT / height)
            resized = cv2.resize(frame, (round(width * scale), round(height * scale)))
            y = (self._TILE_HEIGHT - resized.shape[0]) // 2
            x = (self._TILE_WIDTH - resized.shape[1]) // 2
            tile[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
        cv2.rectangle(tile, (0, 0), (self._TILE_WIDTH - 1, self._TILE_HEIGHT - 1),
                      (80, 220, 80), 2)
        cv2.putText(tile, topic, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (80, 220, 80), 2)
        return tile

    def _mosaic(self):
        with self._lock:
            frames = dict(self._frames)
        columns = math.ceil(math.sqrt(len(self._topics)))
        rows = math.ceil(len(self._topics) / columns)
        canvas = np.zeros(
            (rows * self._TILE_HEIGHT, columns * self._TILE_WIDTH, 3), dtype=np.uint8)
        for index, topic in enumerate(self._topics):
            row, column = divmod(index, columns)
            y, x = row * self._TILE_HEIGHT, column * self._TILE_WIDTH
            canvas[y:y + self._TILE_HEIGHT, x:x + self._TILE_WIDTH] = self._tile(
                topic, frames.get(topic))
        return canvas

    def _publish_mosaic(self):
        mosaic = self._mosaic()
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "camera_mosaic"
        message.height, message.width = mosaic.shape[:2]
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = message.width * 3
        message.data = mosaic.tobytes()
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = CameraMosaicViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
