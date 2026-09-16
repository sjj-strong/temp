"""将多个 ROS Image 话题显示为一个可缩放、可平移的 OpenCV 拼接窗口。"""

import math
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraMosaicViewer(Node):
    """只读图像查看器：滚轮缩放、左键拖拽平移、R 重置、Q/Esc 关闭窗口。"""

    _WINDOW_NAME = "UR Teleop Camera Mosaic"
    _TILE_WIDTH = 640
    _TILE_HEIGHT = 480

    def __init__(self):
        super().__init__("camera_mosaic_viewer")
        self.declare_parameter("topics", [])
        self._topics = [str(topic) for topic in self.get_parameter("topics").value]
        if not self._topics:
            raise RuntimeError("未配置 cameras.visualization.topics")
        self._bridge = CvBridge()
        self._frames = {}
        self._lock = threading.Lock()
        self._zoom, self._offset, self._drag_origin, self._closed = 1.0, [0, 0], None, False
        self._subscriptions = [
            self.create_subscription(Image, topic,
                                     lambda msg, current=topic: self._image_cb(current, msg), 5)
            for topic in self._topics
        ]
        cv2.namedWindow(self._WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._WINDOW_NAME, 1280, 800)
        cv2.setMouseCallback(self._WINDOW_NAME, self._mouse_cb)
        self._timer = self.create_timer(1.0 / 30.0, self._draw)
        self.get_logger().info("单窗口相机查看器：滚轮缩放、左键拖拽、R 重置、Q/Esc 关闭")

    def _image_cb(self, topic, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._frames[topic] = frame
        except Exception as error:
            self.get_logger().warning(f"无法转换 {topic} 图像: {error}")

    def _mouse_cb(self, event, x, y, flags, _):
        if event == cv2.EVENT_MOUSEWHEEL:
            self._zoom = min(4.0, max(0.25, self._zoom * (1.15 if flags > 0 else 1 / 1.15)))
        elif event == cv2.EVENT_LBUTTONDOWN:
            self._drag_origin = (x, y, *self._offset)
        elif event == cv2.EVENT_MOUSEMOVE and self._drag_origin is not None:
            origin_x, origin_y, offset_x, offset_y = self._drag_origin
            self._offset = [offset_x + x - origin_x, offset_y + y - origin_y]
        elif event == cv2.EVENT_LBUTTONUP:
            self._drag_origin = None

    def _tile(self, topic, frame):
        tile = np.zeros((self._TILE_HEIGHT, self._TILE_WIDTH, 3), dtype=np.uint8)
        if frame is None:
            cv2.putText(tile, "等待图像", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 180), 2)
        else:
            height, width = frame.shape[:2]
            scale = min(self._TILE_WIDTH / width, self._TILE_HEIGHT / height)
            resized = cv2.resize(frame, (round(width * scale), round(height * scale)))
            y, x = (self._TILE_HEIGHT - resized.shape[0]) // 2, (self._TILE_WIDTH - resized.shape[1]) // 2
            tile[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
        cv2.rectangle(tile, (0, 0), (self._TILE_WIDTH - 1, self._TILE_HEIGHT - 1), (80, 220, 80), 2)
        cv2.putText(tile, topic, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (80, 220, 80), 2)
        return tile

    def _mosaic(self):
        with self._lock:
            frames = dict(self._frames)
        columns = math.ceil(math.sqrt(len(self._topics)))
        rows = math.ceil(len(self._topics) / columns)
        canvas = np.zeros((rows * self._TILE_HEIGHT, columns * self._TILE_WIDTH, 3), dtype=np.uint8)
        for index, topic in enumerate(self._topics):
            row, column = divmod(index, columns)
            y, x = row * self._TILE_HEIGHT, column * self._TILE_WIDTH
            canvas[y:y + self._TILE_HEIGHT, x:x + self._TILE_WIDTH] = self._tile(topic, frames.get(topic))
        return canvas

    def _draw(self):
        if self._closed:
            return
        mosaic = self._mosaic()
        scaled = cv2.resize(mosaic, None, fx=self._zoom, fy=self._zoom)
        viewport = np.zeros_like(mosaic)
        source_x, source_y = max(0, -self._offset[0]), max(0, -self._offset[1])
        target_x, target_y = max(0, self._offset[0]), max(0, self._offset[1])
        width, height = min(scaled.shape[1] - source_x, viewport.shape[1] - target_x), min(scaled.shape[0] - source_y, viewport.shape[0] - target_y)
        if width > 0 and height > 0:
            viewport[target_y:target_y + height, target_x:target_x + width] = scaled[source_y:source_y + height, source_x:source_x + width]
        cv2.imshow(self._WINDOW_NAME, viewport)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            self._closed = True
            cv2.destroyWindow(self._WINDOW_NAME)
        elif key in (ord("r"), ord("R")):
            self._zoom, self._offset = 1.0, [0, 0]

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


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
