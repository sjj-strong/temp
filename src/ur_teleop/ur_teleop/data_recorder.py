"""LeRobot data recorder (record mode). Owns the keyboard; the first Enter
starts episode 1 AND sends /teleop/enable so teleop_node begins control
(spec §6). Keys: Enter=开始 S=保存并结束 D=丢弃并重置 Q=退出并 finalize.
"""

import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image as RosImage
from std_msgs.msg import Bool, Float64MultiArray

try:
    from lerobot.datasets import LeRobotDataset
except ImportError:
    LeRobotDataset = None

from ur_teleop.config import UR_GRIPPER_JOINT, UR_JOINT_NAMES, default_config_path, load_config
from ur_teleop.frame_builder import FrameBuilder
from ur_teleop.keyboard import KeyboardReader


class DataRecorderNode(Node):
    def __init__(self):
        super().__init__("data_recorder")
        if LeRobotDataset is None:
            raise RuntimeError("LeRobot 未安装：source /opt/lerobot_venv/bin/activate")
        self.declare_parameter("config_file", default_config_path())
        cfg = load_config(self.get_parameter("config_file").value)
        self._rec = cfg.get("recorder", {})
        self._fps = int(self._rec.get("fps", 50))
        self._min_frames = int(self._rec.get("min_frames_per_episode", 2))
        self._cameras = self._rec.get("cameras", {})
        self._builder = FrameBuilder(self._rec, cfg.get("gripper", {}))

        self._lock = threading.Lock()
        self._ur_joints = None
        self._ur_gripper_rad = 0.0
        self._teleop_cmd = None
        self._ur_ee_pose = None
        self._camera_frames = {}
        self._enable_sent = False
        self._ee_warned = False
        self._missing_cam_warned = set()

        self._features, _, _ = self._builder.features()
        self._joint_sub = self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self._cmd_sub = self.create_subscription(Float64MultiArray, "/teleop/commands", self._cmd_cb, 10)
        self._enable_pub = self.create_publisher(Bool, "/teleop/enable", 10)

        self._ee_source = self._rec.get("ee_pose_source", "tf")
        if self._ee_source == "tf":
            from tf2_ros.buffer import Buffer
            from tf2_ros.transform_listener import TransformListener
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        elif self._ee_source == "topic":
            from geometry_msgs.msg import PoseStamped
            self.create_subscription(PoseStamped, self._rec.get("ee_pose_topic", "/tcp_pose"),
                                     self._tcp_cb, 10)

        for cam_name, cam_cfg in self._cameras.items():
            topic = cam_cfg.get("topic", "")
            if topic:
                self.create_subscription(RosImage, topic,
                                         lambda msg, cn=cam_name: self._camera_cb(cn, msg), 10)

        self._dataset = None
        self._episode_count = 0
        self._recording = False
        self._frame_count = 0
        self._kb = KeyboardReader()

    # ---------- callbacks ----------

    def _joint_cb(self, msg: JointState):
        names = set(msg.name)
        if not all(n in names for n in UR_JOINT_NAMES):
            return
        with self._lock:
            self._ur_joints = [msg.position[msg.name.index(n)] for n in UR_JOINT_NAMES]
            if UR_GRIPPER_JOINT in names:
                self._ur_gripper_rad = msg.position[msg.name.index(UR_GRIPPER_JOINT)]

    def _cmd_cb(self, msg: Float64MultiArray):
        with self._lock:
            self._teleop_cmd = list(msg.data) if msg.data else None

    def _tcp_cb(self, msg):
        p = msg.pose
        with self._lock:
            self._ur_ee_pose = [p.position.x, p.position.y, p.position.z,
                                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]

    def _camera_cb(self, cam_name: str, msg: RosImage):
        try:
            from cv_bridge import CvBridge
            img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="rgb8")
            with self._lock:
                self._camera_frames[cam_name] = img
        except Exception:
            pass

    def _get_ee_pose(self):
        """7 维 [x,y,z,qx,qy,qz,qw] 或 None（source=none 恒 None → NaN 段）。"""
        if self._ee_source == "topic":
            with self._lock:
                return list(self._ur_ee_pose) if self._ur_ee_pose else None
        if self._ee_source == "none":
            return None
        try:
            import rclpy.time
            t = self._tf_buffer.lookup_transform(
                self._rec.get("ee_pose_parent_frame", "base_link"),
                self._rec.get("ee_pose_child_frame", "gripper_tcp"),
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            tr, rot = t.transform.translation, t.transform.rotation
            return [tr.x, tr.y, tr.z, rot.x, rot.y, rot.z, rot.w]
        except Exception:
            return None

    # ---------- dataset ----------

    def _init_dataset(self):
        if self._dataset is not None:
            return
        repo_id = self._rec.get("repo_id", "my_user/ur_teleop")
        root = Path(self._rec["root"]) if self._rec.get("root") else None
        kwargs = dict(
            repo_id=repo_id, fps=self._fps, features=self._features, root=root,
            robot_type=self._rec.get("robot_type", "ur10e_alicia_teleop"),
            use_videos=self._rec.get("use_videos", True),
            image_writer_processes=self._rec.get("image_writer_processes", 0),
            image_writer_threads=self._rec.get("image_writer_threads", 2),
        )
        try:
            self._dataset = LeRobotDataset.create(**kwargs)
            self.get_logger().info(f"创建数据集: {repo_id}")
        except FileExistsError:
            import datetime
            new_id = f"{repo_id}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
            kwargs["repo_id"] = new_id
            self._dataset = LeRobotDataset.create(**kwargs)
            self.get_logger().warn(f"数据集已存在，新建带时间戳: {new_id}")

    # ---------- episode control ----------

    def _start_episode(self):
        if self._recording:
            return
        missing = [name for name in self._cameras if name not in self._camera_frames]
        if missing:
            self.get_logger().error(
                f"相机未收到帧，拒绝开始 episode: {missing}。检查相机 topic 后重新按 Enter")
            return
        self._missing_cam_warned = set()
        self._init_dataset()
        if not self._enable_sent:
            self._enable_pub.publish(Bool(data=True))
            self._enable_sent = True
            self.get_logger().info("已发送 /teleop/enable → teleop_node 开始控制")
        self._recording = True
        self._frame_count = 0
        self.get_logger().info(f"Episode {self._episode_count + 1} 开始（S=保存 D=丢弃 Q=退出）")

    def _save_episode(self):
        if not self._recording:
            return
        if self._frame_count < self._min_frames:
            self.get_logger().warn(f"少于 {self._min_frames} 帧，自动丢弃")
            self._dataset.clear_episode_buffer()
        else:
            self._dataset.save_episode()
            self._episode_count += 1
            self.get_logger().info(f"Episode {self._episode_count} 已保存（{self._frame_count} 帧）")
        self._recording = False
        self._missing_cam_warned = set()

    def _discard_episode(self):
        if not self._recording:
            return
        self._dataset.clear_episode_buffer()
        self.get_logger().info(f"Episode 已丢弃（{self._frame_count} 帧）")
        self._recording = False
        self._missing_cam_warned = set()

    def _record_frame(self):
        with self._lock:
            ur = list(self._ur_joints) if self._ur_joints else None
            cmd = list(self._teleop_cmd) if self._teleop_cmd else None
            gripper_rad = self._ur_gripper_rad
            cameras = dict(self._camera_frames)
        ee = self._get_ee_pose()
        if ee is None and not self._ee_warned and self._ee_source != "none":
            self._ee_warned = True
            self.get_logger().warn("EE 位姿查询失败，该段以 NaN 记录（仅警告一次）")
        frame = self._builder.build(ur, ee, gripper_rad, cmd)
        if frame is None:
            return
        for cam_name, cam_cfg in self._cameras.items():
            img = cameras.get(cam_name)
            if img is None:
                if cam_name not in self._missing_cam_warned:
                    self._missing_cam_warned.add(cam_name)
                    self.get_logger().warn(
                        f"相机 {cam_name} 无帧，本 episode 该相机的图像帧将被跳过")
                continue
            frame[f"observation.images.{cam_cfg.get('image_key', cam_name)}"] = img
        try:
            self._dataset.add_frame(frame)
            self._frame_count += 1
        except Exception as e:
            self.get_logger().error(f"add_frame 失败: {e}")

    def finalize(self):
        if self._recording:
            self._save_episode()
        if self._dataset is not None:
            self._dataset.finalize()
            self.get_logger().info(f"数据集 finalize 完成（{self._episode_count} episodes）")


def main():
    rclpy.init()
    node = DataRecorderNode()
    node.get_logger().info("=" * 60)
    node.get_logger().info("Data Recorder 就绪 — Enter=开始 S=保存 D=丢弃 Q=退出")
    node.get_logger().info("=" * 60)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    period = 1.0 / node._fps
    try:
        while rclpy.ok():
            t0 = time.time()
            if node._recording:
                node._record_frame()
            key = node._kb.read_key(0.0)
            if key == "enter":
                node._start_episode()
            elif key == "s":
                node._save_episode()
            elif key == "d":
                node._discard_episode()
            elif key == "q":
                node.get_logger().info("Q 按下，退出")
                break
            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
