"""LeRobot 录制器：Alicia 键盘控制，Xbot 手柄事件及 abs/rel 位姿 action。"""

import os
import sys
import threading
import time
import queue
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image as RosImage
from std_msgs.msg import Bool, Float64MultiArray, String
from geometry_msgs.msg import WrenchStamped

try:
    from lerobot.datasets import LeRobotDataset
except ImportError:
    LeRobotDataset = None

from ur_teleop.config import UR_GRIPPER_JOINT, UR_JOINT_NAMES, default_config_path, load_config
from ur_teleop.frame_builder import FrameBuilder
from ur_teleop.cartesian_action import action_label
from ur_teleop.controller_frame import controller_base_frame
from ur_teleop.keyboard import KeyboardReader


class DataRecorderNode(Node):
    def __init__(self):
        super().__init__("data_recorder")
        if LeRobotDataset is None:
            raise RuntimeError("LeRobot 未安装：source /opt/lerobot_venv/bin/activate")
        self.declare_parameter("config_file", default_config_path())
        cfg = load_config(self.get_parameter("config_file").value)
        self._xbot = cfg['teleop'].get('control_source', 'alicia') == 'xbot'
        self._rec = cfg.get("recorder", {})
        if self._xbot:
            self._rec = dict(self._rec, action_space='cartesian_pose', record_action_joints=True)
        self._reference_link = (controller_base_frame(cfg['xbot']['controller_config_file'])
                                if self._xbot else None)
        self._tcp_link = self._rec.get('ee_pose_child_frame', 'tool0')
        self._fps = int(self._rec.get("fps", 50))
        self._min_frames = int(self._rec.get("min_frames_per_episode", 2))
        self._cameras = {name: camera for name, camera in self._rec.get('cameras', {}).items()
                         if camera.get('enabled', True)}
        self._builder = FrameBuilder(self._rec, cfg.get("gripper", {}))

        self._lock = threading.Lock()
        self._ur_joints = None
        self._joint_velocity = None
        self._joint_effort = None
        self._wrench = None
        self._wrench_reference_link = None
        self._ur_gripper_rad = 0.0
        self._teleop_cmd = None
        self._ur_ee_pose = None
        self._ee_at = -float('inf')
        self._camera_frames = {}
        self._enable_sent = False
        self._ee_warned = False
        self._missing_cam_warned = set()
        self._events = queue.Queue(maxsize=32)
        self._ready = False
        self._ready_at = self._cmd_at = self._joint_at = -float('inf')
        self._gripper_at = -float('inf')
        self._joint_velocity_at = self._joint_effort_at = self._wrench_at = -float('inf')
        self._camera_at = {}
        self._data_timeout = float(self._rec.get('data_timeout_s', .5))

        self._features, _, _ = self._builder.features()
        self._action_size = (len(self._builder._cart_names) if self._xbot else
                             self._features.get('action', {}).get('shape', (0,))[0])
        self._action_label = (action_label(self._rec.get('action_mode', 'abs'), self._reference_link)
                              if self._xbot else None)
        flags = self._builder.observation_flags() if self._xbot else {}
        need_joints = not self._xbot or any(flags[name] for name in (
            'joint_position', 'joint_velocity', 'joint_effort', 'gripper'))
        self._joint_sub = (self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
                           if need_joints else None)
        self._cmd_sub = self.create_subscription(Float64MultiArray, "/teleop/commands", self._cmd_cb, 10)
        self._enable_pub = self.create_publisher(Bool, "/teleop/enable", 10)
        self._finished_pub = self.create_publisher(Bool, '/teleop/record_finished', 10)
        if self._xbot:
            self.create_subscription(String, '/teleop/record_event', self._event_cb, 10)
            self.create_subscription(Bool, '/teleop/xbot_ready', self._ready_cb, 1)
            if self._builder.observation_flags()['wrench']:
                topic = ('/robotiq_force_torque_sensor_broadcaster/wrench'
                         if cfg.get('cell', {}).get('ft300_enabled', False)
                         else '/force_torque_sensor_broadcaster/ft_data')
                self.create_subscription(WrenchStamped, topic, self._wrench_cb, 10)

        self._ee_source = (self._rec.get("ee_pose_source", "tf")
                           if not self._xbot or flags['tcp_pose'] else 'none')
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
        self._kb = None if self._xbot else KeyboardReader()

    # ---------- callbacks ----------

    def _event_cb(self, msg):
        if msg.data in ('start', 'save', 'discard', 'finalize'):
            try:
                self._events.put_nowait((msg.data, time.monotonic()))
            except queue.Full:
                self.get_logger().error('录制操作队列已满，请等待当前操作完成')

    def _ready_cb(self, msg):
        self._ready, self._ready_at = msg.data, time.monotonic()

    def _xbot_data_ready(self):
        now = time.monotonic()
        flags = self._builder.observation_flags()
        return (self._ready and now - self._ready_at < self._data_timeout and
                self._teleop_cmd is not None and len(self._teleop_cmd) == self._action_size and
                all(math.isfinite(v) for v in self._teleop_cmd) and
                now - self._cmd_at < self._data_timeout and
                (not flags['joint_position'] or now - self._joint_at < self._data_timeout) and
                (not flags['joint_velocity'] or now - self._joint_velocity_at < self._data_timeout) and
                (not flags['joint_effort'] or now - self._joint_effort_at < self._data_timeout) and
                (not flags['wrench'] or now - self._wrench_at < self._data_timeout) and
                (not flags['gripper'] or
                 now - self._gripper_at < self._data_timeout) and
                all(now - self._camera_at.get(n, -float('inf')) < self._data_timeout
                    for n in self._cameras))

    def poll_event(self):
        """只在录制主线程执行数据集写入，不阻塞 ROS 回调线程。"""
        try:
            event, stamp = self._events.get_nowait()
        except queue.Empty:
            return False
        if event == 'finalize':
            self._finished_pub.publish(Bool(data=True))
            return True
        if time.monotonic() - stamp > 2.:
            self.get_logger().warn('忽略过期录制操作，请重新按键')
        elif event == 'start':
            self._start_episode()
        elif event == 'save':
            self._save_episode()
        elif event == 'discard':
            self._discard_episode()
        return False

    def _joint_cb(self, msg: JointState):
        names = set(msg.name)
        with self._lock:
            if all(n in names for n in UR_JOINT_NAMES):
                indices = [msg.name.index(n) for n in UR_JOINT_NAMES]
                for values, field, stamp in ((msg.position, '_ur_joints', '_joint_at'),
                                             (msg.velocity, '_joint_velocity', '_joint_velocity_at'),
                                             (msg.effort, '_joint_effort', '_joint_effort_at')):
                    if len(values) == len(msg.name) and all(math.isfinite(values[i]) for i in indices):
                        setattr(self, field, [values[i] for i in indices])
                        setattr(self, stamp, time.monotonic())
            # 实机夹爪是独立 controller_manager，可能单独发布 JointState。
            if (UR_GRIPPER_JOINT in names and len(msg.position) == len(msg.name) and
                    math.isfinite(msg.position[msg.name.index(UR_GRIPPER_JOINT)])):
                self._ur_gripper_rad = msg.position[msg.name.index(UR_GRIPPER_JOINT)]
                self._gripper_at = time.monotonic()

    def _wrench_cb(self, msg: WrenchStamped):
        force, torque = msg.wrench.force, msg.wrench.torque
        values = [force.x, force.y, force.z, torque.x, torque.y, torque.z]
        if not msg.header.frame_id or not all(math.isfinite(value) for value in values):
            return
        with self._lock:
            self._wrench = values
            self._wrench_reference_link = msg.header.frame_id
            self._wrench_at = time.monotonic()

    def _cmd_cb(self, msg: Float64MultiArray):
        with self._lock:
            if self._xbot and (len(msg.data) != self._action_size or len(msg.layout.dim) != 1 or
                              msg.layout.dim[0].label != self._action_label or
                              not all(math.isfinite(v) for v in msg.data)):
                self._teleop_cmd = None
                self._cmd_at = -float('inf')
                return
            self._teleop_cmd = list(msg.data) if msg.data else None
            self._cmd_at = time.monotonic()

    def _tcp_cb(self, msg):
        p = msg.pose
        values = [p.position.x, p.position.y, p.position.z,
                  p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        if self._xbot and (msg.header.frame_id != self._reference_link or
                           not all(math.isfinite(value) for value in values)):
            return
        with self._lock:
            self._ur_ee_pose = values
            self._ee_at = time.monotonic()

    def _camera_cb(self, cam_name: str, msg: RosImage):
        try:
            from cv_bridge import CvBridge
            img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="rgb8")
            with self._lock:
                self._camera_frames[cam_name] = img
                self._camera_at[cam_name] = time.monotonic()
        except Exception:
            pass

    def _get_ee_pose(self):
        """7 维 [x,y,z,qx,qy,qz,qw] 或 None（source=none 恒 None → NaN 段）。"""
        if self._ee_source == "topic":
            with self._lock:
                if self._xbot and time.monotonic() - self._ee_at >= self._data_timeout:
                    return None
                return list(self._ur_ee_pose) if self._ur_ee_pose else None
        if self._ee_source == "none":
            return None
        try:
            import rclpy.time
            t = self._tf_buffer.lookup_transform(
                self._reference_link if self._xbot else self._rec.get("ee_pose_parent_frame", "base_link"),
                self._tcp_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0. if self._xbot else 0.5))
            if self._xbot:
                age = (self.get_clock().now() - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds / 1e9
                if not 0 <= age <= self._data_timeout:
                    return None
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
            suffix = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            new_id = f"{repo_id}_{suffix}"
            kwargs["repo_id"] = new_id
            if root is not None:
                kwargs['root'] = root.with_name(root.name + '_' + suffix)
            self._dataset = LeRobotDataset.create(**kwargs)
            self.get_logger().warn(f"数据集已存在，新建带时间戳: {new_id}")

    # ---------- episode control ----------

    def _start_episode(self):
        if self._recording:
            return
        need_tcp = self._xbot and self._builder.observation_flags()['tcp_pose']
        if self._xbot and (not self._xbot_data_ready() or (need_tcp and self._get_ee_pose() is None)):
            self.get_logger().warn('遥操作/状态/相机未就绪，拒绝开始 episode；就绪后重新按 Menu')
            return
        missing = [name for name in self._cameras if name not in self._camera_frames]
        if missing:
            self.get_logger().error(
                f"相机未收到帧，拒绝开始 episode: {missing}。检查相机 topic 后重新按 Enter")
            return
        self._missing_cam_warned = set()
        self._init_dataset()
        if not self._xbot and not self._enable_sent:
            self._enable_pub.publish(Bool(data=True))
            self._enable_sent = True
            self.get_logger().info("已发送 /teleop/enable → teleop_node 开始控制")
        self._recording = True
        self._frame_count = 0
        hint = 'Y=保存 B=丢弃 View长按=退出' if self._xbot else 'S=保存 D=丢弃 Q=退出'
        self.get_logger().info(f"Episode {self._episode_count + 1} 开始（{hint}）")

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
        if self._xbot and not self._xbot_data_ready():
            return
        with self._lock:
            ur = list(self._ur_joints) if self._ur_joints else None
            velocity = list(self._joint_velocity) if self._joint_velocity else None
            effort = list(self._joint_effort) if self._joint_effort else None
            wrench = list(self._wrench) if self._wrench else None
            wrench_link = self._wrench_reference_link
            cmd = list(self._teleop_cmd) if self._teleop_cmd else None
            gripper_rad = self._ur_gripper_rad
            cameras = dict(self._camera_frames)
        need_tcp = not self._xbot or self._builder.observation_flags()['tcp_pose']
        ee = self._get_ee_pose() if need_tcp else None
        if self._xbot and need_tcp and ee is None:
            return
        if ee is None and not self._ee_warned and self._ee_source != "none":
            self._ee_warned = True
            self.get_logger().warn("EE 位姿查询失败，该段以 NaN 记录（仅警告一次）")
        frame = self._builder.build(ur, ee, gripper_rad, cmd,
                                    joint_velocity=velocity, joint_effort=effort, wrench=wrench,
                                    wrench_reference_link=wrench_link,
                                    reference_link=self._reference_link, tcp_link=self._tcp_link)
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
    # ros2 launch 用 /usr/bin/python3 启动节点（console_scripts shebang），
    # 而 lerobot 只装在 /opt/lerobot_venv。当前解释器缺 lerobot 时，用 venv
    # python os.execv 原地重启本进程：PID 不变、launch 的日志捕获与进程管理
    # 不受影响；环境变量（含 PYTHONPATH）与 ros args 原样继承。
    # 注意：venv 的 python 是指向 /usr/bin/python3 的 symlink，realpath 比较
    # 恒等，不能作为"已在 venv"的判据。UR_TELEOP_REEXEC 才是防无限重启的
    # 唯一护栏（venv 损坏时第二次进入直接走下面的 RuntimeError）。
    if LeRobotDataset is None and os.environ.get("UR_TELEOP_REEXEC") != "1":
        venv_python = "/opt/lerobot_venv/bin/python"
        if os.path.exists(venv_python):
            os.environ["UR_TELEOP_REEXEC"] = "1"
            os.execv(venv_python, [venv_python] + sys.argv)
    rclpy.init()
    node = DataRecorderNode()
    node.get_logger().info("=" * 60)
    node.get_logger().info(f"Data Recorder 就绪 — 录制频率 {node._fps} Hz")
    node.get_logger().info("  手柄: Menu=开始 Y=保存 B=丢弃 View长按=退出" if node._xbot else
                           "  键盘: Enter=开始  S=保存  D=丢弃  Q=退出")
    node.get_logger().info("=" * 60)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    period = 1.0 / node._fps
    hint_interval = 5.0          # 定期重印键盘提示，保证提示始终在终端可见
    last_hint = 0.0
    try:
        while rclpy.ok():
            t0 = time.time()
            if node._recording:
                node._record_frame()
            if node._xbot and node.poll_event():
                break
            key = node._kb.read_key(0.0) if node._kb is not None else None
            if key == "enter":
                node._start_episode()
            elif key == "s":
                node._save_episode()
            elif key == "d":
                node._discard_episode()
            elif key == "q":
                node.get_logger().info("Q 按下，退出")
                break
            if t0 - last_hint >= hint_interval:
                last_hint = t0
                state = "录制中" if node._recording else "待机"
                hint = '[手柄] Menu=开始 Y=保存 B=丢弃 View长按=退出' if node._xbot else '[键盘] Enter=开始 S=保存 D=丢弃 Q=退出'
                print(f"{hint} | 状态: {state} "
                      f"| episodes={node._episode_count} frames={node._frame_count} "
                      f"| {node._fps} Hz", flush=True)
            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
