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

from ur_teleop.session_logging import debug_log, log_event, EpisodeProgress
from ur_teleop.config import UR_JOINT_NAMES, default_config_path, load_config
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
        self._debug = cfg.get("debug", False)
        self._progress = None
        self._xbot = cfg['teleop'].get('control_source', 'alicia') == 'xbot'
        self._rec = cfg.get("recorder", {})
        if self._xbot:
            self._rec = dict(self._rec, action_space='cartesian_pose', record_action_joints=True)
        self._reference_link = (controller_base_frame(cfg['xbot']['controller_config_file'])
                                if self._xbot else None)
        self._tcp_link = self._rec.get('ee_pose_child_frame', 'tool0')
        self._fps = int(self._rec.get("fps", 50))
        if self._fps <= 0:
            raise ValueError("recorder.fps 必须为正数")
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
        self._teleop_cmd = None
        self._camera_frames = {}
        self._enable_sent = False
        self._ee_warned = False
        self._missing_cam_warned = set()
        self._events = queue.Queue(maxsize=32)
        self._ready = False
        self._ready_at = self._cmd_at = self._joint_at = -float('inf')
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
            'joint_position', 'joint_velocity', 'joint_effort'))
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

        if not self._xbot or flags['tcp_pose']:
            from tf2_ros.buffer import Buffer
            from tf2_ros.transform_listener import TransformListener
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

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
                log_event(self, "error", level="error", message='录制操作队列已满，请等待当前操作完成')

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
                all(now - self._camera_at.get(n, -float('inf')) < self._data_timeout
                    for n in self._cameras))

    def poll_event(self):
        """只在录制主线程执行数据集写入，不阻塞 ROS 回调线程。"""
        try:
            event, stamp = self._events.get_nowait()
        except queue.Empty:
            return False
        if event == 'finalize':
            log_event(self, 'keyboard', action='quit', message='View 长按，退出采集')
            self._finished_pub.publish(Bool(data=True))
            return True
        if time.monotonic() - stamp > 2.:
            log_event(self, "warn", level="warn", message='忽略过期录制操作，请重新按键')
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
        """从 TF 获取 7 维 [x,y,z,qx,qy,qz,qw]，不可用时返回 None。"""
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
            debug_log(self, f"创建数据集: {repo_id}")
        except FileExistsError:
            import datetime
            suffix = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f')
            new_id = f"{repo_id}-{suffix}"
            kwargs["repo_id"] = new_id
            if root is not None:
                kwargs['root'] = root.with_name(root.name + '-' + suffix)
            self._dataset = LeRobotDataset.create(**kwargs)
            log_event(self, "warn", level="warn", message=f"数据集已存在，新建带时间戳: {new_id}")

    # ---------- episode control ----------

    def _start_episode(self):
        if self._recording:
            return
        need_tcp = self._xbot and self._builder.observation_flags()['tcp_pose']
        if self._xbot and (not self._xbot_data_ready() or (need_tcp and self._get_ee_pose() is None)):
            log_event(self, "warn", level="warn", message='遥操作/状态/相机未就绪，拒绝开始 episode；就绪后重新按 Menu')
            return
        missing = [name for name in self._cameras if name not in self._camera_frames]
        if missing:
            log_event(self, "error", level="error", message=f"相机未收到帧，拒绝开始 episode: {missing}。检查相机 topic 后重新按 Enter")
            return
        self._missing_cam_warned = set()
        self._init_dataset()
        if not self._xbot and not self._enable_sent:
            self._enable_pub.publish(Bool(data=True))
            self._enable_sent = True
            debug_log(self, "已发送 /teleop/enable → teleop_node 开始控制")
        self._recording = True
        self._frame_count = 0
        hint = 'Y=保存 B=丢弃 View长按=退出' if self._xbot else 'S=保存 D=丢弃 Q=退出'
        log_event(self, "keyboard", action="start", episode=self._episode_count + 1, message=hint)
        self._progress = EpisodeProgress(self._episode_count + 1, self._fps)

    def _save_episode(self):
        if not self._recording:
            return
        self._close_progress()
        if self._frame_count < self._min_frames:
            log_event(self, "warn", level="warn", message=f"少于 {self._min_frames} 帧，自动丢弃")
            self._dataset.clear_episode_buffer()
        else:
            self._dataset.save_episode()
            self._episode_count += 1
            log_event(self, "keyboard", action="save", episode=self._episode_count, frames=self._frame_count)
        self._recording = False
        self._close_progress()
        self._missing_cam_warned = set()

    def _discard_episode(self):
        if not self._recording:
            return
        self._close_progress()
        self._dataset.clear_episode_buffer()
        log_event(self, "keyboard", action="discard", frames=self._frame_count)
        self._recording = False
        self._close_progress()
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
            cameras = dict(self._camera_frames)
        need_tcp = not self._xbot or self._builder.observation_flags()['tcp_pose']
        ee = self._get_ee_pose() if need_tcp else None
        if self._xbot and need_tcp and ee is None:
            return
        if need_tcp and ee is None and not self._ee_warned:
            self._ee_warned = True
            log_event(self, "warn", level="warn", message="EE 位姿查询失败，该段以 NaN 记录（仅警告一次）")
        frame = self._builder.build(ur, ee, cmd,
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
                    log_event(self, "warn", level="warn", message=f"相机 {cam_name} 无帧，本 episode 该相机的图像帧将被跳过")
                continue
            frame[f"observation.images.{cam_cfg.get('image_key', cam_name)}"] = img
        try:
            self._dataset.add_frame(frame)
            self._frame_count += 1
            if getattr(self, "_progress", None) is not None:
                self._progress.update()
        except Exception as e:
            log_event(self, "error", level="error", message=f"add_frame 失败: {e}")

    def _close_progress(self):
        if getattr(self, "_progress", None) is not None:
            self._progress.close()
            self._progress = None

    def finalize(self):
        try:
            if self._recording:
                self._save_episode()
            if self._dataset is not None:
                self._dataset.finalize()
                debug_log(self, f"数据集 finalize 完成（{self._episode_count} episodes）")
        finally:
            self._close_progress()


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
    log_event(node, "collection_frequency", target_hz=node._fps)
    log_event(node, "keyboard", message="手柄: Menu=开始 Y=保存 B=丢弃 View长按=退出" if node._xbot else
              "键盘: Enter=开始 S=保存 D=丢弃 Q=退出")

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    period = 1.0 / node._fps
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
                log_event(node, "keyboard", action="quit", message="Q 按下，退出")
                break
            if node._progress is not None:
                node._progress.tick()
            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.finalize()
        finally:
            node._close_progress()
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
