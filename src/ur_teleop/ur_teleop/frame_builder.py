"""LeRobot 帧组装：Alicia 关节目标或 Xbot abs/rel 笛卡尔位姿 action。"""

import numpy as np

from ur_teleop.config import UR_JOINT_NAMES
from ur_teleop.cartesian_action import action_names


class FrameBuilder:
    """Builds observation.state + action numpy arrays for LeRobotDataset.add_frame."""

    EE_NAMES = ["ee_x", "ee_y", "ee_z", "ee_qx", "ee_qy", "ee_qz", "ee_qw"]

    def __init__(self, recorder_config: dict, gripper_config: dict):
        self._rec = recorder_config
        self._gripper = gripper_config
        self._state_threshold = float(recorder_config.get("state_threshold_rad", 0.4))
        self._task = recorder_config.get("task", "teleoperation")
        self._use_videos = bool(recorder_config.get("use_videos", True))
        self._cartesian = recorder_config.get('action_space') == 'cartesian_pose'
        self._cart_names = action_names(recorder_config.get('action_mode', 'abs')) if self._cartesian else []

    def observation_flags(self):
        """Xbot 的各项观测可独立启用；Alicia 沿用原有配置。"""
        return {name: bool(self._rec.get(key, default)) for name, key, default in (
            ('joint_position', 'record_joint_position', True),
            ('joint_velocity', 'record_joint_velocity', False),
            ('joint_effort', 'record_joint_effort', False),
            ('tcp_pose', 'record_tcp_pose', True),
            ('gripper', 'record_ur_gripper', True),
            ('wrench', 'record_wrench', False),
        )}

    def features(self):
        features = {}
        state_names, action_names = [], []
        if self._cartesian:
            flags = self.observation_flags()
            if flags['joint_position']:
                state_names.extend(f'joint_position.{name}' for name in UR_JOINT_NAMES)
            if flags['joint_velocity']:
                state_names.extend(f'joint_velocity.{name}' for name in UR_JOINT_NAMES)
            if flags['joint_effort']:
                state_names.extend(f'joint_effort.{name}' for name in UR_JOINT_NAMES)
            if flags['tcp_pose']:
                state_names.extend(f'tcp_{name}' for name in self.EE_NAMES)
            if flags['gripper']:
                state_names.append('gripper_state')
            if flags['wrench']:
                state_names.extend(('force.x', 'force.y', 'force.z',
                                    'torque.x', 'torque.y', 'torque.z'))
                features['observation.wrench_reference_link'] = {'dtype': 'string', 'shape': (1,)}
            if flags['tcp_pose']:
                features['observation.tcp_reference_link'] = {'dtype': 'string', 'shape': (1,)}
                features['observation.tcp_link'] = {'dtype': 'string', 'shape': (1,)}
        else:
            if self._rec.get("record_ur_joints", True):
                state_names.extend(UR_JOINT_NAMES)
            if self._rec.get("record_ur_ee_pose", True):
                state_names.extend(self.EE_NAMES)
            if self._rec.get("record_ur_gripper", True):
                state_names.append("gripper_state")
        if state_names:
            features["observation.state"] = {
                "dtype": "float32", "shape": (len(state_names),), "names": state_names,
            }
        if self._rec.get("record_action_joints", True):
            if self._cartesian:
                action_names.extend(self._cart_names[:-1])
            else:
                action_names.extend([f"cmd_{n}" for n in UR_JOINT_NAMES])
        if self._rec.get("record_action_gripper", True):
            action_names.append("cmd_gripper")
        if action_names:
            features["action"] = {
                "dtype": "float32", "shape": (len(action_names),), "names": action_names,
            }
        if self._cartesian:
            features['action.reference_link'] = {'dtype': 'string', 'shape': (1,)}
        for cam_name, cam_cfg in self._rec.get("cameras", {}).items():
            if not cam_cfg.get('enabled', True):
                continue
            key = f"observation.images.{cam_cfg.get('image_key', cam_name)}"
            features[key] = {
                "dtype": "video" if self._use_videos else "image",
                "shape": (int(cam_cfg.get("height", 480)), int(cam_cfg.get("width", 640)), 3),
                "names": ["height", "width", "channels"],
            }
        return features, state_names, action_names

    def build(self, ur_joints, ee_pose, gripper_state_rad, teleop_cmd,
              joint_velocity=None, joint_effort=None, wrench=None,
              wrench_reference_link=None, reference_link=None, tcp_link=None):
        """None when essential data missing; ee_pose None → NaN segment."""
        if teleop_cmd is None or (not self._cartesian and ur_joints is None):
            return None
        if self._cartesian and (len(teleop_cmd) != len(self._cart_names) or not np.isfinite(teleop_cmd).all()):
            return None
        state_parts = []
        if self._cartesian:
            flags = self.observation_flags()
            sources = ((flags['joint_position'], ur_joints, 6),
                       (flags['joint_velocity'], joint_velocity, 6),
                       (flags['joint_effort'], joint_effort, 6),
                       (flags['tcp_pose'], ee_pose, 7),
                       (flags['wrench'], wrench, 6))
            if any(enabled and (values is None or len(values) != size or
                                not np.isfinite(values).all())
                   for enabled, values, size in sources):
                return None
            if flags['gripper'] and (gripper_state_rad is None or not np.isfinite(gripper_state_rad)):
                return None
            for name, values in (('joint_position', ur_joints), ('joint_velocity', joint_velocity),
                                 ('joint_effort', joint_effort), ('tcp_pose', ee_pose)):
                if flags[name]:
                    state_parts.extend(values)
            if flags['gripper']:
                state_parts.append(1.0 if gripper_state_rad > self._state_threshold else 0.0)
            if flags['wrench']:
                if not wrench_reference_link:
                    return None
                state_parts.extend(wrench)
        else:
            if self._rec.get("record_ur_joints", True):
                state_parts.extend(ur_joints[:6])
            if self._rec.get("record_ur_ee_pose", True):
                state_parts.extend(ee_pose if ee_pose is not None else [np.nan] * 7)
            if self._rec.get("record_ur_gripper", True):
                state_parts.append(1.0 if gripper_state_rad > self._state_threshold else 0.0)
        action_parts = []
        motion_size = len(self._cart_names) - 1 if self._cartesian else 6
        if self._rec.get("record_action_joints", True):
            action_parts.extend(teleop_cmd[:motion_size])
        if self._rec.get("record_action_gripper", True):
            action_parts.append(float(teleop_cmd[motion_size]) if len(teleop_cmd) > motion_size else 0.0)
        frame = {'action': np.array(action_parts, dtype=np.float32), 'task': self._task}
        if state_parts:
            frame['observation.state'] = np.array(state_parts, dtype=np.float32)
        if self._cartesian:
            if not reference_link:
                return None
            frame['action.reference_link'] = reference_link
            if flags['tcp_pose']:
                if not tcp_link:
                    return None
                frame['observation.tcp_reference_link'] = reference_link
                frame['observation.tcp_link'] = tcp_link
            if flags['wrench']:
                frame['observation.wrench_reference_link'] = wrench_reference_link
        return frame
