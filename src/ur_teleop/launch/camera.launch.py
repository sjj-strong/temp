"""按自定义相机名称独立启动 USB、RealSense 和选定的图像预览。"""

import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=os.path.join(
            get_package_share_directory("ur_teleop"), "config", "camera.yaml")),
        OpaqueFunction(function=_camera_actions),
    ])


def _camera_actions(context):
    config_file = LaunchConfiguration("config_file").perform(context)
    with open(config_file) as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("相机配置必须是以自定义相机名称为键的映射")
    actions, image_topics = [], []
    for name, camera in config.items():
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
            raise ValueError("相机名称必须以字母开头，仅包含字母、数字和下划线")
        if not isinstance(camera, dict) or camera.get('type') not in ('usb', 'realsense'):
            raise ValueError(f"相机 {name} 的 type 必须为 usb 或 realsense")
        for key, default in (('enabled', True), ('visualize', False)):
            if not isinstance(camera.get(key, default), bool):
                raise ValueError(f"相机 {name} 的 {key} 必须为布尔值")
        if not camera.get('enabled', True):
            continue
        width, height, fps = (camera.get(key, default) for key, default in
                              (('width', 640), ('height', 480), ('fps', 30)))
        if any(type(value) is not int or value <= 0 for value in (width, height, fps)):
            raise ValueError(f"相机 {name} 的 width、height、fps 必须为正整数")
        topic = camera.get('topic', f'/camera/{name}/color/image_raw')
        if not isinstance(topic, str) or not topic.startswith('/'):
            raise ValueError(f"相机 {name} 的 topic 必须为绝对 ROS 话题")
        if camera['type'] == 'usb':
            device = camera.get('device')
            if not isinstance(device, (str, int)) or isinstance(device, bool) or not str(device).strip():
                raise ValueError(f"USB 相机 {name} 必须配置 device")
            parameters = dict(camera_name=name, device=str(device), image_topic=topic,
                              frame_id=camera.get('frame_id', f'{name}_color_optical_frame'),
                              width=width, height=height, fps=float(fps))
            for key in ('fourcc', 'publish_compressed', 'compressed_quality',
                        'auto_exposure', 'exposure_time_absolute'):
                if key in camera:
                    parameters[key] = camera[key]
            actions.append(Node(package='data_collection', executable='opencv_camera_node',
                                name=name, output='screen', parameters=[parameters]))
        else:
            serial = camera.get('serial_no')
            if not isinstance(serial, str) or not serial.strip():
                raise ValueError(f"RealSense 相机 {name} 的 serial_no 必须为非空字符串，请加引号")
            parameters = {
                'camera_name': name,
                'serial_no': ParameterValue(serial, value_type=str),
                'enable_color': camera.get('enable_color', True),
                'enable_depth': camera.get('enable_depth', False),
                'rgb_camera.color_profile': f'{width},{height},{fps}',
                'enable_infra1': False, 'enable_infra2': False,
                'enable_gyro': False, 'enable_accel': False,
            }
            actions.append(Node(package='realsense2_camera', executable='realsense2_camera_node',
                                namespace='camera', name=name, output='screen',
                                parameters=[parameters],
                                remappings=[(f'/camera/{name}/color/image_raw', topic)]))
        if camera.get('visualize', False):
            image_topics.append(topic)
    if image_topics:
        # 仅将已启用并选择可视化的相机加入拼图，避免等待未启动的设备。
        actions.extend([
            Node(package='ur_teleop', executable='camera_mosaic_viewer',
                 name='camera_mosaic_viewer', output='screen',
                 parameters=[{'topics': image_topics}]),
            ExecuteProcess(cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view',
                                '/camera_mosaic/image_raw'], output='screen'),
        ])
    return actions
