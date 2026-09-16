"""独立相机发布入口：可选启动 RealSense 与 USB/OpenCV 相机，不依赖机器人栈。"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _yaml_default(config_file: str, *path: str, fallback: str):
    try:
        with open(config_file) as f:
            data = yaml.safe_load(f)
        for key in path:
            data = data[key]
        return str(data).lower() if isinstance(data, bool) else str(data)
    except Exception:
        return fallback


def _yaml_topics(config_file: str) -> list[str]:
    """读取需要显示的原始 Image 话题，忽略空值和非字符串项。"""
    try:
        with open(config_file) as f:
            topics = yaml.safe_load(f)["cameras"]["visualization"]["topics"]
        return [str(topic) for topic in topics if isinstance(topic, str) and topic]
    except Exception:
        return []


def generate_launch_description():
    pkg_share = get_package_share_directory("ur_teleop")
    config_file = os.path.join(pkg_share, "config", "ur_teleop.yaml")
    opencv_config = _yaml_default(
        config_file, "cameras", "opencv", "config_file", fallback="")
    if not opencv_config:
        opencv_config = os.path.join(pkg_share, "config", "opencv_cameras.yaml")
    data_collection_share = get_package_share_directory("data_collection")
    image_topics = _yaml_topics(config_file)

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config_file),
        DeclareLaunchArgument("launch_realsense", default_value=_yaml_default(
            config_file, "cameras", "realsense", "enabled", fallback="false")),
        DeclareLaunchArgument("launch_opencv_cameras", default_value=_yaml_default(
            config_file, "cameras", "opencv", "enabled", fallback="false")),
        DeclareLaunchArgument("launch_image_viewers", default_value=_yaml_default(
            config_file, "cameras", "visualization", "enabled", fallback="false")),
        DeclareLaunchArgument("opencv_camera_config", default_value=opencv_config),
        DeclareLaunchArgument("d435i_serial", default_value=_yaml_default(
            config_file, "cameras", "realsense", "d435i_serial", fallback="")),
        DeclareLaunchArgument("d455_serial", default_value=_yaml_default(
            config_file, "cameras", "realsense", "d455_serial", fallback="")),
        DeclareLaunchArgument("enable_d435i", default_value=_yaml_default(
            config_file, "cameras", "realsense", "enable_d435i", fallback="true")),
        DeclareLaunchArgument("enable_camera_color", default_value=_yaml_default(
            config_file, "cameras", "realsense", "enable_color", fallback="true")),
        DeclareLaunchArgument("enable_camera_depth", default_value=_yaml_default(
            config_file, "cameras", "realsense", "enable_depth", fallback="false")),
        DeclareLaunchArgument("camera_color_profile", default_value=_yaml_default(
            config_file, "cameras", "realsense", "color_profile", fallback="640,480,30")),
        DeclareLaunchArgument("camera_namespace", default_value=_yaml_default(
            config_file, "cameras", "realsense", "camera_namespace", fallback="camera")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                data_collection_share, "launch", "dual_realsense.launch.py")),
            condition=IfCondition(LaunchConfiguration("launch_realsense")),
            launch_arguments={
                "d435i_serial": LaunchConfiguration("d435i_serial"),
                "d455_serial": LaunchConfiguration("d455_serial"),
                "enable_d435i": LaunchConfiguration("enable_d435i"),
                "enable_color": LaunchConfiguration("enable_camera_color"),
                "enable_depth": LaunchConfiguration("enable_camera_depth"),
                "color_profile": LaunchConfiguration("camera_color_profile"),
                "camera_namespace": LaunchConfiguration("camera_namespace"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                data_collection_share, "launch", "opencv_cameras.launch.py")),
            condition=IfCondition(LaunchConfiguration("launch_opencv_cameras")),
            launch_arguments={"camera_config": LaunchConfiguration("opencv_camera_config")}.items(),
        ),
        # rqt_image_view 一次只能显示一个 topic，无法满足多视角单窗口需求。
        # 此节点将所有 raw Image 拼接为一个窗口，且只订阅图像、不参与控制。
        Node(
            package="ur_teleop", executable="camera_mosaic_viewer",
            name="camera_mosaic_viewer", output="screen",
            condition=IfCondition(LaunchConfiguration("launch_image_viewers")),
            parameters=[{"topics": image_topics}],
        ),
    ])
