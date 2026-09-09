"""Launch the official FT300 and Robotiq 2F-85 hardware test stacks.

The two drivers own separate controller managers, so this launch file can be
used alongside the UR arm controller.  The Robotiq driver activates and
calibrates the physical gripper during startup; keep the gripper clear before
running it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Start the upstream FT300 and gripper drivers with explicit ports."""
    ftdi_id = LaunchConfiguration("ftdi_id")
    gripper_port = LaunchConfiguration("gripper_port")

    ft300_launch = os.path.join(
        get_package_share_directory("robotiq_ft_sensor_hardware"),
        "launch",
        "ft_sensor_standalone.launch.py",
    )
    gripper_launch = os.path.join(
        get_package_share_directory("robotiq_description"),
        "launch",
        "robotiq_control.launch.py",
    )

    return LaunchDescription([
        # The upstream FT300 standalone driver expects the device basename,
        # then opens /dev/<ftdi_id> itself.
        DeclareLaunchArgument(
            "ftdi_id",
            default_value="ttyUSB2",
            description="FT300 port basename; e.g. ttyUSB2 for /dev/ttyUSB2",
        ),
        DeclareLaunchArgument(
            "gripper_port",
            default_value="/dev/ttyUSB0",
            description="Robotiq 2F gripper serial port",
        ),
        LogInfo(msg=["FT300 test port: /dev/", ftdi_id]),
        LogInfo(msg=["Robotiq gripper test port: ", gripper_port]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ft300_launch),
            launch_arguments={
                "ftdi_id": ftdi_id,
                "frame_id": "robotiq_ft_frame_id",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gripper_launch),
            launch_arguments={
                "com_port": gripper_port,
                "launch_rviz": "false",
            }.items(),
        ),
    ])
