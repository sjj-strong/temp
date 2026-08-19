"""Wrapper: ur10e_robotiq_ft_description rsp with mock hardware forced via launch args.

ur_control.launch.py only forwards robot_ip + ur_type to the description
launchfile; use_mock_hardware stays at rsp.launch.py's default ("false") and
the xacro generates URPositionHardwareInterface instead of mock_components.
This thin wrapper forces use_mock_hardware=true so cell.launch (and everything
else tunneled through ur_control) always gets the mock model.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("ur10e_robotiq_ft_description"),
                    "launch", "rsp.launch.py",
                )
            ),
            # ur_control.launch.py only passes robot_ip + ur_type; explicitly
            # forward everything the rsp needs for a mock hardware model.
            launch_arguments={
                "use_mock_hardware": "true",
                "mock_sensor_commands": "false",
            }.items(),
        ),
    ])
