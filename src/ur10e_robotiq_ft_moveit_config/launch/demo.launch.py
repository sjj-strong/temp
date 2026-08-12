from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur10e_robotiq_ft_moveit_config"),
            "launch",
            "bringup.launch.py",
        ])),
        launch_arguments={
            "use_mock_hardware": "true",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        bringup,
    ])
