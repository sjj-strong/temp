from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    combined_xacro = PathJoinSubstitution(
        [FindPackageShare('ur10e_robotiq_description'), 'urdf', 'ur10e_robotiq.urdf.xacro']
    )
    robot_description = {
        'robot_description': ParameterValue(
            Command(
                [
                    FindExecutable(name='xacro'),
                    ' ',
                    combined_xacro,
                    ' ur_type:=',
                    LaunchConfiguration('ur_type'),
                    ' robot_ip:=',
                    LaunchConfiguration('robot_ip'),
                    ' include_ros2_control:=true',
                    ' use_mock_hardware:=true',
                    ' use_fake_hardware:=true',
                    ' use_fake_mode:=true',
                ]
            ),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument('ur_type', default_value='ur10e'),
            DeclareLaunchArgument('robot_ip', default_value='0.0.0.0'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[robot_description],
                output='screen',
            ),
        ]
    )
