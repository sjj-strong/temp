from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare('ur10e_robotiq_description')
    controllers_file = PathJoinSubstitution(
        [description_share, 'config', 'ur_controllers_mock.yaml']
    )
    rsp_launch_file = PathJoinSubstitution(
        [description_share, 'launch', 'robot_state_publisher.launch.py']
    )
    ur_control_launch = PathJoinSubstitution(
        [FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py']
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_control_launch),
                launch_arguments={
                    'ur_type': 'ur10e',
                    'robot_ip': '0.0.0.0',
                    'use_mock_hardware': 'true',
                    'launch_dashboard_client': 'false',
                    'launch_rviz': 'false',
                    'controllers_file': controllers_file,
                    'description_launchfile': rsp_launch_file,
                }.items(),
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'robotiq_gripper_controller',
                    '--controller-manager',
                    '/controller_manager',
                ],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'robotiq_force_torque_sensor_broadcaster',
                    '--controller-manager',
                    '/controller_manager',
                ],
                output='screen',
            ),
        ]
    )
