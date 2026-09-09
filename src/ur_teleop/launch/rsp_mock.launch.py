"""Robot state publisher for the combined UR10e + FT300 + Robotiq mock model.

Drop-in replacement for ur10e_robotiq_ft_description/rsp.launch.py as the
description_launchfile passed to ur_control.launch.py.

ur_ros2_control hardcodes calculate_dynamics=true in the mock hardware section,
which prevents mock_components/GenericSystem from instant-mirroring commands
to states.  We pipe the xacro output through sed to flip it to false so the
joint states actually follow trajectory commands in simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")

    xacro_file = PathJoinSubstitution([
        FindPackageShare("ur10e_robotiq_ft_description"),
        "urdf", "ur10e_robotiq_ft.urdf.xacro",
    ])

    # xacro | sed：把 calculate_dynamics 从 true → false
    robot_description_content = Command([
        FindExecutable(name="bash"), " -c \"",
        FindExecutable(name="xacro"), " ",
        xacro_file,
        " name:=ur10e_robotiq_ft",
        " ur_type:=ur10e",
        " tf_prefix:=",
        " robot_ip:=0.0.0.0",
        " script_filename:=",
        PathJoinSubstitution([
            FindPackageShare("ur_client_library"),
            "resources", "external_control.urscript",
        ]),
        " output_recipe_filename:=",
        PathJoinSubstitution([
            FindPackageShare("ur_robot_driver"),
            "resources", "rtde_output_recipe.txt",
        ]),
        " input_recipe_filename:=",
        PathJoinSubstitution([
            FindPackageShare("ur_robot_driver"),
            "resources", "rtde_input_recipe.txt",
        ]),
        " use_mock_hardware:=", use_mock_hardware,
        " mock_sensor_commands:=", mock_sensor_commands,
        # 组合模型中的 FT300 也必须使用 mock 硬件。仅 use_mock_hardware
        # 不会传递给独立的 robotiq_fts_ros2_control macro；若遗漏此参数，
        # controller_manager 会在启动时枚举真实 USB FT300 并阻塞。
        " ft_sensor_use_fake_mode:=true",
        " | sed 's/calculate_dynamics\\\">true</calculate_dynamics\\\">false</g'",
        "\"",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        DeclareLaunchArgument("mock_sensor_commands", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(
                    robot_description_content, value_type=str,
                ),
            }],
        ),
    ])
