#!/usr/bin/env python3

"""Robot state publisher only: UR + FT300 + Robotiq 2F-85 combined model.

Used by ur_teleop's cell.launch.py as `description_launchfile` so that rviz
(and the controller manager via /robot_description) sees the full assembly —
UR arm, FT300, 2F-85 gripper — instead of the bare UR model.

The xacro is ur10e_robotiq_ft.urdf.xacro, which reuses the official
`ur_ros2_control` macro (ur_robot_driver/urdf/ur.ros2_control.xacro), so the
combined model carries the same <ros2_control> tag as the official flow and
works as a full cell description (mock/real hardware auto-selected by
`use_mock_hardware`, inherited from ur_control.launch.py's launch configs).
Argument set mirrors ur_robot_driver/launch/ur_rsp.launch.py so this file is a
drop-in `description_launchfile` replacement.
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

    # ============================================================
    # Launch arguments
    # ============================================================

    name = LaunchConfiguration("name")
    ur_type = LaunchConfiguration("ur_type")
    tf_prefix = LaunchConfiguration("tf_prefix")
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    headless_mode = LaunchConfiguration("headless_mode")
    verify_robot_model = LaunchConfiguration("verify_robot_model")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_voltage = LaunchConfiguration("tool_voltage")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    reverse_ip = LaunchConfiguration("reverse_ip")
    script_command_port = LaunchConfiguration("script_command_port")
    reverse_port = LaunchConfiguration("reverse_port")
    script_sender_port = LaunchConfiguration("script_sender_port")
    trajectory_port = LaunchConfiguration("trajectory_port")
    script_filename = LaunchConfiguration("script_filename")
    input_recipe_filename = LaunchConfiguration("input_recipe_filename")
    output_recipe_filename = LaunchConfiguration("output_recipe_filename")
    joint_limit_params_file = LaunchConfiguration("joint_limit_params_file")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    physical_params_file = LaunchConfiguration("physical_params_file")
    visual_params_file = LaunchConfiguration("visual_params_file")
    transmission_hw_interface = LaunchConfiguration("transmission_hw_interface")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    ft_sensor_use_fake_mode = LaunchConfiguration("ft_sensor_use_fake_mode")
    ft_sensor_max_retries = LaunchConfiguration("ft_sensor_max_retries")
    ft_sensor_read_rate = LaunchConfiguration("ft_sensor_read_rate")
    ft_sensor_ftdi_id = LaunchConfiguration("ft_sensor_ftdi_id")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="ur10e_robotiq_ft"),
        DeclareLaunchArgument("ur_type", default_value="ur10e"),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("robot_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("mock_sensor_commands", default_value="false"),
        DeclareLaunchArgument("headless_mode", default_value="false"),
        DeclareLaunchArgument("verify_robot_model", default_value="false"),
        DeclareLaunchArgument("use_tool_communication", default_value="false"),
        DeclareLaunchArgument("tool_voltage", default_value="0"),
        DeclareLaunchArgument("tool_parity", default_value="0"),
        DeclareLaunchArgument("tool_baud_rate", default_value="115200"),
        DeclareLaunchArgument("tool_stop_bits", default_value="1"),
        DeclareLaunchArgument("tool_rx_idle_chars", default_value="1.5"),
        DeclareLaunchArgument("tool_tx_idle_chars", default_value="3.5"),
        DeclareLaunchArgument("tool_device_name", default_value="/tmp/ttyUR"),
        DeclareLaunchArgument("tool_tcp_port", default_value="54321"),
        DeclareLaunchArgument("reverse_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("script_command_port", default_value="50004"),
        DeclareLaunchArgument("reverse_port", default_value="50001"),
        DeclareLaunchArgument("script_sender_port", default_value="50002"),
        DeclareLaunchArgument("trajectory_port", default_value="50003"),
        DeclareLaunchArgument(
            "script_filename",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_client_library"),
                 "resources", "external_control.urscript"]
            ),
        ),
        DeclareLaunchArgument(
            "input_recipe_filename",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"),
                 "resources", "rtde_input_recipe.txt"]
            ),
        ),
        DeclareLaunchArgument(
            "output_recipe_filename",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"),
                 "resources", "rtde_output_recipe.txt"]
            ),
        ),
        DeclareLaunchArgument(
            "joint_limit_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "config", ur_type,
                 "joint_limits.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "config", ur_type,
                 "default_kinematics.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "physical_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "config", ur_type,
                 "physical_parameters.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "visual_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "config", ur_type,
                 "visual_parameters.yaml"]
            ),
        ),
        DeclareLaunchArgument("transmission_hw_interface", default_value=""),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "config",
                 "initial_positions.yaml"]
            ),
        ),
        # FT300 参数需要由上层 bringup 显式传入；空 ftdi_id 会令硬件插件
        # 扫描全部串口，并在未发现设备时阻塞 controller_manager 的激活。
        DeclareLaunchArgument("ft_sensor_use_fake_mode", default_value="false"),
        DeclareLaunchArgument("ft_sensor_max_retries", default_value="100"),
        DeclareLaunchArgument("ft_sensor_read_rate", default_value="10"),
        DeclareLaunchArgument("ft_sensor_ftdi_id", default_value=""),
    ]

    # ============================================================
    # Xacro
    # ============================================================

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("ur10e_robotiq_ft_description"),
            "urdf",
            "ur10e_robotiq_ft.urdf.xacro",
        ]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " ",
            "name:=",
            name,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "joint_limit_params:=",
            joint_limit_params_file,
            " ",
            "kinematics_params:=",
            kinematics_params_file,
            " ",
            "physical_params:=",
            physical_params_file,
            " ",
            "visual_params:=",
            visual_params_file,
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "verify_robot_model:=",
            verify_robot_model,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "headless_mode:=",
            headless_mode,
            " ",
            "use_tool_communication:=",
            use_tool_communication,
            " ",
            "tool_parity:=",
            tool_parity,
            " ",
            "tool_baud_rate:=",
            tool_baud_rate,
            " ",
            "tool_stop_bits:=",
            tool_stop_bits,
            " ",
            "tool_rx_idle_chars:=",
            tool_rx_idle_chars,
            " ",
            "tool_tx_idle_chars:=",
            tool_tx_idle_chars,
            " ",
            "tool_device_name:=",
            tool_device_name,
            " ",
            "tool_tcp_port:=",
            tool_tcp_port,
            " ",
            "tool_voltage:=",
            tool_voltage,
            " ",
            "reverse_ip:=",
            reverse_ip,
            " ",
            "script_command_port:=",
            script_command_port,
            " ",
            "reverse_port:=",
            reverse_port,
            " ",
            "script_sender_port:=",
            script_sender_port,
            " ",
            "trajectory_port:=",
            trajectory_port,
            " ",
            "transmission_hw_interface:=",
            transmission_hw_interface,
            " ",
            "initial_positions_file:=",
            initial_positions_file,
            " ",
            "ft_sensor_use_fake_mode:=",
            ft_sensor_use_fake_mode,
            " ",
            "ft_sensor_max_retries:=",
            ft_sensor_max_retries,
            " ",
            "ft_sensor_read_rate:=",
            ft_sensor_read_rate,
            " ",
            "ft_sensor_ftdi_id:=",
            ft_sensor_ftdi_id,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    # ============================================================
    # robot_state_publisher
    # ============================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    return LaunchDescription(
        declared_arguments + [robot_state_publisher]
    )
