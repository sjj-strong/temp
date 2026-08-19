"""Unified ros2_control and MoveIt bringup for UR10e + 2F-85 + FT300."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_name = "ur10e_robotiq_ft_moveit_config"
    package_share = get_package_share_directory(package_name)
    ur_driver_share = get_package_share_directory("ur_robot_driver")
    ur_client_share = get_package_share_directory("ur_client_library")

    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = LaunchConfiguration("ur_type")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    description_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "ur10e_robotiq_ft.urdf.xacro"]
    )
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ", description_file,
            " name:=ur10e_robotiq_ft",
            " ur_type:=", ur_type,
            " tf_prefix:=", LaunchConfiguration("tf_prefix"),
            " robot_ip:=", robot_ip,
            " use_mock_hardware:=", use_mock_hardware,
            " ft_use_fake_mode:=", use_mock_hardware,
            " gripper_com_port:=", LaunchConfiguration("gripper_com_port"),
            " ftdi_id:=", LaunchConfiguration("ftdi_id"),
            " ft_max_retries:=", LaunchConfiguration("ft_max_retries"),
            " ft_read_rate:=", LaunchConfiguration("ft_read_rate"),
            " script_filename:=", PathJoinSubstitution([ur_client_share, "resources", "external_control.urscript"]),
            " input_recipe_filename:=", PathJoinSubstitution([ur_driver_share, "resources", "rtde_input_recipe.txt"]),
            " output_recipe_filename:=", PathJoinSubstitution([ur_driver_share, "resources", "rtde_output_recipe.txt"]),
            " reverse_ip:=", LaunchConfiguration("reverse_ip"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # The MoveIt config provides SRDF, kinematics and planning settings.  The
    # live description below overrides its static setup-assistant description.
    moveit_config = MoveItConfigsBuilder(
        "ur10e_robotiq_ft", package_name=package_name
    ).robot_description(mappings={"name": "ur10e_robotiq_ft", "ur_type": "ur10e"}).to_moveit_configs()
    moveit_parameters = [moveit_config.to_dict(), robot_description]

    ur_controllers = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "config", "ur_controllers.yaml"]
    )
    integrated_controllers = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "ur10e_robotiq_ft_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ParameterFile(ur_controllers, allow_substs=True), integrated_controllers],
        output="screen",
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # These UR nodes are needed only by the physical UR hardware interface.
    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip}],
        condition=UnlessCondition(use_mock_hardware),
        output="screen",
    )
    dashboard_client = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        parameters=[{"robot_ip": robot_ip}],
        condition=UnlessCondition(use_mock_hardware),
        output="screen",
    )
    robot_state_helper = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        parameters=[{"robot_ip": robot_ip, "headless_mode": False}],
        condition=UnlessCondition(use_mock_hardware),
        output="screen",
    )
    trajectory_until = Node(
        package="ur_robot_driver",
        executable="trajectory_until_node",
        name="trajectory_until_node",
        parameters=[{"motion_controller": "scaled_joint_trajectory_controller"}],
        output="screen",
    )

    controller_names = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "ur_configuration_controller",
        "gravity_update_controller",
        "friction_model_controller",
        "scaled_joint_trajectory_controller",
        "robotiq_activation_controller",
        "robotiq_gripper_controller",
        "robotiq_force_torque_sensor_broadcaster",
    ]
    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
            output="screen",
        )
        for name in controller_names
    ]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=moveit_parameters,
        output="screen",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([FindPackageShare(package_name), "config", "moveit.rviz"])],
        parameters=moveit_parameters,
        condition=IfCondition(launch_rviz),
        output="screen",
    )

    arguments = [
        DeclareLaunchArgument("robot_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("ur_type", default_value="ur10e", choices=["ur10e"]),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("gripper_com_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("ftdi_id", default_value="_"),
        DeclareLaunchArgument("ft_max_retries", default_value="100"),
        DeclareLaunchArgument("ft_read_rate", default_value="10"),
        DeclareLaunchArgument("reverse_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
    ]
    return LaunchDescription(
        arguments
        + [state_publisher, control_node, dashboard_client, robot_state_helper, urscript_interface, trajectory_until]
        + controller_spawners
        + [move_group, rviz]
    )
