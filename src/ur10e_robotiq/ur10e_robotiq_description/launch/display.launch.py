from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_file = PathJoinSubstitution(
        [
            FindPackageShare("ur10e_robotiq_description"),
            "urdf",
            "ur10e_robotiq.urdf.xacro",
        ]
    )
    display_rviz = PathJoinSubstitution(
        [FindPackageShare("ur10e_robotiq_description"), "rviz", "display.rviz"]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            description_file,
            " include_ros2_control:=false",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", display_rviz],
                output="screen",
            ),
        ]
    )
