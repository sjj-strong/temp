from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur10e_robotiq_ft", package_name="ur10e_robotiq_ft_moveit_config").robot_description(mappings={"name": "ur10e_robotiq_ft", "ur_type": "ur10e"}).to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
