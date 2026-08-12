"""UR cell: sim (mock + rviz) or real (hardware + gripper + FT300).

Persistent — started by home.launch and shared with teleop.launch (spec §4.1).
Launch args win over ur_teleop.yaml defaults.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def _yaml_default(config_file: str, *path: str, fallback: str):
    try:
        with open(config_file) as f:
            data = yaml.safe_load(f)
        for p in path:
            data = data[p]
        # yaml 布尔（true/false）经 str() 成 "True"/"False"（大写），而本文件所有
        # 条件判断都用小写 "true"/"false" —— 这里把布尔归一为小写，避免 sim 等
        # 开关失配（曾导致 rviz 不启动、tcp_pose 在 mock 下刷 NaN、sim 下误拉 robotiq）。
        return str(data).lower() if isinstance(data, bool) else str(data)
    except Exception:
        return fallback


def _description_launchfile():
    """rviz 模型：自包含 mock rsp，硬编码 use_mock_hardware=true。

    ur_control.launch.py 只转发 robot_ip + ur_type 给 description launchfile，
    不转发 use_mock_hardware。直接指向 ur10e_robotiq_ft_description/rsp.launch.py
    会让 xacro 回退到 rsp 自己的默认值 (use_mock_hardware=false)，生成真机硬件而非 mock。
    因此用自包含的 rsp_mock.launch.py 硬编码 mock 参数来绕过这个问题。
    """
    pkg_share = get_package_share_directory("ur_teleop")
    mock_rsp = os.path.join(pkg_share, "launch", "rsp_mock.launch.py")
    if os.path.exists(mock_rsp):
        return mock_rsp
    # 回退：同时支持旧的 wrapper 名称
    wrapper = os.path.join(pkg_share, "launch", "rsp_wrapper.launch.py")
    if os.path.exists(wrapper):
        return wrapper
    try:
        return os.path.join(get_package_share_directory("ur10e_robotiq_ft_description"),
                            "launch", "rsp.launch.py")
    except Exception:
        return os.path.join(get_package_share_directory("ur_robot_driver"),
                            "launch", "ur_rsp.launch.py")


def generate_launch_description():
    pkg_share = get_package_share_directory("ur_teleop")
    config_file = os.path.join(pkg_share, "config", "ur_teleop.yaml")

    sim_default = _yaml_default(config_file, "sim", fallback="true")
    ip_default = _yaml_default(config_file, "cell", "robot_ip", fallback="0.0.0.0")
    grip_default = _yaml_default(config_file, "cell", "gripper_port", fallback="/dev/ttyUSB1")
    ftdi_default = _yaml_default(config_file, "cell", "ftdi_id", fallback="")
    rviz_default = _yaml_default(config_file, "cell", "launch_rviz", fallback="true")
    alicia_default = _yaml_default(config_file, "cell", "alicia_port", fallback="")
    launch_alicia_default = _yaml_default(config_file, "cell", "launch_alicia", fallback="true")
    ur_type_default = _yaml_default(config_file, "cell", "ur_type", fallback="ur10e")

    sim = LaunchConfiguration("sim")
    is_sim = PythonExpression(["'", sim, "' == 'true'"])          # "false" 字符串真值陷阱防护

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config_file),
        DeclareLaunchArgument("sim", default_value=sim_default),
        DeclareLaunchArgument("robot_ip", default_value=ip_default),
        DeclareLaunchArgument("gripper_port", default_value=grip_default),
        DeclareLaunchArgument("ftdi_id", default_value=ftdi_default),
        DeclareLaunchArgument("launch_rviz", default_value=rviz_default),
        DeclareLaunchArgument("alicia_port", default_value=alicia_default),
        DeclareLaunchArgument("launch_alicia", default_value=launch_alicia_default),
        DeclareLaunchArgument("ur_type", default_value=ur_type_default),
        DeclareLaunchArgument("description_sim",
                              default_value=_description_launchfile()),
        # rviz2 必须放在 ur_control include 之前：后者传 launch_rviz:="false" 会把
        # 全局 LaunchConfiguration("launch_rviz") 改写成 "false"（launch 的
        # LaunchConfiguration 在 include 树里按名字共享、不按 include 隔离），若 rviz
        # 在 include 之后求值，条件会读到 "false" → 不启动。
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", os.path.join(pkg_share, "config", "rviz", "ur_teleop.rviz")],
            condition=IfCondition(PythonExpression(
                ["'", sim, "' == 'true' and '", LaunchConfiguration("launch_rviz"), "' == 'true'"])),
        ),
        # ur_control 分两路：sim 用组合模型（rsp_mock），real 用官方默认
        # （bare UR）。ur_control.launch.py 只向 description_launchfile 转发
        # robot_ip + ur_type，不转发 recipe 文件路径等参数。官方 ur_rsp.launch.py
        # 将 recipe 路径硬编码为本地变量，不依赖继承，故 real 下不传
        # description_launchfile 让 ur_control 走默认逻辑即可。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ur_robot_driver"),
                             "launch", "ur_control.launch.py")
            ),
            condition=IfCondition(is_sim),
            launch_arguments={
                "ur_type": LaunchConfiguration("ur_type"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "use_mock_hardware": sim,
                "mock_sensor_commands": "false",
                "launch_dashboard_client": "false",
                "launch_rviz": "false",
                "description_launchfile": LaunchConfiguration("description_sim"),
                "initial_joint_controller": "scaled_joint_trajectory_controller",
                "activate_joint_controller": "true",
                "controllers_file": os.path.join(pkg_share, "config", "ur_controllers_sim.yaml"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ur_robot_driver"),
                             "launch", "ur_control.launch.py")
            ),
            condition=UnlessCondition(is_sim),
            launch_arguments={
                "ur_type": LaunchConfiguration("ur_type"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "use_mock_hardware": sim,
                "mock_sensor_commands": "false",
                "launch_dashboard_client": "false",
                "launch_rviz": "false",
                "initial_joint_controller": "scaled_joint_trajectory_controller",
                "activate_joint_controller": "true",
                # 显式传 description_launchfile 防止 sim 分支通过
                # LaunchConfiguration 全局改写漏到 real 分支（参见
                # launch-arg-global-clobber memory）。
                "description_launchfile": os.path.join(
                    get_package_share_directory("ur_robot_driver"),
                    "launch", "ur_rsp.launch.py"),
            }.items(),
        ),
        # Sim 模式：将 parallel_gripper_action_controller spawn 到 UR 的
        # controller_manager，驱动 mock_components/GenericSystem 暴露的
        # robotiq_85_left_knuckle_joint 关节。
        # -p 把 yaml 传给 controller_manager 使其能读到 type 与关节参数；
        # controller_manager 将 type 以下部分转发给控制器节点自身。
        Node(
            package="controller_manager",
            executable="spawner",
            condition=IfCondition(is_sim),
            arguments=[
                "robotiq_gripper_controller",
                "-c", "/controller_manager",
                "-p", os.path.join(pkg_share, "config", "gripper_sim_controller.yaml"),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("robotiq_description"),
                             "launch", "robotiq_control.launch.py")
            ),
            condition=UnlessCondition(is_sim),
            launch_arguments={
                "com_port": LaunchConfiguration("gripper_port"),
                "launch_rviz": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("robotiq_ft_sensor_hardware"),
                             "launch", "ft_sensor_standalone.launch.py")
            ),
            condition=UnlessCondition(is_sim),
            launch_arguments={
                "ftdi_id": LaunchConfiguration("ftdi_id"),
                "frame_id": "robotiq_ft_frame_id",
            }.items(),
        ),
        # 主臂 Alicia（始终真实；端口可配，默认自动检测）。
        # 由 launch_alicia 开关控制；纯 sim 无实体 Alicia 时设 launch_alicia:=false。
        # 注意：alicia_d_driver 的 CMakeLists 用 install(DIRECTORY launch/)（带尾斜杠）
        # 把 launch 文件装到了 share 根目录，而非约定的 share/<pkg>/launch/，故此处不加 "launch" 子目录。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("alicia_d_driver"),
                             "alicia_d_driver.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("launch_alicia")),
            launch_arguments={
                "port": LaunchConfiguration("alicia_port"),
            }.items(),
        ),
    ])
