from setuptools import find_packages, setup

package_name = "ur_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/ur_teleop.yaml",
            "config/ur_controllers_sim.yaml",
            "config/ur_controllers_impedance_sim.yaml",
            "config/opencv_cameras.yaml",
            "config/xbot_teleop.yaml",
            "config/xbot_mock_controllers.yaml",
            "config/xbot_cartesian_sim.yaml",
            "config/xbot_cartesian_type.yaml",
        ]),
        ("share/" + package_name + "/config/rviz", [
            "config/rviz/ur_teleop.rviz",
        ]),
        ("share/" + package_name + "/launch", [
            "launch/cell.launch.py",
            "launch/home.launch.py",
            "launch/teleop.launch.py",
            "launch/camera.launch.py",
            "launch/rsp_mock.launch.py",
            "launch/ft300_gripper_test.launch.py",
            "launch/xbot_cell.launch.py",
        ]),
        ("share/ament_index/resource_index/packages", ["resource/ur_teleop"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Alicia-D-ROS2 to UR10e+Robotiq teleoperation with LeRobot data collection",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "home_node = ur_teleop.home_node:main",
            "teleop_node = ur_teleop.teleop_node:main",
            "data_recorder = ur_teleop.data_recorder:main",
            "ruckig_node = ur_teleop.ruckig_node:main",
            "camera_mosaic_viewer = ur_teleop.camera_mosaic_viewer:main",
        ],
    },
)
