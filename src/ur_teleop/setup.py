from setuptools import find_packages, setup

package_name = "ur_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/joint_mapping.yaml",
            "config/calibration_offset.yaml",
            "config/robotiq_gripper.yaml",
            "config/teleop_params.yaml",
            "config/recorder_params.yaml",
            "config/ur_teleop.yaml",
        ]),
        ("share/" + package_name + "/launch", [
            "launch/teleop.launch.py",
            "launch/teleop_only.launch.py",
            "launch/record.launch.py",
            "launch/calibrate.launch.py",
            "launch/view.launch.py",
            "launch/alicia_display.launch.py",
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
            "teleop_node = ur_teleop.teleop_node:main",
            "data_recorder = ur_teleop.data_recorder:main",
            "fake_alicia = ur_teleop.fake_alicia:main",
            "calibrate = ur_teleop.calibration:main",
        ],
    },
)
