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
        ],
    },
)
