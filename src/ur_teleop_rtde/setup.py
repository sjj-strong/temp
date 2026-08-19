from setuptools import find_packages, setup

package_name = "ur_teleop_rtde"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/ur_teleop_rtde.yaml",
        ]),
        ("share/" + package_name + "/launch", [
            "launch/cell.launch.py",
            "launch/home.launch.py",
            "launch/teleop.launch.py",
        ]),
        ("share/ament_index/resource_index/packages", ["resource/ur_teleop_rtde"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Alicia-D to UR10e teleoperation via RTDE servoJ, with LeRobot data recording",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "home_node = ur_teleop_rtde.home_node:main",
            "teleop_node = ur_teleop_rtde.teleop_node:main",
            "data_recorder = ur_teleop_rtde.data_recorder:main",
        ],
    },
)
