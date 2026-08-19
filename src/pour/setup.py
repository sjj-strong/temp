from setuptools import find_packages, setup

package_name = "pour"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/pour_params.yaml",
        ]),
        ("share/" + package_name + "/launch", [
            "launch/pour.launch.py",
        ]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jaron-G",
    maintainer_email="2205250117@qq.com",
    description="电子秤 PD 倾倒控制:串口电子秤节点 + 核心 PD 控制节点",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scale_serial_node = pour.scale_serial_node:main",
            "pour_control_node = pour.pour_control_node:main",
        ],
    },
)
