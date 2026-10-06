"""仅展开启动动作，验证真实设备开关，不连接机器人。"""

import runpy
from pathlib import Path

import pytest


@pytest.mark.parametrize("use_gripper,use_ft300", [(False, False), (True, True), (False, True)])
def test_real_peripheral_switches(use_gripper, use_ft300):
    pytest.importorskip("launch")
    from launch import LaunchContext

    launch_file = Path(__file__).resolve().parents[1] / "launch/standalone_real.launch.py"
    build = runpy.run_path(str(launch_file))["_start_real_cell"]
    context = LaunchContext()
    context.launch_configurations.update({
        "robot_ip": "169.254.138.15",
        "use_gripper": str(use_gripper).lower(),
        "use_ft300": str(use_ft300).lower(),
        "gripper_com_port": "/dev/ttyUSB0",
        "ft_sensor_ftdi_id": "ttyUSB2",
        "launch_rviz": "false",
    })

    actions = build(context)
    assert len(actions) == 1
    arguments = dict(actions[0].launch_arguments)
    assert arguments["launch_gripper"] == str(use_gripper).lower()
    assert arguments["ft_sensor_use_fake_mode"] == str(not use_ft300).lower()
    assert arguments["use_cartesian_impedance"] == "true"
    assert arguments["robot_ip"] == "169.254.138.15"
