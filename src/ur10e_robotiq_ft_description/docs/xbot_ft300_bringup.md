# Xbot 组合启动的控制器参数

`real_bringup.launch.py` 接受 `controllers_file` 参数，并将其转发给 UR 官方的 `ur_control.launch.py`。默认值仍是 UR 官方的 `ur_controllers.yaml`。

Xbot 组合启动传入 `ur_teleop/config/xbot_ur_controllers.yaml`，使同一 `/controller_manager` 注册 FT300 力传感器 broadcaster。启用 FT300 时，由组合 URDF 的 ros2_control 传感器接口占用设备串口，只需加载 broadcaster；不要再启动独立 FT300 驱动。

可先检查启动参数而不连接真机：

```bash
ros2 launch ur10e_robotiq_ft_description real_bringup.launch.py --show-args
```
