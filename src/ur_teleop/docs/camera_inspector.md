# Camera setup before dataset collection

A minimal desktop interface using PySide6 and PyQtGraph. Select a camera, preview its image and adjust its native controls. All interface text is in English.

## Run

Install dependencies in the Python environment used to run the tool:

```bash
python3 -m pip install -r /ros2_ws/src/ur_teleop/requirements-camera.txt
sudo apt install v4l-utils libxcb-cursor0
python3 /ros2_ws/src/ur_teleop/ur_teleop/camera_inspector.py
```

The tool uses PySide6 widgets and a PyQtGraph ImageItem. Capture runs in a background thread; a Qt timer updates the preview. Display levels remain fixed at 0-255 so camera exposure changes remain visible.

To print device information without opening a window:

```bash
python3 /ros2_ws/src/ur_teleop/ur_teleop/camera_inspector.py --list
```

After rebuilding the package and sourcing the workspace, `ros2 run ur_teleop camera_inspector` is also available. The desktop requires a working display connection and access to the camera devices.

## Use

1. Select a camera. Its model, device path, physical port and serial number appear above the preview.
2. Set Width, Height, FPS and USB Format. Click Start / Restart. Supported Formats shows the device's available profiles.
3. Select a Camera Control. Change its numeric value and press Enter, or select a menu entry. The setting applies immediately and the status displays device readback. Disable automatic exposure or white balance before changing the corresponding manual values.
4. Click YAML Parameters and copy the fields into the matching configuration. Use a unique Name for each USB camera.
5. Click Stop or close the window before starting dataset collection.

## Configuration fields

- USB 配置统一放在 `config/camera.yaml` 的顶层 `opencv_cameras` 列表。`device` 优先使用 `/dev/v4l/by-path`；宽高、帧率和 FourCC 采用实际采集参数。每台相机的 name、topic 和 frame_id 必须唯一，只有两台 USB 相机时禁用或删除第三项。
- RealSense 配置放在同一文件的 `cameras.realsense` 中，填写检测到的序列号、启用开关和 color_profile；`cameras.visualization.topics` 填写实际图像话题。
- 调参工具中的曝光、白平衡等原生控件设置只在当前采集会话生效；复制配置时仅写入驱动支持的字段。

The existing RealSense launch always starts D455 and optionally starts D435i. A single D455 can use enable_d435i=false. A single D435i requires a separate launch change to disable the absent D455. Other RealSense models cannot map directly to the current launch configuration.

One USB camera may have multiple video nodes; metadata nodes are excluded from the camera selector. RealSense nodes are grouped by serial number. Two USB cameras can share the same serial number; use their physical USB paths to distinguish them. If stable path links are missing, the displayed `/dev/videoN` fallback may change after reconnecting.

## Verification

Logic tests cover native control parsing and configuration mapping. Qt tests use simulated frames and controls to verify RGB rendering, immediate control application, device switching and stream cleanup without connecting to cameras or robots. All 14 logic, Qt interface and launch regression tests passed. Live 640x480 previews were verified on both USB cameras and the RealSense D455. Brightness changes were accepted and read back on all three devices, then restored to their original values. The RealSense SDK serial is 311322303190; when USB descriptors differ, the SDK identifier takes priority for stream configuration.

References: [PyQtGraph ImageItem](https://pyqtgraph.readthedocs.io/en/latest/api_reference/graphicsItems/imageitem.html), [Qt QTimer](https://doc.qt.io/qtforpython-6/PySide6/QtCore/QTimer.html).
