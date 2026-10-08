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
4. 点击 YAML Parameters，将生成的配置复制到 `camera.yaml`；USB 与 RealSense 均可通过 Name 自定义唯一相机名称。
5. Click Stop or close the window before starting dataset collection.

## Configuration fields

- `config/camera.yaml` 的每个顶层键为自定义相机名。调参工具为 USB 和 RealSense 都使用 Name 字段生成对应的配置项，直接复制到该文件。
- USB 使用 `type: usb`，`device` 优先填写 `/dev/v4l/by-path`；RealSense 使用 `type: realsense` 和加引号的 `serial_no`。各设备独立配置，不固定 D435i 或 D455 的组合。
- `enabled` 控制是否发布，`visualize` 控制是否加入预览；默认图像话题为 `/camera/<名称>/color/image_raw`，无需额外配置全局话题列表。
- 导出的 `resize` 默认关闭，`resize_width/resize_height` 默认 320×240；需要改变保存尺寸时在相机配置中开启缩放。宽高和帧率填写实际采集参数；USB 的 FourCC 放在 `fourcc`。USB 曝光可通过 `auto_exposure` 和 `exposure_time_absolute` 配置，调参界面的其他控件设置仅在当前会话生效。

One USB camera may have multiple video nodes; metadata nodes are excluded from the camera selector. RealSense nodes are grouped by serial number. Two USB cameras can share the same serial number; use their physical USB paths to distinguish them. If stable path links are missing, the displayed `/dev/videoN` fallback may change after reconnecting.

## Verification

Logic tests cover native control parsing and configuration mapping. Qt tests use simulated frames and controls to verify RGB rendering, immediate control application, device switching and stream cleanup without connecting to cameras or robots. 本次逐相机配置修改的 27 项逻辑、界面及启动测试通过，测试使用模拟设备，不访问真实相机或机器人。 Live 640x480 previews were verified on both USB cameras and the RealSense D455. Brightness changes were accepted and read back on all three devices, then restored to their original values. The RealSense SDK serial is 311322303190; when USB descriptors differ, the SDK identifier takes priority for stream configuration.

References: [PyQtGraph ImageItem](https://pyqtgraph.readthedocs.io/en/latest/api_reference/graphicsItems/imageitem.html), [Qt QTimer](https://doc.qt.io/qtforpython-6/PySide6/QtCore/QTimer.html).
