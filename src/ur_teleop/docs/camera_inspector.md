# 采集前相机检查与调参

此工具独立访问相机，不启动 ROS、遥操作或机械臂。检查结束后关闭预览，释放设备再启动数据采集。

## 启动

直接运行源码，无需 colcon 构建：

```bash
python3 /ros2_ws/src/ur_teleop/ur_teleop/camera_inspector.py
# 只查看型号、视频节点、稳定端口、序列号：
python3 /ros2_ws/src/ur_teleop/ur_teleop/camera_inspector.py --list
```

构建并加载工作区后也可使用 `ros2 run ur_teleop camera_inspector`。

依赖：当前 Python 环境需有 `opencv-python`（或系统 `python3-opencv`）、`numpy`、`PyYAML`、`tkinter`。系统需安装 `v4l-utils`，用于 USB 原生参数和格式查询。RealSense 可从系统 USB 信息读取完整型号和序列号，SDK 预览需要同一 Python 环境中的 `pyrealsense2`。GUI 使用 Tk，可搭配无 HighGUI 的 OpenCV，不依赖 `cv2.imshow`。

```bash
sudo apt install python3-tk python3-opencv python3-yaml v4l-utils
# 在实际运行脚本的虚拟环境中安装 RealSense Python SDK：
python3 -m pip install pyrealsense2
```

## 操作

1. 刷新设备，查看型号、`/dev/videoN`、`path`（优先 `/dev/v4l/by-path`）、`by_id` 和物理端口。RealSense 选 SDK 条目，可查看序列号。
2. 点击“支持的格式”查看设备支持的分辨率和帧率；输入宽、高、帧率和 USB FourCC，例如 `MJPG`。点击“打开/重启预览”生效。状态栏显示设备返回的实际配置。
3. 右侧选择曝光、自动曝光、白平衡、亮度、对比度、增益等设备实际支持的参数。拖动滑块，点击“应用参数”实时生效；状态栏显示设备回读值。菜单参数必须选提示中存在的数值。手动曝光/白平衡前先关闭对应自动控制；控制名称和数值单位由驱动定义。
4. 设置配置名称（例如 `usb_front`、`usb_left`），点击“生成 YAML 参数”和“复制文本”，将片段合并到对应配置。可切换另一台相机重复检查；若需同时查看多台，可启动多个工具窗口，每个窗口选择不同设备。
5. 关闭所有预览再启动相机发布/采集，避免设备占用。调参可能在断电或采集程序启动时重置。

## 两个配置应填写什么

- `config/opencv_cameras.yaml`：使用 USB 条目的 `path` 填写 `device`；填写已验证的 `width`、`height`、`fps`、`fourcc`。保留每台相机唯一的 `name`、`topic`、`frame_id`。只有两台 USB 时，移除或禁用不存在的第三台条目。
- `config/camera.yaml`：填写 RealSense 的 `d435i_serial` 或 `d455_serial`、`enabled`、`enable_d435i`、`color_profile`。生成的是需要合并的片段，不是完整文件。保留 `opencv` 和 `visualization`，将 `visualization.topics` 改为实际需要的 USB 和 RealSense 话题，例如 `/camera/d455/color/image_raw`。
- 曝光、白平衡等设置不属于这两个配置目前支持的字段，工具不会生成无效 YAML 参数；设置仅用于本次设备调试，不能据此声称采集程序会保持同样设置。

现有 `dual_realsense.launch.py` 总会启动 D455，D435i 可通过 `enable_d435i` 禁用。因此单 D455 可直接使用；仅接 D435i 时还需要调整采集启动逻辑以禁用不存在的 D455。两台 USB 的序列号可能相同，因此应优先按物理端口绑定。工具会识别其他 RealSense 型号，但拒绝生成无法映射到当前启动配置的序列号字段。

## 设备与容器

一台物理相机可能对应多个 `/dev/videoN`，其中部分是元数据或深度节点，不是额外的 USB 相机。RealSense 视频节点按序列号合并为一个条目，彩色预览使用 SDK。无法打开的节点会显示错误，不会自动替换配置。

如果只有 `/dev/videoN` 而没有稳定路径，工具暂用视频节点；它可能随重插改变，应在宿主机确认 `/dev/v4l/by-path` 并映射到容器。容器还需能访问相机视频设备，RealSense 通常还需要 USB 设备访问。无设备时界面仍能打开并提示原因；有设备但预览失败时检查权限、占用、格式支持和 USB 带宽。图形界面需有效 `DISPLAY` 和显示服务授权。

## 验证范围

新增逻辑测试与相机启动配置回归测试共 9 项通过；界面创建、设备刷新和绘图循环冒烟检查通过。逻辑测试覆盖 V4L2 控制解析、菜单和只读过滤、USB 配置映射、RealSense 序列号映射与不支持型号。开发容器能通过系统信息识别相机，但未暴露视频节点且未安装 RealSense SDK，因此实机帧率、曝光效果与 USB/RealSense 驱动行为仍需连接设备验证。

原生接口参考：[Linux V4L2 控制](https://cdn.kernel.org/doc/html/latest/userspace-api/media/v4l/control.html)、[RealSense 传感器控制](https://github.com/realsenseai/librealsense/blob/master/examples/sensor-control/api_how_to.h)。
