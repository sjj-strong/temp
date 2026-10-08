# 相机检查与调参

独立桌面工具使用 PySide6 和 PyQtGraph，界面文字为英文。选择 USB 或 RealSense，预览图像并调整原生控制项；不启动机器人控制。

## 安装与运行

```bash
python3 -m pip install -r /ros2_ws/src/ur_teleop/requirements-camera.txt
sudo apt install v4l-utils libxcb-cursor0
python3 /ros2_ws/src/ur_teleop/ur_teleop/camera_inspector.py
# 仅列出设备，不打开窗口：
python3 /ros2_ws/src/ur_teleop/ur_teleop/camera_inspector.py --list
```

构建本包并 source 工作区后，也可运行 `ros2 run ur_teleop camera_inspector`。桌面需显示连接和设备访问权限；RealSense 预览需要可用的 pyrealsense2 SDK。存在源码或 USB 描述不代表当前容器能访问采集节点。

## 操作

1. 选择相机，核对型号、设备节点、物理端口和序列号。
2. 设置 Width、Height、FPS 和 USB Format，点击 Start / Restart；Supported Formats 显示设备支持的采集模式。
3. 选择 Camera Control，修改值并按 Enter或选菜单项，检查设备回读。修改手动曝光/白平衡前关闭相应自动项。
4. 对 USB 和 RealSense 都可用 Name 自定义相机名。点击 YAML Parameters，将生成项复制到 `config/camera.yaml` 顶层，相机名应唯一。
5. 关闭预览或点击 Stop 释放设备，再启动正式相机发布。

该工具的实时控制项默认仅作用于当前会话；USB 正式发布可读取 auto_exposure 和 exposure_time_absolute，其他控制项不要假定会随 YAML 自动恢复。

## 配置对应

顶层键即相机名，无 cameras 包装层；type 为 usb/realsense，enabled 控制发布，visualize 控制正式拼图预览。USB device 优先用稳定的 /dev/v4l/by-path，RealSense serial_no 必须为字符串。默认话题为 `/camera/<名称>/color/image_raw`，可在配置中用 topic 覆盖。

导出的 width/height/fps 来自活动采集模式；未启动时使用界面输入。resize 默认 false，resize_width/resize_height 默认 320×240，修改它们只改变录制保存尺寸，不能改变此工具预览或驱动采集尺寸。完整启动参数见[相机发布](launch.md#相机)，录制选择见[数据采集](data_recorder.md)。

USB 的 metadata 节点不是独立相机，工具按节点 index 过滤；RealSense 按序列号分组，并优先采用 SDK 序列号。两台 USB 可以有相同序列号，用物理端口区分；缺少稳定路径时 /dev/videoN 可能随插拔改变。

## 正式发布无法打开相机

调参工具可预览而发布节点提示 `Cannot open OpenCV camera` 时，先列出当前设备：

```bash
python3 /ros2_ws/src/ur_teleop/ur_teleop/camera_inspector.py --list
```

将每台 USB 的 `path` 填入 `camera.yaml` 对应的 `device`，RealSense 的 SDK `serial` 填入加引号的 `serial_no`。USB 更换物理端口后 by-path 会改变；相同 USB 序列号不能区分两台相机。不存在的相机设 `enabled: false`。名称不代表检测到的物理方向，需预览后确认 front/left 对应关系。

关闭调参工具以释放设备，再重新启动[相机发布](launch.md#相机)。`camera ready` 仅表示 ROS 节点初始化，不代表设备打开成功；用图像话题确认实际出图：

```bash
ros2 topic hz /camera/usb_front/color/image_raw
ros2 topic hz /camera/usb_left/color/image_raw
ros2 topic hz /camera/d455/color/image_raw
```

话题名称随配置相机名改变。路径正确仍无法打开时，检查容器设备映射、访问权限和是否被其他进程占用。Ctrl-C 后出现 `rcl_shutdown already called` 是 `data_collection` 的退出处理报错，与相机打开失败无关。
