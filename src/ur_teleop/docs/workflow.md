# 从硬件准备到数据采集

顺序为：准备硬件与配置 → 相机调试 → 启动 Home → 用 teleop 验证 → 用 record 采集。Xbot 和 Alicia 二选一，不同时启动。

以下机器人运动示例使用 **UR mock**，不连接 UR 真机；Xbot 手柄和 Alicia 主臂仍是物理设备。真机测试仅允许 `wrist_3_joint`，完整 Home/遥操作示例不用于真机测试。硬件接入与依赖见[硬件准备](hardware.md)，控制器选择见[控制器说明](controllers.md)。

## 1. 准备硬件和本包

按[硬件准备](hardware.md)确认依赖、UR 网络、输入设备及可选外设。构建本包：

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --packages-select ur_teleop --symlink-install
```

每个 ROS 终端均加载以下环境；普通调试无需额外设置 `ROS_DOMAIN_ID`，沿用当前环境即可：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

Xbot 首次使用先标定并运行[独立手柄测试](xbot_joy_test.md)。Alicia 核对串口、`home.master` 与关节映射。记录实际 UR 当前位置作为 `home.slave` 的只读工具见[读取初始位置](capture_slave_home.md)；它读取真机时需在实际硬件所在 ROS 域中单独运行。

## 2. 相机调试与发布

先装好[相机调试依赖](camera_inspector.md#run)，打开独立预览：

```bash
python3 /ros2_ws/src/ur_teleop/ur_teleop/camera_inspector.py
```

逐台确认型号、稳定设备路径、分辨率、帧率和曝光效果，将界面生成的字段合并到 `config/camera.yaml`（顶层键为自定义相机名，每台相机独立填写 `type`、设备参数、`enabled` 和 `visualize`）。只有两台 USB 时禁用不存在的第三台。关闭调参预览，释放设备。

新开相机终端，加载上述 ROS 环境后执行并保持运行：

```bash
ros2 launch ur_teleop camera.launch.py \
  config_file:=/ros2_ws/src/ur_teleop/config/camera.yaml
```

在所选遥操作配置的 `recorder.cameras` 中启用需要保存的图像，名称与 `camera.yaml` 的顶层相机名一致即可。采集尺寸、话题和保存缩放统一在 `camera.yaml` 设置；开启 `resize` 后用 `resize_width/resize_height` 指定保存尺寸。USB 可通过各相机的 `auto_exposure` 和 `exposure_time_absolute` 设置曝光，其他调参值需在采集前重新确认。细节见[相机调试](camera_inspector.md)、[相机发布](launch.md#相机)与[数据字段](data_recorder.md)。

## 3. 生成本次 mock 配置

以下命令生成两份临时配置，不改变源码中的真机设置，并将 mock 数据写入独立目录。运行一次即可，后面仅选择其中一份：

```bash
/usr/bin/python3 - <<'PY'
from pathlib import Path
import yaml

base = Path('/ros2_ws/src/ur_teleop/config')
common = dict(sim=True, debug=False, cell=dict(ft300_enabled=False),
              gripper=dict(enabled=False))
alicia = dict(common, base_config=str(base / 'alicia_teleop.yaml'),
              teleop=dict(controller='joint_impedance'),
              recorder=dict(root='/ros2_ws/dataset/mock/alicia',
                            repo_id='my_user/ur10e_alicia_mock'))
xbot = dict(common, base_config=str(base / 'xbot_teleop.yaml'),
            xbot=dict(controller_config_file=
                      '/ros2_ws/src/cartesian_impedance_controller/config/ur10e_xbot_sim_cartesian_impedance.yaml'),
            recorder=dict(root='/ros2_ws/dataset/mock/xbot',
                          repo_id='my_user/ur10e_xbot_mock', record_wrench=False))
for name, config in [('alicia', alicia), ('xbot', xbot)]:
    Path(f'/tmp/ur_teleop_{name}_mock.yaml').write_text(
        yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
PY
```

配置合并规则见[配置加载](alicia_teleop_config.md)。需要采集图像时，先在对应源码配置中设置 `recorder.cameras`；临时配置会继承它。

## 4. 先运行 Home 和 teleop 测试

### Xbot

硬件终端运行 Home，到 `HOME REACHED` 后保持终端运行：

```bash
ros2 launch ur_teleop home.launch.py \
  config_file:=/tmp/ur_teleop_xbot_mock.yaml
```

遥操作终端加载同一环境，显式选择 teleop 模式：

```bash
ros2 launch ur_teleop teleop.launch.py \
  config_file:=/tmp/ur_teleop_xbot_mock.yaml mode:=teleop
```

反馈就绪后自动切换到笛卡尔阻抗控制器。先松开 RB，再按住 RB 操作，检查方向和松开后的目标保持；按键细节见[Xbot 操作](xbot_control.md)。

### Alicia

硬件终端启动 Home。UR 为 mock，Alicia 会接收真实 Home 指令：

```bash
ros2 launch ur_teleop home.launch.py \
  config_file:=/tmp/ur_teleop_alicia_mock.yaml sim:=true \
  controller:=joint_impedance enable_gripper:=false enable_ft300:=false
```

看到 `HOME REACHED` 后，遥操作终端执行：

```bash
ros2 launch ur_teleop teleop.launch.py \
  config_file:=/tmp/ur_teleop_alicia_mock.yaml mode:=teleop
```

等静止、offset 捕获完成并提示 Enter 后按回车，检查 mock UR 对主臂的跟随。Home 与遥操作必须使用同一配置，Alicia Home 的 `controller` 参数也必须与 YAML 一致；更多参数见[启动说明](launch.md)。

## 5. 停止 teleop，再启动 record

先 Ctrl-C 停止第二终端的 teleop，保留硬件和相机终端。控制器切换和 Home 校验仍需满足；若当前位置已离开 Home，先停止硬件终端并重新运行对应 mock Home，不能绕过校验。

采集终端加载 ROS 环境及 LeRobot 环境，然后根据输入源选择一条命令：

```bash
source /opt/lerobot_venv/bin/activate
# Xbot：
ros2 launch ur_teleop teleop.launch.py \
  config_file:=/tmp/ur_teleop_xbot_mock.yaml mode:=record
```

```bash
# Alicia：不要与上面的 Xbot 命令同时运行。
ros2 launch ur_teleop teleop.launch.py \
  config_file:=/tmp/ur_teleop_alicia_mock.yaml mode:=record
```

| 操作 | Xbot | Alicia |
| --- | --- | --- |
| 开始 episode | Menu | Enter |
| 保存 | Y | S |
| 丢弃 | B | D |
| 保存并结束采集 | View 长按 | Q |

默认 `debug: false`，正常输出包含操作、配置控制频率、tqdm 帧进度和实际采集 Hz。采集速度、数据字段、文件位置、相机选择与调试日志见[数据采集](data_recorder.md)。模拟数据仅用于流程验证，不当作真机任务数据。

## 6. 结束

先保存并结束采集，确认数据集完成 finalize，再停止相机和硬件终端。不并行运行 teleop 和 record 两套遥操作节点。
