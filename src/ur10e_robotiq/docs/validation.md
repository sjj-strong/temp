# UR10e + Robotiq 集成验证记录

## 2026-08-09 — Stage 0/1

- ROS 发行版：Jazzy
- 工作区：`/ros2_ws`
- 总体结果：**PARTIAL（带已知环境阻塞）**；Stage 0、FT300 与隔离 build-base 验证 **PASS**，默认 build-base 验证 **FAIL（环境构建基线冲突）**

### Stage 0：上游基线

执行命令：

```bash
git -C /ros2_ws/src/Universal_Robots_ROS2_Driver status --short
git -C /ros2_ws/src/Universal_Robots_ROS2_Description status --short
git -C /ros2_ws/src/ros2_robotiq_gripper status --short
git -C /ros2_ws/src/rq_fts_ros2_driver status --short
```

结果：

- `Universal_Robots_ROS2_Driver`：无输出，干净（PASS）
- `Universal_Robots_ROS2_Description`：无输出，干净（PASS）
- `ros2_robotiq_gripper`：保留任务开始前已有的修改与未跟踪文件（PASS）
- `rq_fts_ros2_driver`：任务开始前已有未跟踪的 `docs/`（PASS）

`ros2_robotiq_gripper` 的既有基线：

```text
 M .github/workflows/ci-coverage-build.yml
 M .github/workflows/ci-format.yml
 M .github/workflows/ci-ros-lint.yml
 M .github/workflows/prerelease-check.yml
 M .github/workflows/reusable-industrial-ci-with-cache.yml
 M .github/workflows/reusable-ros-tooling-source-build.yml
 M README.md
 M robotiq_controllers/CMakeLists.txt
 M robotiq_description/CMakeLists.txt
 M robotiq_description/config/robotiq_controllers.yaml
 M robotiq_description/launch/robotiq_control.launch.py
 M robotiq_driver/CMakeLists.txt
 M robotiq_driver/include/robotiq_driver/hardware_interface.hpp
 M robotiq_driver/src/hardware_interface.cpp
 M robotiq_hardware_tests/CMakeLists.txt
?? .github/workflows/jazzy-binary-build-main.yml
?? .github/workflows/jazzy-binary-build-testing.yml
?? .github/workflows/jazzy-semi-binary-build-main.yml
?? .github/workflows/jazzy-semi-binary-build-testing.yml
?? .github/workflows/jazzy-source-build.yml
?? docs/
?? robotiq_description/config/robotiq_update_rate.yaml
?? ros2_robotiq_gripper-not-released.jazzy.repos
?? ros2_robotiq_gripper.jazzy.repos
```

### Stage 1：FT300 依赖

执行命令：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to robotiq_ft_sensor_description robotiq_ft_sensor_hardware
source /ros2_ws/install/setup.bash
ros2 pkg prefix robotiq_ft_sensor_description
ros2 pkg prefix robotiq_ft_sensor_hardware
```

关键输出：

```text
Summary: 3 packages finished [3.69s]
/ros2_ws/install/robotiq_ft_sensor_description
/ros2_ws/install/robotiq_ft_sensor_hardware
```

结果：两个 FT package 均真实构建并可从工作区前缀发现（PASS）。构建仅产生上游 CMake 弃用/策略警告。

### Description 骨架

#### 默认 build-base：FAIL（环境构建基线冲突）

首次执行 brief 原命令：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to ur10e_robotiq_description
```

实际关键输出（保存于 `/ros2_ws/log/build_2026-08-09_18-38-01/`）：

```text
failed to create symbolic link '/ros2_ws/build/ur_dashboard_msgs/ament_cmake_python/ur_dashboard_msgs/ur_dashboard_msgs' because existing path cannot be removed: Is a directory
gmake[2]: *** [CMakeFiles/ament_cmake_python_symlink_ur_dashboard_msgs.dir/build.make:70: CMakeFiles/ament_cmake_python_symlink_ur_dashboard_msgs] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:577: CMakeFiles/ament_cmake_python_symlink_ur_dashboard_msgs.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
Failed   <<< ur_dashboard_msgs [2.86s, exited with code 2]
Aborted  <<< ur_msgs [4.08s]
Aborted  <<< ur_description [4.80s]
Aborted  <<< ur_client_library [18.8s]

Summary: 4 packages finished [18.9s]
  1 package failed: ur_dashboard_msgs
  3 packages aborted: ur_client_library ur_description ur_msgs
  2 packages had stderr output: ur_dashboard_msgs ur_description
  3 packages not processed
```

对应 `events.log` 原始状态：

```text
[2.864597] (ur_dashboard_msgs) JobEnded: {'identifier': 'ur_dashboard_msgs', 'rc': 2}
[4.084196] (ur_msgs) JobEnded: {'identifier': 'ur_msgs', 'rc': 'SIGINT'}
[4.813017] (ur_description) JobEnded: {'identifier': 'ur_description', 'rc': 'SIGINT'}
[18.796175] (ur_client_library) JobEnded: {'identifier': 'ur_client_library', 'rc': 'SIGINT'}
[18.796921] (ur10e_robotiq_description) JobSkipped: {'identifier': 'ur10e_robotiq_description'}
```

结果：**FAIL（环境构建基线冲突）**。工作区旧 `build/ur_dashboard_msgs` 中已有普通目录，和当前 symlink-install 目标冲突，命令退出码为 2，目标包未被执行。未删除旧构建产物，也未修改上游。

#### 隔离 build-base：PASS

使用全新 build/log 前缀并保持目标 install 前缀后重新验证：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/ur10e-task1-probe.NLGSHg/task-log build --symlink-install --build-base /tmp/ur10e-task1-probe.NLGSHg/task-build --install-base /ros2_ws/install --packages-up-to ur10e_robotiq_description
source /ros2_ws/install/setup.bash
ros2 pkg prefix ur10e_robotiq_description
```

关键输出：

```text
Summary: 11 packages finished [1min 25s]
Finished <<< ur10e_robotiq_description [0.88s]
/ros2_ws/install/ur10e_robotiq_description
```

结果：骨架在隔离 build-base 中真实构建成功，且唯一目标 package 前缀正确（PASS）。

### 剩余问题

- 默认 `/ros2_ws/build` 中的既有普通目录仍与 `--symlink-install` 冲突；本任务未删除或修复用户的既有构建产物。
- 因此，使用默认 build-base 的最终全工作区验证仍受阻并保持 **FAIL**；隔离 build-base 的 **PASS** 仅证明依赖与 `ur10e_robotiq_description` 骨架可从干净构建基线成功构建和安装。

## 2026-08-09 — Stage 2

- 目标：实现所有模式共用的 UR10e + FT300 + 2F-85 组合 Xacro，并验证 display/mock 两种 flattened URDF。
- 总体结果：**PASS**；TDD RED 原因符合预期，两份 URDF 均通过静态解析，display 含 0 个、mock 恰含 3 个 `<ros2_control>`。

### TDD RED：目标 Xacro 尚不存在

在创建文件前执行：

```bash
source /ros2_ws/install/setup.bash
xacro /ros2_ws/src/ur10e_robotiq/ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro include_ros2_control:=false > /tmp/ur10e_robotiq.urdf
```

命令退出码为 1。关键输出：

```text
FileNotFoundError: [Errno 2] No such file or directory: '/ros2_ws/src/ur10e_robotiq/ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro'
xacro.XacroException: No such file or directory: /ros2_ws/src/ur10e_robotiq/ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro
AttributeError: module 'xml' has no attribute 'parsers'
```

结果：**FAIL（预期 RED）**。第一失败原因精确为待实现文件不存在；末行是本环境 xacro 在处理前述异常时触发的次生异常，不改变 RED 判定。

### GREEN：生成并静态解析两种 URDF

执行 brief 指定命令：

```bash
source /ros2_ws/install/setup.bash
xacro /ros2_ws/src/ur10e_robotiq/ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro include_ros2_control:=false > /tmp/ur10e_robotiq_display.urdf
check_urdf /tmp/ur10e_robotiq_display.urdf
xacro /ros2_ws/src/ur10e_robotiq/ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro include_ros2_control:=true use_mock_hardware:=true use_fake_hardware:=true use_fake_mode:=true > /tmp/ur10e_robotiq_mock.urdf
check_urdf /tmp/ur10e_robotiq_mock.urdf
rg -n 'gripper_tcp|ft300_sensor|robotiq_85_base_link|shoulder_pan_joint' /tmp/ur10e_robotiq_display.urdf
rg -n '<ros2_control' /tmp/ur10e_robotiq_display.urdf /tmp/ur10e_robotiq_mock.urdf
```

整组命令退出码为 0。两次 `check_urdf` 均输出：

```text
robot name is: ur10e_robotiq
---------- Successfully Parsed XML ---------------
root Link: world has 1 child(ren)
```

解析树关键链路为：

```text
world -> base_link -> ... -> tool0 -> ft300_mounting_plate -> ft300_sensor
ft300_sensor -> robotiq_85_base_link -> gripper_tcp
```

display 关键名称检索输出：

```text
215:  <joint name="shoulder_pan_joint" type="revolute">
303:  <link name="ft300_sensor">
355:  <link name="robotiq_85_base_link">
606:  <link name="gripper_tcp"/>
607:  <joint name="robotiq_85_base_link-gripper_tcp" type="fixed">
```

`<ros2_control>` 原始检索只在 mock 文件命中：

```text
/tmp/ur10e_robotiq_mock.urdf:355:  <ros2_control name="robotiq_2f85" type="system">
/tmp/ur10e_robotiq_mock.urdf:659:  <ros2_control name="ur" rw_rate="0" type="system">
/tmp/ur10e_robotiq_mock.urdf:997:  <ros2_control name="robotiq_ft_sensor" type="sensor">
```

结果：两份 flattened URDF 均为有效树，机械臂、FT300、夹爪与 TCP 关键名称均存在（**PASS**）。

### 不变量与精确计数

执行：

```bash
awk '/<ros2_control/{n++} END{print "display_ros2_control_count=" n+0}' /tmp/ur10e_robotiq_display.urdf
awk '/<ros2_control/{n++} END{print "mock_ros2_control_count=" n+0}' /tmp/ur10e_robotiq_mock.urdf
awk '/<ros2_control name=/{print "mock_component=" $0}' /tmp/ur10e_robotiq_mock.urdf
awk '/<sensor name=/{print "mock_sensor=" $0}' /tmp/ur10e_robotiq_mock.urdf
rg -n '<xacro:(ur_robot|ur_ros2_control|robotiq_ft300|robotiq_fts_ros2_control|robotiq_gripper)' /ros2_ws/src/ur10e_robotiq/ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro
rg -n '<xacro:arg name="(ft_xyz|ft_rpy|gripper_xyz|gripper_rpy|tcp_xyz|tcp_rpy)"' /ros2_ws/src/ur10e_robotiq/ur10e_robotiq_description/urdf/ur10e_robotiq.urdf.xacro
test -f /ros2_ws/install/ur_client_library/share/ur_client_library/resources/external_control.urscript
test -f /ros2_ws/install/ur_robot_driver/share/ur_robot_driver/resources/rtde_input_recipe.txt
test -f /ros2_ws/install/ur_robot_driver/share/ur_robot_driver/resources/rtde_output_recipe.txt
```

整组命令退出码为 0。关键输出：

```text
display_ros2_control_count=0
mock_ros2_control_count=3
mock_component=  <ros2_control name="robotiq_2f85" type="system">
mock_component=  <ros2_control name="ur" rw_rate="0" type="system">
mock_component=  <ros2_control name="robotiq_ft_sensor" type="sensor">
mock_sensor=    <sensor name="robotiq_ft_sensor">
```

源文件检索同时命中 `ur_robot`、`ur_ros2_control`、`robotiq_ft300`、`robotiq_fts_ros2_control`、`robotiq_gripper` 五个上游 macro 调用；六个安装参数默认值保持为：

```text
ft_xyz="0 0 0"
ft_rpy="0 0 0"
gripper_xyz="0 0 0"
gripper_rpy="0 0 0"
tcp_xyz="0 0 0.15"
tcp_rpy="0 0 0"
```

结果：display 无控制块，mock 恰有 UR、夹爪、FT 三个控制块；FT hardware component 与 FT sensor 均精确命名为 `robotiq_ft_sensor`；UR Client Library 脚本及 UR Driver 两份 recipe 均真实存在（**PASS**）。

### 剩余问题

- Stage 1 已记录的默认 `/ros2_ws/build` symlink-install 基线冲突仍存在；Stage 2 只做 Xacro/URDF 静态验证，没有删除用户构建产物或重跑全工作区构建。
- RED 的预期“文件不存在”之后伴随 xacro 异常报告路径中的 `xml.parsers` 次生异常；GREEN 的两次正常展开与解析均不受影响。
