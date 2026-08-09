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
