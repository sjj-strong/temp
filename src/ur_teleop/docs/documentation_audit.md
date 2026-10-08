# 文档与代码核对记录

核对日期：2026-10-09。范围为功能包 README 与 docs 全部说明，依据当前工作区 Python 实现、launch、安装入口和 YAML；源码设备值及数据目录可能由用户修改，文档示例不等同于通用默认值。

## 已纠正的主要差异

| 范围 | 对齐后的说明 |
| --- | --- |
| 启动 | Home/teleop/camera 已安装入口、参数适用分支和 CLI/YAML 优先级；移除不存在的 cell ruckig 参数 |
| Alicia | 配置频率与代码默认频率分开；切换重查和五次失败；夹爪重试 30 秒及非 ACTIVE 门控 |
| Ruckig | 独立节点 100 Hz 与阶段 2 launch 500 Hz 区分；初始化后持续输出，不订阅软件停止 |
| Xbot | 自动切换与 RB 输入分开；控制器空阻尼生成临时文件；退出不自动恢复 trajectory |
| 录制 | Alicia 未订阅 status、未实施 Xbot 的数据龄门控；Alicia TCP 可为 NaN；当前默认关节+TCP 为 13 维 |
| 相机 | 顶层相机名、独立类型和预览；resize 与保存尺寸唯一来源；录制配置只按名称选择 |
| 工具 | 键盘使用 /dev/tty + cbreak；新标定文件示例避免覆盖拒绝；手柄 Menu/View 映射说明与当前文件对齐 |

## 验证方式与边界

- 所有文档本地链接与锚点检查，Shell 示例仅用 bash -n 做语法检查，不执行其中运动指令。
- 包内 ros2 run 名称与 setup.py 的 console_scripts 比对；启动文件与参数名同 launch 声明逐项比对。
- 已安装 home、teleop、camera 使用 --show-args 核对入口；xbot_calibrate、xbot_joy_test、capture_slave_home 使用 --help 核对工具参数。
- 直接读取控制、录制、相机和配置实现核对行为；未修改控制代码或用户当前配置，未启动机器人运动或进行真实机械臂测试。

本次只证明文档与当前接口和实现一致，不把语法检查当作硬件、完整启动、视频编码或数据落盘验收。工作区已有的 Ruckig launch 开关与 YAML 路径不一致等边界已明确记录，不在本次文档更新中改变实现。
