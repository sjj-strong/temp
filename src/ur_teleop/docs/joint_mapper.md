# Alicia 关节映射

实现：`ur_teleop/joint_mapper.py`，纯逻辑。`JointMapper(mapping_config, master_home, slave_home)` 使用会话实际捕获位置作为基准，按数组索引映射：

```text
raw[i] = slave_home[i] + sign[i] × scale[i] × (master_q[i] − master_home[i])
command[i] = clamp(raw[i], limits[ur_joint_order[i]][0] + margin,
                          limits[ur_joint_order[i]][1] − margin)
```

构造参数中的 `safety` 包含 `limits` 和 `clamp_margin_rad`，由 teleop 的 `build_mapping_config()` 合并。margin 代码缺省为 0.1 rad。两侧顺序、sign、scale 和两个 Home 数组都须为六项；限位必须包含配置的从臂名称，否则构造抛 ValueError。`master_to_slave(master_q)` 也校验输入长度。

名称列表用于描述顺序和查询限位，mapper 不按名称重新排列输入。当前 teleop 回调实际按 `ALICIA_JOINT_NAMES` 读取主臂、按 `UR_JOINT_NAMES` 读取从臂；输出消息仍使用 UR 标准顺序，因此不应仅修改 mapping 名称顺序来重排关节。

`ur_joint_order`、`alicia_joint_order`、`get_master_home()` 和 `get_slave_home()` 返回副本；`num_joints` 为从臂数组长度。SessionOffset 复制实际位置但不校验长度，最终由 mapper 校验。

mapper 只做静态仿射变换和数值截断，不做速度限制、滤波或物理合理性检测。Ruckig 平滑仅在所选路径启用时存在；关节阻抗关闭 Ruckig 后由控制器处理其内部参考和力矩限制。参数见[Alicia 配置](alicia_teleop_config.md)，单元用例见 `tests/test_joint_mapper.py`。
