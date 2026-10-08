# Alicia 关节映射

主臂关节变化按比例和方向映射到 UR，基准为每次启动后实际捕获的双臂位置：

```text
UR 目标 = 捕获的 UR 位置 + sign × scale × (主臂位置 − 捕获的主臂位置)
```

## 参数

在 `alicia_teleop.yaml` 中配置：

| 参数 | 作用 |
| --- | --- |
| `mapping.sign` | 六个方向系数；1 同向、-1 反向 |
| `mapping.scale` | 六个位移比例；例如 0.8 表示主臂转动 1 rad，对应 UR 转动 0.8 rad |
| `mapping.alicia_joint_order` | 保持 Joint1 至 Joint6 顺序 |
| `mapping.ur_joint_order` | 保持 shoulder_pan、shoulder_lift、elbow、wrist_1、wrist_2、wrist_3 顺序 |
| `safety.limits` | 每个 UR 关节允许的最小/最大角度，rad |
| `safety.clamp_margin_rad` | 限位内缩余量，rad |

以上数组均须为六项。映射按数组索引对应，修改名称顺序不会自动重排输入。目标超出限位时截断到内缩后的范围。

映射本身不限制速度；平滑设置见[Ruckig](ruckig_node.md)。运行命令见[完整流程](workflow.md#4-遥操作)，参数修改后重启遥操作。
