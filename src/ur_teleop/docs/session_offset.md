# 会话级偏移（offset.py）

> 路径：`ur_teleop/ur_teleop/offset.py` —— 纯逻辑（无 rclpy，全文件 23 行），保存本次会话双臂 home 的实际位置，作为 JointMapper 的映射基准。

## 概述

SessionOffset 只在 home 校验通过、双臂静止后被 `capture()` 一次，之后把捕获值作为 home 偏移注入 JointMapper。设计要点（模块 docstring 与设计文档 §3）：**偏移是会话级的，永不写盘**——每次启动都重新捕获实际到位位置，不依赖配置文件里的目标 home（`home.master`/`home.slave` 只是用户预设目标，实际到位值可能略有偏差）。

## 公开接口

```python
class SessionOffset:
    def __init__(self):
        self.master_home: Optional[list[float]] = None   # 捕获前为 None
        self.slave_home:  Optional[list[float]] = None

    @property
    def captured(self) -> bool:
        # master_home 与 slave_home 均非 None
        return self.master_home is not None and self.slave_home is not None

    def capture(self, master_q: list[float], slave_q: list[float]) -> None:
        # 存副本：list(...)，外部后续修改不影响内部
        self.master_home = list(master_q)
        self.slave_home = list(slave_q)
```

接口语义：

- `capture(master_q, slave_q)`：一次调用同时捕获双臂位置。**存的是副本**（`list()` 拷贝），调用方后续改动 `_master_q`/`_slave_q` 不会污染捕获值——这是测试明确锁定的行为。
- `captured`：二者齐全才为 True。注意不校验长度（6 项），长度合法性由下游 JointMapper 构造期兜底。
- 无 `apply()` 之类的独立方法：**应用方式就是把 `master_home`/`slave_home` 直接传给 `JointMapper(...)` 构造参数**（见下）。

## 关键逻辑

"捕获 → 应用"链在 teleop_node 中（teleop_node.py:219-227，`_capture_offset`）：

```python
self._offset.capture(list(self._master_q), list(self._slave_q))
self._mapper = JointMapper(
    build_mapping_config(self._cfg), self._offset.master_home, self._offset.slave_home
)
```

- 触发时机：状态机 `CAPTURE_OFFSET`，前置条件是 VERIFY_HOME 通过 + SETTLING 静止达标（`settle_time_s` 内逐关节运动小于 `settle_motion_threshold_rad`）。
- 捕获后立即构造 mapper，随后进入 ARMED。会话内 mapper 不再重建，即 offset 固定为本次会话捕获值。
- 捕获值本身不参与 clamp：clamp 限位来自配置文件 `safety.limits`，与捕获值无关。

### 下游长度校验兜底

SessionOffset 不检查长度；若上游给了 5 项或 7 项，`JointMapper.__init__` 的 `master_home/slave_home must have 6 values` 校验（joint_mapper.py:25-26）会直接抛 `ValueError`——偏移模块保持最简，形状契约由消费方强制执行。

## 数据流 / 消费方

- 生产者：teleop_node `_capture_offset`（每会话一次）。
- 消费者：JointMapper（映射基准）；无其他模块直接读取 offset。
- 不参与记录/回放：`data_recorder` 不感知 offset（录的是原始关节数据与指令）。

## 错误处理 / 已知边界

- 无异常路径：capture 不校验、不返回错误；"未捕获就使用"由 teleop_node 状态机保证（只有经过 CAPTURE_OFFSET 才会 ARMED）。
- 设计边界：偏移只在会话启动时捕获一次，运行中不重捕获；若运行中双臂被外力移动而主臂未变，映射基准不自动修正（属刻意简化，与"会话级"语义一致）。
- 不校验捕获时双臂是否真的在目标 home——校验责任在 VERIFY_HOME/SETTLING 阶段，不在此模块。

## 测试覆盖（tests/test_offset.py）

- `test_capture_stores_copies`：capture 后 `captured` 为 True、值相等；**外部修改原列表不影响内部**（副本语义）。
- `test_not_captured_by_default`：默认 `captured` 为 False。
- `test_mapper_with_captured_offset_identity`：捕获非理想 home（如 slave 为 `[0.01, -1.58, ...]`）后，主臂回到捕获位时 `master_to_slave` 输出 == 捕获的 slave 位（sign=1、scale=1 时恒等；证明"实际到位值"而非"配置目标"是映射基准）。
