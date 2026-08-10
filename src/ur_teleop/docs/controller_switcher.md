# 控制器切换（controller_switcher.py）

> 路径：`ur_teleop/ur_teleop/controller_switcher.py` —— controller_manager 的异步客户端封装，**从不 spin executor**（设计文档 §5 问题 10）；所有调用 `call_async` 返回 Future，由调用方在自己的状态机里轮询。

## 概述

ControllerSwitcher 把 `/controller_manager` 的三个服务（`switch_controller` / `list_controllers` / `load_controller`）封装成"发起即返回 Future"的接口，解决嵌套 spin 问题（历史版本在 50 Hz 回调里 `spin_until_future_complete` 与外部 spin 嵌套导致死锁，问题 10）。本模块自身不阻塞、不 spin；`wait_for_services` 是唯一的阻塞 API，仅留给一次性/主线程进程（目前无调用方，见下）。

## 公开接口

### 构造与就绪检测

```python
def __init__(self, node, timeout_s: float = 10.0):
    # 建三个 client：
    #   /controller_manager/switch_controller  (SwitchController)
    #   /controller_manager/list_controllers   (ListControllers)
    #   /controller_manager/load_controller    (LoadController)

def services_ready(self) -> bool:
    # 三 client 的 service_is_ready() 全部为真
```

`services_ready()` 是 teleop_node 使用的唯一就绪探测（WAITING_CELL 阶段判 controller_manager 在线，teleop_node.py:167）。

### 异步请求（返回 Future，客户端未就绪返回 None）

```python
def list_controllers(self):            # Future → ListControllers.Response（controller 列表）
def load_controller(self, controller_name: str):   # Future → LoadController.Response（.ok 为结果）
def switch(self, activate: list[str], deactivate: list[str]):  # Future → SwitchController.Response
    # strictness = SwitchController.Request.STRICT（严格模式：失败即整体失败）
```

三个方法都有"客户端未就绪 → 返回 None"的契约，调用方必须处理 None（teleop_node 视为失败回 ARMED）。

### 静态结果解读（契约见下）

```python
@staticmethod
def list_result(future) -> dict:      # {controller_name: state}，如 {"forward_position_controller": "active"}
    if future is None or not future.done() or future.result() is None:
        return {}
    return {c.name: c.state for c in future.result().controller}

@staticmethod
def switch_ok(future) -> bool:        # 成功且 future.result().ok 为真
    return (future is not None and future.done()
            and future.result() is not None and future.result().ok)
```

**静态方法契约（与集成测试 B 组锁定，test_integration.py:900-920）**：

- 未完成（`not done`）或 `None` 或结果为 None → **不抛**，返回安全默认（`{}` / `False`）。
- cancelled 或携带异常的 future → `future.result()` **会把 `concurrent.futures.CancelledError` / 原异常透传抛出**（不是吞掉）——因为 `done()` 为真时会调用 `result()`。**调用侧必须守卫**：teleop_node 所有解读点都包在 `try/except Exception` 里（list 异常视为未加载走 load 路径、switch 异常视为一次失败参与重试），异常从不逃逸出 `_tick`（集成测试 A4a/A4b）。

### `wait_for_services(timeout_s: float | None = None) -> bool`（controller_switcher.py:33-48）

阻塞等待三个服务逐个就绪（每个 `wait_for_service(timeout_sec=0.5)` 轮询，整体 deadline 为 `timeout_s` 或构造时 `timeout_s`）；超时记 error 日志返回 False。**目前无调用方**——teleop_node 只用非阻塞的 `services_ready()`；该方法是为未来一次性/标定类进程预留的 API（模块 docstring 注明"main thread / one-shot processes only"）。

## 数据流 / 调用方轮询模式

teleop_node 的 SWITCHING 链（teleop_node.py:244-297）：

```text
_begin_switch:  _switch_phase="list"; future = list_controllers(); → SWITCHING
_switching（每 50 Hz tick 轮询 future.done()）：
  list 完成:
    controllers = list_result(fut)（异常 → {}）
    fwd_ctrl 不在列表 → phase="load", future = load_controller("forward_position_controller")
    fwd_ctrl 已在列表 → phase="switch", future = switch([fwd], [traj])
  load 完成:
    ok=true → phase="switch", switch([fwd], [traj])
    ok=false → 回 ARMED（load 失败不计入 switch 重试次数）
  switch 完成:
    switch_ok(fut) 为真 → 发布 demo/status → ACTIVE
    为假 → _switch_attempt += 1；满 5 次回 ARMED（"5 次失败"），否则重发 switch（最多 5 次）
```

- **从不阻塞**：future 未完成时 `_switching` 直接返回（集成测试 A6 锁定：in-flight 时 `_tick` 耗时 < 0.5s）。
- **5 次重试**：`_switch_attempt` 仅在 switch 阶段失败时递增，满 5（`>= 5`）回 ARMED（A3）；load 失败、list 异常、future=None（客户端未就绪）都不计次（A2/A4a/A5）。
- **退出恢复**：`shutdown()`（teleop_node.py:360-382）在 `restore_controller_on_exit` 且状态为 ACTIVE/INACTIVE/SWITCHING 时反向 `switch([traj], [fwd])`；这是唯一使用 `SingleThreadedExecutor.spin_once` 的地方（临时 executor 等 future，属一次性退出路径，不违反"主循环不 spin"）。
- 控制器名常量：`_traj_ctrl = "scaled_joint_trajectory_controller"`、`_fwd_ctrl = "forward_position_controller"`（teleop_node.py:71-72）。

## 错误处理 / 已知边界

- 客户端未就绪时不发请求、返回 None——调用方自行决定重试或失败（teleop_node 回 ARMED 等待再次 enable）。
- 静态解读不吞异常（见契约）：吞异常的责任在调用侧统一 `try/except`，保证异常不会逃逸 50 Hz `_tick`。
- `list_result` 内部对 `future.result()` 调用两次（`or` 短路外的 `result()` 与 dict 推导里的 `result()`）——语义一致，仅轻微冗余。
- 不维护重试状态：重试计数完全由调用方（teleop_node `_switch_attempt`）管理，本模块无状态。

## 测试覆盖（tests/test_integration.py）

- A1 `test_switching_chain_list_load_switch_active`：list → load → switch → ACTIVE 的 phase 顺序。
- A2 `test_switching_load_failure_returns_to_armed`：load `ok=false` → ARMED，`_switch_attempt` 保持 0。
- A3 `test_switching_five_failures_caps_to_armed`：switch 连败 5 次 → ARMED，attempt == 5。
- A4a `test_switching_list_exception_falls_through_to_load`：list 异常 → 视为未加载走 load。
- A4b `test_switching_switch_exception_retries_then_armed`：switch 异常视为失败重试，异常不逃逸 `_tick`。
- A5 `test_switching_none_future_returns_to_armed`：future=None → 直接 ARMED。
- A6 `test_switching_inflight_future_does_not_block`：in-flight 不阻塞。
- B `test_switcher_statics_contract_on_bad_futures`：cancelled/异常 future 抛 `CancelledError`/`RuntimeError`；未完成/None/空结果返回 `{}`/`False`。
