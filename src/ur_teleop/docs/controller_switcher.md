# 控制器异步切换

实现：`ur_teleop/controller_switcher.py`。构造 `ControllerSwitcher(node, timeout_s=10.0)` 建立 `/controller_manager/list_controllers`、`load_controller`、`switch_controller` 三个客户端。

`services_ready()` 只检查三服务发现状态。`list_controllers()`、`load_controller(name)`、`switch(activate, deactivate)` 都调用 `call_async` 并返回 future；对应客户端未就绪时返回 None。模块不 spin 执行器，调用者自行推进回调和轮询 future。

`switch()` 使用 STRICT，并复制 activate/deactivate 名称数组。它不自动加载控制器、不决定冲突控制器、不重试，也不把构造的 timeout_s 写进 SwitchController 请求；这些逻辑由 Alicia 或 Xbot 调用者负责。

`list_result(future)` 在 None、未完成或空结果时返回 `{}`，否则返回名称到状态的字典。`switch_ok(future)` 同样在无有效结果时返回 False，完成时读取 `.ok`。future 自身包含异常或取消时，`.result()` 可能抛异常，调用侧必须捕获。

`wait_for_services(timeout_s=None)` 是阻塞 API，逐个以 0.5 秒等待服务，三个客户端共享一个 deadline；缺省使用构造时 timeout_s。它不能放进控制定时器回调中。Alicia 的重查/加载/五次切换重试见[遥操作节点](teleop_node.md)，Xbot 自动接管和控制器状态确认见[手柄说明](xbot_control.md)。
