# 键盘读取

实现：`ur_teleop/keyboard.py`。`KeyboardReader(stream=None)` 默认优先打开控制终端 `/dev/tty`，避免 `ros2 launch` 把子进程 stdin 重定向后收不到输入；打开失败才回退 `sys.stdin`。测试可传入管道或其他流。

打开控制终端后切换 cbreak，单键无需额外回车，并保留 Ctrl-C；保存原终端属性，退出时恢复。Home 等回车结束时也主动恢复属性。并非始终读取 stdin，也并非“不修改终端模式”。

`read_key(timeout=0.0)` 用 select 检查可读性，再以 `os.read(fd, 1)` 取一个字节，绕过 Python 缓冲造成的待输入丢失。零超时立即返回；正超时最多等待对应秒数。换行统一返回 `enter`，普通键转小写；无输入、EOF 或读取异常返回 None。

## 操作归属

| 阶段/模式 | 读取者 | 输入 |
| --- | --- | --- |
| 真机 Home | home_node | External Control 确认后 Enter |
| Alicia teleop | teleop_node | ARMED 中 Enter 开始切换 |
| Alicia record | data_recorder | Enter 开始，S 保存，D 丢弃，Q 保存并 finalize |
| Xbot | 手柄节点/采集器事件 | 不读录制键盘，使用 Menu/Y/B/View |

Alicia record 的 teleop_node 虽创建 KeyboardReader，但不会读 Enter；采集器在首次开始 episode 时发 `/teleop/enable`，并非启动采集器就自动使能。测试覆盖在 `tests/test_integration.py` 的键盘用例中，不存在独立 `test_keyboard.py`。
