# 键盘读取（keyboard.py）

> 路径：`ur_teleop/ur_teleop/keyboard.py` —— 纯逻辑（仅标准库），非阻塞单键 stdin 读取器，`timeout=0` 时开销足够小可在 50 Hz timer 回调里调用。

## 概述

KeyboardReader 解决两个问题：(1) 在 ROS 节点主循环里以**非阻塞**方式读单个按键（节点不能因为等键盘而卡住 50 Hz 控制）；(2) 修复历史丢键 bug——不能走 Python 文件对象的缓冲读，必须 `os.read` 直读 fd（见"关键逻辑"）。

## 公开接口

```python
class KeyboardReader:
    def __init__(self, stream=None):
        # 默认 sys.stdin；测试注入 os.pipe 的读端
        self._stream = stream if stream is not None else sys.stdin

    def read_key(self, timeout: float = 0.0) -> str | None:
        ...
```

`read_key` 语义（keyboard.py:16-35）：

- `timeout=0.0`：非阻塞轮询，无输入立即返回 `None`（teleop_node 在 `_armed` 里以此实现"按 Enter 开始"的轻量门控）。
- `timeout>0`：阻塞至多 `timeout` 秒等一个键；超时仍返回 `None`。
- 返回值为**小写**单字符，或归一化后的 `"enter"`；EOF 与异常一律 `None`。

## 关键逻辑

```python
# keyboard.py:18-35（要点）
ready, _, _ = select.select([self._stream], [], [], timeout)   # 1. 先 select
if not ready:
    return None
ch = os.read(self._stream.fileno(), 1)     # 2. os.read 直读 fd，一次一个字节
# ...空/解码失败 → None
if text in ("\n", "\r"):
    return "enter"                          # 3. 换行归一
return text.lower()                         # 4. 小写归一
```

### 为什么必须 `os.read` 直读 fd（丢键修复背景）

模块 docstring 与代码注释（keyboard.py:22-24）明确记录：若用缓冲流 `read(1)`，Python 会把内核送来的**整块**字节（比如一次按键连发的 `"ab"`）吸进自己的缓冲，而 `select` 只认 fd 层——剩余字节会被"卡在"Python 缓冲里，select 认为无数据，于是丢键。`os.read(fd, 1)` 绕过缓冲流，每字节严格从 fd 取，配合 select 的"有数据才读"保证一次取一字节、不残留。测试 `test_multiple_chars_pending_not_lost` 正是这条路径的回归用例。

### 归一化细节

- `"\n"` 与 `"\r"`（终端回车两种形态）统一映射为 `"enter"`。
- 其余字节 `utf-8` 解码（`errors="ignore"`，非 UTF-8 字节丢弃），再 `lower()`（`S` → `s`）。
- 空字节（EOF）→ `None`；`ValueError`/`OSError`（流已关闭等）→ `None`。select 可能抛异常的场景同样被吞掉。

## 数据流 / 消费方

单实例 `KeyboardReader()` 在 teleop_node（teleop_node.py:96）与 data_recorder 中各建一个，共享 `sys.stdin` 直读：

- **teleop_node `_armed`（teleop 模式 Enter 门控，teleop_node.py:240）**：`if self._mode == "teleop" and self._kb.read_key(0.0) == "enter": self._begin_switch()`。仅 teleop 模式消费键盘；record 模式不读 Enter——开始信号改由 `/teleop/enable`（recorder 启动时发）驱动（teleop_node.py:228-229, 234-238）。
- **data_recorder 主循环（data_recorder.py:242-251）**：每帧 `read_key(0.0)`，按键归属：
  - `enter` → 开始新 episode（`_start_episode`）；
  - `s` → 保存当前 episode（`_save_episode`）；
  - `d` → 丢弃当前 episode（`_discard_episode`）；
  - `q` → 退出（break，随后 `finalize`）。

## 错误处理 / 已知边界

- 读取失败（EOF、流关闭）静默返回 `None`，不抛异常——调用方无需 try/except。
- 终端原始模式（raw mode）不在此模块：回车在行缓冲下也能被读到（按一次 Enter 产生 `\n` 或 `\r` 已足够），本包不设置 noecho/raw。
- `read_key` 每次最多消费 1 字节；多字节 UTF-8 字符会按字节多次返回（对单键命令足够，非完整终端输入解析器）。
- 输入源归属：teleop_node 与 data_recorder 是**独立进程**（各自 `main()` 建 `KeyboardReader()` 读同一个 `sys.stdin`）——同时只跑一个进程消费键盘，无互斥问题。

## 测试覆盖（tests/test_keyboard.py）

- `test_reads_available_char`：管道写入 `b"S"` → 返回 `"s"`（小写化）。
- `test_enter_key_normalized`：`b"\n"` → `"enter"`。
- `test_no_input_returns_none`：0.05s 无输入 → `None`（超时语义）。
- `test_eof_returns_none`：管道关闭 → `None`。
- `test_multiple_chars_pending_not_lost`：一次写入 `b"ab"`，两次 `read_key` 依次得 `"a"`、`"b"`——缓冲流会丢 `"b"`，os.read 直读不丢（丢键 bug 回归）。
