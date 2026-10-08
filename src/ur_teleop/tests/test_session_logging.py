"""采集日志和进度测试，不启动控制器或写入用户数据集。"""
import json
from types import SimpleNamespace

from ur_teleop import session_logging as module


def test_debug_switch_and_structured_events():
    messages = []
    logger = SimpleNamespace(info=lambda value, **kwargs: messages.append((value, kwargs)))
    node = SimpleNamespace(_debug=False, get_logger=lambda: logger)
    module.debug_log(node, '位姿诊断')
    assert messages == []
    module.log_event(node, 'keyboard', action='save', frames=42)
    assert json.loads(messages[-1][0]) == dict(event='keyboard', action='save', frames=42)
    node._debug = True
    module.debug_log(node, '位姿诊断', throttle_duration_sec=2.)
    assert json.loads(messages[-1][0])['event'] == 'debug'
    assert messages[-1][1] == dict(throttle_duration_sec=2.)


def test_progress_counts_only_success_and_measures_stalls(monkeypatch):
    now = [0.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    progress = module.EpisodeProgress(2, 20)
    try:
        assert progress.bar.total is None
        for _ in range(10):
            progress.update()
        now[0] = 1.
        progress.tick()
        assert progress.bar.n == 10
        assert 'collect_hz=10.0' in progress.bar.postfix
        assert 'target_hz=20' in progress.bar.postfix
        now[0] = 2.
        progress.tick()
        assert progress.bar.n == 10
        assert 'collect_hz=0.0' in progress.bar.postfix
    finally:
        progress.close()
    assert progress.bar.disable
