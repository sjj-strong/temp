"""采集日志：结构化事件、可关闭的调试输出及实时帧进度。"""
import hashlib
import json
import time


def log_event(node, event, *, level='info', message=None, **fields):
    """事件内容为 JSON；ROS 日志等级、时间戳由原日志器保留。"""
    options = {key: fields.pop(key) for key in ('throttle_duration_sec', 'once', 'skip_first') if key in fields}
    if message is not None:
        fields['message'] = message
    # ROS 按调用位置缓存等级和过滤参数；为不同事件及设置保留独立日志上下文。
    key = (event, level, tuple(sorted(options.items())))
    if not hasattr(node, '_event_loggers'):
        node._event_loggers = {}
    if key not in node._event_loggers:
        suffix = hashlib.sha256(repr(key).encode()).hexdigest()[:16]
        node._event_loggers[key] = node.get_logger().get_child('event_' + suffix)
    getattr(node._event_loggers[key], level)(
        json.dumps(dict(event=event, **fields), ensure_ascii=False), **options)


def debug_log(node, message, **options):
    if getattr(node, '_debug', False):
        log_event(node, 'debug', message=message, **options)


class EpisodeProgress:
    """无固定时长的 episode 只显示真实帧数与每秒成功写入帧数。"""
    def __init__(self, episode, fps, num_episodes=0):
        from tqdm import tqdm
        description = f'episode={episode}/{num_episodes}' if num_episodes else f'episode={episode}'
        self.bar = tqdm(total=None, desc=description, unit='frame',
                        dynamic_ncols=True, mininterval=0.5,
                        bar_format='{desc} | frames={n} | elapsed={elapsed}{postfix}',
                        postfix=dict(collect_hz='0.0', target_hz=fps))
        self.fps = fps
        self._stamp = time.monotonic()
        self._frames = 0

    def update(self):
        self.bar.update(1)

    def tick(self, force=False):
        now = time.monotonic()
        elapsed = now - self._stamp
        if elapsed >= 1.0 or (force and elapsed > 0):
            hz = (self.bar.n - self._frames) / elapsed
            self.bar.set_postfix(collect_hz=f'{hz:.1f}', target_hz=self.fps, refresh=False)
            self.bar.refresh()
            self._stamp, self._frames = now, self.bar.n

    def close(self):
        self.tick(force=True)
        self.bar.close()
