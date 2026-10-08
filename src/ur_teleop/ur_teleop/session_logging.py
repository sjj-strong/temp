"""采集日志：结构化事件、可关闭的调试输出及实时帧进度。"""
import json
import time


def log_event(node, event, *, level='info', message=None, **fields):
    """事件内容为 JSON；ROS 日志等级、时间戳由原日志器保留。"""
    options = {key: fields.pop(key) for key in ('throttle_duration_sec', 'once', 'skip_first') if key in fields}
    if message is not None:
        fields['message'] = message
    getattr(node.get_logger(), level)(json.dumps(dict(event=event, **fields), ensure_ascii=False), **options)


def debug_log(node, message, **options):
    if getattr(node, '_debug', False):
        log_event(node, 'debug', message=message, **options)


class EpisodeProgress:
    """无固定时长的 episode 只显示真实帧数与每秒成功写入帧数。"""
    def __init__(self, episode, fps):
        from tqdm import tqdm
        self.bar = tqdm(total=None, desc=f'episode={episode}', unit='frame',
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
