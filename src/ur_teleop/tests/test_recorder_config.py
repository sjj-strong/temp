"""录制参数与片段限额测试，不连接机械臂或写用户数据集。"""
from types import SimpleNamespace

import pytest
from ur_teleop.recorder_config import CREATE_DEFAULTS, dataset_create_options, validate_recorder


@pytest.mark.parametrize('field,value', [
    ('num_episodes', -1), ('num_episodes', True), ('num_episodes', 2.5),
    ('num_episodes', '10'), ('fps', 0), ('min_frames_per_episode', 0),
    ('tolerance_s', float('nan')), ('tolerance_s', -1), ('data_timeout_s', 0),
    ('streaming_encoding', 'false'), ('use_videos', 1), ('batch_encoding_size', 0),
    ('metadata_buffer_size', 0), ('image_writer_processes', -1),
    ('image_writer_threads', True), ('encoder_queue_maxsize', 0),
    ('encoder_threads', 0), ('video_backend', 42), ('vcodec', ''),
])
def test_invalid_recorder_fields(field, value):
    with pytest.raises(ValueError, match=field):
        validate_recorder({field: value})


def test_complete_create_mapping_and_defaults():
    validate_recorder({})
    assert dataset_create_options({}) == CREATE_DEFAULTS
    rec = dict(num_episodes=50, fps=30, tolerance_s=.01, streaming_encoding=True,
               batch_encoding_size=5, vcodec='h264', video_backend='pyav',
               image_writer_threads=4, encoder_threads=2, encoder_queue_maxsize=60,
               metadata_buffer_size=20, image_writer_processes=1, use_videos=True)
    validate_recorder(rec)
    mapped = dataset_create_options(rec)
    assert set(mapped) == set(CREATE_DEFAULTS)
    assert all(mapped[key] == rec.get(key, default) for key, default in CREATE_DEFAULTS.items())
    assert 'num_episodes' not in mapped and 'fps' not in mapped


@pytest.mark.parametrize('source', ['alicia', 'xbot'])
def test_episode_target_counts_saved_only_and_finalizes(monkeypatch, source):
    from ur_teleop.data_recorder import DataRecorderNode
    node = object.__new__(DataRecorderNode)
    node._xbot = source == 'xbot'
    node._num_episodes = 2
    node._finish_requested = False
    node._episode_count = 0
    node._min_frames = 2
    node._progress = None
    calls, finished = [], []
    node._dataset = SimpleNamespace(save_episode=lambda: calls.append('save'),
                                   clear_episode_buffer=lambda: calls.append('discard'),
                                   finalize=lambda: calls.append('finalize'))
    node._finished_pub = SimpleNamespace(publish=lambda message: finished.append(message.data))
    monkeypatch.setattr(DataRecorderNode, 'get_logger', lambda self: SimpleNamespace(
        info=lambda *args: None, warn=lambda *args: None))
    for method, frames in [('_save_episode', 1), ('_discard_episode', 5), ('_save_episode', 3)]:
        node._recording, node._frame_count = True, frames
        getattr(node, method)()
    assert node._episode_count == 1 and not node._finish_requested
    assert finished == []
    node._recording, node._frame_count = True, 4
    node._save_episode()
    assert node._episode_count == 2 and node._finish_requested
    assert finished == [True]
    node._start_episode()
    assert not node._recording
    node.finalize()
    assert calls == ['discard', 'discard', 'save', 'save', 'finalize']
    assert finished == [True]


def test_unlimited_and_save_failure_do_not_finish(monkeypatch):
    from ur_teleop.data_recorder import DataRecorderNode
    node = object.__new__(DataRecorderNode)
    node._num_episodes = 0
    node._episode_count = 100
    node._finish_requested = False
    node._min_frames = 2
    node._progress = None
    node._recording, node._frame_count = True, 5
    node._dataset = SimpleNamespace(save_episode=lambda: None)
    monkeypatch.setattr(DataRecorderNode, 'get_logger', lambda self: SimpleNamespace(info=lambda *args: None))
    node._save_episode()
    assert node._episode_count == 101 and not node._finish_requested
    node._num_episodes = 102
    node._recording = True
    node._dataset.save_episode = lambda: (_ for _ in ()).throw(RuntimeError('模拟保存失败'))
    with pytest.raises(RuntimeError):
        node._save_episode()
    assert node._episode_count == 101 and not node._finish_requested
