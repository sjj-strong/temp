"""录制参数与片段限额测试，不连接机械臂或写用户数据集。"""
from types import SimpleNamespace

import pytest
from ur_teleop.recorder_config import CREATE_DEFAULTS, dataset_create_options, validate_recorder, recording_cameras


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
    node._episode_end_pub = SimpleNamespace(publish=lambda msg: None)
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
    monkeypatch.setattr(DataRecorderNode, 'get_logger', lambda self: _child_logger(SimpleNamespace(
        info=lambda *args: None, warn=lambda *args: None)))
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
    node._xbot = False
    node._num_episodes = 0
    node._episode_count = 100
    node._finish_requested = False
    node._min_frames = 2
    node._progress = None
    node._recording, node._frame_count = True, 5
    node._dataset = SimpleNamespace(save_episode=lambda: None)
    monkeypatch.setattr(DataRecorderNode, 'get_logger', lambda self: _child_logger(SimpleNamespace(info=lambda *args: None)))
    node._save_episode()
    assert node._episode_count == 101 and not node._finish_requested
    node._num_episodes = 102
    node._recording = True
    node._dataset.save_episode = lambda: (_ for _ in ()).throw(RuntimeError('模拟保存失败'))
    with pytest.raises(RuntimeError):
        node._save_episode()
    assert node._episode_count == 101 and not node._finish_requested


@pytest.mark.parametrize('camera', [dict(resize='false'), dict(resize=True, resize_width=0),
                                    dict(resize=True, resize_height=-1),
                                    dict(resize=True, resize_width=1.5)])
def test_invalid_camera_resize(tmp_path, camera):
    import yaml
    path = tmp_path / 'camera.yaml'
    path.write_text(yaml.safe_dump({'front': camera}))
    with pytest.raises(ValueError, match='相机 front'):
        recording_cameras(dict(cameras={'front': {}}), path)


def test_recording_camera_settings_have_one_source(tmp_path):
    """相机文件控制尺寸和话题，各相机缩放独立，录制配置不重复定义。"""
    import yaml
    path = tmp_path / 'camera.yaml'
    path.write_text(yaml.safe_dump(dict(
        front=dict(resize=True, resize_width=200, resize_height=100, topic='/custom/image'),
        wrist=dict(resize=False, width=1280, height=720))))
    cameras = recording_cameras(dict(cameras=dict(
        front=dict(image_key='front', width=999, resize=False), wrist={}, off=dict(enabled=False))), path)
    assert (cameras['front']['width'], cameras['front']['height']) == (200, 100)
    assert cameras['front']['resize'] is True
    assert cameras['front']['topic'] == '/custom/image'
    assert (cameras['wrist']['width'], cameras['wrist']['height']) == (1280, 720)
    assert 'off' not in cameras
    assert recording_cameras({}, tmp_path / 'missing.yaml') == {}


@pytest.mark.parametrize('config', [{}, {'front': dict(enabled=False)}])
def test_recording_unknown_or_disabled_camera(tmp_path, config):
    import yaml
    path = tmp_path / 'camera.yaml'
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='不存在或未启用'):
        recording_cameras(dict(cameras={'front': {}}), path)


def _child_logger(logger):
    """模拟 ROS 子日志器接口，保留原有消息收集。"""
    logger.get_child = lambda name: logger
    return logger
