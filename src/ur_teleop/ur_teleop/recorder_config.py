"""录制参数校验与当前 LeRobot 创建接口的参数映射。"""
import math

CREATE_DEFAULTS = dict(
    use_videos=True, tolerance_s=1e-4,
    image_writer_processes=0, image_writer_threads=2,
    video_backend=None, batch_encoding_size=1, vcodec='libsvtav1',
    metadata_buffer_size=10, streaming_encoding=False,
    encoder_queue_maxsize=30, encoder_threads=None,
)


def validate_recorder(rec):
    """在连接机器人或创建数据集前报告类型和范围错误。"""
    if not isinstance(rec, dict):
        raise ValueError('recorder 必须为映射')
    for name, camera in rec.get('cameras', {}).items():
        if not isinstance(camera, dict) or not isinstance(camera.get('resize', False), bool):
            raise ValueError(f'recorder.cameras.{name}.resize 必须为布尔值')
        if camera.get('enabled', True) and camera.get('resize', False):
            for key, default in (('width', 640), ('height', 480)):
                value = camera.get(key, default)
                if type(value) is not int or value <= 0:
                    raise ValueError(f'recorder.cameras.{name}.{key} 必须为正整数')
    integer_fields = dict(num_episodes=(0, 0), fps=(50, 1), min_frames_per_episode=(2, 1),
                          image_writer_processes=(0, 0), image_writer_threads=(2, 0),
                          batch_encoding_size=(1, 1), metadata_buffer_size=(10, 1),
                          encoder_queue_maxsize=(30, 1))
    for key, (default, minimum) in integer_fields.items():
        value = rec.get(key, default)
        if type(value) is not int or value < minimum:
            raise ValueError(f'recorder.{key} 必须为不小于 {minimum} 的整数')
    threads = rec.get('encoder_threads')
    if threads is not None and (type(threads) is not int or threads < 1):
        raise ValueError('recorder.encoder_threads 必须为正整数或 null')
    for key in ('use_videos', 'streaming_encoding', 'record_action_joints',
                'record_action_gripper', 'record_ur_joints', 'record_ur_ee_pose'):
        if key in rec and not isinstance(rec[key], bool):
            raise ValueError(f'recorder.{key} 必须为布尔值')
    for key, default, minimum in [('tolerance_s', 1e-4, 0), ('data_timeout_s', .5, 0)]:
        value = rec.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'recorder.{key} 必须为有限数值')
        if value < minimum or (key == 'data_timeout_s' and value == 0):
            raise ValueError(f'recorder.{key} 超出允许范围')
    for key in ('video_backend', 'robot_type'):
        value = rec.get(key)
        if value is not None and (not isinstance(value, str) or not value.strip()):
            raise ValueError(f'recorder.{key} 必须为非空字符串或 null')
    codec = rec.get('vcodec', CREATE_DEFAULTS['vcodec'])
    if not isinstance(codec, str) or not codec.strip():
        raise ValueError('recorder.vcodec 必须为非空字符串')


def dataset_create_options(rec):
    """只传入 LeRobotDataset.create 接受的选项，不混入本包录制字段。"""
    return {key: rec.get(key, default) for key, default in CREATE_DEFAULTS.items()}
