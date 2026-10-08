"""传输协议的无损性和拒绝非法动作行为。"""
import numpy as np
import pytest
from protocol import ActionChunk, ImageData, Observation


def test_png_roundtrip():
    pixels = np.random.default_rng(2).integers(0, 256, (13, 17, 3), dtype=np.uint8)
    assert np.array_equal(ImageData.encode(pixels).decode(), pixels)
    assert np.array_equal(ImageData.encode(pixels, 'bgr8').decode(), pixels[..., ::-1])


@pytest.mark.parametrize('actions', [[], [[0.] * 5], [[0.] * 6, [0.] * 7], [[float('nan')] * 6], [[float('inf')] * 7]])
def test_bad_actions(actions):
    with pytest.raises(ValueError):
        ActionChunk(step_id=0, actions=actions)


def test_observation_rejects_nonfinite():
    with pytest.raises(ValueError):
        Observation(step_id=0, values={'x': float('nan')}, reference_frame='base', tcp_link='tool0')


def test_invalid_image():
    with pytest.raises(ValueError):
        ImageData(png='invalid').decode()
