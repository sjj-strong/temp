"""原始观测与物理量动作的 JSON 协议，不依赖模型或 ROS。"""
import base64
import io
from typing import Literal

import numpy as np
from PIL import Image
from pydantic import BaseModel, ConfigDict, Field, field_validator

POSE_ACTION = ['dx', 'dy', 'dz', 'drx', 'dry', 'drz']


class Message(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)


class ImageData(Message):
    encoding: Literal['rgb8', 'bgr8'] = 'rgb8'
    png: str

    @classmethod
    def encode(cls, pixels, encoding='rgb8'):
        """仅封装像素，不进行缩放或模型归一化。"""
        pixels = np.asarray(pixels)
        if pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] != 3:
            raise ValueError('图像必须为 H×W×3 的 uint8')
        stream = io.BytesIO()
        Image.fromarray(pixels).save(stream, format='PNG')
        return cls(encoding=encoding, png=base64.b64encode(stream.getvalue()).decode('ascii'))

    def decode(self):
        """服务器解码，统一为 RGB。"""
        try:
            with Image.open(io.BytesIO(base64.b64decode(self.png, validate=True))) as image:
                if image.format != 'PNG' or image.mode != 'RGB':
                    raise ValueError('必须使用三通道 PNG')
                pixels = np.array(image)
        except Exception as exc:
            raise ValueError('PNG/Base64 图像无效') from exc
        return pixels if self.encoding == 'rgb8' else pixels[..., ::-1].copy()


class Observation(Message):
    step_id: int = Field(ge=0, strict=True)
    values: dict[str, float]
    images: dict[str, ImageData] = Field(default_factory=dict)
    reference_frame: str = Field(min_length=1)
    tcp_link: str = Field(min_length=1)
    wrench_frame: str | None = None
    instruction: str = ''


class ActionChunk(Message):
    step_id: int = Field(ge=0, strict=True)
    actions: list[list[float]]

    @field_validator('actions')
    @classmethod
    def validate_actions(cls, actions):
        if not actions or len(actions[0]) not in (6, 7):
            raise ValueError('动作必须是非空的六维或七维 chunk')
        if any(len(row) != len(actions[0]) for row in actions):
            raise ValueError('动作各行维度必须一致')
        return actions


class Health(Message):
    ready: Literal[True] = True
    policy_type: str
    state_names: list[str]
    cameras: dict[str, list[int]]
    action_names: list[str]
    reference_frame: str
    tcp_link: str
    wrench_frame: str | None = None
    fps: float = Field(gt=0)

    @field_validator('action_names')
    @classmethod
    def validate_names(cls, names):
        if names not in (POSE_ACTION, POSE_ACTION + ['cmd_gripper']):
            raise ValueError('只接受按约定排列的 rel pose，可附带夹爪')
        return names
