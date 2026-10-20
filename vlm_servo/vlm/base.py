#!/usr/bin/env python3
"""VLM 抽象接口。

替换模型时，只需新增一个继承 BaseVLM 的实现类，并在 vlm/__init__.py 的
build_vlm() 工厂里注册即可，其余代码（camera / drawing / main）无需改动。
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class Detection:
    """一次检测结果。bbox 为原图像素坐标 [x1, y1, x2, y2]。"""

    target: Optional[str] = None                       # 目标名称（英文）
    bbox: Optional[Tuple[int, int, int, int]] = None   # (x1, y1, x2, y2) 像素坐标
    raw: Optional[str] = None                          # 模型原始输出，便于调试
    confidence: Optional[float] = None                 # 可选置信度

    @property
    def found(self) -> bool:
        return self.target is not None and self.bbox is not None


@dataclass
class VisionResult:
    """一次视觉理解结果：匹配到的目标列表。"""

    detections: List[Detection] = field(default_factory=list)


class BaseVLM(ABC):
    """视觉大模型统一接口：图像 + 自然语言指令 -> 所有匹配目标。

    只做视觉定位，不做意图/动作分类。
    返回空 detections 表示未找到；多个元素表示画面里有多个匹配目标。
    """

    @abstractmethod
    def detect(self, image_bgr, instruction: str) -> VisionResult:
        """
        :param image_bgr: BGR 彩色图像 (H, W, 3)
        :param instruction: 用户自然语言指令，如 "打磨三角块"
        :return: VisionResult（匹配目标列表）
        """
        raise NotImplementedError
