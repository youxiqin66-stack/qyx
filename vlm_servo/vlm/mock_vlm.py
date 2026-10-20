#!/usr/bin/env python3
"""模拟 VLM：不调用任何 API，返回固定 bbox 列表。

用途：没有 API Key / 没有网络 / 想先验证「多目标交互 + 3D 位姿链路」时，
把 config.yaml 里的 vlm.type 改成 "mock"，并可在 mock_bboxes 里写多个框演示多目标。
"""

import time

from .base import BaseVLM, Detection, VisionResult


class MockVLM(BaseVLM):
    def __init__(self, bboxes=None):
        # bboxes: 像素坐标列表 [[x1,y1,x2,y2], ...]
        self.bboxes = bboxes or [[220, 160, 420, 360]]

    def detect(self, image_bgr, instruction):
        # 模拟网络/推理延迟，让流程行为更接近真实调用
        time.sleep(0.5)

        # 简单规则抠出"目标名"（仅用于演示流程，不代表真实理解能力）
        target = instruction
        for ch in ("抓取", "拿取", "取出", "靠近", "接近", "挨进", "贴近", "打磨",
                   "加工", "磨削", "精磨", "抓", "拿", "取", "递", "找", "跟",
                   "磨", "挨"):
            target = target.replace(ch, "")
        target = target.strip() or "object"

        dets = []
        for i, bb in enumerate(self.bboxes):
            name = target if len(self.bboxes) == 1 else f"{target} #{i + 1}"
            dets.append(Detection(target=name, bbox=tuple(int(v) for v in bb),
                                  raw="mock"))

        return VisionResult(detections=dets)
