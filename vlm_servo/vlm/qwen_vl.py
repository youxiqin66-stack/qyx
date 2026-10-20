#!/usr/bin/env python3
"""Qwen3-VL 实现：通过阿里云百炼 DashScope 的 OpenAI 兼容接口调用。

不占用本地显存，适合 RTX 4050 6GB 还要跑 FoundationPose 的场景。
detect() 只做视觉定位，一次返回所有匹配目标(detections)，不做意图/动作分类。
"""

import base64
import json
import re

import cv2

from .base import BaseVLM, Detection, VisionResult

_DEFAULT_SYSTEM = (
    "You are a robot vision assistant. Given an image and a natural language "
    "instruction, list all objects that match the instruction and localize each "
    "with a 2D bounding box."
)

_DETECT_PROMPT = (
    "Instruction: {instruction}\n"
    "Find the target object the instruction refers to in the image and draw a tight "
    "2D bounding box around it. "
    "The instruction may start with an action verb (打磨/加工/磨削/抛光/抓取/靠近/..."
    " = polish / grind / machine / grasp / pick up / approach) — ignore the action "
    "and focus on WHICH physical object it refers to; use its color and shape to "
    "locate it.\n"
    "Return ONLY a JSON object (no markdown, no extra text) in this exact format:\n"
    '{{"objects": [{{"target": "<object name in English>", "bbox": [x1, y1, x2, y2]}}, ...]}}\n'
    "Each bbox uses NORMALIZED coordinates on a 0-1000 scale: (0,0) is the "
    "top-left corner and (1000,1000) is the bottom-right corner of the image. "
    "(x1,y1) is the top-left and (x2,y2) is the bottom-right of the object.\n"
    "If NO object matches, return: {{\"objects\": []}}"
)


class QwenVLM(BaseVLM):
    def __init__(self, base_url, api_key, model, timeout=30,
                 max_image_size=1024, min_image_size=0, extra_body=None):
        try:
            from openai import OpenAI
        except ImportError as e:  # pragma: no cover
            raise ImportError("缺少 openai 库，请先安装：pip install openai") from e

        self.client = OpenAI(api_key=api_key, base_url=base_url, timeout=timeout)
        self.model = model
        self.max_image_size = max_image_size
        self.min_image_size = min_image_size
        self.extra_body = extra_body or {}

    # ---------- 对外接口 ----------
    def detect(self, image_bgr, instruction):
        h, w = image_bgr.shape[:2]
        img = self._prepare_image(image_bgr)
        b64 = self._encode_image(img)
        prompt = _DETECT_PROMPT.format(instruction=instruction)

        messages = [
            {"role": "system", "content": _DEFAULT_SYSTEM},
            {"role": "user", "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url",
                 "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
            ]},
        ]

        text = self._chat(messages)
        print(f"    [VLM raw] 送入 {img.shape[1]}x{img.shape[0]} | {text[:500]}")
        return self._parse(text, w, h)

    # ---------- 内部实现 ----------
    def _prepare_image(self, image_bgr):
        """把长边规整到 [min_image_size, max_image_size] 之间再送模型。

        小图必须放大：VLM 按固定 patch 网格切图，640x480 送进去时小目标
        （比如 45px 高的三角块）分不到几个 patch，模型会直接返回空 objects。
        放大不会增加真实信息，但能让目标落到足够的 patch 上。
        """
        img = image_bgr
        h, w = img.shape[:2]
        long_side = max(h, w)

        if long_side > self.max_image_size:
            scale = self.max_image_size / long_side
            interp = cv2.INTER_AREA          # 缩小用 AREA，抗锯齿更好
        elif self.min_image_size and long_side < self.min_image_size:
            scale = self.min_image_size / long_side
            interp = cv2.INTER_CUBIC         # 放大用 CUBIC，边缘更平滑
        else:
            return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        img = cv2.resize(img, (int(round(w * scale)), int(round(h * scale))),
                         interpolation=interp)
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    @staticmethod
    def _encode_image(img_rgb):
        # imencode 期望 BGR，这里再转回去；JPEG 比 PNG 更小，省 token 与延迟
        ok, buf = cv2.imencode(".jpg", img_rgb[..., ::-1],
                               [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            raise RuntimeError("图像编码失败")
        return base64.b64encode(buf.tobytes()).decode("ascii")

    def _chat(self, messages):
        resp = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            extra_body=self.extra_body,
        )
        return resp.choices[0].message.content or ""

    def _parse(self, text, w, h):
        raw = text.strip()
        value = self._extract_json(raw)
        dets = []

        if isinstance(value, dict):
            objs = value.get("objects")
            if isinstance(objs, list):
                for item in objs:
                    d = self._item_to_detection(item, w, h)
                    if d is not None:
                        dets.append(d)
            else:
                # 兼容旧格式：单个 target/bbox 放在顶层
                d = self._item_to_detection(value, w, h)
                if d is not None:
                    dets.append(d)
        elif isinstance(value, list):
            # 兼容旧格式：纯 JSON 数组
            for item in value:
                d = self._item_to_detection(item, w, h)
                if d is not None:
                    dets.append(d)

        # 兜底：Qwen grounding token 单框
        if not dets:
            bbox = self._parse_grounding_token(raw, w, h)
            if bbox is not None:
                dets.append(Detection(target=None, bbox=bbox, raw=raw))

        return VisionResult(detections=dets)

    def _item_to_detection(self, item, w, h):
        if not isinstance(item, dict):
            return None
        target = item.get("target") or item.get("label")
        vals = item.get("bbox") or item.get("bbox_2d")
        bbox = self._to_bbox(vals, w, h)
        if target is None and bbox is None:
            return None
        return Detection(target=target, bbox=bbox,
                         raw=json.dumps(item, ensure_ascii=False))

    def _parse_grounding_token(self, raw, w, h):
        m = re.search(
            r"<\|box_start\|>\s*\((\d+),(\d+)\),\((\d+),(\d+)\)\s*<\|box_end\|>",
            raw,
        )
        if not m:
            return None
        nums = [int(v) for v in m.groups()]
        return (self._clamp(nums[0], w), self._clamp(nums[1], h),
                self._clamp(nums[2], w), self._clamp(nums[3], h))

    @staticmethod
    def _extract_json(text):
        """提取第一个平衡的 {...} 或 [...] JSON 值，容忍 markdown 包裹。"""
        text = re.sub(r"```(?:json)?", "", text).strip()
        starts = [i for i in (text.find("{"), text.find("[")) if i != -1]
        if not starts:
            return None
        start = min(starts)
        depth = 0
        for i in range(start, len(text)):
            if text[i] in "{[":
                depth += 1
            elif text[i] in "}]":
                depth -= 1
                if depth == 0:
                    try:
                        return json.loads(text[start:i + 1])
                    except json.JSONDecodeError:
                        return None
        return None

    @staticmethod
    def _to_bbox(vals, w, h):
        if not isinstance(vals, (list, tuple)) or len(vals) != 4:
            return None
        try:
            x1, y1, x2, y2 = [float(v) for v in vals]
        except (TypeError, ValueError):
            return None

        x1, x2 = sorted((x1, x2))
        y1, y2 = sorted((y1, y2))

        # Qwen3-VL 默认输出 0~1000 归一化坐标，转回像素
        if max(abs(x1), abs(y1), abs(x2), abs(y2)) <= 1.5:
            # 少数模型返回 0~1 归一化坐标
            x1, x2 = x1 * w, x2 * w
            y1, y2 = y1 * h, y2 * h
        else:
            x1, x2 = x1 / 1000.0 * w, x2 / 1000.0 * w
            y1, y2 = y1 / 1000.0 * h, y2 / 1000.0 * h

        return (int(round(QwenVLM._clamp(x1, w))), int(round(QwenVLM._clamp(y1, h))),
                int(round(QwenVLM._clamp(x2, w))), int(round(QwenVLM._clamp(y2, h))))

    @staticmethod
    def _clamp(v, hi):
        return max(0, min(hi - 1, v))
