#!/usr/bin/env python3
"""VLM 工厂：根据配置创建具体的 VLM 实现。

新增模型时：写一个继承 BaseVLM 的类，然后在这里的 build_vlm() 里加一个分支。
"""

import os

from .base import BaseVLM, Detection, VisionResult
from .mock_vlm import MockVLM
from .qwen_vl import QwenVLM

__all__ = ["BaseVLM", "Detection", "VisionResult", "build_vlm"]


def build_vlm(cfg) -> BaseVLM:
    """cfg 为 config.yaml 里的 vlm 段（dict）。"""
    vlm_type = (cfg.get("type") or "qwen").lower()

    if vlm_type == "mock":
        return MockVLM(bboxes=cfg.get("mock_bboxes", [[220, 160, 420, 360]]))

    if vlm_type == "qwen":
        key_env = cfg.get("api_key_env", "DASHSCOPE_API_KEY")
        api_key = os.environ.get(key_env)
        if not api_key:
            raise RuntimeError(
                f"未找到 API Key：请先设置环境变量 {key_env}，"
                f"例如 export {key_env}=你的密钥"
            )
        return QwenVLM(
            base_url=cfg.get(
                "base_url", "https://dashscope.aliyuncs.com/compatible-mode/v1"
            ),
            api_key=api_key,
            model=cfg.get("model", "qwen3-vl-plus"),
            timeout=cfg.get("timeout", 30),
            max_image_size=cfg.get("max_image_size", 1024),
            min_image_size=cfg.get("min_image_size", 0),
            extra_body=cfg.get("extra_body", {}),
        )

    raise ValueError(f"未知的 VLM 类型: {vlm_type}（可选: qwen / mock）")
