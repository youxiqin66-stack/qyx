#!/usr/bin/env python3
"""把 VLM 的 2D bbox 转成 FoundationPose register() 需要的物体掩码（阶段三）。

两种模式：
  - rect：直接把 bbox 内部标为 True。最快、零额外显存/依赖，适合瓶子/三角等
          “饱满”物体；细长物体（笔）会带入较多背景，配准时可用 sam 更准。
  - sam ：用 segment_anything 以 bbox 为 prompt 生成精细掩码（更准，占显存，
          首次加载 SAM 模型较慢）。
"""

import numpy as np


def mask_from_bbox_rect(shape_hw, bbox):
    """矩形掩码：bbox 内部为 True。shape_hw=(H, W)。"""
    h, w = shape_hw
    x1, y1, x2, y2 = [int(v) for v in bbox]
    x1, x2 = max(0, x1), min(w, x2)
    y1, y2 = max(0, y1), min(h, y2)
    mask = np.zeros((h, w), dtype=bool)
    if x1 < x2 and y1 < y2:
        mask[y1:y2, x1:x2] = True
    return mask


# SAM 预测器缓存（同一 checkpoint 只加载一次）
_sam_predictor = None
_sam_checkpoint = None


def mask_from_bbox_sam(color_bgr, bbox, checkpoint):
    """SAM 以 bbox 为 prompt 分割。color_bgr: (H,W,3) BGR。"""
    global _sam_predictor, _sam_checkpoint
    if _sam_predictor is None or _sam_checkpoint != checkpoint:
        from segment_anything import sam_model_registry, SamPredictor
        sam = sam_model_registry["vit_b"](checkpoint=checkpoint)
        _sam_predictor = SamPredictor(sam)
        _sam_checkpoint = checkpoint
        print(f"[mask] SAM 模型已加载: {checkpoint}")

    import cv2
    rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
    _sam_predictor.set_image(rgb)
    masks, _, _ = _sam_predictor.predict(
        box=np.array(bbox, dtype=np.float64), multimask_output=False
    )
    return masks[0].astype(bool)


def mask_from_bbox(color_bgr, bbox, mode="rect", sam_checkpoint=None):
    """按 mode 生成掩码。bbox=(x1,y1,x2,y2) 像素坐标。"""
    if mode == "sam":
        if not sam_checkpoint:
            raise ValueError("sam 模式需要提供 sam_checkpoint 路径")
        return mask_from_bbox_sam(color_bgr, bbox, sam_checkpoint)
    return mask_from_bbox_rect(color_bgr.shape[:2], bbox)
