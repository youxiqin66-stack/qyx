#!/usr/bin/env python3
"""OpenCV 画框与画面叠加（阶段一画 bbox，阶段二叠加 XYZ）。

注意：cv2.putText 用 Hershey 字体，只能画 ASCII，画不了中文（会显示成方块）。
所以中文指令/结果打印到终端；画面文字只用英文目标名 + 数字坐标 + XYZ 数值。
"""

import time

import cv2

# 多目标时循环使用的框颜色（BGR）
_PALETTE = [
    (0, 255, 0),    # 绿
    (0, 165, 255),  # 橙
    (255, 0, 255),  # 品红
    (0, 255, 255),  # 黄
    (255, 0, 0),    # 蓝
]


def _safe_ascii(s, fallback="object"):
    """把可能含中文的标签转成 cv2 能画的 ASCII；含非 ASCII 就退回 fallback。"""
    s = str(s or "").strip()
    if not s or any(ord(c) > 127 for c in s):
        return fallback
    return s


def draw_detections(frame_bgr, result, box_color=(0, 255, 0),
                    font_scale=0.7, thickness=2, xyz_list=None):
    """把 VisionResult.detections 的 bbox 画到 frame 上（原地修改）。

    bbox 为像素坐标 (x1, y1, x2, y2)。xyz_list 是与 detections 对齐的
    相机系 XYZ 列表（可含 None），用于阶段二在框旁显示 3D 坐标。
    """
    dets = result.detections or []
    for i, det in enumerate(dets):
        if not det.found:
            continue
        x1, y1, x2, y2 = det.bbox
        color = box_color if len(dets) == 1 else _PALETTE[i % len(_PALETTE)]

        # 1) 画矩形框
        cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), color, thickness)

        # 2) 标签：目标名(英文) + bbox 坐标 [+ XYZ]
        target = _safe_ascii(det.target)
        text = f"{target}  [{x1},{y1},{x2},{y2}]"
        if xyz_list is not None and i < len(xyz_list) and xyz_list[i] is not None:
            x, y, z = xyz_list[i]
            text += f"  XYZ=({x:.3f},{y:.3f},{z:.3f})m"

        (tw, th), _ = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness
        )
        ty = y1 - 8
        if ty < th + 4:
            ty = y1 + th + 8
        cv2.rectangle(frame_bgr, (x1, ty - th - 4), (x1 + tw + 4, ty + 4), color, -1)
        cv2.putText(frame_bgr, text, (x1 + 2, ty), cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, (0, 0, 0), thickness, cv2.LINE_AA)
    return frame_bgr


def draw_hud(frame_bgr, status="", font_scale=0.55, thickness=1):
    """顶部信息条（只用 ASCII）：状态 + FPS。中文信息在终端打印。"""
    h = 26
    cv2.rectangle(frame_bgr, (0, 0), (frame_bgr.shape[1], h), (40, 40, 40), -1)
    if status:
        cv2.putText(frame_bgr, status, (8, h - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
    return frame_bgr


class FpsCounter:
    """粗略 FPS 统计：每秒刷新一次。"""

    def __init__(self):
        self._t0 = time.time()
        self._n = 0
        self.fps = 0.0

    def tick(self):
        self._n += 1
        now = time.time()
        dt = now - self._t0
        if dt >= 1.0:
            self.fps = self._n / dt
            self._n = 0
            self._t0 = now
        return self.fps
