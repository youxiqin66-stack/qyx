#!/usr/bin/env python3
"""把 VLM 的 2D bbox + 对齐深度 + 相机内参 转成相机系 3D XYZ（阶段二核心）。

针孔模型反投影（深度已对齐到彩色，共用同一像素坐标系）：
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    Z = depth                        （米）

抗噪策略：不在单个像素取深度，而在 bbox 中心子区域取【有效深度的中位数】，
再用 bbox 中心像素反投影，避免框边缘的背景/遮挡污染。

相机坐标系 = camera_color_optical_frame（+X 右 +Y 下 +Z 前）。
"""

import numpy as np

DEFAULT_MIN_DEPTH = 0.1   # 米
DEFAULT_MAX_DEPTH = 3.0   # 米


def pixel_to_xyz(u, v, z, K):
    """单个像素 (u, v) + 深度 z(米) -> 相机系 XYZ(米)。K = {fx,fy,cx,cy}。"""
    x = (float(u) - K["cx"]) * z / K["fx"]
    y = (float(v) - K["cy"]) * z / K["fy"]
    return (float(x), float(y), float(z))


def bbox_center(bbox):
    x1, y1, x2, y2 = bbox
    return (int(round((x1 + x2) / 2.0)), int(round((y1 + y2) / 2.0)))


def bbox_to_xyz(bbox, depth, K, min_depth=DEFAULT_MIN_DEPTH,
                max_depth=DEFAULT_MAX_DEPTH, center_ratio=0.5):
    """bbox + 对齐深度 + 内参 -> 相机系 XYZ。

    :param bbox: 像素坐标 (x1, y1, x2, y2)
    :param depth: 对齐深度图 (H, W)，单位米
    :param K: 内参 {"fx","fy","cx","cy"}
    :param center_ratio: 取 bbox 中心多大比例区域做深度统计（0~1）
    :return: (xyz, info)。xyz=(X,Y,Z) 米；失败时 xyz=None，info 含失败原因。
    """
    h, w = depth.shape[:2]
    x1, y1, x2, y2 = [int(v) for v in bbox]
    x1, x2 = max(0, min(x1, w - 1)), max(0, min(x2, w - 1))
    y1, y2 = max(0, min(y1, h - 1)), max(0, min(y2, h - 1))
    if x1 >= x2 or y1 >= y2:
        return None, {"error": "bbox 无效"}

    # 中心子区域，避开框边缘
    cw, ch = x2 - x1, y2 - y1
    pad_x = int(cw * (1 - center_ratio) / 2)
    pad_y = int(ch * (1 - center_ratio) / 2)
    cx0, cx1 = max(x1, x1 + pad_x), min(x2, x2 - pad_x)
    cy0, cy1 = max(y1, y1 + pad_y), min(y2, y2 - pad_y)

    region = depth[cy0:cy1, cx0:cx1]
    d = np.asarray(region, dtype=np.float32)
    mask = (d > min_depth) & (d < max_depth) & np.isfinite(d)
    n_valid = int(mask.sum())
    if n_valid == 0:
        return None, {"error": "框内无有效深度", "n_valid": 0}

    z = float(np.median(d[mask]))
    cu, cv = bbox_center((x1, y1, x2, y2))
    xyz = pixel_to_xyz(cu, cv, z, K)
    return xyz, {"n_valid": n_valid, "median_depth": z, "center": (cu, cv)}
