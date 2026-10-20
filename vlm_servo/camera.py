#!/usr/bin/env python3
"""相机抽象：为阶段一/二提供实时 RGB + 对齐深度 + 相机内参。

阶段一/二直接用 pyrealsense2 取 D455（不装 realsense-ros、不用 cv_bridge，
避免 cv_bridge 与你环境 NumPy 2.x 的 ABI 冲突），且与你 FoundationPose 里
camera_module.py 的取图方式保持一致：
  - rs.align(rs.stream.color)：深度对齐到彩色，二者共用同一像素坐标系
  - 深度 = 原始 z16 * depth_scale，单位【米】
  - 内参：直接用设备彩色流内参（cam_K.txt 与设备内参差别极小，无需读文件）

相机坐标系 = camera_color_optical_frame：+X 右，+Y 下，+Z 前（光轴朝外）。
阶段二输出的 XYZ 就是这个坐标系下的坐标；阶段三再经手眼标定转到机械臂基座。
"""

import threading
import time

import numpy as np


class BaseCamera:
    """所有相机源的统一接口。

    start() 后由后台线程持续抓帧；latest_frame / latest_depth / latest_pair
    始终返回最新帧的拷贝，pair 保证彩色与深度来自同一帧。
    """

    def start(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    @property
    def latest_frame(self):
        """最新 BGR 彩色图 (H, W, 3)。"""
        raise NotImplementedError

    @property
    def latest_depth(self):
        """最新对齐深度图 (H, W)，单位米。"""
        raise NotImplementedError

    def latest_pair(self):
        """同一帧的 (BGR, depth_meters)。"""
        return self.latest_frame, self.latest_depth

    @property
    def intrinsics(self):
        """相机内参 dict：{"fx","fy","cx","cy"}。无内参返回 None。"""
        return None

    @property
    def depth_scale(self):
        return None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()


class RealsenseCamera(BaseCamera):
    """Intel RealSense D455：pyrealsense2 直取彩色 + 对齐深度 + 设备内参。"""

    def __init__(self, width=640, height=480, fps=30):
        import pyrealsense2 as rs  # 延迟导入：没插相机时仍可用 image 模式
        self._rs = rs
        self.width = width
        self.height = height
        self.fps = fps
        self._pipeline = None
        self._config = None
        self._align = None
        self._depth_scale = None
        self._intrinsics = None
        self._lock = threading.Lock()
        self._color = None
        self._depth = None
        self._thread = None
        self._running = False

    def start(self):
        rs = self._rs
        self._pipeline = rs.pipeline()
        self._config = rs.config()
        self._config.enable_stream(
            rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps
        )
        self._config.enable_stream(
            rs.stream.depth, self.width, self.height, rs.format.z16, self.fps
        )
        profile = self._pipeline.start(self._config)

        # 深度对齐到彩色：对齐后的深度与彩色共用同一像素坐标系
        self._align = rs.align(rs.stream.color)

        # 深度尺度：z16 原始值 * scale = 米
        depth_sensor = profile.get_device().first_depth_sensor()
        self._depth_scale = depth_sensor.get_depth_scale()

        # 设备彩色流内参（对齐后深度在彩色像素系，用彩色内参反投影）
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()
        self._intrinsics = {
            "fx": float(intr.fx),
            "fy": float(intr.fy),
            "cx": float(intr.ppx),
            "cy": float(intr.ppy),
        }

        # 预热几十帧，让自动曝光/白平衡稳定，画面不闪
        for _ in range(30):
            self._pipeline.wait_for_frames()

        self._running = True
        self._thread = threading.Thread(
            target=self._poll, name="realsense-poll", daemon=True
        )
        self._thread.start()
        print(
            f"[camera] RealSense D455 已启动: {self.width}x{self.height}@{self.fps}fps "
            f"(depth_scale={self._depth_scale:.5f} m)"
        )

    def _poll(self):
        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
                aligned = self._align.process(frames)
                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                # get_data() 返回的是会被下一帧复用的缓冲区，必须 copy，
                # 否则 self._color 存的是“视图”，显示时会看到错帧/撕裂
                color = np.asanyarray(color_frame.get_data()).copy()  # BGR (H, W, 3)
                depth = (
                    np.asanyarray(depth_frame.get_data()).astype(np.float32)
                    * self._depth_scale
                )  # 米
                with self._lock:
                    self._color = color
                    self._depth = depth
            except Exception as e:  # noqa: BLE001
                if self._running:
                    print(f"[camera] 抓帧异常: {e}")
                time.sleep(0.01)

    @property
    def latest_frame(self):
        with self._lock:
            return None if self._color is None else self._color.copy()

    @property
    def latest_depth(self):
        with self._lock:
            return None if self._depth is None else self._depth.copy()

    def latest_pair(self):
        with self._lock:
            if self._color is None or self._depth is None:
                return None, None
            return self._color.copy(), self._depth.copy()

    @property
    def intrinsics(self):
        return None if self._intrinsics is None else dict(self._intrinsics)

    @property
    def depth_scale(self):
        return self._depth_scale

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:  # noqa: BLE001
                pass
            self._pipeline = None
        print("[camera] RealSense D455 已停止")


class ImageCamera(BaseCamera):
    """离线相机：反复返回同一张静态图 + 合成深度，用于无相机时验证 XYZ 计算。"""

    def __init__(self, image_path, mock_depth_meters=0.6, intrinsics=None):
        import cv2
        img = cv2.imread(image_path)
        if img is None:
            raise FileNotFoundError(f"无法读取测试图片: {image_path}")
        self._frame = img
        # 合成常量深度（米），只为离线验证 bbox->XYZ 的数学是否正确
        self._depth = np.full(img.shape[:2], float(mock_depth_meters), np.float32)

        h, w = img.shape[:2]
        self.width = w
        self.height = h
        if intrinsics:
            self._intrinsics = {k: float(v) for k, v in intrinsics.items()}
        else:
            f = float(max(h, w))
            self._intrinsics = {"fx": f, "fy": f, "cx": w / 2.0, "cy": h / 2.0}
        self._depth_scale = 1.0

        print(
            f"[camera] 使用静态图片: {image_path} ({w}x{h}), "
            f"合成深度 {mock_depth_meters} m"
        )

    def start(self):
        pass

    def stop(self):
        pass

    @property
    def latest_frame(self):
        return self._frame.copy()

    @property
    def latest_depth(self):
        return self._depth.copy()

    def latest_pair(self):
        return self._frame.copy(), self._depth.copy()

    @property
    def intrinsics(self):
        return dict(self._intrinsics)

    @property
    def depth_scale(self):
        return self._depth_scale


def build_camera(cfg):
    """根据 config.yaml 的 camera 段创建相机实例。"""
    cam_type = (cfg.get("type") or "realsense").lower()
    if cam_type == "realsense":
        return RealsenseCamera(
            width=int(cfg.get("width", 1280)),
            height=int(cfg.get("height", 720)),
            fps=int(cfg.get("fps", 30)),
        )
    if cam_type == "image":
        return ImageCamera(
            cfg["image_path"],
            mock_depth_meters=float(cfg.get("mock_depth_meters", 0.6)),
            intrinsics=cfg.get("intrinsics"),
        )
    raise ValueError(f"未知的相机类型: {cam_type}（可选: realsense / image）")
