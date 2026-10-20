#!/usr/bin/env python3
"""FoundationPose 封装（阶段三核心）。不修改你 FoundationPose/ 里任何代码。

做法：把 FoundationPose 目录临时加入 sys.path 后 import estimater/Utils，
封装成 PoseEstimator：register() / track() / draw()，屏蔽 4x4 位姿、to_origin、
内参矩阵转换等细节。

位姿约定（与你 ros_demo.py / POSE_TRANSFORM_GUIDE.md 一致）：
  pose 为 4x4，物体「原始建模坐标系」在相机坐标系下的位姿，单位米，
  父坐标系 = camera_color_optical_frame。
"""

import logging
import os
import sys

import numpy as np

FP_DIR = os.environ.get("FOUNDATIONPOSE_DIR", "/home/qyx/LLM/code/FoundationPose")


def _load_fp():
    """把 FoundationPose 目录加入 sys.path 并返回需要的符号（惰性，避免启动即加载）。"""
    if FP_DIR not in sys.path:
        sys.path.insert(0, FP_DIR)
    from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
    from Utils import draw_posed_3d_box, draw_xyz_axis, set_logging_format, set_seed
    return (FoundationPose, ScorePredictor, PoseRefinePredictor,
            draw_posed_3d_box, draw_xyz_axis, set_logging_format, set_seed)


def K_dict_to_matrix(K):
    """内参 dict {fx,fy,cx,cy} -> 3x3 numpy（FoundationPose 需要矩阵形式）。"""
    return np.array(
        [[K["fx"], 0.0, K["cx"]],
         [0.0, K["fy"], K["cy"]],
         [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def match_mesh(target, meshes, default_mesh, fp_dir):
    """把 VLM 目标名匹配到网格文件（返回绝对路径）。

    meshes 是 {关键词: 相对fp_dir的路径}。匹配规则（大小写不敏感）：
      1) 关键词是目标名的子串（"bottle" in "red bottle"）；
      2) 关键词与目标名里某个单词共享 >=4 字符前缀
         （"triangle" 与 "triangular block" 里的 "triangular" 共享 "triang"）。
    目标名可能是英文（Qwen 输出）或中文（mock）。匹配不到就用 default_mesh。
    """
    t = (target or "").lower()
    t_words = t.split()

    def _matches(kw):
        kw = kw.lower()
        if not kw:
            return False
        if kw in t:
            return True
        for kw_w in kw.split():
            for t_w in t_words:
                n = min(len(kw_w), len(t_w))
                if n >= 4 and kw_w[:n] == t_w[:n]:
                    return True
        return False

    for kw, rel in (meshes or {}).items():
        if _matches(kw):
            return os.path.join(fp_dir, rel)
    if default_mesh:
        return os.path.join(fp_dir, default_mesh)
    return None


class PoseEstimator:
    """单个物体网格的 FoundationPose 估计器（首帧 register + 后续 track）。"""

    def __init__(self, mesh_file, est_refine_iter=5, track_refine_iter=2,
                 debug=0, debug_dir=None):
        (FoundationPose, ScorePredictor, PoseRefinePredictor,
         draw_posed_3d_box, draw_xyz_axis, set_logging_format, set_seed) = _load_fp()

        set_logging_format(level=logging.WARNING)
        set_seed(0)
        self._draw_posed_3d_box = draw_posed_3d_box
        self._draw_xyz_axis = draw_xyz_axis

        import trimesh
        self.mesh = trimesh.load(mesh_file)
        if self.mesh.scale > 1:  # 模型单位是毫米 -> 米（与你 run_demo_prb.py 一致）
            self.mesh.apply_scale(0.001)

        # 可视化用的 OBB：物体建模坐标系 -> OBB 中心坐标系
        self.to_origin, extents = trimesh.bounds.oriented_bounds(self.mesh)
        self.bbox3d = np.stack([-extents / 2, extents / 2], axis=0).reshape(2, 3)

        scorer = ScorePredictor()
        refiner = PoseRefinePredictor()
        import nvdiffrast.torch as dr
        glctx = dr.RasterizeCudaContext()

        self.est = FoundationPose(
            model_pts=self.mesh.vertices,
            model_normals=self.mesh.vertex_normals,
            mesh=self.mesh,
            scorer=scorer,
            refiner=refiner,
            debug_dir=debug_dir or os.path.join(FP_DIR, "debug"),
            debug=debug,
            glctx=glctx,
        )
        self.est_refine_iter = est_refine_iter
        self.track_refine_iter = track_refine_iter
        print(f"[fp] FoundationPose 估计器就绪: {mesh_file}")

    def register(self, rgb, depth, K, mask):
        """首帧配准。rgb=BGR, depth=米, K=3x3, mask=bool(H,W)。返回 4x4 pose。"""
        return self.est.register(
            K=K, rgb=rgb, depth=depth, ob_mask=mask, iteration=self.est_refine_iter
        )

    def track(self, rgb, depth, K):
        """后续帧跟踪。返回 4x4 pose。"""
        return self.est.track_one(
            rgb=rgb, depth=depth, K=K, iteration=self.track_refine_iter
        )

    def draw(self, img_bgr, pose, K, axis_scale=0.1):
        """在 BGR 图上叠加 3D 框 + 坐标轴，返回 BGR 图。"""
        center_pose = pose @ np.linalg.inv(self.to_origin)
        vis = self._draw_posed_3d_box(
            K, img=img_bgr, ob_in_cam=center_pose, bbox=self.bbox3d
        )
        vis = self._draw_xyz_axis(
            vis, ob_in_cam=center_pose, scale=axis_scale, K=K,
            thickness=3, transparency=0, is_input_rgb=False,
        )
        return vis
