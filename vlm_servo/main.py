#!/usr/bin/env python3
"""阶段一 + 阶段二入口。

阶段一：RealSense D455 -> 实时 RGB -> 终端自然语言 -> VLM -> bbox -> 画框
阶段二：bbox + D455 对齐深度 + 内参 -> 相机系 3D XYZ

相机坐标系 = camera_color_optical_frame（+X 右 +Y 下 +Z 前）。
阶段三再把这里的 XYZ 经手眼标定转到机械臂基座（本阶段不乘任何矩阵）。

流程（三个线程）：
  1. 后台线程：pyrealsense2 持续抓取彩色 + 对齐深度
  2. 主线程  ：OpenCV 实时显示，叠加最近一次 bbox 与 XYZ
  3. 输入线程：终端输入自然语言 -> 取同一帧(color+depth) -> VLM -> bbox -> XYZ

退出：终端输入 q / quit / exit，或显示窗口内按 q。
"""

import sys
import threading

import cv2

from config import load_config
from camera import build_camera
from depth3d import bbox_to_xyz
from visualizer import draw_detections, draw_hud, FpsCounter
from vlm import build_vlm


def _fmt_xyz(xyz):
    if xyz is None:
        return "XYZ = N/A"
    return f"XYZ = ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) m"


def main():
    cfg = load_config()

    camera = build_camera(cfg["camera"])
    vlm = build_vlm(cfg["vlm"])
    disp = cfg.get("display", {})
    depth_cfg = cfg.get("depth", {})

    win_name = disp.get("window_name", "VLM Servo - Phase 2")
    font_scale = float(disp.get("font_scale", 0.7))
    thickness = int(disp.get("line_thickness", 2))
    box_color = tuple(int(c) for c in disp.get("box_color", [0, 255, 0]))
    show_fps = bool(disp.get("show_fps", True))

    min_depth = float(depth_cfg.get("min_depth", 0.1))
    max_depth = float(depth_cfg.get("max_depth", 3.0))
    center_ratio = float(depth_cfg.get("center_ratio", 0.5))

    try:
        camera.start()
    except Exception as e:  # noqa: BLE001
        print(f"[!] 相机启动失败: {e}")
        print("    请确认 D455 已连接；或临时把 config.yaml 里 camera.type 改成 image 测试。")
        sys.exit(1)

    K = camera.intrinsics
    if K is None:
        print("[!] 相机无内参，无法计算 XYZ（仍可画 bbox）")
    else:
        print(
            f"[camera] 内参 fx={K['fx']:.1f} fy={K['fy']:.1f} "
            f"cx={K['cx']:.1f} cy={K['cy']:.1f}"
        )

    # 共享状态（输入线程写，显示线程读）
    state = {"result": None, "xyz_list": None,
             "status": "IDLE - type command in terminal"}
    state_lock = threading.Lock()
    stop_event = threading.Event()

    def input_worker():
        print("\n===== VLM Servo 阶段一+二 =====")
        print("在【终端】输入自然语言指令，例如：抓蓝色的笔 / 找到红色的瓶子")
        print("输入 q / quit / exit 退出；显示窗口内按 q 也可退出。\n")
        while not stop_event.is_set():
            try:
                instruction = input(">> ").strip()
            except EOFError:
                break
            if not instruction:
                continue
            if instruction.lower() in ("q", "quit", "exit", "退出"):
                stop_event.set()
                break

            # 取同一帧的彩色 + 对齐深度，保证 bbox 与深度像素对齐
            color, depth = camera.latest_pair()
            if color is None:
                print("[!] 还没拿到相机画面，请确认 D455 已连接并稍候重试")
                continue

            with state_lock:
                state["status"] = "DETECTING..."
            print(f"[*] 指令: {instruction}  (识别中...)")

            try:
                result = vlm.detect(color, instruction)
            except Exception as e:  # noqa: BLE001
                print(f"[!] VLM 识别失败: {e}")
                with state_lock:
                    state["status"] = "ERROR"
                continue

            dets = result.detections or []

            # 阶段二：对每个 bbox 反投影相机系 XYZ
            xyz_list = []
            for det in dets:
                if det.found and depth is not None and K is not None:
                    xyz, info = bbox_to_xyz(
                        det.bbox, depth, K,
                        min_depth=min_depth, max_depth=max_depth,
                        center_ratio=center_ratio,
                    )
                    if xyz is None:
                        print(f"    [!] {det.target}: {info.get('error', '深度失败')}")
                    xyz_list.append(xyz)
                else:
                    xyz_list.append(None)

            with state_lock:
                state["result"] = result
                state["xyz_list"] = xyz_list

            if not dets:
                print("    -> 未找到匹配目标")
                with state_lock:
                    state["status"] = "NOT FOUND"
                continue

            with state_lock:
                state["status"] = "FOUND: %d" % len(dets)
            for i, det in enumerate(dets):
                if det.found:
                    print(f"    -> target = {det.target}, "
                          f"bbox = {list(det.bbox)}, "
                          f"{_fmt_xyz(xyz_list[i] if i < len(xyz_list) else None)}")
                else:
                    print(f"    -> 目标 {det.target} 但无 bbox")

    t = threading.Thread(target=input_worker, name="input", daemon=True)
    t.start()

    fps = FpsCounter()
    # AUTOSIZE：窗口紧贴图像尺寸，避免四周留白边
    cv2.namedWindow(win_name, cv2.WINDOW_AUTOSIZE)

    print("\n[display] 显示窗口已打开，在终端输入指令，窗口内按 q 退出。")
    try:
        while not stop_event.is_set():
            frame = camera.latest_frame
            if frame is None:
                cv2.waitKey(30)
                continue

            with state_lock:
                result = state["result"]
                xyz_list = state["xyz_list"]
                status = state["status"]

            if result is not None:
                draw_detections(frame, result, box_color=box_color,
                                font_scale=font_scale, thickness=thickness,
                                xyz_list=xyz_list)
            if show_fps:
                status = f"{status}  {fps.tick():.1f} FPS"
            draw_hud(frame, status=status)

            cv2.imshow(win_name, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                stop_event.set()
                break
    finally:
        stop_event.set()
        cv2.destroyAllWindows()
        camera.stop()
        print("\n[exit] 已退出。")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[exit] 收到 Ctrl+C，退出。")
        sys.exit(0)
