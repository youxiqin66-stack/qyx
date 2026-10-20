#!/usr/bin/env python3
"""ZeroMQ 位姿发布器：把 6D Pose 按你 ros_demo.py::send_pose() 的消息格式发出。

消息字段与 ros_demo.py 保持一致，只发位姿，不含意图/动作信息。

ZeroMQ PUB 绑定 tcp://*:5555，foundationpose_node.py（foundationpose.launch.py 启动）
用 SUB 连 tcp://localhost:5555。

只发位姿，不做任何机械臂执行/确认命令。
"""

import time

import numpy as np


class PosePublisher:
    """PUB 模式发布位姿 JSON。"""

    def __init__(self, address="tcp://*:5555"):
        import zmq
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.PUB)
        self.sock.bind(address)
        time.sleep(1)  # 等订阅者建立连接（与 ros_demo.py 一致）
        self.frame_count = 0
        print(f"[zmq] 位姿发布中: {address}")

    def send(self, pose, K, to_origin, bbox3d, image_shape):
        """发布一帧位姿。pose=4x4, K=3x3, to_origin=4x4, bbox3d=(2,3)。"""
        message = {
            "timestamp": time.time(),
            "frame_id": self.frame_count,
            "pose": np.asarray(pose).tolist(),
            "camera_K": np.asarray(K).tolist(),
            "to_origin": np.asarray(to_origin).tolist(),
            "bbox": np.asarray(bbox3d).tolist(),
            "image_shape": {"height": int(image_shape[0]), "width": int(image_shape[1])},
        }
        self.sock.send_json(message)
        self.frame_count += 1
        return self.frame_count - 1

    def close(self):
        self.sock.close()
        self.ctx.term()
