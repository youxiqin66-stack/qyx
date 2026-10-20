#!/usr/bin/env python3
"""读取 config.yaml 并返回一个 dict。

所有模块都通过 build_camera() / build_vlm() 拿到自己需要的配置段，
运行期不直接 import 本文件，方便后续换配置来源（如 ROS2 参数）。
"""

import os

import yaml

# config.yaml 与本文件同目录
DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "config.yaml"
)


def load_config(path=None):
    """加载配置。path 为空时用默认的 config.yaml。"""
    path = path or DEFAULT_CONFIG_PATH
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"找不到配置文件: {path}\n"
            "请确认在 vlm_servo 目录下运行，或显式传入 --config 路径。"
        )
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    return cfg
