#!/usr/bin/env python3
"""自然语言指令归一化：把「孔加工」类说法映射到红瓶目标。

为什么需要这层：
    当前视觉链路只拿指令去「定位物体」（VLM bbox -> FoundationPose），
    不再做动作/意图路由。用户说「打磨孔 / 钻孔 / 加工孔」时，字面目标是
    「孔」，而场景里要跟随的其实是红瓶（瓶口就是那个孔）。如果把原话
    直接送给 VLM，模型会去找孔洞的像素区域，bbox 往往只框住瓶口甚至
    返回空结果，FoundationPose 首帧注册就会失败或跟着漂。

    因此送 VLM 之前先做一次确定性改写：只要指令里出现孔加工相关说法，
    就统一改写为「找到红色瓶子」，并把目标名固定成 "red bottle"，
    保证网格匹配（bottle_red_centered.obj）不会跑偏。
"""

# 孔加工相关说法。保留裸「孔」作为兜底：本实验场景里不存在别的孔，
# 只要提到孔就应当跟随红瓶。
HOLE_KEYWORDS = (
    "打磨孔", "磨孔", "钻孔", "打孔", "钻削", "镗孔", "铰孔", "锪孔",
    "攻丝", "加工孔", "孔加工", "孔位", "孔口", "孔洞", "瓶口", "孔",
)

# 改写后送给 VLM 的指令（越直白越容易框准）
RED_BOTTLE_INSTRUCTION = "找到画面中的红色瓶子"

# 固定目标名：与 config.yaml 里 foundationpose.meshes 的 "red bottle" 对应
RED_BOTTLE_TARGET = "red bottle"


def normalize_instruction(text):
    """返回 (送给 VLM 的指令, 需要强制设置的目标名或 None)。

    >>> normalize_instruction("打磨孔")
    ('找到画面中的红色瓶子', 'red bottle')
    >>> normalize_instruction("找到三角形")
    ('找到三角形', None)
    """
    text = (text or "").strip()
    if any(kw in text for kw in HOLE_KEYWORDS):
        return RED_BOTTLE_INSTRUCTION, RED_BOTTLE_TARGET
    return text, None
