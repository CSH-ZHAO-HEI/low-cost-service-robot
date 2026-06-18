#!/usr/bin/env python3
"""
獨立測 VLM 對「卡在 CoffeeTable」場景的判讀能力。
不需要 ROS，不跑 Gazebo。只調 VLM API 比對三種 prompt 變體。

用法：
  python3 vlm_capability_test.py [image_path]
  預設讀當前 image/image.jpg + image/trash_inside.jpg
"""
import os
import sys
import json
import base64

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from openai import OpenAI
from config import VLM_API_KEY, VLM_API_BASE_URL, VLM_MODEL
from prompt.vlm_prompt import VLM_SYSTEM_PROMPT


def encode(path):
    if not os.path.exists(path):
        return ""
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")


def call_vlm(system_prompt, user_text, image_b64_main, image_b64_trash):
    client = OpenAI(api_key=VLM_API_KEY, base_url=VLM_API_BASE_URL)
    content = [{"type": "text", "text": user_text}]
    if image_b64_main:
        content.append({"type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{image_b64_main}"}})
    if image_b64_trash:
        content.append({"type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{image_b64_trash}"}})

    resp = client.chat.completions.create(
        model=VLM_MODEL,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": content},
        ],
        temperature=0.1,
        top_p=0.9,
    )
    return resp.choices[0].message.content


def main():
    img_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "image", "image.jpg")
    trash_path = os.path.join(HERE, "image", "trash_inside.jpg")

    print(f"[test] main image: {img_path}")
    print(f"[test] trash image: {trash_path}")
    print(f"[test] using VLM model: {VLM_MODEL}\n")

    b64_main = encode(img_path)
    b64_trash = encode(trash_path)

    if not b64_main:
        print(f"[ERROR] main image not found at {img_path}")
        return 1

    # ── Test A: 模擬實際 Judge 流程（含 rule_check 暗示）─────────────
    print("=" * 70)
    print("Test A: 含 rule_check 暗示（模擬實際 flow）")
    print("=" * 70)
    user_a = """
User Input:
Global Instruction: "Pick all ground blocks red/blue/yellow and put them into Trash_01_001"
Current Action: Move to yellow_block with offset (0, 0)
Observation: {'robot_x': 2.10, 'robot_y': -1.53, 'robot_orientation': -0.73, 'holding': False}
Rule Check Result: {'pass': False, 'failure_code': 'localization_error', 'error_m': 4.65, 'threshold_m': 0.35}
"""
    out_a = call_vlm(VLM_SYSTEM_PROMPT, user_a, b64_main, b64_trash)
    print(out_a)
    print()

    # ── Test B: 同 prompt，但拿掉 rule_check 暗示 ───────────────────
    print("=" * 70)
    print("Test B: 拿掉 rule_check 暗示（純視覺判斷）")
    print("=" * 70)
    user_b = """
User Input:
Global Instruction: "Pick all ground blocks red/blue/yellow and put them into Trash_01_001"
Current Action: Move to yellow_block with offset (0, 0)
Observation: {'robot_x': 2.10, 'robot_y': -1.53, 'robot_orientation': -0.73, 'holding': False}
Rule Check Result: (omitted — judge by image only)
"""
    out_b = call_vlm(VLM_SYSTEM_PROMPT, user_b, b64_main, b64_trash)
    print(out_b)
    print()

    # ── Test C: 直白問「車卡哪了」─────────────────────────────────
    print("=" * 70)
    print("Test C: 直白問「車是否卡在家具上」（測 VLM 視覺上限）")
    print("=" * 70)
    sys_c = """You are a careful visual inspector. Look at the side-view image of a
mobile robot in a Gazebo simulation. Describe in detail:
1. Where is the robot in the scene relative to nearby furniture?
2. Is the robot's chassis, base, or any wheel visibly overlapping, touching,
   or wedged against any static obstacle (table, sofa, wall, bin, etc.)?
3. If yes, name the obstacle as specifically as you can.
4. Is the robot's arm raised or lowered?
Answer in plain English, 4-6 sentences. Do not output JSON."""
    user_c = "Please inspect this image."
    out_c = call_vlm(sys_c, user_c, b64_main, "")  # 只給主視角
    print(out_c)
    print()

    print("=" * 70)
    print("[summary]")
    print("=" * 70)
    print("Compare error_type / reason in A vs B → 看 rule_check 是否干擾判斷")
    print("Read C 的描述 → 看 VLM 本身能不能識別物理重疊")


if __name__ == "__main__":
    sys.exit(main() or 0)
