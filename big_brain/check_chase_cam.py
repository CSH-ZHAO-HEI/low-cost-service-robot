#!/usr/bin/env python3
"""
check_chase_cam.py — 啟動後快速健檢

驗證兩件事：
  1. VLM camera topic 有在發布（judge camera plugin 跑起來了）
  2. image/image.png 在最近 2 秒內被更新（vlm_image_writer.py 跑起來了）

用法：
  python3 check_chase_cam.py [topic]

退出碼：
  0 = 都 OK
  1 = topic 沒人發
  2 = image.png 沒在更新
  3 = ROS master 沒接上
"""
import os
import sys
import time

import rospy
from sensor_msgs.msg import Image

TOPIC      = sys.argv[1] if len(sys.argv) > 1 else "/judge_camera/rgb/image_raw"
IMAGE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "image", "image.png")
WAIT_SEC   = 5.0      # 等 topic 多久才放棄
STALE_SEC  = 3.0      # image.png mtime 距現在超過這個算 stale

_msg_count = 0
_last_msg_t = None
_last_dims = None


def _cb(msg):
    global _msg_count, _last_msg_t, _last_dims
    _msg_count += 1
    _last_msg_t = time.time()
    _last_dims = (msg.width, msg.height, msg.encoding)


def check_topic() -> bool:
    """訂閱 TOPIC 等 WAIT_SEC 秒，看有沒有訊息進來。"""
    print(f"[1/2] 檢查 topic {TOPIC} ...")
    rospy.Subscriber(TOPIC, Image, _cb, queue_size=1)
    deadline = time.time() + WAIT_SEC
    while time.time() < deadline and _msg_count == 0:
        time.sleep(0.1)

    if _msg_count == 0:
        print(f"  ✗ {WAIT_SEC:.0f}s 內沒收到任何訊息")
        print(f"    → 確認 Gazebo 已重啟（URDF 改了必須重啟）")
        print(f"    → 檢查：rostopic list | grep -E 'judge_camera|chase_cam'")
        return False

    rate = _msg_count / WAIT_SEC
    w, h, enc = _last_dims
    print(f"  ✓ 收到 {_msg_count} 張，~{rate:.1f} Hz，{w}x{h} {enc}")
    return True


def check_image_file() -> bool:
    """檢查 image.png 存在且 mtime 是新的。"""
    print(f"[2/2] 檢查 {IMAGE_PATH} ...")
    if not os.path.exists(IMAGE_PATH):
        print(f"  ✗ 檔案不存在")
        print(f"    → 確認 vlm_image_writer.py 已啟動")
        return False

    age = time.time() - os.path.getmtime(IMAGE_PATH)
    size_kb = os.path.getsize(IMAGE_PATH) / 1024.0

    if age > STALE_SEC:
        print(f"  ✗ 檔案最後更新 {age:.1f}s 前（> {STALE_SEC}s 視為 stale）")
        print(f"    → vlm_image_writer.py 可能掛了，或沒在收 chase_cam 訊息")
        return False

    print(f"  ✓ 最後更新 {age:.1f}s 前，{size_kb:.0f} KB")
    return True


def main():
    print("=" * 50)
    print("  VLM camera + vlm_image_writer 健檢")
    print("=" * 50)

    try:
        rospy.init_node("check_chase_cam", anonymous=True, disable_signals=True)
    except Exception as e:
        print(f"✗ 無法連 ROS master：{e}")
        print(f"  → 確認 roscore / Gazebo 在跑")
        sys.exit(3)

    topic_ok = check_topic()
    image_ok = check_image_file()

    print("=" * 50)
    if topic_ok and image_ok:
        print("  ✓ 全部通過 — Judge 可以正常用 VLM")
        sys.exit(0)
    elif not topic_ok:
        sys.exit(1)
    else:
        sys.exit(2)


if __name__ == "__main__":
    main()
