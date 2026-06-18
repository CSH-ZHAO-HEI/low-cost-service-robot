#!/usr/bin/env python3
"""
vlm_image_writer.py — 把車載 judge camera + trash 內視鏡頭持續寫成 image/*.jpg

JudgeLLM.judge_VLM() 讀以下兩張圖作為當前場景視覺證據：
  - image/image.jpg        ← 車載 judge_camera 側拍
  - image/trash_inside.jpg ← Trash_01_001 上方俯拍桶內

本節點同時訂閱兩個 topic（trash 沒接時自動跳過，不影響主圖）。

啟動：
  python3 vlm_image_writer.py [main_topic] [trash_topic]
  預設：
    main_topic  = /judge_camera/rgb/image_raw
    trash_topic = /trash_inside/cam/image_raw

依賴：cv_bridge 不需要（手動 numpy reshape 解碼）
"""
import os
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image

HERE = os.path.dirname(os.path.abspath(__file__))
IMG_DIR = os.path.join(HERE, "image")

MAIN_TOPIC  = sys.argv[1] if len(sys.argv) > 1 else "/judge_camera/rgb/image_raw"
TRASH_TOPIC = sys.argv[2] if len(sys.argv) > 2 else "/trash_inside/cam/image_raw"

MAIN_OUT  = os.path.join(IMG_DIR, "image.jpg")
TRASH_OUT = os.path.join(IMG_DIR, "trash_inside.jpg")

_MIN_WRITE_INTERVAL = 0.2  # 最快 5Hz 寫檔
_last_write_main  = 0.0
_last_write_trash = 0.0


def _decode_msg(msg) -> np.ndarray:
    """ROS sensor_msgs/Image → BGR ndarray (None if encoding unsupported)."""
    if msg.encoding not in ("rgb8", "bgr8"):
        rospy.logwarn_throttle(5.0, f"[vlm_image_writer] unexpected encoding: {msg.encoding}")
        return None
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else arr


def cb_main(msg):
    global _last_write_main
    now = rospy.Time.now().to_sec()
    if now - _last_write_main < _MIN_WRITE_INTERVAL:
        return
    bgr = _decode_msg(msg)
    if bgr is None:
        return
    cv2.imwrite(MAIN_OUT, bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
    _last_write_main = now


def cb_trash(msg):
    global _last_write_trash
    now = rospy.Time.now().to_sec()
    if now - _last_write_trash < _MIN_WRITE_INTERVAL:
        return
    bgr = _decode_msg(msg)
    if bgr is None:
        return
    cv2.imwrite(TRASH_OUT, bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
    _last_write_trash = now


def main():
    rospy.init_node("vlm_image_writer", anonymous=True)
    os.makedirs(IMG_DIR, exist_ok=True)
    rospy.Subscriber(MAIN_TOPIC,  Image, cb_main,  queue_size=1, buff_size=2**24)
    rospy.Subscriber(TRASH_TOPIC, Image, cb_trash, queue_size=1, buff_size=2**24)
    rospy.loginfo(f"[vlm_image_writer] main subscribed:  {MAIN_TOPIC}  → {MAIN_OUT}")
    rospy.loginfo(f"[vlm_image_writer] trash subscribed: {TRASH_TOPIC} → {TRASH_OUT}")
    rospy.loginfo(f"[vlm_image_writer] max rate:         {1.0/_MIN_WRITE_INTERVAL:.1f} Hz / topic")
    rospy.spin()


if __name__ == "__main__":
    main()
