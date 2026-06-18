#!/usr/bin/python3
"""
小視窗顯示 /chase_cam/image_raw
比 rqt_image_view 更輕量，窗口可拖動、可縮放
用法: python3 pip_cam.py [topic]  (預設 /chase_cam/image_raw)
"""
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image

TOPIC  = sys.argv[1] if len(sys.argv) > 1 else '/chase_cam/image_raw'
WIN    = 'Chase Cam'
W, H   = 480, 270   # 窗口初始大小

latest = None

def cb(msg):
    global latest
    # 手動解碼 RGB8
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    img = arr.reshape((msg.height, msg.width, 3))
    latest = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

def main():
    rospy.init_node('pip_cam', anonymous=True)
    rospy.Subscriber(TOPIC, Image, cb, queue_size=1, buff_size=2**24)

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, W, H)
    print(f"[pip_cam] 訂閱 {TOPIC}  |  按 Q 關閉")

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if latest is not None:
            cv2.imshow(WIN, latest)
        else:
            # 等待畫面時顯示黑底提示
            blank = np.zeros((H, W, 3), dtype=np.uint8)
            cv2.putText(blank, 'Waiting...', (10, H//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (80, 80, 80), 2)
            cv2.imshow(WIN, blank)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        try:
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
