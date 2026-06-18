#!/usr/bin/env python3
"""
ground_truth_tf.py
訂閱 /ground_truth/odom（frame_id=world），
發布 odom → base_link 的 TF，讓 RTAB-Map 和導航棧可以用。
限速 10Hz 避免與 p3d publishTf 產生 TF_REPEATED_DATA 衝突。
"""
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

_br = None
_last_pub = 0.0
_MIN_INTERVAL = 0.1  # 10Hz

def odom_cb(msg):
    global _last_pub
    now = msg.header.stamp.to_sec()
    if now - _last_pub < _MIN_INTERVAL:
        return
    _last_pub = now

    t = TransformStamped()
    t.header.stamp    = msg.header.stamp
    t.header.frame_id = "odom"
    t.child_frame_id  = "base_link"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation      = msg.pose.pose.orientation
    _br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('ground_truth_tf')
    _br = tf2_ros.TransformBroadcaster()
    rospy.Subscriber('/ground_truth/odom', Odometry, odom_cb, queue_size=10)
    rospy.spin()
