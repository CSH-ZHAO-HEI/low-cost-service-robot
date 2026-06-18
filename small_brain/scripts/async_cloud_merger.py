#!/usr/bin/env python3
"""
async_cloud_merger.py
功能：
  1. 過濾相機點雲近距離點（手臂幽靈點，z < ARM_MIN_DEPTH）
  2. 將點雲從 camera_depth_optical_frame 轉換到 map frame（純 numpy）
  3. 發布到 /ego_cloud_input 供 EGO-Planner 使用（world frame）
"""

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

ARM_MIN_DEPTH = 0.55
THROTTLE_HZ   = 15

_pub           = None
_tf_buffer     = None
_last_pub_time = 0.0


def _transform_to_matrix(t):
    """TransformStamped → 4x4 numpy matrix"""
    tr = t.transform.translation
    ro = t.transform.rotation
    # quaternion → rotation matrix
    x, y, z, w = ro.x, ro.y, ro.z, ro.w
    R = np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = [tr.x, tr.y, tr.z]
    return T


def cb_camera(msg):
    global _pub, _tf_buffer, _last_pub_time

    now = rospy.Time.now().to_sec()
    if now - _last_pub_time < 1.0 / THROTTLE_HZ:
        return

    try:
        pts = np.array(list(
            pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ), dtype=np.float32)
    except Exception:
        return

    if len(pts) == 0:
        return

    # 過濾手臂近距離點
    filtered = pts[pts[:, 2] >= ARM_MIN_DEPTH]
    if len(filtered) == 0:
        return

    # TF：camera_depth_optical_frame → map
    try:
        t = _tf_buffer.lookup_transform(
            "map",
            msg.header.frame_id,
            msg.header.stamp,
            rospy.Duration(0.05)
        )
    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException) as e:
        rospy.logwarn_throttle(5.0, f"[cloud_filter] TF failed: {e}")
        return

    T = _transform_to_matrix(t)
    ones = np.ones((len(filtered), 1), dtype=np.float32)
    pts_h = np.hstack([filtered, ones])          # Nx4
    pts_world = (T @ pts_h.T).T[:, :3]           # Nx3

    header = std_msgs.msg.Header()
    header.stamp    = msg.header.stamp
    header.frame_id = "map"
    _pub.publish(pc2.create_cloud_xyz32(header, pts_world.astype(np.float32)))
    _last_pub_time = now

    rospy.loginfo_throttle(15.0,
        f"[cloud_filter] {len(pts)} → {len(filtered)} pts → map frame")


def main():
    global _pub, _tf_buffer
    rospy.init_node('async_cloud_merger')

    _tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(_tf_buffer)

    _pub = rospy.Publisher('/ego_cloud_input', PointCloud2, queue_size=1)
    rospy.Subscriber('/camera/depth/points', PointCloud2, cb_camera, queue_size=1)
    rospy.loginfo("[cloud_filter] 就緒，過濾手臂點並轉換到 map frame")
    rospy.spin()


if __name__ == '__main__':
    main()
