#!/usr/bin/env python3
"""
static_map_generator.py
將 RTAB-Map 2D 佔用格柵地圖轉換為 3D 靜態點雲，供 EGO-Planner 感知牆壁。

輸入：/rtabmap/grid_map (OccupancyGrid)
輸出：/map_static_cloud (PointCloud2, frame=map, ~1Hz)

設計原則：
- Z 層：0.1~1.7m，每 0.2m（9 層），平衡點數與覆蓋度
- 過濾機器人 1m 內的點（相機看得到）
- 限制最大範圍 5m（超遠的牆 EGO 用不到，省算力）
- numpy 向量化，無 Python for 迴圈
"""

import math
import threading

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg


# Z 層：0.1 到 1.7m，每 0.2m，共 9 層
# 保留高度覆蓋（EGO 無人機基因，牆需到 1.7m 否則 optimizer 從上飛過）
Z_LEVELS = np.array([round(0.1 + i * 0.2, 1) for i in range(9)])  # [0.1,0.3,...,1.7]

OCCUPIED_THRESHOLD  = 90   # 只取高置信度牆（過渡帶 50~89 排除，防止路徑被誤封）
ROBOT_FILTER_RADIUS = 1.0  # 機器人 1m 內不加靜態點
MAP_MAX_RANGE       = 2.5  # 只保留機器人 2.5m 內的牆（避免走廊被遠處靜態點塞滿）

_robot_x   = 0.0
_robot_y   = 0.0
_pose_lock = threading.Lock()
_pub       = None


def odom_cb(msg):
    global _robot_x, _robot_y
    with _pose_lock:
        _robot_x = msg.pose.pose.position.x
        _robot_y = msg.pose.pose.position.y


def grid_cb(msg):
    global _last_grid_msg
    _last_grid_msg = msg
    with _pose_lock:
        rx, ry = _robot_x, _robot_y

    res = msg.info.resolution
    ox  = msg.info.origin.position.x
    oy  = msg.info.origin.position.y
    w   = msg.info.width
    h   = msg.info.height

    grid       = np.array(msg.data, dtype=np.int8).reshape(h, w)
    rows, cols = np.where(grid > OCCUPIED_THRESHOLD)

    if len(rows) == 0:
        rospy.loginfo_throttle(30.0, "[static_map_gen] 尚無佔用格，等待地圖...")
        return

    wx = ox + (cols + 0.5) * res
    wy = oy + (rows + 0.5) * res

    dist = np.sqrt((wx - rx) ** 2 + (wy - ry) ** 2)
    mask = (dist >= ROBOT_FILTER_RADIUS) & (dist <= MAP_MAX_RANGE)
    wx = wx[mask]
    wy = wy[mask]

    if len(wx) == 0:
        return

    # 每格 × 9 Z 層
    wx_arr = np.repeat(wx, len(Z_LEVELS))
    wy_arr = np.repeat(wy, len(Z_LEVELS))
    z_arr  = np.tile(Z_LEVELS, len(wx))
    points = np.column_stack([wx_arr, wy_arr, z_arr])

    header = std_msgs.msg.Header()
    header.stamp    = rospy.Time.now()
    header.frame_id = "map"
    _pub.publish(pc2.create_cloud_xyz32(header, points))

    rospy.loginfo_throttle(10.0,
        f"[static_map_gen] 牆格 {len(wx)} 個 → 發布 {len(points)} 點")


def main():
    global _pub
    rospy.init_node('static_map_generator')
    _pub = rospy.Publisher('/map_static_cloud', PointCloud2, queue_size=1, latch=True)
    rospy.Subscriber('/ground_truth/odom', Odometry,      odom_cb,  queue_size=10)
    rospy.Subscriber('/rtabmap/grid_map',  OccupancyGrid, grid_cb,  queue_size=1)
    rospy.loginfo("[static_map_gen] 就緒，等待 /rtabmap/grid_map ...")

    # 若訂閱後沒收到地圖（navigation 模式只發一次），主動等待再拉一次
    rospy.sleep(3.0)
    if _pub.get_num_connections() > 0:
        try:
            msg = rospy.wait_for_message('/rtabmap/grid_map', OccupancyGrid, timeout=5.0)
            rospy.loginfo("[static_map_gen] 主動拉取 grid_map 成功，立即處理")
            grid_cb(msg)
        except rospy.ROSException:
            rospy.logwarn("[static_map_gen] 等待 grid_map 超時，繼續等訂閱觸發")

    # 每 5 秒重新發布一次（機器人移動後過濾範圍跟著更新）
    rospy.Timer(rospy.Duration(5.0), lambda e: _republish_last())
    rospy.spin()


_last_grid_msg = None

def _republish_last():
    global _last_grid_msg
    if _last_grid_msg is not None:
        grid_cb(_last_grid_msg)


if __name__ == '__main__':
    main()
