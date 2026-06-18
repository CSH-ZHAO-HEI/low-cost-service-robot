#!/usr/bin/env python3
"""
pure_nav.py — 純全局導航
用 Gazebo get_model_state 取機器人位置（不需要 subscriber/spin）
用 move_base make_plan 取全局路徑
自己發 cmd_vel 跟隨
"""

import math
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from gazebo_msgs.srv import GetModelState

LINEAR_SPEED  = 0.25
WAYPOINT_DIST = 0.15
GOAL_DIST     = 0.15
TIMEOUT       = 60.0

_cmd_pub   = None
_path_pub  = None
_get_state = None
_make_plan = None


def _init():
    global _cmd_pub, _path_pub, _get_state, _make_plan
    if _cmd_pub is not None:
        return
    _cmd_pub  = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    _path_pub = rospy.Publisher('/pure_nav/path', Path, queue_size=1, latch=True)
    rospy.wait_for_service('/gazebo/get_model_state', timeout=10.0)
    _get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    rospy.wait_for_service('/move_base/make_plan', timeout=10.0)
    _make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)


def _get_robot_pose():
    resp = _get_state('mini_mec_six_arm', 'world')
    x = resp.pose.position.x
    y = resp.pose.position.y
    q = resp.pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )
    return x, y, yaw


def _dist(ax, ay, bx, by):
    return math.sqrt((ax-bx)**2 + (ay-by)**2)


def _stop():
    _cmd_pub.publish(Twist())


def navigate_to(goal_x, goal_y, timeout=TIMEOUT, stop_dist=None):
    _init()

    rx, ry, _ = _get_robot_pose()
    print(f"[pure_nav] 從 ({rx:.2f},{ry:.2f}) 規劃到 ({goal_x:.2f},{goal_y:.2f})")

    start = PoseStamped()
    start.header.frame_id = 'map'
    start.header.stamp = rospy.Time.now()
    start.pose.position.x = rx
    start.pose.position.y = ry
    start.pose.orientation.w = 1.0

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = goal_x
    goal.pose.position.y = goal_y
    goal.pose.orientation.w = 1.0

    try:
        resp = _make_plan(start, goal, 0.1)
        waypoints = [(p.pose.position.x, p.pose.position.y) for p in resp.plan.poses]
    except Exception as e:
        print(f"[pure_nav] 規劃失敗：{e}")
        return False

    if not waypoints:
        print("[pure_nav] 路徑為空")
        return False

    print(f"[pure_nav] {len(waypoints)} 個路徑點")

    # 發布到 RViz
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.header.stamp = rospy.Time.now()
    for wx, wy in waypoints:
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = wx
        ps.pose.position.y = wy
        ps.pose.orientation.w = 1.0
        path_msg.poses.append(ps)
    _path_pub.publish(path_msg)

    rate    = rospy.Rate(10)
    t_start = rospy.Time.now().to_sec()
    wp_idx  = 0

    while not rospy.is_shutdown():
        if rospy.Time.now().to_sec() - t_start > timeout:
            print("[pure_nav] 超時")
            _stop()
            return False

        rx, ry, ryaw = _get_robot_pose()

        check_dist = stop_dist if stop_dist is not None else GOAL_DIST
        if _dist(rx, ry, goal_x, goal_y) < check_dist:
            print("[pure_nav] 到達！")
            _stop()
            return True

        # 推進 waypoint
        while wp_idx < len(waypoints) - 1:
            wx, wy = waypoints[wp_idx]
            if _dist(rx, ry, wx, wy) < WAYPOINT_DIST:
                wp_idx += 1
            else:
                break

        wx, wy = waypoints[wp_idx]
        dx = wx - rx
        dy = wy - ry
        d  = math.sqrt(dx*dx + dy*dy)

        # 減速：距目標 0.8m 內開始減速
        dist_to_goal = _dist(rx, ry, goal_x, goal_y)
        speed = max(0.04, min(LINEAR_SPEED, dist_to_goal * 0.4))

        # 世界座標 → 機器人座標系
        cos_y = math.cos(-ryaw)
        sin_y = math.sin(-ryaw)
        vx_r  =  dx * cos_y - dy * sin_y
        vy_r  =  dx * sin_y + dy * cos_y

        if d > 0.01:
            scale = speed / d
            vx = max(-LINEAR_SPEED, min(LINEAR_SPEED, vx_r * scale))
            vy = max(-LINEAR_SPEED, min(LINEAR_SPEED, vy_r * scale))
        else:
            vx = vy = 0.0

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        _cmd_pub.publish(cmd)

        rate.sleep()

    _stop()
    return False


if __name__ == '__main__':
    import sys
    rospy.init_node('pure_nav', anonymous=True)
    if len(sys.argv) >= 3:
        navigate_to(float(sys.argv[1]), float(sys.argv[2]))
    else:
        print("用法：python3 pure_nav.py <x> <y>")
