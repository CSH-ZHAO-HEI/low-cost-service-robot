#!/usr/bin/env python3
"""
pick_deliver.py — 抓取 + 搬運互動工具
流程：
  1. 列出 table / chair / trash 物件
  2. 選目的地
  3. EGO-Planner 導航到 A 點（pickup station）
  4. /arm/pick（瞬移方塊到夾爪）
  5. EGO-Planner 導航到 approach 點（車頭對著目標）
  6. /arm/put
"""

import sys
import os
import math
import yaml
import threading

import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_srvs.srv import Trigger

PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

SCENE_PATH = os.environ.get("SCENE_PATH", os.path.join(PROJECT_ROOT, "gazebo_scene.yaml"))
KEYWORDS   = ['table', 'chair', 'trash']

# ── A 點：拾取站坐標（把機器人開到想要位置後用 get_robot_pose.py 取得）──
PICKUP_X = -0.098
PICKUP_Y =  3.340

# ── 全域狀態 ──────────────────────────────────────────────────────────────
_robot_x   = 0.0
_robot_y   = 0.0
_robot_yaw = 0.0
_odom_lock = threading.Lock()

_goal_pub  = None   # /ego_planner/goal publisher
_cmd_pub   = None   # /cmd_vel publisher (for in-place rotation)

_arm_lock_active = False  # arm 操作期間鎖速旗標

WAYPOINT_SPACING  = 3.0   # 全局路徑中繼點間距（公尺）
WAYPOINT_TOL      = 1.2   # 到達中繼點判定距離（公尺）
WAYPOINT_MIN_CLEARANCE = 0.6  # 中繼點距牆最小安全距離（公尺）
_make_plan_srv    = None  # /move_base/make_plan service proxy（可選）
_grid_map         = None  # /rtabmap/grid_map 最新快取
_grid_map_lock    = threading.Lock()


def _odom_cb(msg):
    global _robot_x, _robot_y, _robot_yaw
    ox = msg.pose.pose.orientation
    # quaternion → yaw
    siny = 2.0 * (ox.w * ox.z + ox.x * ox.y)
    cosy = 1.0 - 2.0 * (ox.y * ox.y + ox.z * ox.z)
    with _odom_lock:
        _robot_x   = msg.pose.pose.position.x
        _robot_y   = msg.pose.pose.position.y
        _robot_yaw = math.atan2(siny, cosy)


def get_robot_pose():
    with _odom_lock:
        return _robot_x, _robot_y, _robot_yaw


def _grid_map_cb(msg):
    global _grid_map
    with _grid_map_lock:
        _grid_map = msg


def _push_waypoint_from_wall(wx, wy, target_clearance=0.8, max_steps=20):
    """
    把 waypoint 從最近的牆垂直推開，直到距牆 >= target_clearance。
    用 RTAB-Map grid_map 計算梯度方向（最近障礙的反方向）。
    若 grid_map 不可用或已夠遠，直接回傳原點。
    """
    with _grid_map_lock:
        gm = _grid_map
    if gm is None:
        return wx, wy

    res  = gm.info.resolution
    ox   = gm.info.origin.position.x
    oy   = gm.info.origin.position.y
    w    = gm.info.width
    h    = gm.info.height
    data = np.array(gm.data, dtype=np.int8).reshape(h, w)

    step_size = res * 2  # 每步推 2 個格子距離

    px, py = wx, wy
    for _ in range(max_steps):
        # 目前距最近牆的距離與方向
        r_cells = int(target_clearance / res) + 2
        cx = int((px - ox) / res)
        cy = int((py - oy) / res)
        x0 = max(0, cx - r_cells);  x1 = min(w, cx + r_cells + 1)
        y0 = max(0, cy - r_cells);  y1 = min(h, cy + r_cells + 1)
        patch = data[y0:y1, x0:x1]

        occ_rows, occ_cols = np.where(patch > 50)
        if len(occ_rows) == 0:
            break  # 附近沒有障礙，不需要推

        occ_wx = ox + (occ_cols + x0 + 0.5) * res
        occ_wy = oy + (occ_rows + y0 + 0.5) * res
        dists  = np.sqrt((occ_wx - px)**2 + (occ_wy - py)**2)
        min_idx = np.argmin(dists)

        if dists[min_idx] >= target_clearance:
            break  # 已夠遠

        # 推離方向：從最近障礙指向 waypoint
        dx = px - occ_wx[min_idx]
        dy = py - occ_wy[min_idx]
        norm = math.sqrt(dx**2 + dy**2) + 1e-6
        px += (dx / norm) * step_size
        py += (dy / norm) * step_size

    return px, py


def _point_clearance(wx, wy):
    """回傳 (wx, wy) 距最近佔用格的距離（公尺），無地圖時回傳 999"""
    with _grid_map_lock:
        gm = _grid_map
    if gm is None:
        return 999.0
    res = gm.info.resolution
    ox  = gm.info.origin.position.x
    oy  = gm.info.origin.position.y
    w   = gm.info.width
    h   = gm.info.height
    data = np.array(gm.data, dtype=np.int8).reshape(h, w)

    # 以 WAYPOINT_MIN_CLEARANCE 為半徑，用 numpy 向量化搜尋
    r_cells = int(WAYPOINT_MIN_CLEARANCE / res) + 2
    cx = int((wx - ox) / res)
    cy = int((wy - oy) / res)

    x0 = max(0, cx - r_cells);  x1 = min(w, cx + r_cells + 1)
    y0 = max(0, cy - r_cells);  y1 = min(h, cy + r_cells + 1)
    patch = data[y0:y1, x0:x1]

    occ_rows, occ_cols = np.where(patch > 50)
    if len(occ_rows) == 0:
        return 999.0

    occ_wx = ox + (occ_cols + x0 + 0.5) * res
    occ_wy = oy + (occ_rows + y0 + 0.5) * res
    dists  = np.sqrt((occ_wx - wx)**2 + (occ_wy - wy)**2)
    return float(dists.min())


# ── 場景 ──────────────────────────────────────────────────────────────────
def load_scene(path):
    if not os.path.exists(path):
        print(f"[錯誤] 找不到場景檔：{path}")
        sys.exit(1)
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    return data


def filter_scene(scene):
    return {n: v for n, v in scene.items()
            if any(kw in n.lower() for kw in KEYWORDS)}


def print_objects(scene):
    print("\n" + "=" * 65)
    print("  放置目的地（table / chair / trash）：")
    print("=" * 65)
    for i, (name, info) in enumerate(sorted(scene.items()), 1):
        ox   = info.get("object_x",   0)
        oy   = info.get("object_y",   0)
        ax   = info.get("approach_x", 0)
        ay   = info.get("approach_y", 0)
        ayaw = info.get("approach_yaw", 0)
        print(f"  {i:2d}. {name:<36s}  物件({ox:.2f},{oy:.2f})"
              f"  接近({ax:.2f},{ay:.2f}, {math.degrees(ayaw):.0f}°)")
    print("=" * 65)
    print("  輸入名稱或編號，q 退出，list 重新列出\n")


# ── EGO-Planner 導航 ──────────────────────────────────────────────────────
def _send_ego_goal(x, y):
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp    = rospy.Time.now()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.w = 1.0
    _goal_pub.publish(goal)


def _wait_arrival(x, y, tol, timeout, t0):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rx, ry, _ = get_robot_pose()
        if math.sqrt((rx - x)**2 + (ry - y)**2) <= tol:
            return True
        if rospy.Time.now().to_sec() - t0 > timeout:
            return False
        rate.sleep()
    return False


TURN_ANGLE_THRESHOLD = math.radians(25)  # 路徑轉向超過此角度視為轉彎
TURN_LOOKAHEAD       = 5                  # 平滑方向用的前看點數
PRE_TURN_DIST        = 1.2               # 轉角前插入減速點的距離（公尺）
POST_TURN_DIST       = 0.8               # 轉角後插入目標點的距離（公尺）

def _path_direction(poses, i):
    """計算 poses[i] 附近的行進方向向量（用前後各 TURN_LOOKAHEAD 點平均）"""
    n = len(poses)
    i0 = max(0, i - TURN_LOOKAHEAD)
    i1 = min(n - 1, i + TURN_LOOKAHEAD)
    dx = poses[i1].pose.position.x - poses[i0].pose.position.x
    dy = poses[i1].pose.position.y - poses[i0].pose.position.y
    norm = math.sqrt(dx**2 + dy**2) + 1e-9
    return dx / norm, dy / norm


def _find_pose_at_dist(poses, from_i, dist, forward=True):
    """從 poses[from_i] 沿路徑走 dist 公尺，回傳該點的 (x, y) 和 index"""
    n = len(poses)
    acc = 0.0
    i = from_i
    step = 1 if forward else -1
    while 0 <= i + step < n:
        nx = poses[i + step].pose.position.x
        ny = poses[i + step].pose.position.y
        cx = poses[i].pose.position.x
        cy = poses[i].pose.position.y
        acc += math.sqrt((nx - cx)**2 + (ny - cy)**2)
        i += step
        if acc >= dist:
            break
    return poses[i].pose.position.x, poses[i].pose.position.y, i


def _sample_waypoints(path_poses):
    """
    從全局路徑取中繼點：
    - 每 WAYPOINT_SPACING 公尺取一個普通中繼點
    - 轉角處插入兩個點：
        1. 轉角前 PRE_TURN_DIST 的減速點（讓 EGO 先停在入口）
        2. 轉角後 POST_TURN_DIST 的目標點（EGO 轉頭進入新方向）
    """
    waypoints = []
    if not path_poses:
        return waypoints

    n = len(path_poses)
    last_x  = path_poses[0].pose.position.x
    last_y  = path_poses[0].pose.position.y
    last_dx, last_dy = _path_direction(path_poses, 0)
    last_turn_i = 0

    for i, ps in enumerate(path_poses):
        px, py = ps.pose.position.x, ps.pose.position.y
        dist_since_last = math.sqrt((px - last_x)**2 + (py - last_y)**2)

        dx, dy = _path_direction(path_poses, i)
        dot = last_dx * dx + last_dy * dy
        dot = max(-1.0, min(1.0, dot))
        angle_change = math.acos(dot)

        is_turn = (angle_change > TURN_ANGLE_THRESHOLD and
                   i - last_turn_i > TURN_LOOKAHEAD * 2)

        if dist_since_last >= WAYPOINT_SPACING or is_turn:
            if is_turn:
                # 減速點：轉角前 PRE_TURN_DIST
                pre_x, pre_y, _ = _find_pose_at_dist(path_poses, i, PRE_TURN_DIST, forward=False)
                # 目標點：轉角後 POST_TURN_DIST
                post_x, post_y, _ = _find_pose_at_dist(path_poses, i, POST_TURN_DIST, forward=True)
                waypoints.append((pre_x, pre_y))
                waypoints.append((post_x, post_y))
                last_turn_i = i
                rospy.loginfo(f"[waypoints] 轉角 {math.degrees(angle_change):.0f}° @ i={i} "
                              f"→ 減速點({pre_x:.2f},{pre_y:.2f}) 目標({post_x:.2f},{post_y:.2f})")
            else:
                waypoints.append((px, py))
            last_x, last_y = px, py
            last_dx, last_dy = dx, dy

    return waypoints



def _call_make_plan_with_retry(req, max_retries=3):
    """带重试的 make_plan 服务调用"""
    import time
    for attempt in range(max_retries):
        try:
            return _make_plan_srv(req)
        except Exception as e:
            if attempt < max_retries - 1:
                print(f"    ⚠ 规划服务无响应（尝试 {attempt+1}/{max_retries}），等待 1s 重试...")
                time.sleep(1)
            else:
                raise
    return None

def navigate_to(x, y, tol=0.35, timeout=300.0):
    """
    全局路徑橋接導航：
      若 /move_base/make_plan 可用 → 先取全局路徑，逐中繼點發給 EGO-Planner
      否則 → 直接發終點給 EGO-Planner
    """
    print(f"  → 導航中 ({x:.2f}, {y:.2f}) ...")
    t0 = rospy.Time.now().to_sec()

    # 若服務之前斷線，嘗試重連
    global _make_plan_srv
    if _make_plan_srv is None:
        try:
            rospy.wait_for_service("/move_base/make_plan", timeout=1.0)
            _make_plan_srv = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        except rospy.ROSException:
            pass

    if _make_plan_srv is not None:
        try:
            rx, ry, _ = get_robot_pose()
            start = PoseStamped()
            start.header.frame_id = "map"
            start.header.stamp = rospy.Time.now()
            start.pose.position.x = rx
            start.pose.position.y = ry
            start.pose.orientation.w = 1.0

            goal_ps = PoseStamped()
            goal_ps.header.frame_id = "map"
            goal_ps.header.stamp = rospy.Time.now()
            goal_ps.pose.position.x = x
            goal_ps.pose.position.y = y
            goal_ps.pose.orientation.w = 1.0

            req = GetPlanRequest(start=start, goal=goal_ps, tolerance=0.3)
            resp = _call_make_plan_with_retry(req)
            waypoints = _sample_waypoints(resp.plan.poses)
            waypoints.append((x, y))
            print(f"  → 全局路徑 {len(resp.plan.poses)} 點，採樣 {len(waypoints)} 個中繼點")

            for i, (wx, wy) in enumerate(waypoints):
                is_last = (i == len(waypoints) - 1)
                wp_tol  = tol if is_last else WAYPOINT_TOL

                # 中間 waypoint：推離牆壁到安全位置
                if not is_last:
                    wx_safe, wy_safe = _push_waypoint_from_wall(wx, wy, target_clearance=0.8)
                    if abs(wx_safe - wx) > 0.05 or abs(wy_safe - wy) > 0.05:
                        print(f"     中繼 {i+1}/{len(waypoints)}: ({wx:.2f},{wy:.2f}) → 推離至 ({wx_safe:.2f},{wy_safe:.2f})")
                    wx, wy = wx_safe, wy_safe

                    # 跳過已在到達範圍內的 waypoint
                    rx, ry, _ = get_robot_pose()
                    ego_min = 3 * 0.4 + 0.1
                    if math.sqrt((rx - wx)**2 + (ry - wy)**2) < ego_min:
                        print(f"     中繼 {i+1}/{len(waypoints)}: ({wx:.2f},{wy:.2f}) 跳過（太近）")
                        continue

                _send_ego_goal(wx, wy)
                print(f"     中繼 {i+1}/{len(waypoints)}: ({wx:.2f},{wy:.2f})")
                if not _wait_arrival(wx, wy, wp_tol, timeout, t0):
                    print(f"  ✗ 導航逾時（{timeout:.0f}s）")
                    return False
            print(f"  ✓ 到達！")
            return True

        except Exception as e:
            print(f"  ⚠ 全局規劃失敗 ({e})，改用直接導航")

    # Fallback：直接發給 EGO-Planner
    _send_ego_goal(x, y)
    if _wait_arrival(x, y, tol, timeout, t0):
        rx, ry, _ = get_robot_pose()
        print(f"  ✓ 到達！(距目標 {math.sqrt((rx-x)**2+(ry-y)**2):.2f}m)")
        return True
    print(f"  ✗ 導航逾時（{timeout:.0f}s）")
    return False


def rotate_to_yaw(target_yaw, timeout=8.0, tol=0.06):
    """
    原地轉到 target_yaw（弧度），透過 /ground_truth/odom 取當前 yaw。
    車頭方向 = base_link 的 +X，target_yaw 即 +X 朝向角。
    """
    rate = rospy.Rate(10)
    t0   = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        if rospy.Time.now().to_sec() - t0 > timeout:
            break
        _, _, current_yaw = get_robot_pose()
        err = math.atan2(math.sin(target_yaw - current_yaw),
                         math.cos(target_yaw - current_yaw))
        if abs(err) < tol:
            break
        cmd = Twist()
        cmd.angular.z = max(-1.5, min(1.5, err * 2.5))
        _cmd_pub.publish(cmd)
        rate.sleep()
    _cmd_pub.publish(Twist())   # 停止旋轉


# ── cmd_vel 鎖速（arm 操作期間防止 EGO 移動機器人）──────────────────────
def _cmd_vel_lock_thread():
    zero = Twist()
    rate = rospy.Rate(20)
    while _arm_lock_active and not rospy.is_shutdown():
        _cmd_pub.publish(zero)
        rate.sleep()


def _start_arm_lock():
    global _arm_lock_active
    _arm_lock_active = True
    t = threading.Thread(target=_cmd_vel_lock_thread, daemon=True)
    t.start()
    return t


def _stop_arm_lock():
    global _arm_lock_active
    _arm_lock_active = False


# ── 機械臂服務 ────────────────────────────────────────────────────────────
def call_arm(service_name, target_name):
    rospy.set_param('/arm_task/target_name', target_name)
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        resp = rospy.ServiceProxy(service_name, Trigger)()
        if resp.success:
            print(f"  ✓ {service_name}：{resp.message}")
        else:
            print(f"  ✗ {service_name}：{resp.message}")
        return resp.success
    except rospy.ROSException as e:
        print(f"  ✗ {service_name} 服務不可用：{e}")
        return False


# ── 主程式 ────────────────────────────────────────────────────────────────
def main():
    global _goal_pub, _cmd_pub, _make_plan_srv

    rospy.init_node("pick_deliver", anonymous=True)

    # 訂閱里程計（取機器人位姿）
    rospy.Subscriber("/ground_truth/odom", Odometry, _odom_cb, queue_size=10)

    # 訂閱地圖（用於 waypoint 安全距離檢查）
    rospy.Subscriber("/rtabmap/grid_map", OccupancyGrid, _grid_map_cb, queue_size=1)

    # 發布導航目標給 EGO-Planner
    _goal_pub = rospy.Publisher("/ego_planner/goal", PoseStamped,
                                queue_size=1, latch=False)

    # 發布 cmd_vel（原地轉向）
    _cmd_pub  = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # 嘗試連接全局規劃服務（需要 global_planner_only.launch 已啟動）
    try:
        rospy.wait_for_service("/move_base/make_plan", timeout=3.0)
        _make_plan_srv = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        print("[pick_deliver] 全局規劃服務就緒，啟用 waypoint 橋接導航")
    except rospy.ROSException:
        print("[pick_deliver] ⚠ 全局規劃服務未就緒，使用直接導航（EGO-Planner only）")

    # 等待里程計就緒
    print("[pick_deliver] 等待 /ground_truth/odom ...")
    rospy.wait_for_message("/ground_truth/odom", Odometry, timeout=30.0)
    print("[pick_deliver] 就緒！")

    scene        = filter_scene(load_scene(SCENE_PATH))
    names_sorted = sorted(scene.keys())

    print_objects(scene)

    while not rospy.is_shutdown():
        try:
            raw = input("目的地 > ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not raw:
            continue
        if raw.lower() in ("q", "quit"):
            break
        if raw.lower() in ("list", "ls"):
            print_objects(scene)
            continue

        target = raw
        if raw.isdigit():
            idx = int(raw) - 1
            if 0 <= idx < len(names_sorted):
                target = names_sorted[idx]
            else:
                print("  [錯誤] 編號超出範圍")
                continue

        if target not in scene:
            matches = [n for n in scene if n.lower() == target.lower()]
            target = matches[0] if matches else None
            if not target:
                print("  [錯誤] 找不到物件")
                continue

        info  = scene[target]
        obj_x = info.get("object_x",    0.0)
        obj_y = info.get("object_y",    0.0)
        place_x = info.get("place_x", obj_x)
        place_y = info.get("place_y", obj_y)
        ax    = info.get("approach_x",  obj_x)
        ay    = info.get("approach_y",  obj_y)
        ayaw  = info.get("approach_yaw", 0.0)

        print(f"\n  目的地：{target}")

        # ── Step 0: 強制 ARM_CLAMP（永遠保證導航前姿態正確）──────────
        print("\n  [0] ARM_CLAMP ...")
        _start_arm_lock()
        call_arm('/arm/home', '')
        _stop_arm_lock()

        # ── Step 1: 導航到拾取站 ─────────────────────────────────────
        print(f"\n  [1/4] 導航到拾取站 ({PICKUP_X:.2f}, {PICKUP_Y:.2f}) ...")
        if not navigate_to(PICKUP_X, PICKUP_Y):
            print("  導航到拾取站失敗，取消")
            continue

        # ── Step 2: 抓取（ARM_CLAMP → PICK → ARM_CLAMP）───────────────
        print("\n  [2/4] 抓取 red_block ...")
        _start_arm_lock()
        ok = call_arm('/arm/pick', 'red_block')
        _stop_arm_lock()
        if not ok:
            print("  抓取失敗，取消")
            continue

        # ── Step 3: 導航到目的地接近點 ───────────────────────────────
        print(f"\n  [3/4] 導航到 {target} 的接近點 ({ax:.2f}, {ay:.2f}) ...")
        if not navigate_to(ax, ay):
            print("  導航到目的地失敗，取消")
            continue

        # ── Step 3b: 轉向目標 ─────────────────────────────────────────
        face_yaw = math.atan2(math.sin(ayaw), math.cos(ayaw))
        print(f"  → 轉向目標（yaw={math.degrees(face_yaw):.1f}°）...")
        rotate_to_yaw(face_yaw)

        # ── Step 4: 放下（ARM_PUT → 放下，留在 ARM_PUT 結束）──────────
        print("\n  [4/4] 放下 ...")
        if "surface_z" in info:
            rospy.set_param('/arm_task/surface_z', float(info["surface_z"]))
        rospy.set_param('/arm_task/target_x', float(place_x))
        rospy.set_param('/arm_task/target_y', float(place_y))
        _start_arm_lock()
        call_arm('/arm/put', target)
        _stop_arm_lock()

        # 放下後：轉向拾取站方向 → ARM_CLAMP，下一輪可直接出發
        rx, ry, _ = get_robot_pose()
        to_pickup_yaw = math.atan2(PICKUP_Y - ry, PICKUP_X - rx)
        print(f"  → 轉向拾取站（yaw={math.degrees(to_pickup_yaw):.1f}°）...")
        rotate_to_yaw(to_pickup_yaw)
        print("  → ARM_CLAMP ...")
        _start_arm_lock()
        call_arm('/arm/home', '')
        _stop_arm_lock()
        print("\n  完成！\n")


if __name__ == "__main__":
    main()
