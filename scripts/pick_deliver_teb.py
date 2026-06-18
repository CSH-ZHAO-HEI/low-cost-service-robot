#!/usr/bin/env python3
"""
pick_deliver_teb.py — TEB/move_base 版抓取 + 搬運互動工具

對照 EGO 版 pick_deliver.py：
  - 導航目標送到 move_base action
  - 場景、拾取站、機械臂服務與 EGO 版共用
"""

import math
import os
import sys
import threading

import actionlib
import rospy
import yaml
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

SCENE_PATH = os.environ.get("SCENE_PATH", os.path.join(PROJECT_ROOT, "gazebo_scene.yaml"))
KEYWORDS = ["table", "chair", "trash"]

PICKUP_X = -0.098
PICKUP_Y = 3.340
ARM_PICKUP_DIST = 0.55  # 車體中心距方塊距離（手臂伸出長度，需實測調整）

# TEB/move_base uses inflated 2D costmaps, so the EGO approach points can be
# too close to tables/trash and become unreachable. Keep the base farther away
# while staying inside the arm's practical reach.
TEB_APPROACH_DIST       = 0.75
TEB_APPROACH_DIST_TRASH = 0.50
TEB_APPROACH_DIST_TABLE = 0.60

_robot_x = 0.0
_robot_y = 0.0
_robot_yaw = 0.0
_odom_lock = threading.Lock()

_cmd_pub = None
_move_base_client = None
_model_pub = None
_arm_lock_active = False
_nav_goal_active = False

BLOCK_NAME = "red_block"
BLOCK_HALF = 0.025


def _yaw_to_quat(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qz, qw


def _odom_cb(msg):
    global _robot_x, _robot_y, _robot_yaw
    ox = msg.pose.pose.orientation
    siny = 2.0 * (ox.w * ox.z + ox.x * ox.y)
    cosy = 1.0 - 2.0 * (ox.y * ox.y + ox.z * ox.z)
    with _odom_lock:
        _robot_x = msg.pose.pose.position.x
        _robot_y = msg.pose.pose.position.y
        _robot_yaw = math.atan2(siny, cosy)


def get_robot_pose():
    with _odom_lock:
        return _robot_x, _robot_y, _robot_yaw


def load_scene(path):
    if not os.path.exists(path):
        print(f"[錯誤] 找不到場景檔：{path}")
        sys.exit(1)
    with open(path, "r") as f:
        return yaml.safe_load(f)


def filter_scene(scene):
    return {
        name: info
        for name, info in scene.items()
        if any(kw in name.lower() for kw in KEYWORDS)
    }


def print_objects(scene):
    print("\n" + "=" * 65)
    print("  放置目的地（table / chair / trash）：")
    print("=" * 65)
    for i, (name, info) in enumerate(sorted(scene.items()), 1):
        ox = info.get("object_x", 0)
        oy = info.get("object_y", 0)
        ax = info.get("approach_x", 0)
        ay = info.get("approach_y", 0)
        ayaw = info.get("approach_yaw", 0)
        print(
            f"  {i:2d}. {name:<36s}  物件({ox:.2f},{oy:.2f})"
            f"  接近({ax:.2f},{ay:.2f}, {math.degrees(ayaw):.0f} deg)"
        )
    print("=" * 65)
    print("  輸入名稱或編號，q 退出，list 重新列出\n")


def navigate_to(x, y, yaw=None, timeout=300.0):
    global _nav_goal_active

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    if yaw is None:
        _, _, yaw = get_robot_pose()
    qz, qw = _yaw_to_quat(yaw)
    goal.target_pose.pose.orientation.z = qz
    goal.target_pose.pose.orientation.w = qw

    print(f"  → TEB 導航中 ({x:.2f}, {y:.2f}) ...")
    _move_base_client.send_goal(goal)
    _nav_goal_active = True
    finished = _move_base_client.wait_for_result(rospy.Duration(timeout))
    if not finished:
        _move_base_client.cancel_goal()
        _move_base_client.wait_for_result(rospy.Duration(2.0))
        _move_base_client.stop_tracking_goal()
        _nav_goal_active = False
        print(f"  ✗ 導航逾時（{timeout:.0f}s）")
        return False

    state = _move_base_client.get_state()
    _move_base_client.stop_tracking_goal()
    _nav_goal_active = False
    if state == GoalStatus.SUCCEEDED:
        rx, ry, _ = get_robot_pose()
        dist = math.sqrt((rx - x) ** 2 + (ry - y) ** 2)
        print(f"  ✓ 到達！(距目標 {dist:.2f}m)")
        return True

    print(f"  ✗ move_base 回報失敗：state={state}")
    return False


def adjust_teb_approach(obj_x, obj_y, ax, ay, ayaw, target_name=""):
    lower = target_name.lower()

    dx = ax - obj_x
    dy = ay - obj_y
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < 1e-3:
        dx = -math.cos(ayaw)
        dy = -math.sin(ayaw)
        dist = 1.0

    if "trash" in lower or "bin" in lower:
        # 強制設到指定距離（不管 yaml 原值）
        scale = TEB_APPROACH_DIST_TRASH / dist
        return obj_x + dx * scale, obj_y + dy * scale
    elif "table" in lower:
        scale = TEB_APPROACH_DIST_TABLE / dist
        return obj_x + dx * scale, obj_y + dy * scale
    else:
        # 其他物件：只在太近時推遠
        if dist >= TEB_APPROACH_DIST:
            return ax, ay
        scale = TEB_APPROACH_DIST / dist
        return obj_x + dx * scale, obj_y + dy * scale


def drive_forward(dist_m, speed=0.15, timeout=10.0):
    """沿車頭方向直線前進 dist_m 公尺（用里程計計距）"""
    x0, y0, _ = get_robot_pose()
    rate = rospy.Rate(20)
    t0 = rospy.Time.now().to_sec()
    cmd = Twist()
    cmd.linear.x = speed
    while not rospy.is_shutdown():
        if rospy.Time.now().to_sec() - t0 > timeout:
            break
        x, y, _ = get_robot_pose()
        if math.sqrt((x - x0)**2 + (y - y0)**2) >= dist_m:
            break
        _cmd_pub.publish(cmd)
        rate.sleep()
    _cmd_pub.publish(Twist())


def rotate_to_yaw(target_yaw, timeout=8.0, tol=0.10):
    rate = rospy.Rate(20)
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        if rospy.Time.now().to_sec() - t0 > timeout:
            break
        _, _, current_yaw = get_robot_pose()
        err = math.atan2(
            math.sin(target_yaw - current_yaw),
            math.cos(target_yaw - current_yaw),
        )
        if abs(err) < tol:
            break
        cmd = Twist()
        cmd.angular.z = max(-1.2, min(1.2, err * 1.2))
        _cmd_pub.publish(cmd)
        rate.sleep()
    _cmd_pub.publish(Twist())


def _cmd_vel_lock_thread():
    zero = Twist()
    rate = rospy.Rate(20)
    while _arm_lock_active and not rospy.is_shutdown():
        _cmd_pub.publish(zero)
        rate.sleep()


def _start_arm_lock():
    global _arm_lock_active, _nav_goal_active
    if _nav_goal_active:
        _move_base_client.cancel_goal()
        _move_base_client.wait_for_result(rospy.Duration(1.0))
        _move_base_client.stop_tracking_goal()
        _nav_goal_active = False
    _arm_lock_active = True
    t = threading.Thread(target=_cmd_vel_lock_thread, daemon=True)
    t.start()
    return t


def _stop_arm_lock():
    global _arm_lock_active
    _arm_lock_active = False


def call_arm(service_name, target_name):
    rospy.set_param("/arm_task/target_name", target_name)
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


def teleport_block_to_pickup():
    import time
    msg = ModelState()
    msg.model_name = BLOCK_NAME
    msg.reference_frame = "world"
    msg.pose.position.x = PICKUP_X
    msg.pose.position.y = PICKUP_Y
    msg.pose.position.z = BLOCK_HALF
    msg.pose.orientation.w = 1.0
    t_end = time.time() + 0.5
    while time.time() < t_end:
        _model_pub.publish(msg)
        time.sleep(0.005)
    print(f"  ✓ 紅色方塊瞬移到拾取站 ({PICKUP_X:.2f}, {PICKUP_Y:.2f})")


def main():
    global _cmd_pub, _move_base_client, _model_pub

    rospy.init_node("pick_deliver_teb", anonymous=True)
    rospy.Subscriber("/ground_truth/odom", Odometry, _odom_cb, queue_size=10)
    _cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    _model_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
    rospy.sleep(1.0)  # 等待 publisher 連上 Gazebo

    print("[pick_deliver_teb] 等待 /ground_truth/odom ...")
    rospy.wait_for_message("/ground_truth/odom", Odometry, timeout=30.0)

    _move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    print("[pick_deliver_teb] 等待 move_base action server ...")
    if not _move_base_client.wait_for_server(rospy.Duration(60.0)):
        print("[錯誤] 無法連接 move_base，請確認 ./run_teb_compare.sh 已啟動")
        return

    print("[pick_deliver_teb] 就緒！")
    scene = filter_scene(load_scene(SCENE_PATH))
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

        info = scene[target]
        obj_x = info.get("object_x", 0.0)
        obj_y = info.get("object_y", 0.0)
        place_x = info.get("place_x", obj_x)
        place_y = info.get("place_y", obj_y)
        ax = info.get("approach_x", obj_x)
        ay = info.get("approach_y", obj_y)
        ayaw = math.atan2(
            math.sin(info.get("approach_yaw", 0.0)),
            math.cos(info.get("approach_yaw", 0.0)),
        )
        teb_ax, teb_ay = adjust_teb_approach(obj_x, obj_y, ax, ay, ayaw, target)

        print(f"\n  目的地：{target}")

        # ── 立即瞬移紅色方塊到拾取站 ─────────────────────────────────
        teleport_block_to_pickup()

        # ── Step 0: 轉向拾取站 → ARM_CLAMP ──────────────────────────
        rx, ry, _ = get_robot_pose()
        to_pickup_yaw = math.atan2(PICKUP_Y - ry, PICKUP_X - rx)
        print(f"\n  [0] 轉向拾取站（yaw={math.degrees(to_pickup_yaw):.1f}°）→ ARM_CLAMP ...")
        rotate_to_yaw(to_pickup_yaw)
        _start_arm_lock()
        call_arm("/arm/home", "")
        _stop_arm_lock()

        # 計算讓手臂能到達方塊的停靠點（車體退後 ARM_PICKUP_DIST，車頭對準方塊）
        rx, ry, _ = get_robot_pose()
        α = math.atan2(PICKUP_Y - ry, PICKUP_X - rx)
        pickup_nav_x = PICKUP_X - ARM_PICKUP_DIST * math.cos(α)
        pickup_nav_y = PICKUP_Y - ARM_PICKUP_DIST * math.sin(α)
        # 車頭（-X）朝向方塊：yaw = α + π
        pickup_face_yaw = α

        print(f"\n  [1/4] 導航到拾取點 ({pickup_nav_x:.2f}, {pickup_nav_y:.2f})，朝向方塊 ...")
        if not navigate_to(pickup_nav_x, pickup_nav_y):
            print("  導航到拾取站失敗，取消")
            continue

        rospy.sleep(0.4)
        print(f"  → 對準方塊方向（yaw={math.degrees(pickup_face_yaw):.1f}°）...")
        rotate_to_yaw(pickup_face_yaw)
        print("  → 前進貼近方塊 ...")
        drive_forward(0.15)

        print("\n  [2/4] 抓取 red_block ...")
        _start_arm_lock()
        ok = call_arm("/arm/pick", "red_block")
        _stop_arm_lock()
        if not ok:
            print("  抓取失敗，取消")
            continue

        if abs(teb_ax - ax) > 0.01 or abs(teb_ay - ay) > 0.01:
            print(
                f"\n  [3/4] TEB 接近點拉遠: "
                f"({ax:.2f}, {ay:.2f}) → ({teb_ax:.2f}, {teb_ay:.2f})"
            )
        else:
            print(f"\n  [3/4] 導航到 {target} 的接近點 ({teb_ax:.2f}, {teb_ay:.2f}) ...")
        if not navigate_to(teb_ax, teb_ay):
            print("  導航到目的地失敗，取消")
            continue

        rospy.sleep(0.4)
        print("\n  [4/4] 放下 ...")
        if "surface_z" in info:
            rospy.set_param("/arm_task/surface_z", float(info["surface_z"]))
        rospy.set_param("/arm_task/target_x", float(place_x))
        rospy.set_param("/arm_task/target_y", float(place_y))

        # Step A：先抬臂到 PUT 位置（臂就位，follower 停止）
        _start_arm_lock()
        call_arm("/arm/prepare_put", target)
        _stop_arm_lock()

        # Step B：轉向對準目標（臂已就位，旋轉更穩）
        print(f"  → 轉向：車頭對著 {target}（yaw={math.degrees(ayaw):.1f} deg）...")
        rotate_to_yaw(ayaw)

        # Step C：開夾爪放下（trash 強制用桶口中心）
        lower = target.lower()
        if "trash" in lower or "bin" in lower:
            print("  → 前進貼近垃圾桶 ...")
            drive_forward(0.25)
            rospy.set_param("/arm_task/use_target_xy", True)
        else:
            rospy.set_param("/arm_task/use_target_xy", False)
        _start_arm_lock()
        call_arm("/arm/drop", target)
        _stop_arm_lock()
        rospy.set_param("/arm_task/use_target_xy", False)
        print("\n  完成！（手臂維持 ARM_PUT，下輪 Step 0 會歸位）\n")


if __name__ == "__main__":
    main()
