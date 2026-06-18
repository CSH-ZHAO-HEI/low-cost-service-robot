#!/usr/bin/env python3
"""
test_coke_pick.py — BalconyTable Coke 抓取姿態測試工具

對照 put_on_table_test.py，專門測試從 BalconyTable 夾 Coke 到 NightStand。
改完 --pose 直接重跑，不用重啟 Gazebo / arm_task_server。

用法：
  # 完整流程（導航 + 抓 + 放）
  python3 test_coke_pick.py

  # 跳過導航，直接在原地測試手臂姿態
  python3 test_coke_pick.py --skip-nav

  # 只測試抓取，不去 NightStand 放
  python3 test_coke_pick.py --pick-only

  # 自訂手臂姿態
  python3 test_coke_pick.py --skip-nav --pick-only --pose 0,-0.95,0.66,1.0,0

  # 自訂 CUP_SNAP_DOWN（夾爪在罐子的哪個高度，預設 0.058m = 罐子中間）
  python3 test_coke_pick.py --skip-nav --pick-only --snap-down 0.04

  # 只擺姿態 + 轉向，不 teleport / 不夾（純看手臂位置）
  python3 test_coke_pick.py --skip-nav --pose-only --pose 0,-0.95,0.66,1.0,0

  # 只印參數，不連 ROS
  python3 test_coke_pick.py --dry-run

前提：
  ./run_gazebo.sh 已啟動
  ./run_rtab.sh  已啟動
  ./run_teb.sh   已啟動
"""
import os

import math
import sys
import time
import yaml
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.msg import ModelState, LinkStates
from std_msgs.msg import String


# ── 可調參數 ──────────────────────────────────────────────────────
PICK_POSE  = [0.0, -0.65, 0.60, 1.0, 0.0]   # BalconyTable 抓取姿態
ARM_CLAMP  = [0.0, -1.1,  0.66, 1.0, 0.0]   # 搬運姿態
ARM_PUT    = [0.0, -0.5,  0.3,  0.3, 0.0]   # 放置姿態

COKE_NAME      = 'Coke'
COKE_HALF      = 0.058   # 罐半高（scale 0.8 → 11.6cm / 2）
CUP_SNAP_DOWN  = COKE_HALF  # teleport 偏移：夾爪對齊罐子中間
GRIP_Z_OFFSET  = -0.03      # link7/9 中點到夾爪實際中心的 z 偏移

BALCONY_TABLE  = 'BalconyTable_01_001'
BALCONY_XY     = (-0.556, 4.111)   # 桌子中心
COKE_INIT_POS  = (-0.556, 3.84, 0.278)  # 桌邊初始位置（罐底 z）

NIGHTSTAND     = 'NightStand_01_001'

PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

SCENE_YAML = os.environ.get("SCENE_PATH", os.path.join(PROJECT_ROOT, "gazebo_scene.yaml"))

GRIP_JOINTS = ['joint6','joint7','joint8','joint9','joint10','joint11']
GRIP_OPEN   = [ 0.45, -0.45,  0.45,  0.45,  0.45,  0.45]
GRIP_CLOSE  = [-0.45,  0.45, -0.45, -0.45, -0.45, -0.45]

# ── 全局 ─────────────────────────────────────────────────────────
_odom_xy  = (0.0, 0.0)
_odom_yaw = 0.0
_link_pos: dict = {}
_cmd_pub = _arm_pub = _grip_pub = _model_pub = None
_attach_pub = _detach_pub = None
_mb_client = None


def odom_cb(msg):
    global _odom_xy, _odom_yaw
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    _odom_xy  = (p.x, p.y)
    _odom_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def link_cb(msg):
    for name, pose in zip(msg.name, msg.pose):
        _link_pos[name] = pose.position


def gripper_center():
    p7 = _link_pos.get('mini_mec_six_arm::link7')
    p9 = _link_pos.get('mini_mec_six_arm::link9')
    if p7 is None or p9 is None:
        return None
    return (
        (p7.x + p9.x) / 2.0,
        (p7.y + p9.y) / 2.0,
        (p7.z + p9.z) / 2.0 + GRIP_Z_OFFSET,
    )


def set_arm(pose, secs=2.0):
    traj = JointTrajectory()
    traj.joint_names = ['joint1','joint2','joint3','joint4','joint5']
    traj.header.stamp = rospy.Time.now()
    pt = JointTrajectoryPoint()
    pt.positions = pose
    pt.time_from_start = rospy.Duration(secs)
    traj.points = [pt]
    _arm_pub.publish(traj)
    rospy.sleep(secs + 0.2)
    print(f"    ✓ arm set to {[round(v,3) for v in pose]}")


def set_gripper(pos, secs=1.0):
    traj = JointTrajectory()
    traj.joint_names = GRIP_JOINTS
    traj.header.stamp = rospy.Time.now()
    pt = JointTrajectoryPoint()
    pt.positions = pos
    pt.time_from_start = rospy.Duration(secs)
    traj.points = [pt]
    _grip_pub.publish(traj)
    rospy.sleep(secs + 0.2)


def teleport(name, x, y, z, dur=0.4):
    msg = ModelState()
    msg.model_name = name
    msg.reference_frame = 'world'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    t_end = time.time() + dur
    while time.time() < t_end:
        _model_pub.publish(msg)
        time.sleep(0.005)
    print(f"    ✓ teleport {name} → ({x:.3f}, {y:.3f}, {z:.3f})")


def navigate_to(x, y, yaw, timeout=60):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = math.sin(yaw / 2)
    goal.target_pose.pose.orientation.w = math.cos(yaw / 2)
    print(f"    → nav ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.0f}°)")
    _mb_client.cancel_all_goals()
    rospy.sleep(0.1)
    _mb_client.send_goal(goal)
    if not _mb_client.wait_for_result(rospy.Duration(timeout)):
        print("    ✗ timeout")
        _mb_client.cancel_goal()
        return False
    if _mb_client.get_state() != GoalStatus.SUCCEEDED:
        print(f"    ✗ state={_mb_client.get_state()}")
        return False
    print("    ✓ reached")
    return True


def rotate_to_face(tx, ty, tol=0.03, timeout=10):
    rate = rospy.Rate(20)
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() - t0 < timeout:
        rx, ry = _odom_xy
        target_yaw = math.atan2(ty - ry, tx - rx)
        err = math.atan2(math.sin(target_yaw - _odom_yaw),
                         math.cos(target_yaw - _odom_yaw))
        if abs(err) < tol:
            break
        max_w = 1.2 if abs(err) > 0.15 else max(0.3, abs(err) * 4.0)
        cmd = Twist()
        cmd.angular.z = max(-max_w, min(max_w, err * 1.5))
        _cmd_pub.publish(cmd)
        rate.sleep()
    _cmd_pub.publish(Twist())
    print(f"    ✓ aligned to ({tx:.2f}, {ty:.2f})  yaw={math.degrees(_odom_yaw):.0f}°")


def gripper_z_estimate(pose):
    """粗估夾爪 z（同 put_on_table_test.py）"""
    j2, j3, j4 = pose[1], pose[2], pose[3]
    z = 0.27
    z += 0.21 * math.cos(-j2)
    z += 0.20 * math.cos(-j2 + j3)
    z += 0.06 * math.cos(-j2 + j3 + j4)
    return z


def load_scene():
    with open(SCENE_YAML) as f:
        return yaml.safe_load(f)


def main():
    global _cmd_pub, _arm_pub, _grip_pub, _model_pub, _attach_pub, _detach_pub, _mb_client

    skip_nav  = '--skip-nav'  in sys.argv
    pick_only = '--pick-only' in sys.argv
    pose_only = '--pose-only' in sys.argv   # 只擺姿態，不 teleport 不夾
    dry_run   = '--dry-run'   in sys.argv

    pose = list(PICK_POSE)
    snap_down = CUP_SNAP_DOWN

    if '--pose' in sys.argv:
        i = sys.argv.index('--pose')
        try:
            parts = [float(v) for v in sys.argv[i + 1].split(',')]
            if len(parts) != 5:
                raise ValueError
            pose = parts
        except (IndexError, ValueError):
            print("--pose 格式：--pose 0,-0.95,0.66,1.0,0")
            sys.exit(1)

    if '--snap-down' in sys.argv:
        i = sys.argv.index('--snap-down')
        try:
            snap_down = float(sys.argv[i + 1])
        except (IndexError, ValueError):
            print("--snap-down 格式：--snap-down 0.04")
            sys.exit(1)

    # ── 印參數 ───────────────────────────────────────────────────
    est_z = gripper_z_estimate(pose)
    print("=" * 60)
    print(f"  TABLE:       {BALCONY_TABLE}")
    print(f"  MODEL:       {COKE_NAME}")
    print(f"  COKE init:   {COKE_INIT_POS}")
    print(f"  PICK_POSE:   {[round(v,3) for v in pose]}")
    print(f"  est gripper z ≈ {est_z:.3f} m  (BalconyTable surface = 0.278 m)")
    print(f"  snap_down:   {snap_down:.3f} m  → 夾爪在罐子 {snap_down/0.116*100:.0f}% 高度")
    print(f"  pick_only:   {pick_only}   skip_nav: {skip_nav}   pose_only: {pose_only}")
    print("=" * 60)
    print()
    print("快速調姿態：")
    print("  python3 test_coke_pick.py --skip-nav --pick-only --pose 0,-0.95,0.66,1.0,0")
    print("  python3 test_coke_pick.py --skip-nav --pick-only --snap-down 0.04")
    print()

    if dry_run:
        return

    # ── Init ROS ─────────────────────────────────────────────────
    rospy.init_node('test_coke_pick', anonymous=True)
    rospy.Subscriber('/ground_truth/odom', Odometry,    odom_cb,  queue_size=5)
    rospy.Subscriber('/gazebo/link_states', LinkStates, link_cb,  queue_size=1)
    _cmd_pub    = rospy.Publisher('/cmd_vel',                        Twist,          queue_size=1)
    _arm_pub    = rospy.Publisher('/arm_controller/command',         JointTrajectory, queue_size=1)
    _grip_pub   = rospy.Publisher('/hand_controller/command',        JointTrajectory, queue_size=1)
    _model_pub  = rospy.Publisher('/gazebo/set_model_state',         ModelState,      queue_size=10)
    _attach_pub = rospy.Publisher('/block_follower/attach',          String,          queue_size=1)
    _detach_pub = rospy.Publisher('/block_follower/detach',          String,          queue_size=1)
    _mb_client  = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    print("等待 move_base...")
    if not _mb_client.wait_for_server(rospy.Duration(10)):
        print("✗ move_base 連不上")
        sys.exit(1)
    rospy.sleep(1.0)

    scene = load_scene()
    bt = scene[BALCONY_TABLE]
    ax  = float(bt['approach_x'])
    ay  = float(bt['approach_y'])
    ayaw = float(bt.get('approach_yaw', math.pi/2))
    bx  = float(bt['object_x'])
    by  = float(bt['object_y'])

    ns = scene[NIGHTSTAND]
    ns_ax  = float(ns['approach_x'])
    ns_ay  = float(ns['approach_y'])
    ns_yaw = float(ns.get('approach_yaw', 0.0))
    ns_ox, ns_oy = float(ns['object_x']), float(ns['object_y'])
    ns_px = float(ns.get('place_x', ns_ox))
    ns_py = float(ns.get('place_y', ns_oy))
    ns_sz = float(ns.get('surface_z', 0.369))

    # ── Step 1：起點乾淨 ─────────────────────────────────────────
    print("\n[1] 開夾爪 + ARM_CLAMP")
    set_gripper(GRIP_OPEN, secs=0.5)
    set_arm(ARM_CLAMP, secs=1.5)

    # ── Step 2：teleport Coke 到桌邊 ─────────────────────────────
    print(f"\n[2] teleport {COKE_NAME} → {COKE_INIT_POS}")
    teleport(COKE_NAME, *COKE_INIT_POS, dur=0.4)
    rospy.sleep(0.3)

    # ── BalconyTable 段 ──────────────────────────────────────────
    ay_mid   = ay - 0.25   # 中継：YAML 往後 0.25m（1步）
    ay_final = ay          # 最終：YAML approach 直接到位

    if not skip_nav:
        print(f"\n[3a] nav → BalconyTable 中継 ({ax:.2f}, {ay_mid:.2f})")
        if not navigate_to(ax, ay_mid, ayaw):
            print("✗ 導航失敗")
            return
    else:
        print("\n[3a] --skip-nav 跳過導航")

    print(f"\n[3b] [中継] arm → PICK_POSE {[round(v,3) for v in pose]}")
    set_arm(pose, secs=2.0)
    print(f"         rotate to face BalconyTable ({bx:.2f}, {by:.2f})")
    rotate_to_face(bx, by)

    if pose_only:
        print("\n--pose-only：只看姿態，不 teleport 不夾，結束。")
        print(f"  ↳ 調整命令：python3 test_coke_pick.py --skip-nav --pose-only --pose {','.join(str(round(v,2)) for v in pose)}")
        return

    if not skip_nav:
        print(f"\n[3c] nav → BalconyTable 最終 ({ax:.2f}, {ay_final:.2f})")
        if not navigate_to(ax, ay_final, ayaw):
            print("✗ 導航失敗（最終）")
            return

    # ── Step 4：snap + 夾取 ───────────────────────────────────────
    rospy.sleep(0.3)
    c = gripper_center()
    if c is None:
        rx, ry = _odom_xy
        c = (rx + 0.40 * math.cos(_odom_yaw), ry + 0.40 * math.sin(_odom_yaw), 0.35)
        print("    ! link_states 未就緒，使用粗估")
    snap = (c[0], c[1], c[2] - snap_down)
    print(f"\n[4] gripper=({c[0]:.3f},{c[1]:.3f},{c[2]:.3f})  snap→({snap[0]:.3f},{snap[1]:.3f},{snap[2]:.3f})")
    teleport(COKE_NAME, *snap, dur=0.3)
    set_gripper(GRIP_CLOSE, secs=0.8)
    _attach_pub.publish(String(
        data=f"mini_mec_six_arm::link7,{COKE_NAME},{snap[0]:.4f},{snap[1]:.4f},{snap[2]:.4f}"))
    rospy.sleep(0.5)

    # ── Step 5：轉向 NightStand → ARM_CLAMP → re-snap → attach base_link ──
    # base_link 只隨導航平移/yaw 旋轉，之後換手臂姿態罐子不再跳
    print(f"\n[5] 轉向 NightStand + ARM_CLAMP + re-snap → base_link")
    rotate_to_face(ns_ox, ns_oy)
    set_arm(ARM_CLAMP, secs=2.0)
    _detach_pub.publish(String(data="detach"))
    rospy.sleep(0.4)
    c2 = gripper_center()
    if c2 is None:
        c2 = c
    snap2 = (c2[0], c2[1], c2[2] - snap_down)
    print(f"         re-snap → ({snap2[0]:.3f},{snap2[1]:.3f},{snap2[2]:.3f})")
    teleport(COKE_NAME, *snap2, dur=0.3)
    _attach_pub.publish(String(
        data=f"mini_mec_six_arm::base_link,{COKE_NAME},{snap2[0]:.4f},{snap2[1]:.4f},{snap2[2]:.4f}"))
    rospy.sleep(0.3)

    print("\n✓ 抓取完成（ARM_CLAMP 搬運中）")
    print(f"  調整：python3 test_coke_pick.py --skip-nav --pick-only --pose {','.join(str(round(v,2)) for v in pose)} --snap-down {snap_down:.3f}")

    if pick_only:
        print("\n--pick-only：不去 NightStand，結束。")
        return

    # ── NightStand 段 ────────────────────────────────────────────
    ns_ay_mid   = ns_ay - 0.15   # 中継：YAML 往後 0.25m（1步前）
    ns_ay_final = ns_ay + 0.10   # 最終：YAML 往前 0.10m

    print(f"\n[6a] nav → NightStand 中継 ({ns_ax:.2f}, {ns_ay_mid:.2f})")
    if not navigate_to(ns_ax, ns_ay_mid, ns_yaw, timeout=120):
        print("✗ 導航到 NightStand 中継失敗")
        return

    print(f"\n[6b] [中継] 轉向 NightStand + ARM_PUT")
    rotate_to_face(ns_ox, ns_oy)
    set_arm(ARM_PUT, secs=2.0)

    print(f"\n[6c] nav → NightStand 最終 ({ns_ax:.2f}, {ns_ay_final:.2f})")
    if not navigate_to(ns_ax, ns_ay_final, ns_yaw, timeout=120):
        print("✗ 導航到 NightStand 最終失敗")
        return

    print(f"\n[7] detach + drop")
    _detach_pub.publish(String(data="detach"))
    rospy.sleep(0.4)
    set_gripper(GRIP_OPEN, secs=0.8)
    gc = gripper_center()
    drop_x = gc[0] if gc else ns_px
    drop_y = gc[1] if gc else ns_py
    print(f"    drop → ({drop_x:.3f},{drop_y:.3f},{ns_sz+0.025:.3f})"
          f"  ({'gripper XY' if gc else 'fallback'})")
    teleport(COKE_NAME, drop_x, drop_y, ns_sz + 0.025, dur=0.4)

    print("\n✓ 完成！")


if __name__ == '__main__':
    main()
