#!/usr/bin/env python3
"""
put_on_table_test.py — CoffeeTable 桌面拿取 + NightStand 放置測試（小腦任務，不走 LLM）

預設任務：
  CoffeeTable → 夾起 red_block → NightStand_01_001 → 放下

可配置：
  - --side east|west|north|south：選擇從哪個方向接近（用來避開障礙物）
  - 改檔頂 TABLE_POSE[<key>]：每張桌子可有自己的手臂姿態
  - 改完直接重跑，**不用重啟** Gazebo / arm_task_server

用法：
  python3 put_on_table_test.py coffeetable                  # CoffeeTable 預設從 north 正對桌面
  python3 put_on_table_test.py coffeetable --side east      # 從東邊接近
  python3 put_on_table_test.py coffeetable --side north     # 從北邊接近
  python3 put_on_table_test.py coffeetable --skip-nav       # 跳過導航重測手臂
  python3 put_on_table_test.py coffeetable --pose 0,-0.35,0.95,1.3,0
  python3 put_on_table_test.py coffeetable --buffer 0.30
  python3 put_on_table_test.py coffeetable --nudge 0.08
  python3 put_on_table_test.py coffeetable --edge-frac 0.90
  python3 put_on_table_test.py coffeetable --pick-only      # 只拿起，不去 nightstand 放下
  python3 put_on_table_test.py coffeetable --place-target NightStand_01_001
  python3 put_on_table_test.py coffeetable --dry-run        # 只印 approach/yaw，不連 ROS

前提：
  - Gazebo + RTAB-Map + TEB 已啟動
  - gazebo_scene.yaml 已生成
  - red_block 在 Gazebo（任意位置 — 腳本會 teleport）
"""
import os
import sys
import math
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


# ═══════════════════════════════════════════════════════════════
# 🔧 可調姿態 — 每張桌子一組（改這裡，重跑就生效）
# ═══════════════════════════════════════════════════════════════
#
# 姿態 = [joint1, joint2, joint3, joint4, joint5]
#   joint1: 底座旋轉（z=-1，正值=逆時針）— 通常 0
#   joint2: 大臂俯仰（y=-1，負值=往後仰）
#   joint3: 小臂俯仰（y=+1，正值=往前彎）
#   joint4: 腕部俯仰（y=+1，正值=往下）
#   joint5: 腕部旋轉（z=-1）— 通常 0
#
# 經驗法則：
#   joint2 越「負」→ 大臂越往後仰 → 整體越「往上抬」
#   joint2 越接近 0  → 大臂越垂直 → 夾爪 z 越高
#   joint3 越大 → 小臂越往前 → 夾爪伸更遠 (x 變大)
#   joint4 越大 → 腕越往下 → 夾爪面朝地面
#
# CoffeeTable surface_z = 0.37 m，目標夾爪 z ≈ 0.42 m (略上桌面)
#   起始試值（比 ARM_PUT 低、比 ARM_CLAMP 高）— 隨意試
#
# BalconyTable surface_z = 0.28 m，目標夾爪 z ≈ 0.33 m
#
# 改完直接跑：python3 put_on_table_test.py coffeetable --skip-nav
TABLE_POSE = {
    'coffeetable':  [0.0, -0.35, 0.95, 1.3, 0.0],   # 比 ARM_CLAMP 略抬（j2 -1.1→-0.85）
    'balconytable': [0.0, -0.95, 0.66, 1.0, 0.0],   # 略低（桌面更矮）
    'kitchentable': [0.0, -0.75, 0.66, 1.0, 0.0],   # 略高（桌面更高 0.45m）
    # 沒列的桌子用 DEFAULT_POSE
}
DEFAULT_POSE = [0.0, -0.5, 0.3, 0.3, 0.0]   # ARM_PUT 風格 fallback

# 距離（從物件中心算）= bbox_half + buffer
# 每張桌子可不一樣，沒列的用 DEFAULT_BUFFER
TABLE_BUFFER = {
    'coffeetable':  0.30,    # chassis 距桌邊約 15cm，比 0.20 更穩
    'balconytable': 0.55,
    'kitchentable': 0.55,
}
DEFAULT_BUFFER = 0.55

# 沒指定 --side 時的專用預設。CoffeeTable 從 north 方向接近，
# 車頭朝南，正對桌面長邊，不再沿桌子長軸從側面靠近。
DEFAULT_SIDE = {
    'coffeetable': 'north',
}

# 接近方向 — 把 (dx, dy) 從物件中心指向 approach 點
SIDE_DIR = {
    'east':  ( 1,  0),    # 從東邊接近，車頭朝西
    'west':  (-1,  0),
    'north': ( 0,  1),
    'south': ( 0, -1),
}


# ═══════════════════════════════════════════════════════════════
# 常數（一般不用改）
# ═══════════════════════════════════════════════════════════════
GRIP_JOINTS = ['joint6','joint7','joint8','joint9','joint10','joint11']
GRIP_OPEN   = [ 0.45, -0.45,  0.45,  0.45,  0.45,  0.45]
GRIP_CLOSE  = [-0.45,  0.45, -0.45, -0.45, -0.45, -0.45]

BLOCK_NAME = 'red_block'
BLOCK_HALF = 0.025
DEFAULT_PLACE_TARGET = 'NightStand_01_001'
ARM_CLAMP = [0.0, -1.1, 0.66, 1.0, 0.0]
ARM_PUT = [0.0, -0.5, 0.3, 0.3, 0.0]
COFFEE_FORWARD_NUDGE = 0.0
COFFEE_EDGE_FRAC = 0.90

PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

SCENE_YAML = os.environ.get("SCENE_PATH", os.path.join(PROJECT_ROOT, "gazebo_scene.yaml"))


# ═══════════════════════════════════════════════════════════════
# 全局
# ═══════════════════════════════════════════════════════════════
_odom_xy   = (0.0, 0.0)
_odom_yaw  = 0.0
_cmd_pub      = None
_arm_pub      = None
_gripper_pub  = None
_model_pub    = None
_mb_client    = None
_follower_attach_pub = None
_follower_detach_pub = None
_link_positions = {}
GRIP_Z_OFFSET = -0.03


def odom_cb(msg):
    global _odom_xy, _odom_yaw
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    _odom_xy  = (p.x, p.y)
    _odom_yaw = math.atan2(siny, cosy)


def link_states_cb(msg):
    for name, pose in zip(msg.name, msg.pose):
        _link_positions[name] = pose.position


def get_gripper_center_world():
    p7 = _link_positions.get('mini_mec_six_arm::link7')
    p9 = _link_positions.get('mini_mec_six_arm::link9')
    if p7 is None or p9 is None:
        return None
    return (
        (p7.x + p9.x) / 2.0,
        (p7.y + p9.y) / 2.0,
        (p7.z + p9.z) / 2.0 + GRIP_Z_OFFSET,
    )


def parse_float_list(raw, expected_len, label):
    parts = [p.strip() for p in raw.split(',') if p.strip()]
    if len(parts) != expected_len:
        raise ValueError(f"{label} 需要 {expected_len} 個逗號分隔數值")
    return [float(p) for p in parts]


def find_scene_key(scene, query):
    q = query.lower()
    if q in scene:
        return q
    exact = next((k for k in scene if k.lower() == q), None)
    if exact:
        return exact
    partial = next((k for k in scene if q in k.lower()), None)
    if partial:
        return partial
    raise KeyError(query)


def set_arm(pose, secs=2.0):
    """直接送 trajectory 到 arm_controller，不走 MoveIt。"""
    traj = JointTrajectory()
    traj.joint_names = ['joint1','joint2','joint3','joint4','joint5']
    traj.header.stamp = rospy.Time.now()
    pt = JointTrajectoryPoint()
    pt.positions = pose
    pt.time_from_start = rospy.Duration(secs)
    traj.points = [pt]
    _arm_pub.publish(traj)
    rospy.sleep(secs + 0.2)


def set_gripper(pos, secs=1.0):
    traj = JointTrajectory()
    traj.joint_names = GRIP_JOINTS
    traj.header.stamp = rospy.Time.now()
    pt = JointTrajectoryPoint()
    pt.positions = pos
    pt.time_from_start = rospy.Duration(secs)
    traj.points = [pt]
    _gripper_pub.publish(traj)
    rospy.sleep(secs + 0.2)


def navigate_to(x, y, yaw, timeout=60):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = math.sin(yaw/2)
    goal.target_pose.pose.orientation.w = math.cos(yaw/2)
    print(f"    → move_base goal ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.0f}°)")

    # Clear stale action handles from previous runs/goals. Without this, actionlib
    # can receive a transition callback for an old goal handle and print:
    # "Got a transition callback on a goal handle that we're not tracking".
    _mb_client.cancel_all_goals()
    rospy.sleep(0.1)
    _mb_client.stop_tracking_goal()

    _mb_client.send_goal(goal)
    if not _mb_client.wait_for_result(rospy.Duration(timeout)):
        print(f"    ✗ timeout ({timeout}s)")
        _mb_client.cancel_goal()
        _mb_client.wait_for_result(rospy.Duration(1.0))
        _mb_client.stop_tracking_goal()
        return False
    state = _mb_client.get_state()
    if state != GoalStatus.SUCCEEDED:
        if state == GoalStatus.PREEMPTED:
            print("    ! goal preempted, retry once")
            _mb_client.stop_tracking_goal()
            rospy.sleep(0.5)
            _mb_client.send_goal(goal)
            if _mb_client.wait_for_result(rospy.Duration(timeout)):
                state = _mb_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    _mb_client.stop_tracking_goal()
                    print(f"    ✓ reached")
                    return True
        print(f"    ✗ state={state}")
        _mb_client.stop_tracking_goal()
        return False
    _mb_client.stop_tracking_goal()
    print(f"    ✓ reached")
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
        max_omega = 1.2 if abs(err) > 0.15 else max(0.3, abs(err) * 4.0)
        cmd = Twist()
        cmd.angular.z = max(-max_omega, min(max_omega, err * 1.5))
        _cmd_pub.publish(cmd)
        rate.sleep()
    _cmd_pub.publish(Twist())
    print(f"    ✓ aligned (yaw={math.degrees(_odom_yaw):.0f}°)")


def drive_forward(distance, speed=0.12, timeout=5.0):
    if abs(distance) < 1e-3:
        return
    x0, y0 = _odom_xy
    rate = rospy.Rate(20)
    t0 = rospy.Time.now().to_sec()
    cmd = Twist()
    cmd.linear.x = speed if distance > 0 else -speed
    target = abs(distance)
    while not rospy.is_shutdown():
        if rospy.Time.now().to_sec() - t0 > timeout:
            break
        x, y = _odom_xy
        if math.hypot(x - x0, y - y0) >= target:
            break
        _cmd_pub.publish(cmd)
        rate.sleep()
    _cmd_pub.publish(Twist())
    print(f"    ✓ forward nudge {distance:.2f} m")


def teleport_block(model_name, x, y, z, duration=0.3):
    msg = ModelState()
    msg.model_name = model_name
    msg.reference_frame = 'world'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    t_end = time.time() + duration
    while time.time() < t_end:
        _model_pub.publish(msg)
        time.sleep(0.005)
    print(f"    ✓ teleported {model_name} → ({x:.3f}, {y:.3f}, {z:.3f})")


def get_gripper_z_estimate(pose_values):
    """粗估某個 arm pose 下夾爪的 z 高度（base_link 上方 0.27 + 鏈長三角函數）。
    只用來印出讓使用者參考調姿態用。"""
    # 簡化模型：用 joint2 + joint3 + joint4 算大致角度
    j1, j2, j3, j4, j5 = pose_values
    # base_link 上方 base→joint1 = 0.27m
    z = 0.27
    # joint2 是 y=-1 軸，所以 joint2 為負 = 大臂往後 = 上升
    # 鏈長 joint2→joint3 ≈ 0.21
    z += 0.21 * math.cos(-j2)
    # joint3 鏈長 ≈ 0.20，繼續上去
    z += 0.20 * math.cos(-j2 + j3)
    # joint4 鏈長 ≈ 0.06
    z += 0.06 * math.cos(-j2 + j3 + j4)
    return z


def main():
    global _cmd_pub, _arm_pub, _gripper_pub, _model_pub, _mb_client
    global _follower_attach_pub, _follower_detach_pub

    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    target = sys.argv[1].lower()
    skip_nav = '--skip-nav' in sys.argv
    dry_run = '--dry-run' in sys.argv
    pick_only = '--pick-only' in sys.argv
    pose_override = None
    buffer_override = None
    place_target = DEFAULT_PLACE_TARGET
    coffee_nudge = COFFEE_FORWARD_NUDGE
    edge_frac = COFFEE_EDGE_FRAC

    if '--pose' in sys.argv:
        i = sys.argv.index('--pose')
        if i + 1 >= len(sys.argv):
            print("--pose 格式：--pose 0,-0.35,0.95,1.3,0")
            sys.exit(1)
        try:
            pose_override = parse_float_list(sys.argv[i + 1], 5, "--pose")
        except ValueError as e:
            print(e)
            sys.exit(1)

    if '--buffer' in sys.argv:
        i = sys.argv.index('--buffer')
        if i + 1 >= len(sys.argv):
            print("--buffer 格式：--buffer 0.30")
            sys.exit(1)
        buffer_override = float(sys.argv[i + 1])

    if '--nudge' in sys.argv:
        i = sys.argv.index('--nudge')
        if i + 1 >= len(sys.argv):
            print("--nudge 格式：--nudge 0.08")
            sys.exit(1)
        coffee_nudge = float(sys.argv[i + 1])

    if '--edge-frac' in sys.argv:
        i = sys.argv.index('--edge-frac')
        if i + 1 >= len(sys.argv):
            print("--edge-frac 格式：--edge-frac 0.90")
            sys.exit(1)
        edge_frac = float(sys.argv[i + 1])
        edge_frac = max(0.0, min(1.0, edge_frac))

    if '--place-target' in sys.argv:
        i = sys.argv.index('--place-target')
        if i + 1 >= len(sys.argv):
            print("--place-target 格式：--place-target NightStand_01_002")
            sys.exit(1)
        place_target = sys.argv[i + 1]

    # 解析 --side 旗標
    side = None
    side_from_default = False
    if '--side' in sys.argv:
        i = sys.argv.index('--side')
        if i + 1 < len(sys.argv):
            side = sys.argv[i + 1].lower()
            if side not in SIDE_DIR:
                print(f"--side 必須是 {list(SIDE_DIR.keys())}")
                sys.exit(1)

    # Load scene
    with open(SCENE_YAML) as f:
        scene = yaml.safe_load(f)
    try:
        table_key = find_scene_key(scene, target)
    except KeyError:
        candidates = [k for k in scene
                      if any(t in k.lower() for t in ('table','desk','bench','nightstand'))]
        print(f"找不到 '{target}'。可選的桌子：")
        for c in candidates:
            print(f"  {c}")
        sys.exit(1)

    try:
        place_key = find_scene_key(scene, place_target)
    except KeyError:
        candidates = [k for k in scene if 'nightstand' in k.lower()]
        print(f"找不到放置目標 '{place_target}'。可選 NightStand：")
        for c in candidates:
            print(f"  {c}")
        sys.exit(1)

    info = scene[table_key]
    ox = info['object_x']
    oy = info['object_y']
    sz = info['surface_z']
    px = info.get('place_x', ox)
    py = info.get('place_y', oy)
    bbox_half = max(info.get('bbox_half_x', 0.25), info.get('bbox_half_y', 0.25))

    place_info = scene[place_key]
    ns_ox = place_info['object_x']
    ns_oy = place_info['object_y']
    ns_ax = place_info['approach_x']
    ns_ay = place_info['approach_y']
    ns_yaw = place_info.get('approach_yaw', 0.0)
    ns_px = place_info.get('place_x', ns_ox)
    ns_py = place_info.get('place_y', ns_oy)
    ns_sz = place_info.get('surface_z', 0.0)

    # 從 TABLE_BUFFER 找這張桌子的 buffer（沒列就用 default）
    buf_key = next((k for k in TABLE_BUFFER if k in target), None)
    approach_buffer = TABLE_BUFFER.get(buf_key, DEFAULT_BUFFER)
    if buffer_override is not None:
        approach_buffer = buffer_override

    # CoffeeTable 專用：不加 --side 時也固定正對桌面。
    default_side_key = next((k for k in DEFAULT_SIDE if k in target), None)
    auto_side = DEFAULT_SIDE.get(default_side_key)
    if side is None and auto_side:
        side = auto_side
        side_from_default = True

    # Approach 計算
    if side:
        # --side 覆蓋：從指定方向接近
        dx_dir, dy_dir = SIDE_DIR[side]
        edge_dist = (
            abs(dx_dir) * info.get('bbox_half_x', bbox_half) +
            abs(dy_dir) * info.get('bbox_half_y', bbox_half)
        )
        new_dist = edge_dist + approach_buffer
        ax = ox + dx_dir * new_dist
        ay = oy + dy_dir * new_dist
        ayaw = math.atan2(-dy_dir, -dx_dir)   # 車頭指向物件
        # place_x/y = 桌邊靠 robot 那側；edge_frac 越大越靠近桌邊。
        px = ox + dx_dir * info.get('bbox_half_x', bbox_half) * edge_frac
        py = oy + dy_dir * info.get('bbox_half_y', bbox_half) * edge_frac
    else:
        # 用 yaml 方向，套用本檔 approach_buffer
        ax_y = info['approach_x']
        ay_y = info['approach_y']
        ayaw = info.get('approach_yaw', 0.0)
        new_dist = bbox_half + approach_buffer
        dx, dy = ax_y - ox, ay_y - oy
        dist = math.hypot(dx, dy)
        if dist > 1e-3:
            scale = new_dist / dist
            ax = ox + dx * scale
            ay = oy + dy * scale
        else:
            ax, ay = ax_y, ay_y

    # 選姿態 — 每張桌子查 TABLE_POSE，沒列就用 DEFAULT
    pose_key = None
    for k in TABLE_POSE:
        if k in target:
            pose_key = k
            break
    arm_pose = TABLE_POSE.get(pose_key, DEFAULT_POSE)
    if pose_override is not None:
        arm_pose = pose_override
    pose_label = pose_key if pose_key else 'DEFAULT'
    if pose_override is not None:
        pose_label += ' override'

    # 印參數
    print("=" * 64)
    print(f"  TABLE:           {table_key}")
    print(f"  object center:   ({ox:6.2f}, {oy:6.2f}, sz={sz:.3f})")
    print(f"  place point:     ({px:6.2f}, {py:6.2f})  ← block 會落這裡")
    print(f"  bbox_half (max): {bbox_half:.3f}")
    side_label = f"{side} (coffeetable default)" if side_from_default else side
    print(f"  approach side:   {side_label if side else 'yaml預設'}")
    print(f"  approach point:  ({ax:6.2f}, {ay:6.2f}, yaw={math.degrees(ayaw):.0f}°)")
    print(f"  approach buffer: {approach_buffer}  (bbox+buffer = {new_dist:.2f} m)")
    print(f"  forward nudge:   {coffee_nudge:.2f} m")
    print(f"  edge fraction:   {edge_frac:.2f}")
    print("-" * 64)
    print(f"  🔧 arm pose [{pose_label}]: {arm_pose}")
    print(f"     estimated gripper z ≈ {get_gripper_z_estimate(arm_pose):.2f} m"
          f"  (target桌面 = {sz:.2f} m)")
    print("-" * 64)
    print(f"  PLACE TARGET:    {place_key}")
    print(f"  nightstand obj:  ({ns_ox:6.2f}, {ns_oy:6.2f}, sz={ns_sz:.3f})")
    print(f"  nightstand app:  ({ns_ax:6.2f}, {ns_ay:6.2f}, yaw={math.degrees(ns_yaw):.0f}°)")
    print(f"  nightstand drop: ({ns_px:6.2f}, {ns_py:6.2f}, z={ns_sz + BLOCK_HALF:.3f})")
    print("=" * 64)
    if skip_nav:
        print("  --skip-nav: 跳過導航 + 對齊，直接從第 4 步開始")
    if pick_only:
        print("  --pick-only: 只抓起 red_block，不去 nightstand 放下")
    if dry_run:
        print("  --dry-run: 只檢查參數，不連 ROS")
    print()

    if dry_run:
        return

    # Init ROS
    rospy.init_node('put_on_table_test', anonymous=True)
    rospy.Subscriber('/ground_truth/odom', Odometry, odom_cb, queue_size=5)
    rospy.Subscriber('/gazebo/link_states', LinkStates, link_states_cb, queue_size=1)
    _cmd_pub     = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    _arm_pub     = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)
    _gripper_pub = rospy.Publisher('/hand_controller/command', JointTrajectory, queue_size=1)
    _model_pub   = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    _follower_attach_pub = rospy.Publisher('/block_follower/attach', String, queue_size=1)
    _follower_detach_pub = rospy.Publisher('/block_follower/detach', String, queue_size=1)
    _mb_client   = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print("等待 move_base...")
    if not _mb_client.wait_for_server(rospy.Duration(10)):
        print("✗ move_base 連不上，請先跑 ./run_teb_compare.sh")
        sys.exit(1)
    rospy.sleep(1.0)
    print("就緒。\n")

    # ═══════════════════════════════════════════════════════════════
    # CoffeeTable → pick red_block → NightStand → drop 流程：
    #   1. 開夾爪 + arm 歸 ARM_CLAMP (確保乾淨起點)
    #   2. teleport red_block 到 CoffeeTable 桌面 place 點
    #   3. 導航 (帶著 ARM_CLAMP 姿態走)
    #   4. arm 切換到 TABLE_POSE (專屬抓取姿態)
    #   5. 對齊朝向 table center
    #   6. teleport block 到夾爪 (對齊到夾爪實際位置)
    #   7. 夾緊 + attach follower (block 之後跟著 link5)
    #   8. 導航前才回 ARM_CLAMP (避免抓取後立刻動臂穿模)
    #   9. 導航到 NightStand approach
    #  10. 抬臂到 ARM_PUT
    #  11. 對齊 NightStand
    #  12. detach follower + 開夾爪 + teleport 到 NightStand place 點
    # ═══════════════════════════════════════════════════════════════
    # Step 1: 起點乾淨
    print("[1/8] 起點：開夾爪 + arm 歸 ARM_CLAMP")
    set_gripper(GRIP_OPEN, secs=0.5)
    set_arm(ARM_CLAMP, secs=1.5)

    # Step 2: spawn block 到 CoffeeTable 桌面 place 點
    print(f"\n[2/8] teleport red_block 到 CoffeeTable 桌面 ({px:.2f}, {py:.2f}, {sz+BLOCK_HALF:.3f})")
    teleport_block(BLOCK_NAME, px, py, sz + BLOCK_HALF, duration=0.4)
    rospy.sleep(0.3)

    if not skip_nav:
        # Step 3: nav (arm 還在 ARM_CLAMP)
        print(f"\n[3/8] 導航到 approach ({ax:.2f}, {ay:.2f})")
        if not navigate_to(ax, ay, ayaw):
            print("✗ 導航失敗，中止")
            return
        if coffee_nudge > 0:
            print(f"    → CoffeeTable 前進補距 {coffee_nudge:.2f} m")
            drive_forward(coffee_nudge)

    # Step 4: arm 切換到 TABLE_POSE
    print(f"\n[{'1/4' if skip_nav else '4/8'}] arm 切到 [{pose_label}] 抓取姿態 = {arm_pose}")
    set_arm(arm_pose, secs=2.0)

    if not skip_nav:
        # Step 5: 對齊朝向 table center
        print(f"\n[5/8] 對齊朝向 table center ({ox:.2f}, {oy:.2f})")
        rotate_to_face(ox, oy)

    # Step 6: teleport block 到夾爪 (snap 對齊)
    # 參考 pick_and_place.py：優先用 link7/link9 的真實世界座標。
    c = get_gripper_center_world()
    if c is None:
        rx, ry = _odom_xy
        ryaw = _odom_yaw
        gripper_z = get_gripper_z_estimate(arm_pose)
        gripper_x = rx + 0.40 * math.cos(ryaw)
        gripper_y = ry + 0.40 * math.sin(ryaw)
        print("    ! link_states 還沒拿到夾爪位置，暫用粗估 gripper 位置")
    else:
        gripper_x, gripper_y, gripper_z = c
    print(f"\n[{'2/4' if skip_nav else '6/8'}] snap block 到夾爪位置 ({gripper_x:.2f}, {gripper_y:.2f}, {gripper_z:.2f})")
    teleport_block(BLOCK_NAME, gripper_x, gripper_y, gripper_z, duration=0.3)

    # Step 7: 夾緊 + attach follower
    print(f"\n[{'3/4' if skip_nav else '7/8'}] 夾緊 + attach block_follower")
    set_gripper(GRIP_CLOSE, secs=0.8)
    msg = String()
    msg.data = f"mini_mec_six_arm::link5,{BLOCK_NAME},{gripper_x:.4f},{gripper_y:.4f},{gripper_z:.4f}"
    _follower_attach_pub.publish(msg)
    print(f"    → publish: {msg.data}")
    rospy.sleep(0.5)

    if pick_only:
        print("\n  ✓ pick-only 完成：red_block 已抓起，手臂保持 coffeetable 抓取姿態")
        return

    # Step 8: 保持 coffeetable 姿態先轉向 NightStand，再回搬運姿態。
    print(f"\n[8/8] 保持 coffeetable 姿態，先轉向 {place_key}")
    rotate_to_face(ns_ox, ns_oy)

    print(f"\n[9/12] 轉向後 arm 回 ARM_CLAMP 搬運姿態")
    set_arm(ARM_CLAMP, secs=2.0)

    print(f"\n[10/12] 導航到 {place_key} approach ({ns_ax:.2f}, {ns_ay:.2f})")
    if not navigate_to(ns_ax, ns_ay, ns_yaw, timeout=120):
        print("✗ 導航到 NightStand 失敗，中止")
        return

    print(f"\n[11/12] arm 切到 ARM_PUT 放置姿態 = {ARM_PUT}")
    set_arm(ARM_PUT, secs=2.0)

    print(f"\n[12/13] 對齊朝向 {place_key} center ({ns_ox:.2f}, {ns_oy:.2f})")
    rotate_to_face(ns_ox, ns_oy)

    print(f"\n[13/13] 放到 {place_key} 面上 ({ns_px:.2f}, {ns_py:.2f}, {ns_sz + BLOCK_HALF:.3f})")
    _follower_detach_pub.publish(String(data="detach"))
    rospy.sleep(0.4)
    set_gripper(GRIP_OPEN, secs=0.8)
    teleport_block(BLOCK_NAME, ns_px, ns_py, ns_sz + BLOCK_HALF, duration=0.4)

    print("\n" + "=" * 64)
    print("  ✓ 完成。檢查 Gazebo：")
    print(f"    a. red_block 是否已從 CoffeeTable 被夾起")
    print(f"    b. red_block 是否放在 {place_key} 面上")
    print(f"    c. 放下後手臂保持 ARM_PUT，不自動回 ARM_CLAMP")
    print(f"    d. 調姿勢命令：python3 put_on_table_test.py coffeetable --skip-nav --pick-only --pose 0,-0.35,0.95,1.3,0")
    print("=" * 64)


if __name__ == '__main__':
    main()
