"""
ros_bridge.py - Big Brain 與 ROS/move_base 的橋接層

功能：
  - 初始化 rospy node
  - 訂閱 /odom → 提供 get_current_pos(), get_current_orientation()
  - 讀取 ~/.ros/semantic_map.yaml → 提供 get_obj_info(name)
  - move_to_goal(x, y, yaw=None) → 發 /move_base_simple/goal + 等待結果
  - 失敗或超時拋 RuntimeError

用法：
  import ros_bridge
  ros_bridge.init()          # 初始化（只需調用一次）
  x, y = ros_bridge.get_current_pos()
  ros_bridge.move_to_goal(13.0, -0.2)
"""
import math
import os
import threading
import time

import yaml
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.srv import GetModelState
import actionlib

from config import SCENE_YAML_PATH as GAZEBO_SCENE_PATH
SEMANTIC_MAP_PATH  = os.path.expanduser("~/.ros/semantic_map.yaml")
MOVE_TIMEOUT_SEC  = 120.0  # 導航超時（秒）

# ── 動態物件清單 ─────────────────────────────────────────────
# 這些物件位置會在 runtime 改變（被機器人搬、被瞬移），不能信 yaml。
# 查詢時直接從 Gazebo /gazebo/get_model_state 拿即時座標。
DYNAMIC_OBJECTS = {'red_block', 'yellow_block', 'blue_block', 'Coke'}
# 機器人停車時車體中心到目標物件的距離 (m)，僅用於動態物件（red_block 等小東西）。
ARM_REACH_M    = 0.40

# 靜態家具的 approach 距離 = bbox_half (max of x,y) + buffer
# yaml 預設 buffer ~0.30m，這裡按物件類型微調
# 桌子要離遠一點看起來才不會撞、垃圾桶要靠近一點手才搆得到
APPROACH_BUFFER_TABLE = 0.35    # 桌子：bbox + 0.35m（放置時需夾爪伸到桌面上方）
APPROACH_BUFFER_TRASH = 0.20    # 垃圾桶：bbox + 0.20m，近一點（put 時還會 drive_forward 補）
APPROACH_BUFFER_CHAIR = 0.30    # 椅子/沙發：跟 yaml 預設一致

# ── 全局狀態 ─────────────────────────────────────────────────
_lock       = threading.Lock()
_odom_pos   = (0.0, 0.0)
_odom_yaw   = 0.0
_smap       = {}           # 語義地圖緩存
_initialized = False
_move_client = None        # actionlib client
_get_model_state = None    # /gazebo/get_model_state ServiceProxy
_cmd_pub    = None         # /cmd_vel publisher（給 rotate_to_face 用）
_arm_pub    = None         # /arm_controller/command publisher（給 tuck_arm 用）

# 手臂收納姿態：直立向上，不擋相機 FOV / 不撞牆
ARM_TRAVEL_POSE = [0.0, 0.0, 0.0, 0.0, 0.0]   # joint1-5 都歸零 = 直立
# 手臂 ready/transport 姿態：跟 arm_task_server.ARM_CLAMP 同步
ARM_CLAMP_POSE  = [0.0, -1.1, 0.66, 1.0, 0.0]


def init():
    """初始化 ROS node + 訂閱 /odom。run_teb.sh 已跑時調用。"""
    global _initialized, _move_client, _get_model_state, _cmd_pub, _arm_pub
    if _initialized:
        return
    try:
        rospy.init_node('big_brain', anonymous=False, disable_signals=True)
    except rospy.exceptions.ROSException:
        pass  # 已有 node，忽略

    rospy.Subscriber('/ground_truth/odom', Odometry, _odom_cb, queue_size=5)

    # /cmd_vel publisher (for rotate_to_face)
    _cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # /arm_controller/command publisher (for tuck_arm)
    _arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)

    # actionlib client for move_base
    _move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("[ros_bridge] Waiting for move_base action server...")
    connected = _move_client.wait_for_server(timeout=rospy.Duration(10.0))
    if not connected:
        rospy.logwarn("[ros_bridge] move_base action server not available, "
                      "will use /move_base_simple/goal as fallback")
        _move_client = None

    # /gazebo/get_model_state for DYNAMIC_OBJECTS lookup
    try:
        rospy.wait_for_service('/gazebo/get_model_state', timeout=5.0)
        _get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        rospy.loginfo("[ros_bridge] /gazebo/get_model_state ready (dynamic objects enabled)")
    except rospy.ROSException:
        rospy.logwarn("[ros_bridge] /gazebo/get_model_state unavailable; "
                      "dynamic objects %s will fall back to yaml", DYNAMIC_OBJECTS)
        _get_model_state = None

    _reload_smap()
    _initialized = True
    rospy.loginfo("[ros_bridge] Ready. pos=%.2f,%.2f", *_odom_pos)


# ── 對齊朝向 ──────────────────────────────────────────────────
def rotate_to_face(target_x: float, target_y: float,
                   timeout: float = 10.0, tol: float = 0.03) -> bool:
    """原地旋轉，讓車頭（base_link +X，即手臂與相機方向）對準目標座標。

    tol=0.03 rad (≈ 1.7°) → 在 0.55 m 距離下，橫向偏移 ~1 cm。
    P-controller: gain=1.2, max omega=1.2 rad/s, rate=20 Hz。
    最後階段加 ramp-down 防止震盪（小誤差時自動降速）。

    回傳：True 表示對準（誤差 < tol），False 表示超時。
    """
    if not _initialized:
        raise RuntimeError("ros_bridge.rotate_to_face: not initialized")

    rate = rospy.Rate(20)
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        if rospy.Time.now().to_sec() - t0 > timeout:
            _cmd_pub.publish(Twist())
            rospy.logwarn("[ros_bridge] rotate_to_face timeout")
            return False

        with _lock:
            rx, ry = _odom_pos
            ryaw = _odom_yaw
        target_yaw = math.atan2(target_y - ry, target_x - rx)
        err = math.atan2(math.sin(target_yaw - ryaw),
                         math.cos(target_yaw - ryaw))

        if abs(err) < tol:
            _cmd_pub.publish(Twist())  # 停
            return True

        # P-controller with ramp-down near goal to avoid overshoot at tight tolerance.
        # 當 |err| < 0.15 rad（≈8.6°），最大 omega 線性降到 0.3 rad/s
        max_omega = 1.2 if abs(err) > 0.15 else max(0.3, abs(err) * 4.0)
        cmd = Twist()
        cmd.angular.z = max(-max_omega, min(max_omega, err * 1.5))
        _cmd_pub.publish(cmd)
        rate.sleep()

    _cmd_pub.publish(Twist())
    return False


# ── /odom 回調 ────────────────────────────────────────────────
def _odom_cb(msg):
    global _odom_pos, _odom_yaw
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )
    with _lock:
        _odom_pos = (p.x, p.y)
        _odom_yaw = yaw


def get_current_pos():
    """回傳 (x, y) 機器人當前位置（map 坐標系，單位 m）"""
    with _lock:
        return _odom_pos


def get_current_orientation():
    """回傳 yaw 角（弧度）"""
    with _lock:
        return _odom_yaw


# ── 語義地圖 ──────────────────────────────────────────────────
def _reload_smap():
    global _smap
    for path in [GAZEBO_SCENE_PATH, SEMANTIC_MAP_PATH]:
        if os.path.exists(path):
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            if data:
                _smap = data
                rospy.loginfo("[ros_bridge] Loaded semantic map from %s: %s", path, list(_smap.keys()))
                return
    rospy.logwarn("[ros_bridge] No semantic map found, tried: %s, %s", SEMANTIC_MAP_PATH, GAZEBO_SCENE_PATH)
    _smap = {}


def get_obj_info(name: str) -> dict:
    """
    依物件名稱查詢語義地圖。
    - 動態物件（DYNAMIC_OBJECTS）→ 即時查 Gazebo /gazebo/get_model_state
    - 靜態家具 → yaml 快照
    找不到時重新加載地圖再查一次；仍找不到拋 KeyError。
    """
    # 動態物件：即時拿 Gazebo 真實位置，approach 點根據機器人當下朝向算
    if name in DYNAMIC_OBJECTS and _get_model_state is not None:
        return _get_dynamic_obj_info(name)

    if name in _smap:
        return _smap[name]
    _reload_smap()
    if name in _smap:
        return _smap[name]
    raise KeyError(f"[ros_bridge] Object '{name}' not found in semantic map. "
                   f"Available: {list(_smap.keys())}")


def _get_dynamic_obj_info(name: str) -> dict:
    """從 Gazebo 拿動態物件即時座標，回傳跟靜態 yaml 同 schema 的 dict。

    approach 點計算策略：
        從機器人**當前位置**畫一條線到物件，沿那條線往回退 ARM_REACH_M 公尺。
        車頭朝向那條線（這樣手臂正好可以伸到物件）。

    這個策略對「物件在開放空間」最自然（紅塊在地上、桌邊都適用），
    因為機器人不需要繞遠路就能對準。
    """
    try:
        resp = _get_model_state(name, '')
    except rospy.ServiceException as e:
        raise KeyError(f"[ros_bridge] /gazebo/get_model_state failed for '{name}': {e}")
    if not resp.success:
        raise KeyError(f"[ros_bridge] Gazebo has no model '{name}': {resp.status_message}")

    bx = resp.pose.position.x
    by = resp.pose.position.y
    bz = resp.pose.position.z

    # 從機器人當前位置看過去
    rx, ry = _odom_pos
    dx, dy = bx - rx, by - ry
    dist = math.hypot(dx, dy)
    if dist < 1e-3:
        # 機器人剛好在物件上 → 預設往 +X 方向退
        ax, ay, ayaw = bx - ARM_REACH_M, by, 0.0
    else:
        ux, uy = dx / dist, dy / dist                          # 機器人 → 物件 單位向量
        ax = bx - ARM_REACH_M * ux                             # 從物件往回退 ARM_REACH_M
        ay = by - ARM_REACH_M * uy
        ayaw = math.atan2(uy, ux)                              # 車頭對準物件

    return {
        'object_x': bx, 'object_y': by,
        'approach_x': ax, 'approach_y': ay, 'approach_yaw': ayaw,
        'surface_z': bz,                                       # 物件當前 z（地上 ≈ 0.025，桌上 ≈ 0.5）
        'bbox_half_x': 0.025, 'bbox_half_y': 0.025,            # red_block 5cm 立方
        'size_w': 0.05, 'size_h': 0.05,
        '__dynamic__': True,                                   # debug 標記
    }


def get_obj_approach_pos(name: str):
    """回傳 (approach_x, approach_y, approach_yaw)。

    靜態家具：按類型用 bbox + buffer 重算 approach 距離（保持 yaml 的方向）
    動態物件：用 ros_bridge 算的 ARM_REACH_M

    對稱物件（yaml 有 approach_candidates 列表）：
        從多個候選中挑離機器人最近的可達點，回傳該方向。
        只對 yaml 顯式標 candidates 的物件啟用；其他物件邏輯完全不變。
    """
    info = get_obj_info(name)
    ax, ay = info['approach_x'], info['approach_y']
    ayaw   = info.get('approach_yaw', 0.0)

    # 前置「已就位」檢查只對靜態物件生效。
    # 對動態物件（red/blue/yellow/Coke）必須每次重算 approach，
    # 否則會把「錯誤的、隨車更新的 approach 點」誤判為已就位。
    is_dynamic_target = bool(info.get('__dynamic__')) or name.lower() in {
        'red_block', 'blue_block', 'yellow_block', 'coke'
    }
    if not is_dynamic_target:
        try:
            rx, ry = get_current_pos()
            AT_APPROACH_TOL = 0.30  # m  ← robot 在 approach 點 30cm 內視為「已就位」
            # 蒐集所有合法 approach 點：canonical + candidates（如果有）
            all_approaches = [{'x': ax, 'y': ay, 'yaw': ayaw}]
            cands = info.get('approach_candidates')
            if cands and isinstance(cands, list):
                all_approaches.extend(cands)
            for p in all_approaches:
                d = math.hypot(float(p['x']) - rx, float(p['y']) - ry)
                if d <= AT_APPROACH_TOL:
                    pyaw = float(p.get('yaw', ayaw))
                    rospy.loginfo(
                        "[ros_bridge] '%s' approach: robot already within %.2fm of an "
                        "approach point (dist=%.2f) — staying put at (%.2f,%.2f), no re-nav",
                        name, AT_APPROACH_TOL, d, rx, ry
                    )
                    return (rx, ry, pyaw)
            # 補：LLM 透過 find_reachable_approach_to 自選的 wp 不在 yaml approach_candidates 裡，
            # 但只要 robot 已經在物件 arm-reach 範圍內就視為「已就位」，避免 put_down 又
            # 強行 nav 回 canonical approach（會原地轉圈 + 拉長 TEB 路徑）。
            try:
                obj_x = float(info.get('object_x', ax))
                obj_y = float(info.get('object_y', ay))
                d_obj = math.hypot(obj_x - rx, obj_y - ry)
                ARM_REACH_TOL = 0.60  # m  ← 機器人在物件 60cm 內視為已就位（涵蓋 arm reach）
                if d_obj <= ARM_REACH_TOL:
                    # 以目前位置為 approach，朝向 yaw 指向物件中心
                    face_yaw = math.atan2(obj_y - ry, obj_x - rx)
                    rospy.loginfo(
                        "[ros_bridge] '%s' approach: robot already within %.2fm of object center "
                        "(dist=%.2f) — staying put at (%.2f,%.2f), face yaw=%.2f, no re-nav",
                        name, ARM_REACH_TOL, d_obj, rx, ry, face_yaw
                    )
                    return (rx, ry, face_yaw)
            except Exception:
                pass
        except Exception as _e:
            pass  # 安全退回原邏輯

    # 對稱物件 approach 選擇權交回 LLM — primitive 不再偷偷挑側。
    # LLM 想用不同側時應顯式呼叫 find_reachable_approach_to(name)，
    # 然後 move_to_xy(候選) + rotate_to_face(物件中心)。
    # 這層改動讓 LLM 的空間決策真實可見，也讓「選錯側 → adjust」鏈條能被觸發。
    #
    # 仍保留 BIGBRAIN_AUTO_PICK_SIDE=1 作為「向後相容開關」：設了就恢復舊行為。
    auto_pick = os.environ.get('BIGBRAIN_AUTO_PICK_SIDE', '').strip() in ('1', 'true', 'True')
    candidates = info.get('approach_candidates')
    if auto_pick and candidates and isinstance(candidates, list) and len(candidates) > 1:
        try:
            rx, ry = get_current_pos()
            sorted_cands = sorted(
                candidates,
                key=lambda c: math.hypot(float(c['x']) - rx, float(c['y']) - ry)
            )
            for c in sorted_cands:
                cx, cy = float(c['x']), float(c['y'])
                if is_reachable(cx, cy):
                    cyaw = float(c.get('yaw', ayaw))
                    rospy.loginfo(
                        "[ros_bridge] '%s' approach (AUTO_PICK_SIDE on): picked side=%s "
                        "(%.2f,%.2f) instead of canonical (%.2f,%.2f)",
                        name, c.get('side', '?'), cx, cy, ax, ay
                    )
                    return (cx, cy, cyaw)
            rospy.logwarn("[ros_bridge] '%s': no candidate reachable, using canonical (%.2f,%.2f)",
                          name, ax, ay)
        except Exception as e:
            rospy.logwarn("[ros_bridge] '%s': candidate selection failed: %s, using canonical",
                          name, e)

    if info.get('__dynamic__'):
        return (ax, ay, ayaw)

    # CoffeeTable 已在 gazebo_scene.yaml / get_scene.py 裡手動調好：
    # 從 north 方向正對桌面長邊。不要再用通用 table buffer 拉遠。
    if 'coffeetable' in name.lower():
        return (ax, ay, ayaw)

    # 按類型用 bbox 重算距離（保持 yaml 方向）
    obj_x, obj_y = info['object_x'], info['object_y']
    bbox_half = max(info.get('bbox_half_x', 0.25), info.get('bbox_half_y', 0.25))
    buffer = _type_approach_buffer(name)
    if buffer is not None:
        new_dist = bbox_half + buffer
        dx, dy = ax - obj_x, ay - obj_y
        dist = math.hypot(dx, dy)
        if dist > 1e-3:
            scale = new_dist / dist
            ax = obj_x + dx * scale
            ay = obj_y + dy * scale
            rospy.loginfo("[ros_bridge] '%s' approach: bbox %.2f + buffer %.2f = %.2f m (原 %.2f m)",
                          name, bbox_half, buffer, new_dist, dist)
    return (ax, ay, ayaw)


def _type_approach_buffer(name: str):
    """根據物件名稱回傳 approach buffer (bbox 之外的距離)；無匹配則 None。"""
    lower = name.lower()
    if 'trash' in lower or 'bin' in lower:
        return APPROACH_BUFFER_TRASH
    if any(k in lower for k in ('table', 'desk', 'bench', 'nightstand')):
        return APPROACH_BUFFER_TABLE
    if any(k in lower for k in ('chair', 'sofa')):
        return APPROACH_BUFFER_CHAIR
    return None


def get_obj_xy(name: str):
    """回傳物件本身在地圖中的坐標 (object_x, object_y)"""
    info = get_obj_info(name)
    return (info['object_x'], info['object_y'])


def get_obj_size(name: str):
    """回傳 (size_w, size_h)，若未記錄則回傳 (-1, -1)"""
    info = get_obj_info(name)
    return (info.get('size_w', -1.0), info.get('size_h', -1.0))


def get_obj_color_rgb(name: str):
    """回傳 (r, g, b) 平均顏色，若未記錄則回傳 (-1, -1, -1)"""
    info = get_obj_info(name)
    r = info.get('color_r', -1)
    g = info.get('color_g', -1)
    b = info.get('color_b', -1)
    return (r, g, b)


def get_available_objects():
    """回傳目前語義地圖中所有物件名稱列表"""
    _reload_smap()
    return list(_smap.keys())


# ── 導航 ──────────────────────────────────────────────────────
def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


def _set_arm_pose(pose, secs: float = 1.2) -> None:
    """直接送 5-DOF 關節指令到 /arm_controller/command。"""
    if _arm_pub is None:
        return
    traj = JointTrajectory()
    traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    traj.header.stamp = rospy.Time.now()
    pt = JointTrajectoryPoint()
    pt.positions = pose
    pt.time_from_start = rospy.Duration(secs)
    traj.points = [pt]
    _arm_pub.publish(traj)
    rospy.sleep(secs + 0.1)


def tuck_arm(secs: float = 1.2) -> None:
    """收手臂到 ARM_TRAVEL_POSE（直立），不擋深度相機 FOV。
    用在 nav 之前，把孤伸的手臂收起來；block_follower plugin 仍會跟著 link5。
    """
    _set_arm_pose(ARM_TRAVEL_POSE, secs)


def restore_arm_clamp(secs: float = 1.2) -> None:
    """還原手臂到 ARM_CLAMP_POSE（前下方搬運/待機姿態）。
    用在 nav 之後，恢復「ready」狀態。後續呼叫 /arm/pick 或 /arm/put 會接續。
    """
    _set_arm_pose(ARM_CLAMP_POSE, secs)


def drive_forward(distance: float, speed: float = 0.15, timeout: float = 8.0) -> bool:
    """沿車頭方向直線前進 distance 公尺（用里程計計距）。

    用在 pick/put 前，把 navigate 的殘差補完，讓夾爪對準物件。
    distance < 0 視為倒退（一般不用，這裡留著相容）。
    """
    if not _initialized:
        raise RuntimeError("[ros_bridge] drive_forward: not initialized")
    if abs(distance) < 1e-3:
        return True
    rx0, ry0 = _odom_pos
    rate = rospy.Rate(20)
    t0 = rospy.Time.now().to_sec()
    cmd = Twist()
    cmd.linear.x = speed if distance > 0 else -speed
    target = abs(distance)
    while not rospy.is_shutdown():
        if rospy.Time.now().to_sec() - t0 > timeout:
            break
        rx, ry = _odom_pos
        if math.hypot(rx - rx0, ry - ry0) >= target:
            break
        _cmd_pub.publish(cmd)
        rate.sleep()
    _cmd_pub.publish(Twist())
    return True


def move_to_goal(x: float, y: float, yaw: float = None, tuck: bool = False) -> bool:
    """
    發送導航目標到 move_base，等待結果。
    yaw=None 時使用當前朝向。
    tuck=True 時：nav 前 tuck 手臂直立（清相機 FOV），nav 後還原到 ARM_CLAMP。
        預設 False — 手臂維持 ARM_CLAMP 全程（big_brain.run_once 任務開頭已強制設）。
        若 task 需要極乾淨的相機 FOV（譬如純導航 + 視覺辨識）才設 True。
    成功回傳 True，失敗/超時拋 RuntimeError。
    """
    if not _initialized:
        raise RuntimeError("[ros_bridge] Not initialized. Call ros_bridge.init() first.")

    if tuck:
        rospy.loginfo("[ros_bridge] tucking arm before navigation")
        tuck_arm(secs=1.2)

    if yaw is None:
        yaw = get_current_orientation()

    goal_msg = MoveBaseGoal()
    goal_msg.target_pose.header.frame_id = 'map'
    goal_msg.target_pose.header.stamp    = rospy.Time.now()
    goal_msg.target_pose.pose.position.x = x
    goal_msg.target_pose.pose.position.y = y
    goal_msg.target_pose.pose.position.z = 0.0
    goal_msg.target_pose.pose.orientation = _yaw_to_quaternion(yaw)

    if _move_client is not None:
        _move_client.cancel_all_goals()   # 清除舊 goal，避免 transition callback 警告
        rospy.sleep(0.1)
        rospy.loginfo("[ros_bridge] Sending goal (%.2f, %.2f, yaw=%.2f)", x, y, yaw)
        _move_client.send_goal(goal_msg)

        # 等待結果
        finished = _move_client.wait_for_result(rospy.Duration(MOVE_TIMEOUT_SEC))
        if not finished:
            _move_client.cancel_goal()
            raise RuntimeError(f"[ros_bridge] Navigation timeout ({MOVE_TIMEOUT_SEC}s) "
                               f"to ({x:.2f}, {y:.2f})")

        state = _move_client.get_state()
        from actionlib_msgs.msg import GoalStatus
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("[ros_bridge] Reached goal (%.2f, %.2f)", x, y)
            if tuck:
                rospy.loginfo("[ros_bridge] restoring arm to ARM_CLAMP")
                restore_arm_clamp(secs=1.2)
            return True
        elif state == GoalStatus.PREEMPTED:
            # PREEMPTED = cmd_vel_to_airsim 偵測到卡住取消了目標，重試一次
            rospy.logwarn("[ros_bridge] Goal preempted (stuck detection), retrying once...")
            rospy.sleep(1.0)
            _move_client.send_goal(goal_msg)
            finished = _move_client.wait_for_result(rospy.Duration(MOVE_TIMEOUT_SEC))
            if not finished:
                _move_client.cancel_goal()
                raise RuntimeError(f"[ros_bridge] Navigation timeout on retry to ({x:.2f}, {y:.2f})")
            if _move_client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("[ros_bridge] Reached goal on retry (%.2f, %.2f)", x, y)
                if tuck:
                    rospy.loginfo("[ros_bridge] restoring arm to ARM_CLAMP")
                    restore_arm_clamp(secs=1.2)
                return True
            raise RuntimeError(f"[ros_bridge] Navigation failed after retry (state={_move_client.get_state()}) "
                               f"to ({x:.2f}, {y:.2f})")
        else:
            raise RuntimeError(f"[ros_bridge] Navigation failed (state={state}) "
                               f"to ({x:.2f}, {y:.2f})")
    else:
        # Fallback: 發 PoseStamped topic（不等待）
        _simple_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
        rospy.sleep(0.3)
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp    = rospy.Time.now()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = _yaw_to_quaternion(yaw)
        _simple_pub.publish(ps)
        rospy.loginfo("[ros_bridge] (fallback) Published /move_base_simple/goal (%.2f, %.2f)", x, y)

        # 輪詢 /odom 等待到達
        deadline = time.time() + MOVE_TIMEOUT_SEC
        while time.time() < deadline:
            cx, cy = get_current_pos()
            dist = math.hypot(cx - x, cy - y)
            if dist < 0.5:
                rospy.loginfo("[ros_bridge] Reached goal (%.2f, %.2f) dist=%.2f", x, y, dist)
                return True
            time.sleep(0.5)
        raise RuntimeError(f"[ros_bridge] Navigation timeout ({MOVE_TIMEOUT_SEC}s) "
                           f"to ({x:.2f}, {y:.2f})")


def is_reachable(x: float, y: float, tolerance: float = 0.3) -> bool:
    """查詢 move_base 從當前位置到 (x, y) 是否能規劃出全局路徑。

    不會動車。內部呼叫 /move_base/make_plan service，typical latency 50-150ms。
    回 True：global planner 找得到 path → 點在 free space，可嘗試導航。
    回 False：點在牆裡 / 完全被障礙包圍 / 不可達 → LLM 應換 candidate。
    """
    from nav_msgs.srv import GetPlan
    from geometry_msgs.msg import PoseStamped

    if not _initialized:
        raise RuntimeError("[ros_bridge] Not initialized. Call ros_bridge.init() first.")

    try:
        rospy.wait_for_service('/move_base/make_plan', timeout=2.0)
        plan_srv = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
    except (rospy.ROSException, rospy.ServiceException) as e:
        rospy.logwarn(f"[ros_bridge] is_reachable: make_plan service unavailable: {e}")
        return True  # 服務不可用時退保守：假設可達（讓 LLM 試試看）

    cx, cy = get_current_pos()
    cyaw = get_current_orientation()

    start = PoseStamped()
    start.header.frame_id = 'map'
    start.header.stamp = rospy.Time.now()
    start.pose.position.x = float(cx)
    start.pose.position.y = float(cy)
    start.pose.orientation.z = math.sin(cyaw / 2.0)
    start.pose.orientation.w = math.cos(cyaw / 2.0)

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.orientation.w = 1.0

    try:
        resp = plan_srv(start=start, goal=goal, tolerance=float(tolerance))
        reachable = len(resp.plan.poses) > 0
        rospy.loginfo(f"[ros_bridge] is_reachable({x:.2f},{y:.2f}) → {reachable} "
                      f"(plan poses={len(resp.plan.poses)})")
        return reachable
    except rospy.ServiceException as e:
        rospy.logwarn(f"[ros_bridge] is_reachable: service call failed: {e}")
        return True  # 同上：失敗時保守 True


def list_obstacles_near(x: float, y: float, radius: float = 3.0) -> list:
    """通用空間查詢：回傳指定點周圍 radius 範圍內所有 named obstacles。

    回傳格式 list of dict：
        [{"name": str, "x": float, "y": float, "hx": float, "hy": float, "dist": float}, ...]
    按距離 (dist) 由近到遠排序。

    用途：LLM 規劃 recovery 時要知道附近有什麼障礙，自己決定怎麼繞。
    不寫死哪些物件算「障礙」— 凡是 semantic map 裡有 bbox 的 named entity 都列出來。
    """
    if not _smap:
        _reload_smap()
    out = []
    for name, info in (_smap or {}).items():
        if not isinstance(info, dict):
            continue
        ox = info.get("object_x")
        oy = info.get("object_y")
        if ox is None or oy is None:
            continue
        dist = math.hypot(float(ox) - float(x), float(oy) - float(y))
        if dist <= float(radius):
            out.append({
                "name": name,
                "x": float(ox),
                "y": float(oy),
                "hx": float(info.get("bbox_half_x", 0.0)),
                "hy": float(info.get("bbox_half_y", 0.0)),
                "dist": round(dist, 3),
            })
    out.sort(key=lambda d: d["dist"])
    return out


def find_reachable_approach_to(obj_name: str, dist: float = 0.55,
                               num_angles: int = 8) -> tuple:
    """繞 obj_name 360° 試 num_angles 個候選 approach 點，回最近的可達點。

    用途：當 yaml 寫死的 canonical approach 點被障礙物擋住時（例如 trash 西側
    被茶几堵），LLM 可以查另一側是否可達。對稱物件（trash、ball）任何角度都
    能放，不用死守 canonical 方向。

    參數：
      obj_name: 語義地圖裡的物件名
      dist:     候選點到 obj 中心的距離（公尺）
      num_angles: 360° 內均勻取樣的角度數（8 = 每 45°）

    回傳：
      (x, y) 元組 — 最近的可達點
      None       — 所有候選都不可達
    """
    info = get_obj_info(obj_name)
    ox = float(info.get("object_x"))
    oy = float(info.get("object_y"))
    rx, ry = get_current_pos()

    candidates = []
    for i in range(num_angles):
        theta = 2.0 * math.pi * i / num_angles
        cx = ox + dist * math.cos(theta)
        cy = oy + dist * math.sin(theta)
        d_to_robot = math.hypot(cx - rx, cy - ry)
        candidates.append((cx, cy, d_to_robot))

    candidates.sort(key=lambda c: c[2])
    for cx, cy, d in candidates:
        if is_reachable(cx, cy):
            rospy.loginfo(f"[ros_bridge] find_reachable_approach_to('{obj_name}'): "
                          f"({cx:.2f},{cy:.2f}) reachable, dist_from_robot={d:.2f}")
            return (cx, cy)
    rospy.logwarn(f"[ros_bridge] find_reachable_approach_to('{obj_name}'): "
                  f"NO candidate reachable (tried {num_angles} angles at dist {dist})")
    return None


def clear_costmaps() -> bool:
    """只清 local costmap 的 obstacle_layer（laser/sensor 來源），global 完全不動。

    為什麼不用 /move_base/clear_costmaps：
      預設 Empty service 會把 global + local 兩層全部清掉。global 的 static_layer
      是 map_server 的靜態地圖（房間牆、家具），清掉之後到下次 costmap 更新前，
      global planner 可能規劃出穿牆路徑。
    做法：用 dynamic_reconfigure bounce local_costmap 的 obstacle_layer.enabled
      false → 0.15s → true。這會強制 obstacle_layer 把已累積的 sensor 資料丟掉，
      重新開始接 laser。global static_layer 完全沒被觸碰。
    若 dyn-reconfigure 介面不可用，fall back 回舊的全清行為，並記 warn。
    """
    # 路徑 A（首選）：dyn-reconfigure bounce local obstacle_layer，global 完全不動
    LOCAL_OBSTACLE = '/move_base/local_costmap/obstacle_layer'
    try:
        from dynamic_reconfigure.client import Client as _DynClient
        client = _DynClient(LOCAL_OBSTACLE, timeout=1.5)
        client.update_configuration({'enabled': False})
        rospy.sleep(0.15)
        client.update_configuration({'enabled': True})
        rospy.loginfo("[ros_bridge] clear_costmaps OK (local obstacle_layer bounced, global untouched)")
        return True
    except Exception as e_dyn:
        rospy.logwarn(f"[ros_bridge] local-only clear failed ({e_dyn}); "
                      f"falling back to full clear + static re-sync")

    # 路徑 B（fallback）：全清 → 強制 static_layer 重新從 /map 合成
    # 如果 path A 不可用就走這條：clear 兩層 → bounce global static_layer → wait
    # update tick，這樣全域 costmap 會以「乾淨 static_layer + 重新累積的 sensor」合成。
    full_cleared = False
    try:
        from std_srvs.srv import Empty
        rospy.wait_for_service('/move_base/clear_costmaps', timeout=3.0)
        rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
        full_cleared = True
        rospy.loginfo("[ros_bridge] clear_costmaps OK (full clear)")
    except Exception as e:
        rospy.logwarn(f"[ros_bridge] full clear failed: {e}")
        return False

    # 重新合成 static：bounce global static_layer 讓它從 map_server 重抓地圖
    try:
        from dynamic_reconfigure.client import Client as _DynClient
        sc = _DynClient('/move_base/global_costmap/static_layer', timeout=1.5)
        sc.update_configuration({'enabled': False})
        rospy.sleep(0.10)
        sc.update_configuration({'enabled': True})
        rospy.sleep(0.30)   # 給 1-2 個 update tick 把 static 合進 costmap
        rospy.loginfo("[ros_bridge] static_layer re-synced after full clear")
    except Exception as e_static:
        rospy.logwarn(f"[ros_bridge] static_layer re-sync failed: {e_static} "
                      f"(static will rebuild on next natural update tick ~0.2s)")
        rospy.sleep(0.30)  # 退化做法：靠 update_frequency 自然刷新
    return full_cleared


def rotate_by(angle_rad: float, timeout: float = 10.0, tol: float = 0.03) -> bool:
    """原地旋轉指定角度（相對當前朝向，弧度）。

    angle_rad > 0 → 左轉（CCW）
    angle_rad < 0 → 右轉（CW）
    回 True = 達到目標角度，False = 超時。

    純角度版的 rotate_to_face — 不需要算虛擬目標點。
    用 /cmd_vel 直接控制角速度，繞過 TEB。
    """
    if not _initialized:
        raise RuntimeError("ros_bridge.rotate_by: not initialized")

    with _lock:
        start_yaw = _odom_yaw
    target_yaw = math.atan2(math.sin(start_yaw + angle_rad),
                            math.cos(start_yaw + angle_rad))

    rate = rospy.Rate(20)
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        if rospy.Time.now().to_sec() - t0 > timeout:
            _cmd_pub.publish(Twist())
            rospy.logwarn("[ros_bridge] rotate_by timeout")
            return False

        with _lock:
            cur_yaw = _odom_yaw
        err = math.atan2(math.sin(target_yaw - cur_yaw),
                         math.cos(target_yaw - cur_yaw))

        if abs(err) < tol:
            _cmd_pub.publish(Twist())
            return True

        max_omega = 1.2 if abs(err) > 0.15 else max(0.3, abs(err) * 4.0)
        cmd = Twist()
        cmd.angular.z = max(-max_omega, min(max_omega, err * 1.5))
        _cmd_pub.publish(cmd)
        rate.sleep()
