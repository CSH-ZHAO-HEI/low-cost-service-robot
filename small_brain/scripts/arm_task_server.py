#!/usr/bin/env python3
"""
arm_task_server.py
提供兩個 ROS Service：
  /arm/pick  (std_srvs/Trigger) — 抓取 /arm_task/target_name 指定的物件
  /arm/put   (std_srvs/Trigger) — 放置到 /arm_task/target_name 指定的目標位置

啟動方式：
  rosrun small_brain_sim arm_task_server.py
  或直接：python3 arm_task_server.py
"""
import os

import sys
import time
import threading
import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.msg import ModelState, LinkStates
from gazebo_msgs.srv import SetModelState, GetLinkState
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String

# link_attacher_msgs 是可選的（do_attach/do_detach 目前未被呼叫）
try:
    from gazebo_ros_link_attacher.srv import Attach, AttachRequest
    _HAS_ATTACHER = True
except ImportError:
    _HAS_ATTACHER = False

# ── 手臂 Joint 名稱 ──────────────────────────────────────────────
ARM_JOINTS  = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
GRIP_JOINTS = ['joint6', 'joint7', 'joint8', 'joint9', 'joint10', 'joint11']

GRIP_OPEN  = [ 0.45, -0.45,  0.45,  0.45,  0.45,  0.45]
GRIP_CLOSE = [-0.45,  0.45, -0.45, -0.45, -0.45, -0.45]

# ── 手臂姿態常數 ─────────────────────────────────────────────────
ARM_CLAMP        = [0.0, -1.1, 0.66, 1.0, 0.0]  # Gazebo 初始姿態，搬運姿態
PICK_JOINTS      = [0.0, -1.3, 0.66, 1.0, 0.0]  # 比 clamp 更低，地面抓取深度
ARM_PUT          = [0.0, -0.5, 0.3,  0.3, 0.0]  # 放置姿態：向前向上約 45°，張爪後方塊掉落
# 桌面抓取姿態：手臂往前上伸，夾爪約在 z=0.4-0.5m，剛好桌面高度
# 用在物件位於桌面/沙發等高處時的 pick（source_z > 0.15）
ARM_PICK_HIGH    = [0.0, -0.5, 0.3,  0.3, 0.0]  # 跟 ARM_PUT 同（forward+up 45°）— 之後可微調
PICK_HIGH_LOWER  = [0.0, -0.6, 0.5,  0.5, 0.0]  # 「下降到物件」 — 比 PICK_HIGH 略低

# ── 常數 ─────────────────────────────────────────────────────────
BLOCK_HALF      = 0.025  # 物件半高（m）
GRIP_Z_OFFSET   = -0.03  # 夾爪中心往下偏移量（Z）
# 夾爪中心 XY 校正：若方塊出現在手臂前方偏移，調大此值（正值 = 往機器人後方移）
GRIP_XY_OFFSET  = 0.0    # 沿機器人 X 軸偏移（m），需要實測調整

# ── 桌面高度字典（模糊比對） ──────────────────────────────────────
Z_HEIGHTS = {
    'kitchentable': 0.75,
    'coffeetable':  0.45,
    'desk':         0.75,
    'nightstand':   0.366,
    'trash':        0.40,
    'trashcan':     0.40,
    'bin':          0.40,
    'red_block':    0.025,
}

def get_surface_z(target_name: str) -> float:
    """模糊比對 Z_HEIGHTS，無匹配時回傳 0.025（落地）"""
    lower = target_name.lower()
    for key, z in Z_HEIGHTS.items():
        if key in lower:
            return z
    return 0.025

# ── 全局狀態 ─────────────────────────────────────────────────────
_attached      = False
_current_model = None
_attach_thread = None
_link_positions = {}

# ── ROS 物件（init 後填入） ──────────────────────────────────────
arm                = None
gripper            = None
model_pub          = None
set_model_state_srv = None
_get_link_state    = None
_follower_attach_pub = None   # → /block_follower/attach
_follower_detach_pub = None   # → /block_follower/detach


# ════════════════════════════════════════════════════════════════
# 工具函式
# ════════════════════════════════════════════════════════════════

def _on_link_states(msg):
    """Gazebo 物理頻率回調：更新快取，並在 attach 狀態下即時同步方塊位置。"""
    global _link_positions
    for name, pose in zip(msg.name, msg.pose):
        _link_positions[name] = pose.position

    # 若正在 attach，在同一 callback 直接發布方塊位置 → 延遲僅一個物理步
    if _attached and _current_model and model_pub is not None:
        c = _gripper_center_from_cache()
        if c:
            model_pub.publish(make_model_state(_current_model, c[0], c[1], c[2]))


def _gripper_center_from_cache():
    """從 _link_positions 快取計算夾爪中心（無 RPC 延遲）。"""
    p7 = _link_positions.get('mini_mec_six_arm::link7')
    p9 = _link_positions.get('mini_mec_six_arm::link9')
    if p7 is None or p9 is None:
        return None
    return (
        (p7.x + p9.x) / 2.0 + GRIP_XY_OFFSET,
        (p7.y + p9.y) / 2.0,
        (p7.z + p9.z) / 2.0 + GRIP_Z_OFFSET,
    )


def get_gripper_center():
    """取夾爪中心：優先快取，備用 service。"""
    c = _gripper_center_from_cache()
    if c is not None:
        return c
    try:
        r7 = _get_link_state('mini_mec_six_arm::link7', 'world')
        r9 = _get_link_state('mini_mec_six_arm::link9', 'world')
        if not r7.success or not r9.success:
            return None
        p7 = r7.link_state.pose.position
        p9 = r9.link_state.pose.position
        return (
            (p7.x + p9.x) / 2.0 + GRIP_XY_OFFSET,
            (p7.y + p9.y) / 2.0,
            (p7.z + p9.z) / 2.0 + GRIP_Z_OFFSET,
        )
    except Exception:
        return None


def make_model_state(model_name, x, y, z):
    msg = ModelState()
    msg.model_name = model_name
    msg.reference_frame = 'world'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    return msg


def teleport(model_name, x, y, z, duration=0.3):
    """高頻發送 ModelState 將物件瞬移到指定位置"""
    msg = make_model_state(model_name, x, y, z)
    t_end = time.time() + duration
    while time.time() < t_end:
        model_pub.publish(msg)
        time.sleep(0.005)


def detach_follower_burst(duration=0.5, rate_hz=50.0):
    """連續發 detach，避免單次 ROS 訊息因啟動/queue 時序丟失。"""
    if _follower_detach_pub is None:
        return
    msg = String(data="detach")
    period = 1.0 / rate_hz
    t_end = time.time() + duration
    while time.time() < t_end and not rospy.is_shutdown():
        _follower_detach_pub.publish(msg)
        time.sleep(period)


def set_gripper(pos, secs=1.0):
    traj = JointTrajectory()
    traj.joint_names = GRIP_JOINTS
    traj.header.stamp = rospy.Time.now()
    pt = JointTrajectoryPoint()
    pt.positions = pos
    pt.time_from_start = rospy.Duration(secs)
    traj.points = [pt]
    gripper.publish(traj)
    rospy.sleep(secs + 0.2)


def move_arm(target, is_joints=False):
    if is_joints:
        arm.set_joint_value_target(target)
    else:
        arm.set_named_target(target)
    arm.go(wait=True)
    arm.stop()


def attach_loop():
    """背景執行緒：200Hz 鎖定物件到夾爪中心"""
    global _attached, _current_model
    while _attached and not rospy.is_shutdown():
        c = get_gripper_center()
        if c and _current_model:
            model_pub.publish(make_model_state(_current_model, c[0], c[1], c[2]))
        time.sleep(0.005)  # 200 Hz


def get_link5_pos():
    """取得 link5 在世界座標的位置"""
    try:
        r = _get_link_state('mini_mec_six_arm::link5', 'world')
        if not r.success:
            return None
        p = r.link_state.pose.position
        return (p.x, p.y, p.z)
    except Exception:
        return None


def do_attach(model_name):
    """呼叫 link_attacher 服務，將 model_name 固定到 link5（未使用，備用）"""
    if not _HAS_ATTACHER:
        rospy.logwarn("gazebo_ros_link_attacher 未安裝，do_attach 跳過")
        return False
    req = AttachRequest()
    req.model_name_1 = 'mini_mec_six_arm'
    req.link_name_1  = 'link5'
    req.model_name_2 = model_name
    req.link_name_2  = 'link'
    resp = _attach_srv(req)
    rospy.loginfo(f"  attach {'成功' if resp.ok else '失敗'}")
    return resp.ok


def do_detach(model_name):
    """呼叫 link_attacher 服務，釋放 model_name（未使用，備用）"""
    if not _HAS_ATTACHER:
        rospy.logwarn("gazebo_ros_link_attacher 未安裝，do_detach 跳過")
        return False
    req = AttachRequest()
    req.model_name_1 = 'mini_mec_six_arm'
    req.link_name_1  = 'link5'
    req.model_name_2 = model_name
    req.link_name_2  = 'link'
    resp = _detach_srv(req)
    rospy.loginfo(f"  detach {'成功' if resp.ok else '失敗'}")
    return resp.ok




# ════════════════════════════════════════════════════════════════
# Service Handlers
# ════════════════════════════════════════════════════════════════

def handle_pick(req):
    """從地面或桌面拿物件。

    讀 /arm_task/source_z 決定 pick 姿態：
      source_z <= 0.15  → 地面物件，用 ARM_CLAMP → PICK_JOINTS（原邏輯）
      source_z >  0.15  → 桌面/沙發物件，用 ARM_PICK_HIGH → PICK_HIGH_LOWER
                          teleport 物件到夾爪位置（保持原 z，不會掉地上）
    """
    global _attached, _current_model, _attach_thread

    model_name = rospy.get_param('/arm_task/target_name', 'red_block')
    source_z   = float(rospy.get_param('/arm_task/source_z', BLOCK_HALF))
    high_pick  = source_z > 0.15      # 物件在桌面/沙發等高處

    rospy.loginfo(f"[arm/pick] target: {model_name}, source_z={source_z:.3f}"
                  f" → {'HIGH (table)' if high_pick else 'LOW (floor)'} pose")

    try:
        # 1. 開夾爪
        set_gripper(GRIP_OPEN)

        # 2. 對應姿態就位
        if high_pick:
            move_arm(ARM_PICK_HIGH, is_joints=True)
        else:
            move_arm(ARM_CLAMP, is_joints=True)
        rospy.sleep(0.3)

        # 3. Teleport 物件到夾爪位置
        # 高處 pick 時 teleport 到夾爪當前 z（保持物件不下落）
        # 地面 pick 時 teleport 到 z=BLOCK_HALF（原邏輯）
        c = get_gripper_center()
        if c is None:
            return TriggerResponse(success=False, message="無法取得夾爪位置")
        pick_z = c[2] if high_pick else BLOCK_HALF
        teleport(model_name, c[0], c[1], pick_z, duration=0.3)
        rospy.loginfo(f"  瞬移 → ({c[0]:.3f}, {c[1]:.3f}, {pick_z:.3f})")

        # 4. 等待畫面穩定
        rospy.sleep(2.0)

        # 5. 下降到對應「低位」姿態
        if high_pick:
            move_arm(PICK_HIGH_LOWER, is_joints=True)
        else:
            move_arm(PICK_JOINTS, is_joints=True)
        rospy.sleep(0.3)

        # 6. 再次對齊夾爪中心
        c = get_gripper_center()
        if c:
            teleport(model_name, c[0], c[1], c[2], duration=0.1)

        # 7. 通知 C++ plugin 開始跟隨（帶入物件當前座標，避免 timing race）
        _current_model = model_name
        c2 = get_gripper_center()
        if c2 is None:
            c2 = c if c else (0, 0, 0)
        _follower_attach_pub.publish(
            f"mini_mec_six_arm::link5,{model_name},{c2[0]:.4f},{c2[1]:.4f},{c2[2]:.4f}")
        rospy.loginfo(f"  block_follower attach @ ({c2[0]:.3f},{c2[1]:.3f},{c2[2]:.3f})")

        # 8. 夾緊
        set_gripper(GRIP_CLOSE)

        # 9. 回到搬運姿態（ARM_CLAMP，不擋鏡頭）
        move_arm(ARM_CLAMP, is_joints=True)
        rospy.sleep(0.3)

        rospy.loginfo(f"[arm/pick] 完成：{model_name} 已抓取，已回到 ARM_CLAMP 搬運姿態")
        return TriggerResponse(success=True, message=f"picked {model_name}")

    except Exception as e:
        rospy.logerr(f"[arm/pick] 失敗：{e}")
        _attached = False
        return TriggerResponse(success=False, message=str(e))


def simulate_fall(model_name, start_z, target_z, x, y):
    """模擬方塊從 start_z 落到 target_z + BLOCK_HALF，每步 0.02s"""
    z = start_z
    floor_z = target_z + BLOCK_HALF
    step = 0.05  # 每步下降 5cm
    while z > floor_z:
        z = max(z - step, floor_z)
        model_pub.publish(make_model_state(model_name, x, y, z))
        time.sleep(0.02)
    # 最終定位
    teleport(model_name, x, y, floor_z, duration=0.1)
    rospy.loginfo(f"  simulate_fall 完成：落點 z={floor_z:.3f}")


def handle_home(req):
    """手臂歸位到 ARM_CLAMP（搬運/待機姿態）"""
    try:
        move_arm(ARM_CLAMP, is_joints=True)
        rospy.loginfo("[arm/home] 歸位完成")
        return TriggerResponse(success=True, message="home")
    except Exception as e:
        return TriggerResponse(success=False, message=str(e))


def handle_prepare_put(req):
    """抬臂到 ARM_PUT 姿態，follower 繼續跟隨（方塊跟著臂轉向）。
    由 pick_deliver 在轉向前呼叫，讓臂先就位。"""
    global _current_model
    try:
        move_arm(ARM_PUT, is_joints=True)
        rospy.sleep(0.8)
        # 不停 follower：轉向時方塊繼續跟著夾爪
        rospy.loginfo("[arm/prepare_put] 臂已就位 ARM_PUT，follower 繼續跟隨")
        return TriggerResponse(success=True, message="prepared")
    except Exception as e:
        rospy.logerr(f"[arm/prepare_put] 失敗：{e}")
        return TriggerResponse(success=False, message=str(e))


def handle_drop(req):
    """開夾爪並將方塊 teleport 到桌面。轉向後呼叫。"""
    global _current_model
    target_name = rospy.get_param('/arm_task/target_name', 'ground')
    rospy.loginfo(f"[arm/drop] target: {target_name}")
    try:
        model_name = _current_model or 'red_block'

        c = get_gripper_center()
        if c is None:
            return TriggerResponse(success=False, message="無法取得夾爪位置")
        drop_x, drop_y = c[0], c[1]

        use_target_xy = bool(rospy.get_param('/arm_task/use_target_xy', False))
        if use_target_xy and rospy.has_param('/arm_task/target_x') and rospy.has_param('/arm_task/target_y'):
            drop_x = float(rospy.get_param('/arm_task/target_x'))
            drop_y = float(rospy.get_param('/arm_task/target_y'))

        if rospy.has_param('/arm_task/surface_z'):
            surface_z = rospy.get_param('/arm_task/surface_z')
        else:
            surface_z = get_surface_z(target_name)
        floor_z = surface_z + BLOCK_HALF

        # 垃圾桶/bin 類目標：一律落地，不用 surface_z（halved bbox 高度會讓方塊懸空）
        if 'trash' in target_name.lower() or 'bin' in target_name.lower():
            rospy.loginfo(
                f"[arm/drop] target='{target_name}' 為垂圾桶/bin → 改用落地高度 "
                f"{BLOCK_HALF:.3f}（原 surface_z 高度 {floor_z:.3f}）"
            )
            floor_z = BLOCK_HALF

        start_z = c[2]   # 夾爪當前高度，作為下落起點

        # 停 follower → 開夾爪（blocking）→ 等 0.5s → 固定速度下落
        # 單次 detach 偶爾會被 queue/timing 吃掉，導致方塊被下一幀拉回爪子。
        detach_follower_burst(duration=0.45)
        set_gripper(GRIP_OPEN, secs=0.8)
        rospy.sleep(0.5)

        # 方塊先定位在夾爪正下方
        model_pub.publish(make_model_state(model_name, drop_x, drop_y, start_z))
        rospy.sleep(0.05)

        # 固定速度下落：每 40ms 降 1cm（≈ 0.25 m/s），有速度感但不急
        z = start_z
        while z > floor_z:
            z = max(z - 0.01, floor_z)
            model_pub.publish(make_model_state(model_name, drop_x, drop_y, z))
            time.sleep(0.04)
        teleport(model_name, drop_x, drop_y, floor_z, duration=0.15)  # 最終定位
        detach_follower_burst(duration=0.25)
        teleport(model_name, drop_x, drop_y, floor_z, duration=0.2)   # detach 後再固定一次

        _current_model = None
        rospy.loginfo(f"[arm/drop] 放置於 ({drop_x:.3f}, {drop_y:.3f}, {floor_z:.3f})")
        return TriggerResponse(success=True, message=f"dropped at ({drop_x:.2f},{drop_y:.2f})")
    except Exception as e:
        rospy.logerr(f"[arm/drop] 失敗：{e}")
        return TriggerResponse(success=False, message=str(e))


def handle_put(req):
    """舊介面：prepare_put + drop 合併，向後相容。"""
    r = handle_prepare_put(req)
    if not r.success:
        return r
    return handle_drop(req)


# ════════════════════════════════════════════════════════════════
# 主程式
# ════════════════════════════════════════════════════════════════

def main():
    global arm, gripper, model_pub, _follower_attach_pub, _follower_detach_pub

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_task_server')

    # 訂閱 link states（用於夾爪位置查詢）
    rospy.Subscriber('/gazebo/link_states', LinkStates, _on_link_states, queue_size=1)

    # 發布 set_model_state（用於 pick 時的 teleport）
    model_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    # Block follower plugin publishers（零延遲 C++ 跟隨）
    _follower_attach_pub = rospy.Publisher('/block_follower/attach', String, queue_size=1)
    _follower_detach_pub = rospy.Publisher('/block_follower/detach', String, queue_size=1)

    global set_model_state_srv, _get_link_state
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    rospy.wait_for_service('/gazebo/get_link_state', timeout=10.0)
    _get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)


    # MoveIt arm
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_max_acceleration_scaling_factor(0.3)

    # 夾爪 publisher
    gripper = rospy.Publisher('/hand_controller/command', JointTrajectory, queue_size=1)

    rospy.sleep(1.5)  # 等待連線穩定

    # 註冊 Services
    rospy.Service('/arm/pick',        Trigger, handle_pick)
    rospy.Service('/arm/put',         Trigger, handle_put)
    rospy.Service('/arm/prepare_put', Trigger, handle_prepare_put)
    rospy.Service('/arm/drop',        Trigger, handle_drop)
    rospy.Service('/arm/home',        Trigger, handle_home)

    rospy.loginfo("[arm_task_server] 就緒。等待 /arm/pick 或 /arm/put 呼叫…")
    rospy.spin()

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
