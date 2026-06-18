#!/usr/bin/env python3
"""
sim_pick.py - 抓取 + 放置流程
用法：
  python3 sim_pick.py           # 只執行抓取，完成後等待 /sim_pick/do_place 訊號
  python3 sim_pick.py --place   # 抓取完成後自動接著放置（不等訊號）
"""
import sys
import threading
import rospy
import tf
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64, Bool
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, GetModelState

# ── 手臂姿態 ──────────────────────────────────────────────────
ARM_JOINTS     = ['joint1','joint2','joint3','joint4','joint5']
ARM_CLAMP      = [0,    -1.1,  0.66, 1.0,  0]   # 向前下方抓取
ARM_UPLIFT     = [0,     0.0,  1.0,  1.57, 0]   # 舉起（搬運中）
ARM_ROTATE_PUT = [1.57, -1.1,  0.66, 1.0,  0]   # 旋轉 90°朝側邊放下

# ── 夾爪指令 ──────────────────────────────────────────────────
OPEN  = {'joint6': 0.15,  'joint7':-0.15, 'joint8': 0.15,
         'joint9': 0.15,  'joint10': 0.15,'joint11': 0.15}
CLOSE = {'joint6':-0.45, 'joint7': 0.45, 'joint8':-0.45,
         'joint9':-0.45, 'joint10':-0.45,'joint11': 0.45}

# ── 抓取參數 ──────────────────────────────────────────────────
BLOCK_MODEL   = 'red_block'
GRIPPER_FRAME = 'link5'
MAGNET_RADIUS = 0.12    # 磁吸觸發距離 (m)
GRIP_OFFSET_X = 0.034   # link5 → 夾爪中心偏移
GRIP_OFFSET_Y = 0.0
GRIP_OFFSET_Z = -0.07

# ── 放置參數 ──────────────────────────────────────────────────
BLOCK_HALF     = 0.025   # 方塊半高（m），方塊底面到中心距離
FALL_DURATION  = 0.8     # 模擬落下動畫時長（秒）

_attached = False
_do_place = False         # 由 /sim_pick/do_place topic 設定


# ── 工具函式 ──────────────────────────────────────────────────

def send_gripper(pubs, target):
    for j, val in target.items():
        pubs[j].publish(Float64(data=val))


def get_gripper_center(tl):
    (trans, _) = tl.lookupTransform('odom', GRIPPER_FRAME, rospy.Time(0))
    return (trans[0] + GRIP_OFFSET_X,
            trans[1] + GRIP_OFFSET_Y,
            trans[2] + GRIP_OFFSET_Z)


def send_arm(client, positions, duration):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ARM_JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = positions
    pt.time_from_start = rospy.Duration(duration)
    goal.trajectory.points.append(pt)
    goal.trajectory.header.stamp = rospy.Time.now()
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(duration + 3.0))


# ── 抓取背景執行緒 ────────────────────────────────────────────

def magnet_loop(tl, get_state, set_state):
    """arm_clamp 期間：方塊進入範圍自動漸進吸入夾爪中心"""
    rate = rospy.Rate(50)
    rospy.loginfo(f'[magnet] 磁吸啟動，範圍 {MAGNET_RADIUS}m')
    while not rospy.is_shutdown() and not _attached:
        try:
            resp = get_state(BLOCK_MODEL, '')
            bx, by, bz = (resp.pose.position.x,
                          resp.pose.position.y,
                          resp.pose.position.z)
            cx, cy, cz = get_gripper_center(tl)
            dist = ((cx-bx)**2 + (cy-by)**2 + (cz-bz)**2) ** 0.5
            if dist < MAGNET_RADIUS:
                alpha = 0.3
                s = ModelState()
                s.model_name      = BLOCK_MODEL
                s.reference_frame = ''
                s.pose.position.x = bx + alpha * (cx - bx)
                s.pose.position.y = by + alpha * (cy - by)
                s.pose.position.z = bz + alpha * (cz - bz)
                s.pose.orientation.w = 1.0
                req = SetModelStateRequest(); req.model_state = s
                set_state(req)
        except Exception as e:
            rospy.logwarn_throttle(2.0, f'[magnet] {e}')
        rate.sleep()


def attach_loop(tl, set_state):
    """hand_close 後：方塊鎖定跟隨夾爪"""
    rate = rospy.Rate(50)
    rospy.loginfo('[attach] 鎖定跟隨開始')
    while not rospy.is_shutdown() and _attached:
        try:
            cx, cy, cz = get_gripper_center(tl)
            s = ModelState()
            s.model_name      = BLOCK_MODEL
            s.reference_frame = ''
            s.pose.position.x = cx
            s.pose.position.y = cy
            s.pose.position.z = cz
            s.pose.orientation.w = 1.0
            req = SetModelStateRequest(); req.model_state = s
            set_state(req)
        except Exception as e:
            rospy.logwarn_throttle(2.0, f'[attach] {e}')
        rate.sleep()


# ── 放置流程 ──────────────────────────────────────────────────

def simulate_fall(set_state, get_state, duration=FALL_DURATION):
    """停止吸附後模擬方塊受重力落到桌面（set_model_state 後重力失效，手動補償）"""
    try:
        resp    = get_state(BLOCK_MODEL, '')
        drop_x  = resp.pose.position.x
        drop_y  = resp.pose.position.y
        drop_z  = resp.pose.position.z
    except Exception:
        return

    vel      = 0.0
    dt       = 0.02
    steps    = int(duration / dt)
    rate     = rospy.Rate(1.0 / dt)

    for _ in range(steps):
        vel    += 9.8 * dt
        drop_z  = max(drop_z - vel * dt, BLOCK_HALF)
        s = ModelState()
        s.model_name      = BLOCK_MODEL
        s.reference_frame = ''
        s.pose.position.x = drop_x
        s.pose.position.y = drop_y
        s.pose.position.z = drop_z
        s.pose.orientation.w = 1.0
        req = SetModelStateRequest(); req.model_state = s
        try:
            set_state(req)
        except Exception:
            pass
        rate.sleep()
        if drop_z <= BLOCK_HALF:
            break


def do_place(arm_client, pubs, tl, set_state, get_state):
    """
    放物流程（在機器人已導航到桌子旁後呼叫）：
      1. arm_rotate_put — 旋轉 90°，手臂伸向桌面
      2. 停止 attach_loop — 方塊不再跟隨夾爪
      3. 模擬落下 — 方塊落到桌面
      4. hand_open — 開夾爪
      5. arm_clamp — 歸位
    """
    global _attached

    rospy.loginfo('[place] 1. arm_rotate_put')
    send_arm(arm_client, ARM_ROTATE_PUT, 3.0)
    rospy.sleep(0.5)

    rospy.loginfo('[place] 2. 停止吸附，模擬落下')
    _attached = False
    rospy.sleep(0.1)
    simulate_fall(set_state, get_state)

    rospy.loginfo('[place] 3. hand_open')
    send_gripper(pubs, OPEN)
    rospy.sleep(1.0)

    rospy.loginfo('[place] 4. 歸位 arm_clamp')
    send_arm(arm_client, ARM_CLAMP, 3.0)

    rospy.loginfo('[place] === 放置完成 ===')


# ── 主程式 ────────────────────────────────────────────────────

def main():
    global _attached, _do_place

    auto_place = "--place" in sys.argv

    rospy.init_node('sim_pick', anonymous=False)

    tl = tf.TransformListener()
    rospy.sleep(1.0)

    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    pubs = {}
    for j in OPEN:
        pubs[j] = rospy.Publisher(f'/{j}_pos_controller/command', Float64, queue_size=1)
    rospy.sleep(0.5)

    import subprocess
    subprocess.run(['rosnode', 'kill', '/gripper_init'], capture_output=True)
    rospy.sleep(0.5)

    arm_client = actionlib.SimpleActionClient(
        '/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    arm_client.wait_for_server()

    # 訂閱放置觸發訊號（發送 True 到此 topic 即開始放置）
    def place_cb(msg):
        global _do_place
        if msg.data:
            _do_place = True
    rospy.Subscriber('/sim_pick/do_place', Bool, place_cb, queue_size=1)

    # ── 抓取流程 ──────────────────────────────────────────────
    rospy.loginfo('=== 抓取流程開始 ===')

    rospy.loginfo('1. hand_open')
    send_gripper(pubs, OPEN)
    rospy.sleep(1.5)

    rospy.loginfo('2. arm_clamp + 磁吸啟動')
    threading.Thread(target=magnet_loop, args=(tl, get_state, set_state), daemon=True).start()
    send_arm(arm_client, ARM_CLAMP, 3.0)
    rospy.sleep(0.5)

    rospy.loginfo('3. 夾爪閉合 + 鎖定吸附')
    send_gripper(pubs, CLOSE)
    _attached = True
    threading.Thread(target=attach_loop, args=(tl, set_state), daemon=True).start()
    rospy.sleep(1.0)

    rospy.loginfo('4. arm_uplift')
    send_arm(arm_client, ARM_UPLIFT, 3.0)

    rospy.loginfo('=== 抓取完成，持續吸附中 ===')

    if auto_place:
        rospy.loginfo('[pick] --place 模式：直接進入放置流程')
        do_place(arm_client, pubs, tl, set_state, get_state)
        return

    rospy.loginfo('[pick] 等待放置訊號：rostopic pub /sim_pick/do_place std_msgs/Bool "data: true"')

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        send_gripper(pubs, CLOSE)   # 持續覆蓋，防止 gripper_init 搶佔
        if _do_place:
            _do_place = False
            do_place(arm_client, pubs, tl, set_state, get_state)
            break
        rate.sleep()


if __name__ == '__main__':
    main()
