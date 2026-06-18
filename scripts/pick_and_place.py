#!/usr/bin/python3
import sys
import time
import threading
import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.msg import ModelState, LinkStates
from gazebo_msgs.srv import SetModelState

JOINTS     = ['joint6', 'joint7', 'joint8', 'joint9', 'joint10', 'joint11']
GRIP_CLOSE = [-0.45,  0.45, -0.45, -0.45, -0.45, -0.45]
GRIP_OPEN  = [ 0.45, -0.45,  0.45,  0.45,  0.45,  0.45]

PICK_JOINTS = [0.0, -1.5, 0.66, 1.0, 0.0]
BLOCK_NAME  = 'red_block'
BLOCK_HALF  = 0.025

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_and_place', anonymous=True)

    rospy.wait_for_service('/gazebo/set_model_state')
    set_srv   = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    model_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    _link_positions = {}
    def _on_link_states(msg):
        for name, pose in zip(msg.name, msg.pose):
            _link_positions[name] = pose.position
    rospy.Subscriber('/gazebo/link_states', LinkStates, _on_link_states, queue_size=1)

    arm     = moveit_commander.MoveGroupCommander("arm")
    gripper = rospy.Publisher('/hand_controller/command', JointTrajectory, queue_size=1)
    rospy.sleep(1.5)
    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_max_acceleration_scaling_factor(0.3)

    GRIP_Z_OFFSET = -0.03  # 吸附點往下偏移

    def get_gripper_center_world():
        t7 = _link_positions.get('mini_mec_six_arm::link7')
        t9 = _link_positions.get('mini_mec_six_arm::link9')
        if t7 is None or t9 is None:
            return None
        return (
            (t7.x + t9.x) / 2.0,
            (t7.y + t9.y) / 2.0,
            (t7.z + t9.z) / 2.0 + GRIP_Z_OFFSET,
        )

    def make_state(x, y, z):
        msg = ModelState()
        msg.model_name = BLOCK_NAME
        msg.reference_frame = 'world'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        return msg

    def teleport(x, y, z, duration=0.3):
        """model_pub 高頻發送到指定位置"""
        msg = make_state(x, y, z)
        t_end = time.time() + duration
        while time.time() < t_end:
            model_pub.publish(msg)

    def simulate_fall(x, y, from_z):
        """model_pub 高頻模擬下落到地面"""
        drop_z = from_z
        vel = 0.0
        g = 9.8
        t_last = time.time()
        while drop_z > BLOCK_HALF:
            t_now = time.time()
            dt = t_now - t_last
            t_last = t_now
            vel += g * dt
            drop_z = max(drop_z - vel * dt, BLOCK_HALF)
            model_pub.publish(make_state(x, y, drop_z))

    def set_gripper(pos, secs=1.0):
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        traj.header.stamp = rospy.Time.now()
        pt = JointTrajectoryPoint()
        pt.positions = pos
        pt.time_from_start = rospy.Duration(secs)
        traj.points = [pt]
        gripper.publish(traj)
        rospy.sleep(secs + 0.2)

    def move(target, is_joints=False):
        if is_joints:
            arm.set_joint_value_target(target)
        else:
            arm.set_named_target(target)
        arm.go(wait=True)
        arm.stop()

    # ── STEP 1: arm_clamp，瞬移方塊到夾爪地面位置，等 3 秒 ──
    rospy.loginfo("▶ STEP 1: 初始化")
    set_gripper(GRIP_OPEN)
    move('arm_clamp')
    rospy.sleep(0.3)

    center = get_gripper_center_world()
    if center is None:
        rospy.logerr("無法取得夾爪位置")
        return

    teleport(center[0], center[1], BLOCK_HALF)
    rospy.loginfo(f"  方塊瞬移地面 → x={center[0]:.3f} y={center[1]:.3f} z={BLOCK_HALF}")
    rospy.sleep(3.0)

    # ── STEP 2: 下降到抓取位 ──
    rospy.loginfo("▶ STEP 2: 下降")
    move(PICK_JOINTS, is_joints=True)
    rospy.sleep(0.3)

    # ── STEP 3: 吸附 + 關夾爪 ──
    rospy.loginfo("▶ STEP 3: 吸附")
    center3 = get_gripper_center_world()
    if center3:
        teleport(center3[0], center3[1], center3[2])

    attaching = [True]
    def attach_loop():
        while attaching[0] and not rospy.is_shutdown():
            c = get_gripper_center_world()
            if c:
                model_pub.publish(make_state(c[0], c[1], c[2]))
    t = threading.Thread(target=attach_loop, daemon=True)
    t.start()
    set_gripper(GRIP_CLOSE, secs=1.0)

    # ── STEP 4: 抬升 ──
    rospy.loginfo("▶ STEP 4: 抬升")
    move('arm_clamp')

    # ── STEP 5: 旋轉到放置位 ──
    rospy.loginfo("▶ STEP 5: 旋轉")
    move('arm_rotate_put')
    rospy.sleep(0.3)

    # ── STEP 6: 放下（夾爪鬆開同時開始下落）──
    rospy.loginfo("▶ STEP 6: 放下")
    attaching[0] = False
    t.join(timeout=1.0)
    c = get_gripper_center_world()
    threading.Thread(target=set_gripper, args=(GRIP_OPEN,), daemon=True).start()
    if c:
        rospy.loginfo(f"  模擬下落 from z={c[2]:.3f}")
        simulate_fall(c[0], c[1], c[2])
        rospy.loginfo("  方塊落地")
    rospy.sleep(1.0)

    # ── STEP 7: 歸位 ──
    rospy.loginfo("▶ STEP 7: 歸位")
    move('arm_clamp')

    rospy.loginfo("完成")
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
