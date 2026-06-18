#!/usr/bin/env python3
# Bridge: /teleop/joint_states -> /arm_controller/command + /hand_controller/command
# Teleop only sends joint1-6, we map: joint1-5 -> arm, joint6 -> hand (gripper open/close)
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS  = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
HAND_JOINTS = ['joint6', 'joint7', 'joint8', 'joint9', 'joint10', 'joint11']

pub_arm  = None
pub_hand = None

arm_pos  = [0.0] * 5
hand_pos = [0.0] * 6  # joint6-11, teleop controls joint6 only

def teleop_cb(msg):
    global arm_pos, hand_pos
    updated_arm  = False
    updated_hand = False

    for i, name in enumerate(msg.name):
        if i >= len(msg.position):
            break
        if name in ARM_JOINTS:
            arm_pos[ARM_JOINTS.index(name)] = msg.position[i]
            updated_arm = True
        elif name == 'joint6':
            # teleop joint6 = wrist rotation -> map to all gripper joints symmetrically
            val = msg.position[i]
            hand_pos = [val, val, -val, -val, val, -val]
            updated_hand = True

    if updated_arm:
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = list(arm_pos)
        pt.time_from_start = rospy.Duration(0.15)
        traj.points = [pt]
        pub_arm.publish(traj)

    if updated_hand:
        traj = JointTrajectory()
        traj.joint_names = HAND_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = list(hand_pos)
        pt.time_from_start = rospy.Duration(0.15)
        traj.points = [pt]
        pub_hand.publish(traj)

if __name__ == '__main__':
    rospy.init_node('joint_state_bridge')
    pub_arm  = rospy.Publisher('/arm_controller/command',  JointTrajectory, queue_size=5)
    pub_hand = rospy.Publisher('/hand_controller/command', JointTrajectory, queue_size=5)
    # Subscribe to teleop-only topic (remapped from teleop script)
    rospy.Subscriber('/arm_teleop', JointState, teleop_cb)
    rospy.loginfo("joint_state_bridge: ready, listening on /arm_teleop")
    rospy.spin()
