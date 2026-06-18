#!/usr/bin/env python3
"""啟動時把夾爪鎖在 0 位置，防止自由浮動"""
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['joint6', 'joint7', 'joint8', 'joint9', 'joint10', 'joint11']

def main():
    rospy.init_node('gripper_init', anonymous=False)
    pub = rospy.Publisher('/hand_controller/command', JointTrajectory, queue_size=1)
    rospy.sleep(2.0)  # 等 hand_controller 就緒

    traj = JointTrajectory()
    traj.joint_names = JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = [0.0] * 6
    pt.time_from_start = rospy.Duration(1.0)
    traj.points = [pt]
    traj.header.stamp = rospy.Time.now()

    pub.publish(traj)
    rospy.loginfo('[gripper_init] 夾爪鎖定在 0 位置')
    rospy.sleep(1.0)

if __name__ == '__main__':
    main()
