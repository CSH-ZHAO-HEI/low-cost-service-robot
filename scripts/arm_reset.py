#!/usr/bin/env python3
"""
Reset arm to all-zero position.
Run: python3 arm_reset.py
"""
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_reset():
    rospy.init_node('arm_reset', anonymous=True)
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)

    rospy.sleep(1.0)

    traj = JointTrajectory()
    traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

    pt = JointTrajectoryPoint()
    pt.positions = [0.0, 0.0, 0.0, 0.0, 0.0]
    pt.time_from_start = rospy.Duration(3.0)

    traj.points = [pt]
    traj.header.stamp = rospy.Time.now()

    pub.publish(traj)
    rospy.loginfo("Arm reset to zero position.")
    rospy.sleep(1.0)

if __name__ == '__main__':
    send_reset()
