#!/usr/bin/env python3
# 改這行就好 → [joint1, joint2, joint3, joint4, joint5]
# joint2 越負 = 手臂越高；joint3 越大 = 夾爪越前；joint4 越大 = 夾爪越低
POSE = [0.0, -0.65, 0.60, 1.0, 0.0]

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('set_balcony_pose', anonymous=True)
pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)
rospy.sleep(1.0)

traj = JointTrajectory()
traj.joint_names = ['joint1','joint2','joint3','joint4','joint5']
traj.header.stamp = rospy.Time.now()
pt = JointTrajectoryPoint()
pt.positions = POSE
pt.time_from_start = rospy.Duration(2.0)
traj.points = [pt]
pub.publish(traj)
rospy.sleep(2.2)
print(f"done: {POSE}")
