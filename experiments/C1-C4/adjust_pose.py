#!/usr/bin/python3
import time
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node("adjust_pose", anonymous=True, disable_signals=True)
arm_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
gripper_pub = rospy.Publisher("/hand_controller/command", JointTrajectory, queue_size=1)

# 改这里
POSE = [0.0, -1.2, 0.8, 1.0, 0.0]

time.sleep(1.0)




