#!/usr/bin/env python3
"""Inject Gazebo ground truth pose into RTAB-Map for accurate localization start."""
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('set_initial_pose', anonymous=True)
pub = rospy.Publisher('/rtabmap/global_pose', PoseWithCovarianceStamped, queue_size=1, latch=True)

# Wait for RTAB-Map to be ready
rospy.sleep(2.0)

try:
    odom = rospy.wait_for_message('/ground_truth/odom', Odometry, timeout=10.0)
except rospy.ROSException:
    rospy.logerr("set_initial_pose: /ground_truth/odom timeout")
    exit(1)

msg = PoseWithCovarianceStamped()
msg.header.frame_id = "map"
msg.header.stamp = rospy.Time.now()
msg.pose.pose = odom.pose.pose
msg.pose.covariance[0]  = 0.01
msg.pose.covariance[7]  = 0.01
msg.pose.covariance[35] = 0.01

# Publish multiple times to ensure RTAB-Map receives it
for i in range(5):
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rospy.loginfo("set_initial_pose: [%d/5] x=%.3f y=%.3f -> /rtabmap/global_pose",
                  i+1, msg.pose.pose.position.x, msg.pose.pose.position.y)
    rospy.sleep(0.5)
