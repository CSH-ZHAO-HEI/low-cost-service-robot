#!/usr/bin/python3
"""
Auto-run at Gazebo startup: set arm to arm_clamp pose (forward-down, out of camera FOV).
Called from mec_six_arm_warehouse.launch after controllers start.
"""
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main():
    rospy.init_node('arm_clamp_init', anonymous=True)
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)

    rospy.sleep(3.0)  # wait for controllers to be ready

    traj = JointTrajectory()
    traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

    pt = JointTrajectoryPoint()
    # arm_clamp: 向前下方，建图初始姿态。
    pt.positions = [0.0, -1.1, 0.66, 1.0, 0.0]
    pt.time_from_start = rospy.Duration(3.0)

    traj.points = [pt]
    traj.header.stamp = rospy.Time.now()

    pub.publish(traj)
    rospy.loginfo("Arm set to arm_home pose.")
    rospy.sleep(1.0)

if __name__ == '__main__':
    main()
