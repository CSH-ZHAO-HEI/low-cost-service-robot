#!/usr/bin/env python3
"""
flip_cmd_vel.py
TEB 以 base_footprint（車頭=+X）規劃，但 planar_move 以 base_link 解讀 cmd_vel。
base_footprint 相對 base_link 旋轉 180°，因此 linear.x/y 需取反；angular.z 不變。
"""
import rospy
from geometry_msgs.msg import Twist

pub = None

def callback(msg):
    out = Twist()
    out.linear.x  = -msg.linear.x
    out.linear.y  = -msg.linear.y
    out.linear.z  =  msg.linear.z
    out.angular.x =  msg.angular.x
    out.angular.y =  msg.angular.y
    out.angular.z =  msg.angular.z
    pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("flip_cmd_vel")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/cmd_vel_teb", Twist, callback)
    rospy.spin()
