#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <string>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <wheeltec_arm_pick/pick_and_put.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "wheeltec_four_arm_pick/arm_pick_and_put.h"


uint8_t car_state=0;
int arm_done=1; //置0是表示已经操作完成这个动作，1是表示有新的动作需要完成

std::string arm_state="none";

//接收机械臂运动状态的回调函数
void arm_state_callback(const std_msgs::String &state)
{
  static std::string last_arm_state;
  arm_state=state.data;
  if(last_arm_state!=arm_state) arm_done=1; //机械臂新状态切换过程检测
  last_arm_state=arm_state;
}


int main(int argc, char **argv)
{ 
    //std_msgs::Float32 msg;
    geometry_msgs::Twist msg;
    //wheeltec_arm_pick::pick_and_put msg;
    ros::init(argc, argv, "arm_pick_and_put_dp");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("arm");   
    moveit::planning_interface::MoveGroupInterface hand("hand"); 

    arm.setGoalJointTolerance(0.01);
    //arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.5);

    arm.setNamedTarget("arm_uplift"); arm.move(); sleep(1); //机械臂回到收起的状态
    hand.setNamedTarget("hand_close"); hand.move(); sleep(1); //机械爪关闭

    
    ros::Subscriber arm_state_sub=n.subscribe("arm_state",10,arm_state_callback); //订阅机械臂的运动状态
    car_state=0;
    arm_done=1;
    
   
    //ros::Publisher cmd_vel_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",10);///
    

    while(ros::ok())
   {
        if (arm_done==1&&arm_state=="pick")  arm_pick(),car_state=1,arm_done =0;//机械臂抓取色块
       
        else if (arm_done==1&&arm_state=="put")   arm_put(),car_state=2,arm_done =0; //机械臂放置色块

        else if (arm_done==1&&arm_state=="flag3")  car_state=2,arm_done=0;
        else if (arm_done==1&&arm_state=="no_msg") car_state=0,arm_done=0;

        ros::spinOnce();
   }
    ros::shutdown(); 
    return 0;
}

//一个完整的夹取动作
void arm_pick()
{
    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface hand("hand");
    arm.setGoalJointTolerance(0.01);
    //arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.6);
    hand.setNamedTarget("hand_open");  hand.move(); sleep(1);
    arm.setNamedTarget("arm_clamp");   arm.move();  sleep(1);
    hand.setNamedTarget("hand_close"); hand.move(); sleep(1);
    arm.setNamedTarget("arm_uplift");  arm.move();  

}

//一个完整的放置动作
void arm_put()
{
    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface hand("hand");
    arm.setGoalJointTolerance(0.01);
    //arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.6);
    arm.setNamedTarget("arm_clamp");   arm.move();  sleep(1);
    hand.setNamedTarget("hand_open");  hand.move(); sleep(1);
    arm.setNamedTarget("arm_uplift");  arm.move();  sleep(1);
    hand.setNamedTarget("hand_close"); hand.move(); 
}
