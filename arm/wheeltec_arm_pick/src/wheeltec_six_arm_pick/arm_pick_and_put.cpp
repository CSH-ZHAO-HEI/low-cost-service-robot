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
#include <wheeltec_arm_pick/pick_and_put.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "wheeltec_six_arm_pick/arm_pick_and_put.h"


uint8_t car_state=0;
int arm_done=1; //置0是表示已经操作完成这个动作，1是表示有新的动作需要完成
int action_count;
ros::Publisher joint_states_pub;

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
    wheeltec_arm_pick::pick_and_put msg;
    ros::init(argc, argv, "arm_pick_and_put");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("arm");   
    moveit::planning_interface::MoveGroupInterface hand("hand"); 

    arm.setGoalJointTolerance(0.01);

    arm.setMaxVelocityScalingFactor(1);

    hand.setNamedTarget("hand_open"); hand.move(); sleep(1); //机械爪关闭
    arm.setNamedTarget("arm_uplift"); arm.move(); sleep(1);  //机械臂回到收起的状态


    ros::Publisher car_command_pub=n.advertise<wheeltec_arm_pick::pick_and_put>("car_command",10);//发布机械臂及底盘的运动状态
    joint_states_pub=n.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states",10);     //发布关节状态
    ros::Subscriber arm_state_sub=n.subscribe("arm_state",10,arm_state_callback);                //订阅机械臂的运动状态
    car_state=0;
    arm_done=1;
   
    while(ros::ok())
   {
         if (arm_done==1&&arm_state=="shake_hand")  arm_shake_hand(),car_state=4,arm_done=0;//机械臂转动夹爪
    else if (arm_done==1&&arm_state=="pick")  arm_pick(),car_state=1,arm_done =0;           //机械臂抓取色块
    else if (arm_done==1&&arm_state=="put")   arm_put(),car_state=2,arm_done =0;            //机械臂放置色块
    else if (arm_done==1&&arm_state=="rotate_put")  arm_rotate_put(),car_state=3,arm_done=0;//机械臂旋转放置色块
    else if (arm_done==1&&arm_state=="no_msg") car_state=0,arm_done=0;

    if(car_state==0)  msg.car_state=0,msg.angle= 0;    //无状态
    if(car_state==1)  msg.car_state=1,msg.angle= 1.57 ;//成功抓取色块后，发出底盘左转的命令
    if(car_state==2)  msg.car_state=2,msg.angle=-1.57 ;//成功放置色块后，发出底盘右转的命令
    if(car_state==3)  msg.car_state=3,msg.angle=0 ;    //机械臂旋转放置色块完毕
    if(car_state==4)  msg.car_state=4,msg.angle= 0;    //机械臂旋转夹爪完毕

    car_command_pub.publish(msg);//将底盘的状态发布出去

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
    arm.setMaxVelocityScalingFactor(0.1);
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
    arm.setMaxAccelerationScalingFactor(1);
    arm.setMaxVelocityScalingFactor(1);
    arm.setNamedTarget("arm_clamp");   arm.move();  sleep(1);
    hand.setNamedTarget("hand_open");  hand.move(); sleep(1);
    arm.setNamedTarget("arm_uplift");  arm.move();
}

//一个完整的旋转放置动作
void arm_rotate_put()
{
    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface hand("hand");
    arm.setGoalJointTolerance(0.01);
    arm.setMaxAccelerationScalingFactor(1);
    arm.setMaxVelocityScalingFactor(1);
    arm.setNamedTarget("arm_rotate_uplift");   arm.move();  sleep(1);
    arm.setNamedTarget("arm_rotate_put");      arm.move(); sleep(1);
    hand.setNamedTarget("hand_open");          hand.move(); sleep(1);
    arm.setNamedTarget("arm_rotate_uplift");   arm.move();  sleep(1);
    arm.setNamedTarget("arm_uplift");          arm.move();  
}

//一个完整的机械臂转动夹爪动作
void arm_shake_hand()
{
    action_count=0;
    float joint_step=0.06;  //机械臂运动的步进值
    float temp=0;
    ros::Rate loop_rate(60); //设置程序执行频率（单位：hz）
    while(ros::ok()){
        if  (action_count>=0 && action_count<16) temp-=joint_step; //执行的第1段动作
        else if(action_count>=16 && action_count<48)  temp+=joint_step; //执行的第2段动作
        else if(action_count>=48 && action_count<80)  temp-=joint_step; //执行的第3段动作
        else if (action_count>=80) temp+=joint_step; //执行的第4段动作
        sensor_msgs::JointState arm_joint_msg;  //定义一个机械臂控制信息的消息数据类型
        ros::Time pub_time=ros::Time::now();  //获取当前的ROS时间
        //输入当前的ros时间
        arm_joint_msg.header.stamp=pub_time;
        //输入机械臂臂身的目标关节角度（单位：弧度）
        arm_joint_msg.position.push_back(0);
        arm_joint_msg.position.push_back(0.54);
        arm_joint_msg.position.push_back(1.57);
        arm_joint_msg.position.push_back(1.57);
        arm_joint_msg.position.push_back(temp);
        //输入机械臂夹爪的目标关节角度（单位：弧度）
        arm_joint_msg.position.push_back(0.6);
        arm_joint_msg.position.push_back(-0.6);
        arm_joint_msg.position.push_back(0.6);
        arm_joint_msg.position.push_back(0.6);
        arm_joint_msg.position.push_back(0.6);
        arm_joint_msg.position.push_back(0.6);
        //输入机械臂关节名称
        arm_joint_msg.name.push_back("joint1");
        arm_joint_msg.name.push_back("joint2");
        arm_joint_msg.name.push_back("joint3");
        arm_joint_msg.name.push_back("joint4");
        arm_joint_msg.name.push_back("joint5"); 
        arm_joint_msg.name.push_back("joint6");
        arm_joint_msg.name.push_back("joint7");
        arm_joint_msg.name.push_back("joint8");
        arm_joint_msg.name.push_back("joint9");
        arm_joint_msg.name.push_back("joint10");
        arm_joint_msg.name.push_back("joint11");
        //将关节目标角度发布出去
        joint_states_pub.publish(arm_joint_msg);
        action_count+=1;
        if (action_count>=96) break;
        loop_rate.sleep(); //延时等待
    }

}
