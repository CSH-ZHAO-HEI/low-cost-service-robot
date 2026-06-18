#include "wheeltec_six_arm_pick/voice_control.h"
#include <iostream>
#include <std_msgs/Int8.h>


std::string arm_state="ready",arm_action="voice_control_wait";
std::vector<double> joint_group_positions(5); //关节正解运动用到的关机目标运动数组
ros::Publisher joints_state_publisher;  //初始化关节目标角度的发布者
int preset_flag = 0;  //预设位置标志位



using namespace std;
int time_count=0;
/***********************************
函数功能：机械臂预设位置的执行函数
***********************************/
void arm_preset(void)
{
  //moveit::planning_interface::MoveGroupInterface arm("arm");   //初始化moveit控制规划组
  //moveit::planning_interface::MoveGroupInterface hand("hand");  //初始化moveit控制规划组

  //arm.setNamedTarget("voice_control_wait"); arm.move(); sleep(1);
   static float joint_step=0.05; //机械臂运动的步进值
   sensor_msgs::JointState arm_joint_msg;  //定义一个机械臂控制信息的消息数据类型
   ros::Time pub_time=ros::Time::now();   //获取当前的ROS时间
      //输入当前的ros时间
      arm_joint_msg.header.stamp=pub_time;
      //输入机械臂臂身的目标关节角度（单位：弧度）
      arm_joint_msg.position.push_back(0);
      arm_joint_msg.position.push_back(0);
      arm_joint_msg.position.push_back(0); 
      arm_joint_msg.position.push_back(1.57);
      arm_joint_msg.position.push_back(0);
      //输入机械臂夹爪的目标关节角度（单位：弧度）
      arm_joint_msg.position.push_back(0);
      arm_joint_msg.position.push_back(0); 
      arm_joint_msg.position.push_back(0);
      arm_joint_msg.position.push_back(0);
      arm_joint_msg.position.push_back(0);
      arm_joint_msg.position.push_back(0);
      //输入机械臂关节名称
      arm_joint_msg.name.push_back("joint_1");
      arm_joint_msg.name.push_back("joint_2");
      arm_joint_msg.name.push_back("joint_3"); 
      arm_joint_msg.name.push_back("joint_4");
      arm_joint_msg.name.push_back("joint_5"); 
      arm_joint_msg.name.push_back("joint_6");
      arm_joint_msg.name.push_back("joint_10");
      arm_joint_msg.name.push_back("joint_7");
      arm_joint_msg.name.push_back("joint_11");
      arm_joint_msg.name.push_back("joint_8");
      arm_joint_msg.name.push_back("joint_9");
      //将关节目标角度发布出去
      joints_state_publisher.publish(arm_joint_msg);

}

/**************************************************************************
函数功能：预设位置标志位sub回调函数
入口参数：preset_flag_msg  node_feedback.cpp
返回  值：无
**************************************************************************/
void preset_flag_Callback(std_msgs::Int8 msg)
{
  preset_flag = msg.data;
  if(preset_flag == 1 )
  {
    arm_preset();
  }

}

/*  主函数*/
/***********************************/
int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "arm_preset"); 
    ros::NodeHandle n;
 
    ros::Subscriber preset_flag_sub = n.subscribe("preset_flag", 1, preset_flag_Callback);//预设位置标志位话题
    joints_state_publisher=n.advertise<sensor_msgs::JointState>("voice_joint_states",10);//往控制机械臂运动的话题发布信息

    ros::Rate loop_rate(40); //设置程序执行频率（单位：hz）

    ros::spin();

    //while循环执行
    /*while(ros::ok())
   {
     if(time_count<100) {time_count++;arm_preset();}
     loop_rate.sleep(); //延时等待
  }  */
  return 0;
}






