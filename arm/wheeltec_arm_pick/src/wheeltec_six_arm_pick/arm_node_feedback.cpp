
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

using namespace std;
ros::Publisher preset_flag_pub,color_choose_pub,count_flag_pub,car_zero_pub;

int visual_follow_flag = 0,fk_demo_flag = 0,ik_demo_flag = 0,visual_clamp_flag = 0;
int face_follow_flag = 0,gesture_flag=0,cartesian_flag = 0,voice_flag = 0;//各功能开启标志位
std::string sw = "on";//标志位区分开关
std::string sw_color = "init";//颜色开关
/***********************************
函数功能：获取语音控制指令
***********************************/
void voice_word_callback(const std_msgs::String& msg)
{
	/***指令***/
	std::string str1 = msg.data.c_str();    //取传入数据
	std::string str2 = "机械臂色块跟随";
	std::string str3 = "关闭机械臂色块跟随"; 
	std::string str4 = "机械臂人脸跟随";
	std::string str5 = "关闭机械臂人脸跟随";
	std::string str6 = "机械臂正解";
  	std::string str7 = "机械臂逆解";
  	std::string str8 = "打开手势识别";
	std::string str9 = "关闭手势识别";
	std::string str10 = "机械臂笛卡尔路径规划";
	std::string str11 = "机械臂休眠";
	std::string str12 = "选定红色色块";
	std::string str13 = "选定绿色色块";
	std::string str14 = "选定蓝色色块";
	std::string str15 = "选定黄色色块";
	std::string str16 = "选定黑色色块";
	std::string str17 = "机械臂色块夹取";
	std::string str18 = "关闭色块夹取";
/***********************************
指令：打开机械臂色块跟随

***********************************/
    ros::V_string v_nodes;
    std::string node_name = std::string("/joint_state_publisher");
	if(str1 == str2 && sw == "on")
	{ 
	   ros::master::getNodes(v_nodes);
       auto if_publish = std::find(v_nodes.begin(),v_nodes.end(),node_name.c_str());
       if(if_publish != v_nodes.end()){
       	 system("rosnode kill /joint_state_publisher");
       }
	   sw = "off";
	   sw_color = "follow";
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/opening.wav");
	   std_msgs::Int8 preset_flag_msg;
       preset_flag_msg.data=1;
       preset_flag_pub.publish(preset_flag_msg);
	   system("dbus-launch gnome-terminal -- roslaunch wheeltec_arm_pick voi_color_tracking.launch");
	   cout<<">>>正在打开色块跟随"<<endl;
	}
/***********************************
指令：关闭机械臂色块跟随

***********************************/
	else if(str1 == str3)
	{  
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/OK.wav");
	   system("rosnode kill /color_follower");
	   system("rosnode kill /color_decetor");
	   system("rosnode kill /usb_cam");
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/visual_follow/visual_close.wav");
	   system("gnome-terminal -- roslaunch wheeltec_arm_pick voi_demo.launch");
	   sleep(1);
	   system("rosnode kill /move_group");
	   cout<<">>>已经关闭色块跟随"<<endl;
	   std_msgs::Int8 preset_flag_msg;
       preset_flag_msg.data=1;
       preset_flag_pub.publish(preset_flag_msg);
	   sw = "on";
	   sw_color = "init";		
	}
/***********************************
指令：打开机械臂人脸跟随

***********************************/
	if(str1 == str4 && sw == "on")
	{ 
	   ros::master::getNodes(v_nodes);
       auto if_publish = std::find(v_nodes.begin(),v_nodes.end(),node_name.c_str());
       if(if_publish != v_nodes.end()){
       	 system("rosnode kill /joint_state_publisher");
       }
	   sw = "off";
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/opening.wav");
	   std_msgs::Int8 preset_flag_msg;
       preset_flag_msg.data=1;
       preset_flag_pub.publish(preset_flag_msg);
       //system("rosnode kill /joint_state_publisher");
	   system("dbus-launch gnome-terminal -- roslaunch wheeltec_arm_pick voi_face_tracking.launch");
	   cout<<">>>正在打开人脸跟随"<<endl;
	}
/***********************************
指令：关闭机械臂人脸跟随

***********************************/
	else if(str1 == str5)
	{  
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/OK.wav");
	   system("rosnode kill /face_follower");
	   system("rosnode kill /face_detector");
	   system("rosnode kill /usb_cam");
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/face_follow/face_close.wav");
	   system("gnome-terminal -- roslaunch wheeltec_arm_pick voi_demo.launch");
	   sleep(1);
	   system("rosnode kill /move_group");
	   cout<<">>>已经关闭人脸跟随"<<endl;
	   std_msgs::Int8 preset_flag_msg;
       preset_flag_msg.data=1;
       preset_flag_pub.publish(preset_flag_msg);
	   sw = "on";	
	}
/***********************************
指令：打开机械臂正解

***********************************/
	if(str1 == str6)
	{
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/opening.wav");
	   system("dbus-launch gnome-terminal -- roslaunch wheeltec_arm_pick voi_demo.launch");
	   sleep(3);
	   system("dbus-launch gnome-terminal -- roslaunch wheeltec_arm_pick voi_fk_demo.launch");
	   cout<<">>>正在打开机械臂正解"<<endl;
	}
/***********************************
指令：打开机械臂逆解

***********************************/
	if(str1 == str7)
	{
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/opening.wav");
	   system("dbus-launch gnome-terminal -- roslaunch wheeltec_arm_pick voi_demo.launch");
	   sleep(3); 
	   system("dbus-launch gnome-terminal -- roslaunch wheeltec_arm_pick voi_ik_demo.launch");
	   cout<<">>>正在打开机械臂逆解"<<endl;
	}
/***********************************
指令：打开手势识别

***********************************/
	if(str1 == str8 && sw == "on")
	{ 
	   ros::master::getNodes(v_nodes);
       auto if_publish = std::find(v_nodes.begin(),v_nodes.end(),node_name.c_str());
       if(if_publish != v_nodes.end()){
       	 system("rosnode kill /joint_state_publisher");
       }
	   sw = "off";
	   std_msgs::Int8 preset_flag_msg;
       preset_flag_msg.data=1;
       preset_flag_pub.publish(preset_flag_msg);
       //system("rosnode kill /joint_state_publisher");
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/opening.wav");
	   system("dbus-launch gnome-terminal -- roslaunch wheeltec_arm_pick voi_gesture_recognition.launch");
	   cout<<">>>正在打开手势识别"<<endl;
	}
/***********************************
指令：关闭手势识别

***********************************/
	else if(str1 == str9)
	{  
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/OK.wav");
	   system("rosnode kill /gesture_recognition");
	   system("rosnode kill /gesture_recognition_execute");
	   system("rosnode kill /usb_cam");
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/gesture/gesture_close.wav");
	   system("gnome-terminal -- roslaunch wheeltec_arm_pick voi_demo.launch");
	   sleep(2);
	   system("rosnode kill /move_group");
	   cout<<">>>已经关闭手势识别"<<endl;
	   std_msgs::Int8 preset_flag_msg;
       preset_flag_msg.data=1;
       preset_flag_pub.publish(preset_flag_msg);
	   sw = "on";	
	}
/***********************************
指令：进行笛卡尔路径规划

***********************************/
	else if(str1 == str10)
	{ 
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/opening.wav");
	   system("dbus-launch gnome-terminal -- roslaunch wheeltec_arm_pick voi_demo.launch");
	   sleep(3); 
	   system("dbus-launch gnome-terminal -- roslaunch wheeltec_arm_pick voi_cartesian_demo.launch");
	   cout<<">>>正在进行笛卡尔路径规划"<<endl;
	}

/***********************************
指令：选择红色色块

***********************************/
	else if(str1 == str12 && sw == "off")
	{ 
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/OK.wav");
	   std_msgs::Int8 color_flag_msg;
	   color_flag_msg.data = 1;
	   color_choose_pub.publish(color_flag_msg);
	   cout<<">>>选择红色色块"<<endl;
	}
/***********************************
指令：选择绿色色块

***********************************/
	else if(str1 == str13 && sw == "off")
	{ 
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/OK.wav");
	   if (sw_color == "follow")
	   {
		   std_msgs::Int8 color_flag_msg;
		   color_flag_msg.data = 2;
		   color_choose_pub.publish(color_flag_msg);
	   }
	   else if (sw_color == "clamp")
	   {
	   	   std_msgs::Int8 color_flag_msg;
		   color_flag_msg.data = 3;
		   color_choose_pub.publish(color_flag_msg);
	   }	   
	   cout<<">>>选择绿色色块"<<endl;
	}
/***********************************
指令：选择蓝色色块

***********************************/
	else if(str1 == str14 && sw == "off")
	{ 
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/OK.wav");
	   if (sw_color == "follow")
	   {
		   std_msgs::Int8 color_flag_msg;
		   color_flag_msg.data = 3;
		   color_choose_pub.publish(color_flag_msg);
	   }
	   else if (sw_color == "clamp")
	   {
	   	   std_msgs::Int8 color_flag_msg;
		   color_flag_msg.data = 2;
		   color_choose_pub.publish(color_flag_msg);
	   }	   
	   cout<<">>>选择蓝色色块"<<endl;
	}
/***********************************
指令：选择黄色色块

***********************************/
	else if(str1 == str15 && sw == "off")
	{ 
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/OK.wav");
	   if (sw_color == "follow")
	   {
		   std_msgs::Int8 color_flag_msg;
		   color_flag_msg.data = 4;
		   color_choose_pub.publish(color_flag_msg);
	   }
	   else if (sw_color == "clamp")
	   {
	   	   std_msgs::Int8 color_flag_msg;
		   color_flag_msg.data = 1;
		   color_choose_pub.publish(color_flag_msg);
	   }	   
	   cout<<">>>选择黄色色块"<<endl;
	}
/***********************************
指令：选择黑色色块

***********************************/
	else if(str1 == str16 && sw == "off")
	{ 
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/OK.wav");
	   if (sw_color == "follow")
	   {
		   std_msgs::Int8 color_flag_msg;
		   color_flag_msg.data = 4;
		   color_choose_pub.publish(color_flag_msg);
	   }
	   else if (sw_color == "clamp")
	   {
	   	   std_msgs::Int8 color_flag_msg;
		   color_flag_msg.data = 1;
		   color_choose_pub.publish(color_flag_msg);
	   }	   
	   cout<<">>>选择黑色色块"<<endl;
	}
/***********************************
指令：打开机械臂色块夹取

***********************************/
	else if(str1 == str17 && sw == "on")
	{ 
	   sw = "off";
	   sw_color = "clamp";
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/opening.wav");
	   std_msgs::Int8 preset_flag_msg;
       preset_flag_msg.data=1;
       preset_flag_pub.publish(preset_flag_msg);
       system("dbus-launch gnome-terminal -- roslaunch wheeltec_arm_pick voi_demo.launch");
	   sleep(3); 
	   system("dbus-launch gnome-terminal -- roslaunch wheeltec_arm_pick voi_arm_pick_color.launch");
	   cout<<">>>正在打开色块夹取"<<endl;
	}
/***********************************
指令：关闭机械臂色块夹取

***********************************/
	else if(str1 == str18)
	{  
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/OK.wav");
	   system("rosnode kill /arm_control");
	   system("rosnode kill /car_location_color");
	   system("rosnode kill /visual_tracker");
	   system("rosnode kill /usb_cam");
	   system("rosnode kill /move_group");
	   geometry_msgs::Twist msg;
       msg.linear.x=0;
       msg.linear.y=0;
       msg.angular.z=0;
       car_zero_pub.publish(msg);
	   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/clamp/clamp_close.wav");
	   cout<<">>>已经关闭色块夹取"<<endl;

	   std_msgs::Int8 preset_flag_msg;
       preset_flag_msg.data=1;
       preset_flag_pub.publish(preset_flag_msg);
	   sw = "on";
	   sw_color = "init";	
	}
}
/**************************************************************************
函数功能：色块跟随开启成功标志位sub回调函数
入口参数：visual_follow_flag.msg  visualTracker.py
返回  值：无
**************************************************************************/
void visual_follow_flagCallback(std_msgs::Int8 msg)
{
	visual_follow_flag = msg.data;
	if(visual_follow_flag == 1)
	{
		system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/visual_follow/visual_open.wav");
		cout<<">>>色块跟随打开成功"<<endl;	
	}
		printf("%d\n",visual_follow_flag);
}
/**************************************************************************
函数功能：色块夹取开启成功标志位sub回调函数
入口参数：visual_clamp_flag.msg  wheeltec_arm_pick/visualTracker.py
返回  值：无
**************************************************************************/
void visual_clamp_flagCallback(std_msgs::Int8 msg)
{
	visual_clamp_flag = msg.data;
	if(visual_clamp_flag == 1)
	{
		system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/clamp/clamp_open.wav");
		cout<<">>>色块夹取打开成功"<<endl;	
	}
		printf("%d\n",visual_clamp_flag);
}
/**************************************************************************
函数功能：人脸跟随开启成功标志位sub回调函数
入口参数：face_follow_flag.msg  face_detector.py
返回  值：无
**************************************************************************/
void face_follow_flagCallback(std_msgs::Int8 msg)
{
	face_follow_flag = msg.data;
	
	if(face_follow_flag == 1)
	{
		system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/face_follow/face_open.wav");

		cout<<"人脸跟随打开成功"<<endl;
	}
	printf("%d\n",face_follow_flag);
}

/**************************************************************************
函数功能：机械臂正解开启成功标志位sub回调函数
入口参数：fk_demo_flag.msg  arm_demo/arm_fk_demo.cpp
返回  值：无
**************************************************************************/
void fk_flagCallback(std_msgs::Int8 msg)
{
	fk_demo_flag = msg.data;
	if(fk_demo_flag == 1)
	{
		system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/demo/fk_demo_over.wav");
		cout<<"机械臂正解打开成功"<<endl;
		sleep(5);
		system("rosnode kill /move_group");
	   	//system("rosnode kill /joint_state_publisher");
	}
		printf("%d\n",fk_demo_flag);
}

/**************************************************************************
函数功能：机械臂逆解开启成功标志位sub回调函数
入口参数：ik_demo_flag.msg  arm_demo/arm_ik_demo.cpp
返回  值：无
**************************************************************************/
void ik_flagCallback(std_msgs::Int8 msg)
{
	ik_demo_flag = msg.data;
	if(ik_demo_flag == 1)
	{
		system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/demo/ik_demo_over.wav");
		cout<<"机械臂逆解打开成功"<<endl;
		sleep(5);
		system("rosnode kill /move_group");
	   	//system("rosnode kill /joint_state_publisher");
	}
		printf("%d\n",ik_demo_flag);
}

/**************************************************************************
函数功能：笛卡尔路径规划开启成功标志位sub回调函数
入口参数：cartesian_flag.msg  arm_demo/cartesian_demo.cpp
返回  值：无
**************************************************************************/
void cartesian_flagCallback(std_msgs::Int8 msg)
{
	cartesian_flag = msg.data;
	if(cartesian_flag == 1)
	{
		system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/demo/cartesian_over.wav");
		cout<<"笛卡尔路径规划开启成功"<<endl;
		sleep(5);
		system("rosnode kill /move_group");
	   	//system("rosnode kill /joint_state_publisher");
	}
		printf("%d\n",cartesian_flag);
}

/**************************************************************************
函数功能：手势识别开启成功标志位sub回调函数
入口参数：gesture_flag.msg  wheeltec_tracker_pkg/gesture_recognition.cpp
返回  值：无
**************************************************************************/
void gesture_flag_flagCallback(std_msgs::Int8 msg)
{
	gesture_flag = msg.data;
	if(gesture_flag == 1)
	{
		system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/gesture/gesture_open.wav");
		cout<<"手势识别开启成功"<<endl;
	}
		printf("%d\n",gesture_flag);
}

/**************************************************************************
函数功能：寻找语音开启成功标志位sub回调函数
入口参数：voice_flag_msg  voice_control.cpp
返回  值：无
**************************************************************************/
/*void voice_flag_Callback(std_msgs::Int8 msg)
{
	voice_flag = msg.data;
	if(voice_flag == 1)
	{
		system("rosnode kill /move_group");
	   	system("rosnode kill /joint_state_publisher");
	}
}*/

int main(int argc, char** argv)
{

	

	ros::init(argc, argv, "arm_node_feedback");  //初始化节点  

	ros::NodeHandle nd; //初始化句柄

	ros::Subscriber voice_word_sub = nd.subscribe("voice_words",10,voice_word_callback); //订阅语音输入信息话题
	ros::Subscriber fk_demo_flag_sub = nd.subscribe("fk_demo_flag", 1, fk_flagCallback);//机械臂正解开启标志位订阅
	ros::Subscriber ik_demo_flag_sub = nd.subscribe("ik_demo_flag", 1, ik_flagCallback);//机械臂逆解开启标志位订阅
	ros::Subscriber cartesian_flag_sub = nd.subscribe("cartesian_flag", 1, cartesian_flagCallback);//笛卡尔路径规划开启标志位订阅

	ros::Subscriber face_follow_flag_sub = nd.subscribe("face_follow_flag", 1, face_follow_flagCallback);//人脸跟随开启标志位订阅
	ros::Subscriber visual_follow_flag_sub = nd.subscribe("visual_follow_flag", 1, visual_follow_flagCallback);//色块跟随开启标志位订阅
	ros::Subscriber visual_clamp_flag_sub = nd.subscribe("visual_clamp_flag", 1, visual_clamp_flagCallback);//色块夹取开启标志位订阅
	ros::Subscriber gesture_flag_sub = nd.subscribe("gesture_flag", 1, gesture_flag_flagCallback);//色块跟随开启标志位订阅
	/*ros::Subscriber voice_flag_sub = nd.subscribe("voice_flag", 1, voice_flag_Callback);*/
    car_zero_pub=nd.advertise<geometry_msgs::Twist>("cmd_vel",10);
	count_flag_pub = nd.advertise<std_msgs::Int8>("count_flag", 1);//命令词刷新标志位
	preset_flag_pub = nd.advertise<std_msgs::Int8>("preset_flag",1);//预设标志位发布
	color_choose_pub = nd.advertise<std_msgs::Int8>("color_flag",1);//色块选择标志位

	sleep(10);

	system("rosnode kill /move_group");
	//system("rosnode kill /joint_state_publisher");

	int count = 0;//计数
	double rate = 2;
	ros::Rate loopRate(rate);

	while(ros::ok())
	{
		if (count == 24)//设定刷新命令词时间
		{
			std_msgs::Int8 count_flag_msg;
			count_flag_msg.data = 1;
			count_flag_pub.publish(count_flag_msg);
			count = 0;
		}
		count++;
		ros::spinOnce(); 
		loopRate.sleep();
	}

	return 0;
}

