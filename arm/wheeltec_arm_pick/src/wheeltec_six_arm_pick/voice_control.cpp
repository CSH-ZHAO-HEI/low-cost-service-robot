#include "wheeltec_six_arm_pick/voice_control.h"
#include <iostream>
#include <std_msgs/Int8.h>
#include <cstdlib>
#include <ctime>

#define random(a,b) (rand() % (b-a)+ a + 1)
int game_mode = 0,find_guess = 0,game = 0;
std::string arm_state="ready",arm_action="noon";
std::vector<double> joint_group_positions(5); //关节正解运动用到的关机目标运动数组
ros::Publisher joints_state_publisher;  //初始化关节目标角度的发布者
ros::Publisher guess_flag_publisher;//猜方位标志位
int action_count =0;  //动作执行时用到的计数变量
float patrol_init_value=0,nod_init_value=0,dance_init_value=0;
int voice_target=0,voice_follower_flag=0,voice_control_flag=0; //控制逻辑相关标志位
int patrol_flag=0,nod_flag=0,dance_flag=0;  //控制逻辑相关标志位
float joint_target1=0,joint_target2=0,joint_target3=0,joint_target4=0,joint_target5=0; //赋值给moveit做正解的目标关节值
bool arm_success,hand_close_success,hand_open_success; //moveit正解计算（返回值）是否成功的标志位
int count_flag = 0;  //命令词刷新标志位
std::string sw = "on";//标志位区分开关

void Direction();
void guess_game();
using namespace std;
/***********************************
函数功能：机械臂巡视的执行函数
***********************************/
void arm_patrol(void)
{
   static float joint_step=0.02;  //机械臂运动的步进值
   sensor_msgs::JointState arm_joint_msg;  //定义一个机械臂控制信息的消息数据类型
   ros::Time pub_time=ros::Time::now();  //获取当前的ROS时间
   if  (action_count>=0 && action_count<70) patrol_init_value=patrol_init_value-joint_step; //执行的第1段动作
   else if(action_count>=70 && action_count<210)  patrol_init_value=patrol_init_value+joint_step; //执行的第2段动作
   else if (action_count>=210) patrol_init_value=patrol_init_value-joint_step; //执行的第3段动作
      //输入当前的ros时间
      arm_joint_msg.header.stamp=pub_time;
      //输入机械臂臂身的目标关节角度（单位：弧度）
      arm_joint_msg.position.push_back(patrol_init_value);
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

    if(action_count>280)  //执行完成后将标志位置零
    {
      patrol_flag=0;
      action_count=0;
      system("dbus-launch gnome-terminal -- roslaunch wheeltec_arm_pick voi_demo.launch");
      sleep(1);
      system("rosnode kill /move_group");
      patrol_init_value=0;
      arm_state="ready";
      if (game_mode == 1)
      {
        system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/guess_game/guess.wav");
      }
    }
}
/***********************************
函数功能：机械臂点头的执行函数
***********************************/
void arm_nod(void)
{
   static float joint_step=0.04; //机械臂运动的步进值
   sensor_msgs::JointState arm_joint_msg;  //定义一个机械臂控制信息的消息数据类型
   ros::Time pub_time=ros::Time::now();   //获取当前的ROS时间
   if  (action_count>=0 && action_count<20)      nod_init_value=nod_init_value-joint_step; //执行的第1段动作
   else if(action_count>=20 && action_count<40)  nod_init_value=nod_init_value+joint_step; //执行的第2段动作
   else if(action_count>=40 && action_count<60)  nod_init_value=nod_init_value-joint_step; //执行的第3段动作
   else if (action_count>=60)                    nod_init_value=nod_init_value+joint_step; //执行的第4段动作
      //输入当前的ros时间
      arm_joint_msg.header.stamp=pub_time;
      //输入机械臂臂身的目标关节角度（单位：弧度）
      arm_joint_msg.position.push_back(0);
      arm_joint_msg.position.push_back(0);
      arm_joint_msg.position.push_back(-nod_init_value);
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
 
    if(action_count>80) //执行完成后将标志位置零
    {
      nod_flag=0;
      action_count=0;
      nod_init_value=0;
      arm_state="ready";
      system("dbus-launch gnome-terminal -- roslaunch wheeltec_arm_pick voi_demo.launch");
      sleep(1);
      system("rosnode kill /move_group");
      if (game_mode == 2)
      {
        system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/guess_game/guess.wav");
      }
    } 
}
/***********************************
函数功能：机械臂跳舞的执行函数
***********************************/
void arm_dance(void)  
{
   static float joint_step=0.025; //机械臂运动的步进值
   sensor_msgs::JointState arm_joint_msg;  //定义一个机械臂控制信息的消息数据类型
   ros::Time pub_time=ros::Time::now();   //获取当前的ROS时间
   if  (action_count>=0 && action_count<20)       dance_init_value=dance_init_value-joint_step; //执行的第1段动作
   else if(action_count>=20 && action_count<60)   dance_init_value=dance_init_value+joint_step; //执行的第2段动作
   else if(action_count>=60 && action_count<100)  dance_init_value=dance_init_value-joint_step; //执行的第3段动作
   else if(action_count>=100 && action_count<140) dance_init_value=dance_init_value+joint_step; //执行的第4段动作
   else if(action_count>=140 )                    dance_init_value=dance_init_value-joint_step; //执行的第5段动作
      //输入当前的ros时间
      arm_joint_msg.header.stamp=pub_time;
      //输入机械臂臂身的目标关节角度（单位：弧度）
      arm_joint_msg.position.push_back(-dance_init_value);
      arm_joint_msg.position.push_back(dance_init_value);
      arm_joint_msg.position.push_back(dance_init_value); 
      arm_joint_msg.position.push_back(dance_init_value);
      arm_joint_msg.position.push_back(3*dance_init_value);
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

    if(action_count>160)  //执行完成后将标志位置零
    {
      dance_flag=0;
      action_count=0;
      dance_init_value=0;
      arm_state="ready";
      system("dbus-launch gnome-terminal -- roslaunch wheeltec_arm_pick voi_demo.launch");
      sleep(1);
      system("rosnode kill /move_group");
      if (game_mode == 3)
      {
        system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/guess_game/guess.wav");
      }
    }
}
/***********************************
函数功能：猜谜小游戏
***********************************/
void guess_game()
{
  srand((int)time(0));
  game_mode = random(0, 3);
  if (game_mode == 1)
  {
    voice_control_flag =1;
  }
  else if (game_mode == 2)
  {
   voice_control_flag=3;
  }
  else if (game_mode == 3)
  {
   voice_control_flag=2;
  }
}
/***********************************
函数功能：获取语音唤醒角度
***********************************/
void mic_awak_angle_callback(const std_msgs::Int32 &msg)
{
   voice_target=msg.data;
   ROS_INFO("voice_target_is    :(%d)",voice_target);
   //机械臂正方向180度朝向
   if((voice_target>=60)&&(voice_target<=240))
   {
    joint_target1=0.0174*(voice_target-150);
    joint_target3=-1.57;
   } 
  //机械臂反方向180度朝向
   else   
   {
     if((voice_target>240)&&(voice_target<=360))
      {
         joint_target1=0.0174*(voice_target-150)-3.14;
         joint_target3=1.57; 
      }
      else
      {
         joint_target1=0.0174*(voice_target)+0.785; 
         joint_target3=1.57;
      }
   }
   voice_follower_flag=1;
   joint_target2=0,joint_target4=0,joint_target5=0;
}
/***********************************
函数功能：获取语音控制指令
***********************************/
void voice_words_callback(const std_msgs::String& msg)
{
	/***指令***/
	std::string str1 = msg.data.c_str();    //取传入数据
	std::string str2 = "机械臂巡视";
	std::string str3 = "机械臂跳舞";
  std::string str4 = "机械臂点头";
  std::string str5 = "来玩猜谜游戏";
  std::string str6 = "小微在巡视";
  std::string str7 = "小微在点头";
  std::string str8 = "小微在跳舞";

	if(str1 == str2 && sw == "on")
	{
   voice_control_flag=1;         
   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/arm_patrol.wav");
   cout<<">>>好的：机械臂巡视"<<endl;
	}
/***********************************
指令：机械臂跳舞
动作：跳舞
***********************************/
  else if(str1 == str3 && sw == "on")
  {
   voice_control_flag=2;         
   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/arm_dance.wav");
   cout<<">>>好的：机械臂跳舞"<<endl;
  }
/***********************************
指令：机械臂点头
动作：点头
***********************************/
	else if(str1 == str4 && sw == "on")
	{
   voice_control_flag=3;         
   system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/voice_base/arm_nod.wav");
	 cout<<">>>好的：机械臂点头"<<endl;
	}

/***********************************
指令：猜谜游戏
动作：无动作
***********************************/
  else if(str1 == str5 && sw == "on")
  { 
    sw = "off";
    system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/guess_game/sure.wav");
    sleep(1.5);
    guess_game(); 
    sleep(2);      
    cout<<">>>请说出猜谜答案: "<<endl;
    cout<<">>>小微在巡视|小微在点头|小微在跳舞 "<<endl;
  }

  else if(str1 == str6 && sw == "off")
  { 
    if(game_mode == 1)
    {
      system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/guess_game/patrol.wav");
       game_mode = 0;
       sw = "on";
    }
    else
    {
      system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/guess_game/guess_wrong.wav");
    }
  }

  else if(str1 == str7 && sw == "off")
  { 
    if(game_mode == 2)
    {
      system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/guess_game/nod.wav");
       game_mode = 0;
       sw = "on";
    }
    else
    {
      system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/guess_game/guess_wrong.wav");
    }
  }

  else if(str1 == str8 && sw == "off")
  { 
    if(game_mode == 3)
    {
      system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/guess_game/dance.wav");
      game_mode = 0;
      sw = "on";
      system("dbus-launch gnome-terminal -- roslaunch wheeltec_arm_pick voi_demo.launch");
      sleep(1);
      system("rosnode kill /move_group");
    }
    else
    {
      system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_robot/src/xf_mic_asr_offline_circle/feedback_voice/guess_game/guess_wrong.wav");
    }
  }

}

/**************************************************************************
函数功能：命令词刷新标志位sub回调函数
入口参数：count_flag_msg  node_feedback.cpp
返回  值：无
**************************************************************************/
void count_flag_Callback(std_msgs::Int8 msg)
{
  count_flag = msg.data;
  if(count_flag == 1 && game_mode == 0 &&find_guess == 0)
  {
    Direction();
  }

}
/**************************************************************************
函数功能：打印可执行的语音指令
返回  值：无
**************************************************************************/
void Direction()
{
  cout<<"\n###########################################"<<endl; 
  cout<<">>>输入以下语音指令控制机械臂吧:"<<endl; 
  cout<<">>>机械臂巡视"<<endl;
  cout<<">>>机械臂跳舞"<<endl;  
  cout<<">>>机械臂点头"<<endl;
  cout<<">>>机械臂正解"<<endl;
  cout<<">>>机械臂逆解"<<endl;
  cout<<">>>来玩猜谜游戏"<<endl;
  cout<<">>>机械臂笛卡尔路径规划"<<endl;
  cout<<">>>(打开/关闭)手势识别"<<endl;
  cout<<">>>(机械臂/关闭机械臂)人脸跟随"<<endl;
  cout<<">>>(机械臂/关闭机械臂)色块跟随"<<endl;
  cout<<"↓色块跟随打开后可以进行色块选择↓"<<endl;
  cout<<">>>选定(红色/绿色/蓝色/黄色/黑色)色块"<<endl;
  cout<<">>>(机械臂/关闭)色块夹取"<<endl;
  cout<<"↓色块夹取打开后可以进行色块选择↓"<<endl;
  cout<<">>>选定(绿色/蓝色/黄色)色块"<<endl;
  cout<<"############################################\n"<<endl;
  cout<<">>>输入以下语音指令控制小车吧:"<<endl;
  cout<<"小车前进———————————>向前"<<endl;
  cout<<"小车后退———————————>后退"<<endl;
  cout<<"小车左转———————————>左转"<<endl;
  cout<<"小车右转———————————>右转"<<endl;
  cout<<"小车停———————————>停止"<<endl;
  cout<<"小车休眠———————————>休眠，等待下一次唤醒"<<endl;
  cout<<"小车过来———————————>寻找声源"<<endl;
  cout<<"小车去I点———————————>小车自主导航至I点"<<endl;
  cout<<"小车去J点———————————>小车自主导航至J点"<<endl;
  cout<<"小车去K点———————————>小车自主导航至K点"<<endl;
  cout<<"小车雷达跟随———————————>小车打开雷达跟随"<<endl;
  cout<<"关闭雷达跟随———————————>小车关闭雷达跟随"<<endl;
  cout<<"打开自主建图———————————>小车打开自主建图"<<endl;
  cout<<"关闭自主建图———————————>关闭打开自主建图"<<endl;
  cout<<"开始多点导航———————————>小车开始导航"<<endl;
  cout<<"关闭多点导航———————————>小车关闭导航"<<endl;
}
/***********************************
  主函数
***********************************/
int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "voice_follower"); 
    ros::NodeHandle n;
 
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber voice_words_sub = n.subscribe("voice_words",10,voice_words_callback); //订阅语音输入信息话题
    ros::Subscriber mic_awak_angle_sub=n.subscribe("mic/awake/angle",10,mic_awak_angle_callback); //订阅语音唤醒角度话题
    ros::Subscriber count_flag_sub = n.subscribe("count_flag", 1, count_flag_Callback);//命令词刷新标志位话题
    joints_state_publisher=n.advertise<sensor_msgs::JointState>("voice_joint_states",10);//往控制机械臂运动的话题发布信息
    guess_flag_publisher=n.advertise<std_msgs::Int8>("guess_flag",1);
    //初始化过程打印一遍可执行的语音指令
    Direction();

    ros::Rate loop_rate(40); //设置程序执行频率（单位：hz）
    ros::V_string list_nodes;
    std::string node_name = std::string("/joint_state_publisher");
    //while循环执行
    while(ros::ok())
   {
     if(voice_control_flag==1 &&  arm_state=="ready") //机械臂巡视
      {
             ros::master::getNodes(list_nodes);
             auto if_publish = std::find(list_nodes.begin(),list_nodes.end(),node_name.c_str());
             if(if_publish != list_nodes.end()){
               system("rosnode kill /joint_state_publisher");
               //sleep(1);
             }
             patrol_flag=1;
             voice_control_flag=0;  
       }

       else if(voice_control_flag==3 &&  arm_state=="ready")  //机械臂点头
      {
             ros::master::getNodes(list_nodes);
             auto if_publish = std::find(list_nodes.begin(),list_nodes.end(),node_name.c_str());
             if(if_publish != list_nodes.end()){
               system("rosnode kill /joint_state_publisher");
               //sleep(1);
             }  
             nod_flag=1;
             voice_control_flag=0;
       }

       else if(voice_control_flag==2 &&  arm_state=="ready")  //机械臂跳舞
      {
             ros::master::getNodes(list_nodes);
             auto if_publish = std::find(list_nodes.begin(),list_nodes.end(),node_name.c_str());
             if(if_publish != list_nodes.end()){
               system("rosnode kill /joint_state_publisher");
               //sleep(1);
             }  
             dance_flag=1;
             voice_control_flag=0;
       }

      if(patrol_flag==1)
     {
       arm_patrol(); //执行机械臂巡视
       action_count++;
     } 

      if(nod_flag==1)
     {
       arm_nod(); //执行机械臂点头
       action_count++;
     }  

      if(dance_flag==1)
     {
       arm_dance(); //执行机械臂跳舞 
       action_count++;
     } 
    loop_rate.sleep(); //延时等待
    //ros::spinOnce();
  }  
  return 0;
}






