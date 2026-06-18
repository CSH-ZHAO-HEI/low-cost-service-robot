#include "wheeltec_four_arm_pick/test_param.h"
#include <moveit/move_group_interface/move_group_interface.h>
std::string  target_direction;

int arm_done=1,hand_done=1; //置0是表示已经操作完成这个动作，1是表示有新的动作需要完成

std::string arm_state="none";

//接收机械臂运动状态的回调函数
void car_init::arm_state_callback(const std_msgs::String &state)
{
  static std::string last_arm_state;
  arm_state=state.data;
  if(last_arm_state!=arm_state) arm_done=1; //机械臂新状态切换过程检测
  last_arm_state=arm_state;
}

//rqt工具dynamic_reconfigure实现实时调参
void car_init::reconfigCB(wheeltec_arm_pick::paramsConfig &config, uint32_t level)
{
  static double last_arm_upper_and_lower =0,last_hand_open_and_close=0;
  /*
  ROS_INFO("x_p param : %f",config.x_p);
  ROS_INFO("x_i param : %f",config.x_i);
  ROS_INFO("x_d param : %f",config.x_d);
  ROS_INFO("y_p param : %f",config.y_p);
  ROS_INFO("y_i param : %f",config.y_i);
  ROS_INFO("y_d param : %f",config.y_d);
  ROS_INFO("color_location_x param : %f",config.color_location_x);
  ROS_INFO("color_location_y param : %f",config.color_location_y);
  */
  x_p=config.x_p;
  x_i=config.x_i;
  x_d=config.x_d;
  y_p=config.y_p;
  y_i=config.y_i;
  y_d=config.y_d;
  z_p=config.z_p;
  z_i=config.z_i;
  z_d=config.z_d;
  car_start=config.car_start;
  color_location_x=config.color_location_x;
  color_location_y=config.color_location_y;
  arm_upper_and_lower=config.arm_upper_and_lower;
  hand_open_and_close=config.hand_open_and_close;

  if(last_arm_upper_and_lower!=arm_upper_and_lower) arm_done=1;
  if(last_hand_open_and_close!=hand_open_and_close) hand_done=1;
  last_arm_upper_and_lower=arm_upper_and_lower;
  last_hand_open_and_close=hand_open_and_close;
}

//接收运动底盘的回调函数
void car_init::car_command_callback(const wheeltec_arm_pick::pick_and_put &msg)
{
  car_state=msg.car_state;
  target_angle=msg.angle;
}

//获取色块位置的回调函数，在函数中做pid计算，将色块定位到可夹取位置
void car_init::color_location_callback(const wheeltec_arm_pick::four_arm_position &msg)
{
  static int count=0;
 //mini_mec
 if(car_mode=="mini_mec_four_arm"){
  distance_y = msg.angleX;
  distance_x = msg.angleY;
  if(car_start==1)
  {
    target_liner_x=PID_control_x();
    target_liner_y=PID_control_y();
  }
  else
  {
    target_liner_x=0;
    target_liner_y=0;
  }
    target_angular_z=0; //色块定位过程中只需要x和y轴的线速度
  }
  //mini_tank,mini_4wd
  else if(car_mode=="mini_tank_four_arm"||car_mode=="mini_4wd_four_arm"){
   distance_x = msg.angleY;
   angular_z = msg.angleX;
   if(car_start==1)
  {
    target_liner_x=PID_control_x();
    target_angular_z=PID_control_z();
  }
  else
  {
    target_liner_x=0;
    target_angular_z=0;
  }
    target_liner_y=0; //色块定位过程中只需要x和z轴的线速度
  }

  cmd_vel_publish();//将底盘速度发布出去
}

//运动底盘速度话题发布函数
void car_init::cmd_vel_publish()
{
  geometry_msgs::Twist msg;
  msg.linear.x=target_liner_x;
  msg.linear.y=target_liner_y;
  msg.angular.z=target_angular_z;
  cmd_vel_pub.publish(msg);
}


//底盘在x轴方向（前后）定位色块的PID计算
float car_init::PID_control_x()
{
    static float last_error=0,output=0,sum_error;
    pid_x.error=distance_x-(color_location_x);
    //sum_error=sum_error+pid_x.error;
    //    if  (sum_error>1) sum_error=1;
    //else if (sum_error<-1) sum_error=-1;
    //output=x_d*last_error+x_p*pid_x.error+x_i*sum_error;
    output=x_d*last_error+x_p*pid_x.error;
    last_error=pid_x.error;
    return output;
}

//底盘在y轴方向（左右）定位色块的PID计算
float car_init::PID_control_y()
{
    static float last_error=0,output=0,sum_error;
    pid_y.error=distance_y-(color_location_y);
    //sum_error=sum_error+pid_y.error;
    //    if  (sum_error>1)  sum_error=1;
    //else if (sum_error<-1) sum_error=-1;
    //output=y_d*last_error+y_p*pid_y.error+y_i*sum_error;
    output=y_d*last_error+y_p*pid_y.error;
    last_error=pid_y.error;
    return output;

}

//底盘在z轴方向定位色块的PID计算
float car_init::PID_control_z()
{
    static float last_error=0,output=0;
    pid_z.error=angular_z-(color_location_y);
    output=z_d*last_error+z_p*pid_z.error;
    last_error=pid_z.error;
    return output;

}

//取绝对值函数
float car_init::abs_float(float input)
{

if(input<0) return -input;
else  return input;
}

//循环执行的内容函数
void car_init::control()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface arm("arm");   
    moveit::planning_interface::MoveGroupInterface hand("hand"); 
    hand.setNamedTarget("hand_open"); hand.move();  sleep(1);
    arm.setNamedTarget("arm_uplift");  arm.move();  sleep(1);
  while(ros::ok())
  {
    if (arm_done==1&&arm_upper_and_lower== 1)  
    { 
      arm.setNamedTarget("arm_clamp");   arm.move();  sleep(1);
      arm_done =0;
    }
    else if (arm_done==1&&arm_upper_and_lower==2)
    { 
      arm.setNamedTarget("arm_uplift");  arm.move();  sleep(1);
      arm_done =0; 
    }

    if (hand_done==1&&hand_open_and_close== 1)  
    { 
      hand.setNamedTarget("hand_open");   hand.move();  sleep(1);
      hand_done =0;
    }
    else if (hand_done==1&&hand_open_and_close==2)
    { 
      hand.setNamedTarget("hand_close");  hand.move();  sleep(1);
      hand_done =0; 
    }
    //ROS_INFO("%x",arm_done);

    ros::spinOnce();
  }
}


//构造函数
car_init::car_init()
{
    //参数服务器
    ros::NodeHandle param_nh("~"); //私有命名空间的节点句柄，用于参数服务器
    param_nh.param("x_p",x_p,0.0); //x轴方向的pid参数
    param_nh.param("x_i",x_i,0.0);
    param_nh.param("x_d",x_d,0.0); 
    param_nh.param("y_p",y_p,0.0); //y轴方向的pid参数
    param_nh.param("y_i",y_i,0.0);
    param_nh.param("y_d",y_d,0.0);
    param_nh.param("z_p",z_p,0.0); //z轴方向的pid参数
    param_nh.param("z_i",z_i,0.0);
    param_nh.param("z_d",z_d,0.0);
    param_nh.param("color_location_x",color_location_x,0.0); //色块的x轴方位的目标位置
    param_nh.param("color_location_y",color_location_y,0.0); //色块的y轴方位的目标位置
    param_nh.param("arm_upper_and_lower",arm_upper_and_lower,0.0);
    param_nh.param("hand_open_and_close",hand_open_and_close,0.0);
    param_nh.param("car_start",car_start,0.0);
    param_nh.param<std::string>("car_mode",car_mode,"mini_mec_four_arm");
    //arm_state_pub=n.advertise<std_msgs::String>("arm_state",10);
    cmd_vel_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",10);
    car_command_sub=n.subscribe("car_command",10,&car_init::car_command_callback,this);
    color_location_sub=n.subscribe("object_tracker/current_position",10,&car_init::color_location_callback,this);
    arm_state_sub=n.subscribe("arm_state",10,&car_init::arm_state_callback,this);
    

    dynamic_reconfigure::Server<wheeltec_arm_pick::paramsConfig> *             dynamic_reconfigure_server;
    dynamic_reconfigure::Server<wheeltec_arm_pick::paramsConfig>::CallbackType dynamic_reconfigure_callback;

    dynamic_reconfigure_callback = boost::bind(&car_init::reconfigCB, this, _1, _2);

    dynamic_reconfigure_server = new dynamic_reconfigure::Server<wheeltec_arm_pick::paramsConfig>(param_nh);
    dynamic_reconfigure_server->setCallback(dynamic_reconfigure_callback);

    ROS_INFO_STREAM("car_location_color_node_init_successful");

}

//析购函数
car_init::~car_init()
{
    ROS_INFO_STREAM("car_location_color_node_close");
}


int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "test_param"); //节点初始化
    car_init car_control;  //实例化一个对象
    car_control.control(); //循环执行内容
    return 0;
}




