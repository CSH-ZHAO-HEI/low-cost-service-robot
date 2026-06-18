#include "wheeltec_six_arm_pick/car_location_color.h"
std::string  target_direction;
long int error_flag=1,error_count=0;

//接收小车状态的回调函数
void car_init::car_command_callback(const wheeltec_arm_pick::pick_and_put &msg)
{
  car_state=msg.car_state;
  target_angle=msg.angle;
}

//获取色块位置的回调函数，在函数中做pid计算，将色块定位到可夹取位置，当未非目标色块时执行一次机械臂转动夹爪动作
void car_init::color_location_callback(const wheeltec_arm_pick::six_arm_position &msg)
{
  static int count=0;
  

  //检测到目标色块
  if(msg.correct&&car_state==0&&shake_hand==0){
    //识别到目标色块，将error标志位置零
    error_flag=0;

    /*******************小车底盘定位色块*******************/
    //mini_mec
    if(car_mode=="mini_mec_six_arm")
    {
    if(location_flag==0&&car_state==0){  
    distance_y = msg.angleX;
    distance_x = msg.angleY;
    target_liner_x=PID_control_x();
    target_liner_y=PID_control_y();
    }
    //当pid控制下的小车底盘速度小于0.005时开始计数，计数达到150则认为底盘已定位好位置
    if((abs_float(target_liner_x)<0.003)&&(abs_float(target_liner_y)<0.005)&&car_state==0) count++;
    else count=0;
    if(count>150)  location_flag=1,count=0;//色块定位完成
    if(location_flag==0)  target_angular_z=0; //色块定位过程中只需要x和y轴的线速度
    if(location_flag==1) target_liner_x=0,target_liner_y=0; //色块定位完成后底盘不再定位，防止抖动
    }

    //mini_tank,mini_4wd
    else if((car_mode=="mini_tank_six_arm")||(car_mode=="mini_4wd_six_arm")){
    if(location_flag==0&&car_state==0){
    distance_x = msg.angleY;
    angular_z = msg.angleX;
    target_liner_x=PID_control_x();
    target_angular_z=PID_control_z();
    }
    if((abs_float(target_liner_x)<0.003)&&(abs_float(target_angular_z)<0.005)&&car_state==0) count++;
    else count=0;
    if(count>=150)  location_flag=1,count=0;//色块定位完成
    if(location_flag==0)  target_liner_y=0; //色块定位只需要x轴线速度及z轴角速度
    if(location_flag==1){
      target_liner_x=0;
      target_angular_z=0; //色块定位完成后底盘不再定位，防止抖动
    } 
    else if(location_flag==1&&car_state!=0){
      target_liner_x=0;
    }
    }

    //将底盘速度发布出去
    cmd_vel_publish();
  }

  //检测到非目标色块,仅当小车处于识别色块状态且识别到非目标色块执行
  else if(!msg.correct&&car_state==0&&location_flag==0&&shake_hand==0){
    if (msg.color=="yellow"&&shake_hand_done[0]==0) shake_hand=1,shake_hand_done[0]=1;
    if (msg.color=="blue"&&shake_hand_done[1]==0)   shake_hand=1,shake_hand_done[1]=1;
    if (msg.color=="green"&&shake_hand_done[2]==0)  shake_hand=1,shake_hand_done[2]=1;
  }
}

//获取底盘位置的回调函数，利用odom话题来做底盘夹取到色块后旋转多少角度的计算
void car_init::car_pose_callback(const nav_msgs::Odometry &msg)
{
  static float last_target_angle=0,target_position_z=0;
  
  car_position_x=msg.pose.pose.position.x;//获取底盘的位置信息（编码器积分获得）
  car_position_y=msg.pose.pose.position.y;
  car_position_z=msg.pose.pose.position.z;

  if(last_target_angle!=target_angle) target_position_z=car_position_z + target_angle;//状态发生变化，获取需要转多少角度
  if(car_state==1)//夹取到色块后
  {     //给上0.1的允许误差范围
        if(target_position_z<=(car_position_z-0.1))  target_angular_z= -0.6,move_flag=1; //以0.6rad/s的速度旋转
   else if(target_position_z>=(car_position_z+0.1))  target_angular_z=  0.6,move_flag=1;
   else                     target_angular_z= 0,move_flag=2; //夹取色块后自转完成
   last_target_angle=target_angle;
   cmd_vel_publish();//将底盘速度发布出去
   }

  if(car_state==2)//放置完色块后
  {     //给上0.1的允许误差范围
        if(target_position_z<=(car_position_z-0.1))  target_angular_z= -0.6,move_flag=3; //以0.6rad/s的速度旋转
   else if(target_position_z>=(car_position_z+0.1))  target_angular_z=  0.6,move_flag=3;
   else                     target_angular_z= 0,move_flag=4; //放置色块后自转完成
   last_target_angle=target_angle;  
   cmd_vel_publish();//将底盘速度发布出去
   }
   last_target_angle=target_angle;
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

//机械臂状态的话题发布函数
void car_init::arm_state_publish()
{
  static uint8_t last_move_flag=0,last_location_flag=0;
  std_msgs::String msg;
  msg.data="last_target_angle";

  //当机械臂转动夹爪动作标志位为1时发送"shake_hand"命令，执行转动夹爪动作
  if (shake_hand)                                                    msg.data="shake_hand";
  //机械臂云台旋转并放置物块，之后回到原位
  else if ((car_state==1)&&(rotate_mode=="holder"))                  msg.data="rotate_put";
  //如果底盘对色块定位完成，那么发布机械臂夹取的命令
  else if((location_flag==1)&&(move_flag<2))                         msg.data="pick";
  //底盘自转前，如果是夹取了东西的，那么就执行机械臂放置命令
  else if((move_flag==2||move_flag==3)&&(rotate_mode=="chassis"))    msg.data="put";
  //其他情况则发送"no_msg" 
  else                                                               msg.data="no_msg";

  //将机械臂的命令发布出去
  arm_state_pub.publish(msg);

}
//底盘在x轴方向（前后）定位色块的PID计算
float car_init::PID_control_x()
{
    static float last_error=0,output=0;
    pid_x.error=distance_x-(color_location_x);
    output=x_d*last_error+x_p*pid_x.error;
    last_error=pid_x.error;
    return output;
    
}

//底盘在y轴方向（左右）定位色块的PID计算
float car_init::PID_control_y()
{
    static float last_error=0,output=0;
    pid_y.error=distance_y-(color_location_y);
    output=y_d*last_error+y_p*pid_y.error;
    last_error=pid_y.error;
    return output;

}

//底盘在z轴方向（旋转）定位色块的PID计算
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
  
  while(ros::ok())
  {
    //当小车执行完一套夹取放置色块动作后将标志位清零
    if(move_flag==4||car_state==3) {
      car_state=0,location_flag=0,move_flag=0;
      for (int i=0;i<shake_hand_done.size();i++){
        shake_hand_done[i]=0;
      }
    }

    //当小车执行完一次转动夹爪动作后将标志位清零
    if(car_state==4){
      car_state=0,location_flag=0,move_flag=0,shake_hand=0;
      sleep(1);}


    //发布机械臂状态的命令
    arm_state_publish();
    //error标志位为0，则将error计置零
    if(error_flag==0)      error_count=0;
    //当error标志位为1且处于定位色块阶段，则开始error计数
    else if(error_flag==1&&location_flag==0) error_count++;
    error_flag=1; 
    //当error计数达到10000时，将底盘速度置零防止小车乱跑
    if(error_count>10000) 
    {
     target_liner_x=0;
     target_liner_y=0;
     target_angular_z=0;
     cmd_vel_publish();
     error_count=0;
    }
    ros::spinOnce();
  }
}

//构造函数
car_init::car_init()
{
    //参数服务器
    ros::NodeHandle param_nh("~"); //私有命名空间的节点句柄，用于参数服务器
    param_nh.param("x_p",x_p,0.0); //x轴方向的pid参数
    param_nh.param("x_d",x_d,0.0);
    param_nh.param("y_p",y_p,0.0); //y轴方向的pid参数
    param_nh.param("y_d",y_d,0.0);
    param_nh.param("z_p",z_p,0.0); //z轴方向的pid参数
    param_nh.param("z_d",z_d,0.0);
    param_nh.param("color_location_x",color_location_x,0.0); //色块的x轴方位的目标位置
    param_nh.param("color_location_y",color_location_y,0.0); //色块的y轴方位的目标位置
    param_nh.param<std::string>("car_mode",car_mode,"mini_mec_six_arm");//获取小车的类型
    param_nh.param<std::string>("rotate_mode",rotate_mode,"chassis");//获取夹取色块后旋转方式
    
    arm_state_pub=n.advertise<std_msgs::String>("arm_state",10);
    cmd_vel_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",10);
    car_command_sub=n.subscribe("car_command",10,&car_init::car_command_callback,this);
    color_location_sub=n.subscribe("object_tracker/current_position",10,&car_init::color_location_callback,this);
    if (rotate_mode=="chassis") {car_pose_sub=n.subscribe("odom",10,&car_init::car_pose_callback,this);}

    target_angle=0,car_state=-1,location_flag=0,move_flag=0,shake_hand=0; //标志位初始化
    for(int i=0;i<3;i++){
      shake_hand_done.push_back(0);
    }

    ROS_INFO_STREAM("car_location_color_node_init_successful");
}

//析构函数
car_init::~car_init()
{
    ROS_INFO_STREAM("car_location_color_node_close");
}

//主函数
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "car_location_color"); //节点初始化
    car_init car_control;  //实例化一个对象
    car_control.control(); //循环执行内容
    return 0;
}




