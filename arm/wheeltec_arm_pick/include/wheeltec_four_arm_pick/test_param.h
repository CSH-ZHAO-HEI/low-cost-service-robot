#ifndef __CAR_LOCATION_COLOR_H_
#define __CAR_LOCATION_COLOR_H_

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
#include <nav_msgs/Odometry.h>
#include <wheeltec_arm_pick/pick_and_put.h>
#include <wheeltec_arm_pick/four_arm_position.h>
#include <dynamic_reconfigure/server.h>
#include <wheeltec_arm_pick/paramsConfig.h>
#include <moveit/move_group_interface/move_group_interface.h>

typedef struct _PID_NUMBER_
{
  float error;
  float integral;
  float diff;
}PID_NUMBER;


class car_init
{
  public:
  car_init();
 ~car_init();
  void control();

  private:
  	ros::NodeHandle n;
  	ros::Publisher  arm_state_pub,cmd_vel_pub;
  	ros::Subscriber car_command_sub,color_location_sub,car_pose_sub,arm_state_sub;
  	ros::Time _Now_Time,_last_Time;
  	float sampling_time; //采样时间
    float distance_y;
    float distance_x;
    float angular_z;
    uint8_t   move_flag,location_flag;
    float car_position_x,car_position_y,car_position_z;
    void car_pose_callback(const nav_msgs::Odometry &msg);
    void color_location_callback(const wheeltec_arm_pick::four_arm_position &msg);
    void car_command_callback(const wheeltec_arm_pick::pick_and_put &msg);
    void reconfigCB(wheeltec_arm_pick::paramsConfig &config, uint32_t unused_level);
    void arm_state_callback(const std_msgs::String &state);
    float PID_control_x();
    float PID_control_y();
    float PID_control_z();
    void cmd_vel_publish();
    void arm_state_publish();
    float abs_float(float input);
    double x_p,x_i,x_d;
    double y_p,y_i,y_d;
    double z_p,z_i,z_d;
    double color_location_x,color_location_y,arm_upper_and_lower,hand_open_and_close,car_start;
    PID_NUMBER pid_x,pid_y,pid_z;
    float target_liner_x,target_liner_y,target_angular_z;
    float target_angle;
    uint8_t car_state;
    std::string car_mode;
};


#endif

