#ifndef __WHEELTEC_ARM_H_
#define __WHEELTEC_ARM_H_

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


class arm_init
{
  public:

  //void control();


  private:
 // void arm_pick();
  //void arm_put();
  //string arm_state;
    moveit::planning_interface::MoveGroupInterface arm,hand;
  
};


  void arm_pick();
  void arm_put();
  void arm_rotate_put();
  void arm_shake_hand();

#endif
