#!/bin/bash
source ~/catkin_ws/devel/setup.bash
MODE="${1:-light}"

if [ "$MODE" = "full" ]; then
    roslaunch small_brain_sim map_rviz.launch
else
    roslaunch small_brain_sim map_rviz_light.launch
fi
