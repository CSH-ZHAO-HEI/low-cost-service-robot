#!/bin/bash
# ============================================
# 啟動 MoveIt + RViz
# 請先執行 run_gazebo.sh，再執行此腳本
# ============================================

source ~/catkin_ws/devel/setup.bash

echo "======================================"
echo " 啟動 MoveIt + RViz..."
echo " 關閉方式: Ctrl+C"
echo "======================================"

roslaunch small_brain_sim moveit_rviz_only.launch
