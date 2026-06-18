#!/bin/bash
# 在 RViz 里干净地显示移臂小车模型（白底，不需 Gazebo），用于截图补到 PPT P4。
# 用法：bash show_robot.sh
#   - 弹出的滑块窗口可调机械臂姿态（默认已摆成待机姿态）
#   - RViz 里按住鼠标拖动旋转视角、滚轮缩放
#   - 截图：Windows 用 Win+Shift+S 框选 RViz 窗口即可

source /opt/ros/noetic/setup.bash
source "${CATKIN_WS:-$HOME/catkin_ws}/devel/setup.bash" 2>/dev/null

# WSL2 Mesa 渲染（和 run_gazebo.sh 一致），否则 STL 网格渲染不出来
export LD_LIBRARY_PATH=/usr/local/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export MESA_LOADER_DRIVER_OVERRIDE=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

roslaunch small_brain_sim robot_display.launch
