#!/bin/bash
# 診斷 RViz 無法顯示機器人模型的問題

source ~/catkin_ws/devel/setup.bash

echo "======================================"
echo " RViz 診斷工具"
echo "======================================"

# 1. 確認 ROS Master 在線
echo ""
echo "[1] ROS Master 狀態："
if rostopic list >/dev/null 2>&1; then
    echo "    ✓ ROS Master 在線"
else
    echo "    ✗ ROS Master 不在線！請先執行 run_gazebo.sh"
    exit 1
fi

# 2. 確認 robot_description 存在
echo ""
echo "[2] robot_description 參數："
if rosparam get /robot_description >/dev/null 2>&1; then
    URDF_LEN=$(rosparam get /robot_description | wc -c)
    echo "    ✓ 存在 (長度: ${URDF_LEN} 字元)"
else
    echo "    ✗ 不存在！Gazebo 可能還沒啟動完成"
fi

# 3. 確認 robot_state_publisher
echo ""
echo "[3] robot_state_publisher 節點："
if rosnode list 2>/dev/null | grep -q robot_state_publisher; then
    echo "    ✓ 正在執行"
else
    echo "    ✗ 不在執行！"
fi

# 4. 確認 TF frames
echo ""
echo "[4] TF frames："
TF_FRAMES=$(rostopic echo -n1 /tf_static 2>/dev/null | grep "frame_id" | head -5)
echo "    base_link 是否存在:"
rosrun tf tf_echo base_link link1 2>&1 | head -3

# 5. 確認 joint_states
echo ""
echo "[5] /joint_states 話題："
if rostopic hz /joint_states 2>/dev/null --window=5 -p 2>/dev/null & HZ_PID=$!
   sleep 2
   kill $HZ_PID 2>/dev/null; then
    echo "    ✓ 正在發布"
fi

# 5b. 取得 joint names
echo ""
echo "[5b] joint names:"
rostopic echo -n1 /joint_states 2>/dev/null | grep "name:" | head -5

# 6. 確認 rospack 能找到 mini_mec_six_arm
echo ""
echo "[6] mini_mec_six_arm 套件："
PKG_PATH=$(rospack find mini_mec_six_arm 2>/dev/null)
if [ -n "$PKG_PATH" ]; then
    echo "    ✓ 找到: $PKG_PATH"
    echo "    meshes 目錄: $(ls $PKG_PATH/meshes/ | wc -l) 個檔案"
else
    echo "    ✗ 找不到套件！"
fi

# 7. 確認 TF tree
echo ""
echo "[7] TF tree (前 20 行)："
rosrun tf view_frames 2>/dev/null &
VIEW_PID=$!
sleep 3
kill $VIEW_PID 2>/dev/null
rostopic echo -n1 /tf 2>/dev/null | grep "frame_id\|child_frame_id" | head -20

echo ""
echo "======================================"
echo " 診斷完成"
echo "======================================"
echo ""
echo "如果全部顯示 ✓，請嘗試以下 RViz 操作："
echo "  1. 在 RViz 中點 RobotModel，查看 Status: Error 的詳細訊息"
echo "  2. 或執行: roslaunch small_brain_sim simple_rviz.launch"
