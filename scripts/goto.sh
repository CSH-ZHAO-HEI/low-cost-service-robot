#!/bin/bash
# ============================================
# goto.sh — 互動式導航測試
# 列出 gazebo_scene.yaml 中的物件，輸入名稱小車自動前往
# 用法：./goto.sh
# 需先執行：run_gazebo.sh → run_rtab.sh map → run_teb.sh → get_scene.py
# ============================================

source ~/catkin_ws/devel/setup.bash

# 檢查場景檔是否存在
if [ ! -f ~/catkin_ws/Thesis_Project/gazebo_scene.yaml ]; then
    echo "======================================"
    echo " 錯誤：找不到 gazebo_scene.yaml"
    echo " 請先執行：python3 ~/catkin_ws/Thesis_Project/get_scene.py"
    echo "======================================"
    exit 1
fi

# 等待 move_base 節點上線
echo "======================================"
echo " 等待 move_base 就緒..."
echo "======================================"

WAIT=0
MAX=60
while true; do
    NODE_OK=$(rosnode list 2>/dev/null | grep -c "^/move_base$")
    if [ "$NODE_OK" -ge 1 ]; then
        echo " ✓ move_base 已上線"
        break
    fi
    if [ "$WAIT" -ge "$MAX" ]; then
        echo " ✗ 超時（${MAX}s），move_base 未啟動"
        echo "   請確認 run_teb.sh 已在另一個終端運行"
        exit 1
    fi
    sleep 2
    WAIT=$((WAIT + 2))
done

# 再等 2 秒讓 action server 完全初始化
sleep 2

echo "======================================"
echo " goto.sh — 互動式導航測試"
echo " 場景檔：~/catkin_ws/Thesis_Project/gazebo_scene.yaml"
echo " Ctrl+C 或輸入 q 退出"
echo "======================================"

python3 ~/catkin_ws/Thesis_Project/goto.py
