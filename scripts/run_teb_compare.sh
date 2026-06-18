#!/bin/bash
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# ============================================
# TEB 導航對照版（不改 EGO-Planner 流程）
#
# 需先執行：
#   1. ./run_gazebo.sh
#   2. ./run_rtab.sh map   # 第一次建圖
#      ./run_rtab.sh       # 之後定位
#
# 用法：
#   ./run_teb_compare.sh
# ============================================

source ~/catkin_ws/devel/setup.bash

pkill -f "ego_planner_node"         2>/dev/null
pkill -x "move_base"                2>/dev/null
pkill -x "depthimage_to_laserscan"  2>/dev/null
pkill -f "rviz"                     2>/dev/null
pkill -x "send_mark.py"             2>/dev/null
sleep 1

echo "======================================"
echo " 等待 RTAB-Map 發布 /rtabmap/grid_map..."
echo " 請確認 run_rtab.sh 已在另一個終端運行"
echo "======================================"

WAIT=0
MAX=180
while true; do
    PUB=$(rostopic info /rtabmap/grid_map 2>/dev/null | grep -c "Publishers:")
    if [ "$PUB" -ge 1 ]; then
        echo " ✓ /rtabmap/grid_map 就緒（等了 ${WAIT}s）"
        break
    fi
    if [ "$WAIT" -ge "$MAX" ]; then
        echo " ✗ 超時（${MAX}s），請確認 run_rtab.sh 正常運行"
        exit 1
    fi
    sleep 3
    WAIT=$((WAIT+3))
    [ $((WAIT % 15)) -eq 0 ] && echo "   已等 ${WAIT}s..."
done

echo "======================================"
echo " 啟動 move_base + TEB 對照版"
echo " 關閉方式：Ctrl+C"
echo "======================================"

sleep 2 && "$PROJECT_ROOT/scripts/map_rviz.sh" full &
roslaunch small_brain_sim move_base_teb.launch
