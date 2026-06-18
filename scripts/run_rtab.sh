#!/bin/bash
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# ============================================
# 啟動 RTAB-Map
# 用法：
#   ./run_rtab.sh        → 定位模式（無 RViz）
#   ./run_rtab.sh map    → 建圖模式（開 RViz）
# 請先執行 run_gazebo.sh
# ============================================

source ~/catkin_ws/devel/setup.bash

# ── 清理殘余流程（保留 Gazebo）─────────────────────────────────
echo " 清理舊的 ROS 節點..."
rosnode kill /rtabmap/rtabmap       2>/dev/null || true
rosnode kill /rtabmap/rtabmapviz    2>/dev/null || true
sleep 1
pkill -x "rtabmap"                  2>/dev/null
pkill -x "rtabmap_viz"              2>/dev/null
pkill -x "move_base"                2>/dev/null
pkill -x "depthimage_to_laserscan"  2>/dev/null
pkill -f "map_rviz.sh"              2>/dev/null
pkill -f "rviz"                     2>/dev/null
pkill -x "send_mark.py"             2>/dev/null
sleep 2

# ── 等待相機就緒 ───────────────────────────────────────────────
echo " 等待相機數據..."
WAIT=0
while true; do
    PUB=$(rostopic info /camera/depth/image_raw 2>/dev/null | grep -c "Publishers:")
    if [ "$PUB" -ge 1 ]; then
        echo " ✓ 相機就緒"
        break
    fi
    [ "$WAIT" -ge 30 ] && echo " ✗ 相機超時，請確認 run_gazebo.sh 已啟動" && exit 1
    sleep 2; WAIT=$((WAIT+2))
done

if [ "$1" == "map" ]; then
    echo "======================================"
    echo " RTAB-Map 建圖模式"
    echo " 地圖存於：~/.ros/rtabmap.db"
    echo " 關閉方式：Ctrl+C"
    echo "======================================"
    echo " 建圖性能模式：關閉 pip_cam，使用輕量 RViz"
    pkill -f "pip_cam.py /camera/rgb/image_raw" 2>/dev/null || true
    rm -f ~/.ros/rtabmap.db ~/.ros/rtabmap.db-journal
    sleep 3 && "$PROJECT_ROOT/scripts/map_rviz.sh" light &
    roslaunch small_brain_sim 3d_mapping.launch rtabmapviz:=false
else
    echo "======================================"
    echo " RTAB-Map 定位模式"
    echo " 接著請執行：./run_teb.sh"
    echo " 關閉方式：Ctrl+C"
    echo "======================================"
    if [ ! -f ~/.ros/rtabmap.db ]; then
        echo " 錯誤：找不到地圖，請先執行 ./run_rtab.sh map"
        exit 1
    fi
    roslaunch small_brain_sim 3d_navigation.launch localization:=true rtabmapviz:=false
fi
