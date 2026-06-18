#!/bin/bash
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# ============================================
# 啟動小屋 Gazebo 仿真
# 麥輪六自由度機器人 + AWS 小屋場景
# track_visual 會在機器人 spawn 後自動鎖定視角
# ============================================

source ~/catkin_ws/devel/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export MESA_LOADER_DRIVER_OVERRIDE=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export DISPLAY=:0
export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:1"   # 立刻 ECONNREFUSED，不卡 GUI
# model://coke/... 需要 Thesis_Project 在 model path 裡
export GAZEBO_MODEL_PATH="$PROJECT_ROOT/scene:$PROJECT_ROOT/small_brain/models:$GAZEBO_MODEL_PATH"

MODE="${1:-normal}"
if [ "$MODE" = "map" ] || [ "$MODE" = "perf" ] || [ "$MODE" = "headless" ]; then
    : "${GAZEBO_GUI:=false}"
    : "${START_PIP_CAM:=false}"
else
    : "${GAZEBO_GUI:=true}"
    : "${START_PIP_CAM:=true}"
fi

echo "======================================"
echo " 清理舊進程..."
echo "======================================"
killall gzserver gzclient gazebo roslaunch rosmaster 2>/dev/null
pkill -9 -f "gz camera" 2>/dev/null
sleep 2
echo " 清理完成"

echo "======================================"
echo " 啟動 Gazebo + 機器人..."
echo "======================================"
echo " 模式: $MODE"
echo " Gazebo GUI: $GAZEBO_GUI"
echo " Pip camera: $START_PIP_CAM"

roslaunch small_brain_sim mec_six_arm_house.launch gui:=$GAZEBO_GUI &
LAUNCH_PID=$!

echo "======================================"
echo " Gazebo 已啟動"
echo " 鍵盤控制請另開終端執行: ./start_control.sh"
echo " 抓取測試請另開終端執行: /usr/bin/python3 pick_and_place.py"
echo "======================================"

# 等控制器就緒，初始化機械臂姿態
sleep 8
/usr/bin/python3 "$PROJECT_ROOT/scripts/arm_clamp_init.py" &

# spawn 可樂罐到 BalconyTable_01_001 靠機器人那側邊邊
rosrun gazebo_ros spawn_model \
  -sdf \
  -file "$PROJECT_ROOT/scene/coke/model.sdf" \
  -model Coke \
  -x -0.556 -y 3.84 -z 0.278 &

# 刪掉 DeskPortraitC_01（畫框，不需要）
sleep 12
rosservice call /gazebo/delete_model "model_name: 'DeskPortraitC_01'" &

if [ "$START_PIP_CAM" = "true" ]; then
    # 啟動前置鏡頭小視窗
    /usr/bin/python3 "$PROJECT_ROOT/scripts/pip_cam.py" /camera/rgb/image_raw &
else
    echo " 建圖性能模式：跳過 pip_cam.py，減少相機 topic 顯示負載"
fi

wait $LAUNCH_PID
