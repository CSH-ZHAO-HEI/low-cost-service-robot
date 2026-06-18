#!/bin/bash
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# 顯示機器人後方追蹤視角（小浮動視窗，可拖到角落）
source ~/catkin_ws/devel/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export MESA_LOADER_DRIVER_OVERRIDE=d3d12
export DISPLAY=:0

python3 "$PROJECT_ROOT/scripts/pip_cam.py" /camera/rgb/image_raw
