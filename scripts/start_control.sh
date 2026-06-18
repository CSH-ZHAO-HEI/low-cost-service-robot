#!/bin/bash
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# ============================================
# 啟動麥輪 + 機械臂鍵盤控制
# 請先執行 run_gazebo.sh 再開此腳本
# ============================================

source ~/catkin_ws/devel/setup.bash

echo "======================================"
echo " 鍵盤操作說明"
echo "======================================"
echo ""
echo "  底盤移動:"
echo "    W         = 前進"
echo "    S         = 後退"
echo "    A         = 左轉"
echo "    D         = 右轉"
echo "    Q         = 左前斜移（麥輪）"
echo "    R         = 右前斜移（麥輪）"
echo "    空白鍵     = 剎車（立即停止）"
echo "    Z / X     = 加速 / 減速"
echo ""
echo "  機械臂（發布到 /arm_controller/command）:"
echo "    1 / 2     = joint1 底座旋轉 +/-"
echo "    C / V     = joint2 大臂 +/-"
echo "    B / N     = joint3 小臂 +/-"
echo "    F / G     = joint4 腕部俯仰 +/-"
echo "    T / Y     = joint5 腕部旋轉 +/-"
echo "    O         = 夾爪張開"
echo "    P         = 夾爪夾緊"
echo "    M         = 手臂回到 arm_clamp 姿態"
echo "    ; / '     = 手臂步進精度 -/+"
echo ""
echo "  Ctrl+C 關閉"
echo "======================================"
echo ""

python3 "$PROJECT_ROOT/small_brain/scripts/teleop_key.py"
