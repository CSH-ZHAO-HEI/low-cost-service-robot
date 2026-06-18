#!/usr/bin/env bash
# setup_workspace.sh — 把本代码包的 ROS 包链接进一个 catkin 工作区并编译。
# 用法：
#   ./setup_workspace.sh [catkin_ws_目录]    # 默认 ~/catkin_ws
#
# 完成后：
#   source <catkin_ws>/devel/setup.bash
#   export GAZEBO_MODEL_PATH=<code>/scene:<code>/small_brain/models:$GAZEBO_MODEL_PATH
set -e

CODE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${1:-$HOME/catkin_ws}"
SRC="$WS/src"

echo "[setup] 代码包: $CODE_DIR"
echo "[setup] 工作区: $WS"
mkdir -p "$SRC"

link() {  # link <目标> <链接名>
  local target="$1" name="$2"
  if [ -e "$target" ]; then
    ln -sfn "$target" "$SRC/$name"
    echo "  linked  $name -> $target"
  else
    echo "  [skip]  $target 不存在"
  fi
}

# 小脑（包名 small_brain_sim）+ 机械臂 + 场景
link "$CODE_DIR/small_brain"                              small_brain_sim
link "$CODE_DIR/arm/mini_mec_six_arm"                     mini_mec_six_arm
link "$CODE_DIR/arm/mini_mec_six_arm_moveit_config"       mini_mec_six_arm_moveit_config
link "$CODE_DIR/arm/wheeltec_arm_pick"                    wheeltec_arm_pick
link "$CODE_DIR/arm/wheeltec_arm_rc"                      wheeltec_arm_rc
link "$CODE_DIR/arm/wheeltec_tracker_pkg"                 wheeltec_tracker_pkg
link "$CODE_DIR/scene/aws-robomaker-small-house-world"    aws-robomaker-small-house-world

echo "[setup] catkin_make ..."
( cd "$WS" && catkin_make )

cat <<EOF

[setup] 完成。接下来在每个终端执行：
  source $WS/devel/setup.bash
  export GAZEBO_MODEL_PATH=$CODE_DIR/scene:$CODE_DIR/small_brain/models:\$GAZEBO_MODEL_PATH
  export PROJECT_ROOT=$CODE_DIR          # 大脑读 gazebo_scene.yaml 用（默认已是此值）

启动见 项目文档/00_总览.md「启动流程」。
EOF
