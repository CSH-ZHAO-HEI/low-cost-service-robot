#!/usr/bin/env bash
# smoke_test.sh — 快速自检：关键 ROS topic / service 是否就绪。
# 在 Gazebo + RTAB-Map + move_base + arm_task_server 都起来后运行。
echo "=== ROS topics ==="
need_topics=(
  /camera/rgb/image_raw /camera/depth/image_raw /camera/depth/points
  /ground_truth/odom /cmd_vel /joint_states
  /rtabmap/grid_map /judge_camera/rgb/image_raw
  /arm_controller/command /hand_controller/command
  /block_follower/attach
)
topics=$(rostopic list 2>/dev/null)
for t in "${need_topics[@]}"; do
  if echo "$topics" | grep -qx "$t"; then echo "  [ok]   $t"; else echo "  [MISS] $t"; fi
done

echo "=== ROS services（机械臂）==="
svcs=$(rosservice list 2>/dev/null)
for s in /arm/pick /arm/put /arm/prepare_put /arm/drop /arm/home; do
  if echo "$svcs" | grep -qx "$s"; then echo "  [ok]   $s"; else echo "  [MISS] $s"; fi
done

echo "=== move_base action ==="
echo "$topics" | grep -q "/move_base/goal" && echo "  [ok]   /move_base 已起" || echo "  [MISS] /move_base"

echo "=== L2 资料库 ==="
SCENE="${GAZEBO_SCENE_PATH:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/gazebo_scene.yaml}"
[ -f "$SCENE" ] && echo "  [ok]   $SCENE" || echo "  [MISS] $SCENE（先跑 python3 scripts/get_scene.py）"
