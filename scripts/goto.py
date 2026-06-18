#!/usr/bin/env python3
"""
goto.py — 互動式導航測試工具
列出 gazebo_scene.yaml 中的物件，輸入名稱即發送 move_base 目標
Ctrl+C 或輸入 q 退出
"""

import sys
import os
import math
import yaml
import signal
import threading

import rospy
import actionlib
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

SCENE_PATH = os.environ.get("SCENE_PATH", os.path.join(PROJECT_ROOT, "gazebo_scene.yaml"))


# ── 工具函式 ──────────────────────────────────────────────────
def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


def load_scene(path):
    if not os.path.exists(path):
        print(f"[錯誤] 找不到場景檔：{path}")
        print("       請先執行：python3 get_scene.py")
        sys.exit(1)
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    if not data:
        print("[錯誤] gazebo_scene.yaml 是空的")
        sys.exit(1)
    return data


def print_objects(scene):
    print("\n" + "=" * 65)
    print("  可前往的物件：")
    print("=" * 65)
    for i, (name, info) in enumerate(sorted(scene.items()), 1):
        ox   = info.get("object_x",     0)
        oy   = info.get("object_y",     0)
        ax   = info.get("approach_x",   0)
        ay   = info.get("approach_y",   0)
        ayaw = info.get("approach_yaw", 0)
        print(f"  {i:2d}. {name:<32s}  物件({ox:.2f},{oy:.2f})"
              f"  接近({ax:.2f},{ay:.2f}, {math.degrees(ayaw):.0f}°)")
    print("=" * 65)
    print("  輸入物件名稱或編號前往，輸入 q 退出，輸入 list 重新列出")
    print("=" * 65 + "\n")


def send_goal(client, x, y, yaw):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp    = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation = yaw_to_quaternion(yaw)

    client.cancel_all_goals()
    rospy.sleep(0.1)
    client.send_goal(goal)
    print(f"  → 目標已發送 ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}°)")
    print(f"  → 等待到達... （Ctrl+C 可取消）")

    finished = client.wait_for_result(rospy.Duration(120.0))

    if not finished:
        client.cancel_goal()
        print("  ✗ 超時（120s），已取消目標")
        return False

    state = client.get_state()
    if state == GoalStatus.SUCCEEDED:
        print("  ✓ 到達目標！")
        return True
    elif state == GoalStatus.PREEMPTED:
        print("  ✗ 目標被取消")
        return False
    else:
        print(f"  ✗ 導航失敗（狀態碼 {state}）")
        return False


# ── 主程式 ────────────────────────────────────────────────────
def main():
    rospy.init_node("goto_interactive", anonymous=True, disable_signals=True)

    # 載入場景
    scene = load_scene(SCENE_PATH)
    names_sorted = sorted(scene.keys())

    # 連接 move_base
    print("[goto] 連接 move_base action server...")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    print("[goto] 等待 move_base 初始化（最多 60 秒）...")
    connected = client.wait_for_server(rospy.Duration(60.0))
    if not connected:
        print("[錯誤] 無法連接 move_base（60s 超時），請確認 run_teb.sh 已啟動且地圖已載入")
        sys.exit(1)
    print("[goto] 連接成功！\n")

    print_objects(scene)

    # 主循環
    while not rospy.is_shutdown():
        try:
            raw = input("前往 > ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n[goto] 已退出")
            break

        if not raw:
            continue

        if raw.lower() in ("q", "quit", "exit"):
            print("[goto] 已退出")
            break

        if raw.lower() in ("list", "ls", "l"):
            print_objects(scene)
            continue

        # 支援輸入編號
        target = raw
        if raw.isdigit():
            idx = int(raw) - 1
            if 0 <= idx < len(names_sorted):
                target = names_sorted[idx]
            else:
                print(f"  [錯誤] 編號 {raw} 超出範圍（1~{len(names_sorted)}）")
                continue

        if target not in scene:
            # 模糊比對（不分大小寫）
            matches = [n for n in scene if n.lower() == target.lower()]
            if matches:
                target = matches[0]
            else:
                print(f"  [錯誤] 找不到物件「{target}」，輸入 list 查看可用物件")
                continue

        info = scene[target]
        ax   = info.get("approach_x",   info.get("object_x", 0))
        ay   = info.get("approach_y",   info.get("object_y", 0))
        ayaw = info.get("approach_yaw", 0.0)

        print(f"\n  目標：{target}")
        send_goal(client, ax, ay, ayaw)
        print()

    client.cancel_all_goals()


if __name__ == "__main__":
    main()
