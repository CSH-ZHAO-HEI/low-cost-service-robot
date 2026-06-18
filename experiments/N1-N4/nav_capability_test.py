#!/usr/bin/env python3
"""
Manual navigation capability test for N1-N4.

This file tests the small-brain navigation stack only:
  - no LLM
  - no VLM
  - no JudgeLLM
  - no arm service

Prerequisites:
  1. ./run_gazebo.sh
  2. ./run_rtab.sh
  3. ./run_teb.sh or ./run_teb_compare.sh
  4. python3 get_scene.py

Usage:
  python3 N1-N4/nav_capability_test.py N1
  python3 N1-N4/nav_capability_test.py N2
  python3 N1-N4/nav_capability_test.py N3
  python3 N1-N4/nav_capability_test.py N4
  python3 N1-N4/nav_capability_test.py all
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from typing import Dict, Iterable, List, Tuple

import yaml


PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
BIG_BRAIN_DIR = os.path.join(PROJECT_ROOT, "big_brain")
SCENE_PATH = os.path.join(PROJECT_ROOT, "gazebo_scene.yaml")

if BIG_BRAIN_DIR not in sys.path:
    sys.path.insert(0, BIG_BRAIN_DIR)

import ros_bridge  # noqa: E402


Point = Tuple[float, float]


def load_scene() -> Dict[str, dict]:
    with open(SCENE_PATH, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def names_containing(scene: Dict[str, dict], *keywords: str) -> List[str]:
    result = []
    for name in scene:
        lower = name.lower()
        if any(key.lower() in lower for key in keywords):
            result.append(name)
    return sorted(result)


def obj_xy(scene: Dict[str, dict], name: str) -> Point:
    info = scene[name]
    return float(info["object_x"]), float(info["object_y"])


def approach_pose(scene: Dict[str, dict], name: str) -> Tuple[float, float, float]:
    info = scene[name]
    return (
        float(info["approach_x"]),
        float(info["approach_y"]),
        float(info.get("approach_yaw", 0.0)),
    )


def choose_min_abs_sum(scene: Dict[str, dict], names: Iterable[str]) -> str:
    candidates = list(names)
    if not candidates:
        raise RuntimeError("No candidate objects found.")
    return min(candidates, key=lambda n: abs(obj_xy(scene, n)[0]) + abs(obj_xy(scene, n)[1]))


def dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def goto(x: float, y: float, yaw: float | None = None, label: str = "") -> Point:
    if label:
        print(f"[goto] {label}: ({x:.3f}, {y:.3f})")
    else:
        print(f"[goto] ({x:.3f}, {y:.3f})")
    ros_bridge.move_to_goal(x, y, yaw)
    pos = ros_bridge.get_current_pos()
    print(f"[odom] ({pos[0]:.3f}, {pos[1]:.3f}), error={dist(pos, (x, y)):.3f}m")
    return pos


def follow_waypoints(points: List[Tuple[float, float, float | None, str]]) -> bool:
    for x, y, yaw, label in points:
        goto(x, y, yaw, label)
    return True


def task_n1(_: Dict[str, dict]) -> bool:
    """N1: navigate to a fixed reachable coordinate.

    Original task says (100, 200). In this codebase all distances are meters,
    so this capability test uses (1.0, 2.0), equivalent to 100cm, 200cm.
    """
    goto(1.0, 2.0, None, "N1 target (1.0, 2.0)")
    return True


def task_n2(scene: Dict[str, dict]) -> bool:
    """N2: navigate next to a trash can."""
    trash_names = names_containing(scene, "trash", "bin")
    target = choose_min_abs_sum(scene, trash_names)
    ax, ay, ayaw = approach_pose(scene, target)
    print(f"[N2] selected trash: {target}")
    goto(ax, ay, ayaw, f"N2 approach {target}")
    return True


def task_n3(scene: Dict[str, dict]) -> bool:
    """N3: move around the sofa in a 4x4 meter square."""
    sofa_names = names_containing(scene, "sofa")
    target = choose_min_abs_sum(scene, sofa_names)
    sx, sy = obj_xy(scene, target)
    half = 2.0
    print(f"[N3] selected sofa: {target}, center=({sx:.3f}, {sy:.3f})")
    points = [
        (sx + half, sy + half, None, "N3 square corner 1"),
        (sx + half, sy - half, None, "N3 square corner 2"),
        (sx - half, sy - half, None, "N3 square corner 3"),
        (sx - half, sy + half, None, "N3 square corner 4"),
        (sx + half, sy + half, None, "N3 close square"),
    ]
    return follow_waypoints(points)


def task_n4(scene: Dict[str, dict]) -> bool:
    """N4: go to the trash can with min abs(x)+abs(y), then return."""
    start = ros_bridge.get_current_pos()
    trash_names = names_containing(scene, "trash", "bin")
    target = choose_min_abs_sum(scene, trash_names)
    tx, ty = obj_xy(scene, target)
    ax, ay, ayaw = approach_pose(scene, target)

    print(f"[N4] start=({start[0]:.3f}, {start[1]:.3f})")
    print(f"[N4] selected trash: {target}, object=({tx:.3f}, {ty:.3f})")
    goto(ax, ay, ayaw, f"N4 approach {target}")
    goto(start[0], start[1], None, "N4 return to start")
    return True


TASKS = {
    "N1": task_n1,
    "N2": task_n2,
    "N3": task_n3,
    "N4": task_n4,
}


def run_task(task_id: str, scene: Dict[str, dict]) -> bool:
    print(f"\n========== {task_id} ==========")
    t0 = time.time()
    try:
        ok = TASKS[task_id](scene)
        elapsed = time.time() - t0
        print(f"[{task_id}] PASS, time={elapsed:.1f}s")
        return ok
    except Exception as e:
        elapsed = time.time() - t0
        print(f"[{task_id}] FAIL, time={elapsed:.1f}s, reason={e}")
        return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("task", choices=["N1", "N2", "N3", "N4", "all"])
    args = parser.parse_args()

    scene = load_scene()
    ros_bridge.init()

    task_ids = list(TASKS) if args.task == "all" else [args.task]
    results = {task_id: run_task(task_id, scene) for task_id in task_ids}

    print("\n========== summary ==========")
    for task_id, ok in results.items():
        print(f"{task_id}: {'PASS' if ok else 'FAIL'}")
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
