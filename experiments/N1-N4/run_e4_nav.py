#!/usr/bin/env python3
"""
Collect E4 manual-baseline data for navigation tasks N1-N4.

E4 means:
  - manual code / no LLM
  - Gazebo execution
  - no VLM
  - navigation-only metrics

Before every repeat, the robot is returned to a fixed home pose (0, 0).
That reset time is NOT counted in total_time_s.

Prerequisites:
  1. ./run_gazebo.sh
  2. ./run_rtab.sh
  3. ./run_teb.sh or ./run_teb_compare.sh
  4. python3 get_scene.py

Usage:
  python3 N1-N4/run_e4_nav.py --all
  python3 N1-N4/run_e4_nav.py --tasks N1 N2 N3 N4 --repeats 3
  python3 N1-N4/run_e4_nav.py --tasks N4 --repeats 1

Output:
  N1-N4/e4_nav_results.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import yaml
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
BIG_BRAIN_DIR = os.path.join(PROJECT_ROOT, "big_brain")
SCENE_PATH = os.path.join(PROJECT_ROOT, "gazebo_scene.yaml")
DEFAULT_OUTPUT = os.path.join(PROJECT_ROOT, "N1-N4", "e4_nav_results.csv")
ROBOT_MODEL_NAME = "mini_mec_six_arm"
HOME_Z = 0.05

if BIG_BRAIN_DIR not in sys.path:
    sys.path.insert(0, BIG_BRAIN_DIR)

import ros_bridge  # noqa: E402


Point = Tuple[float, float]

TASK_INSTRUCTIONS = {
    "N1": "Navigate to (1.0, 2.0)",
    "N2": "Navigate next to the trash can",
    "N3": "Move around the sofa in a 4x4 meter square",
    "N4": "Go to the trash can with the smallest |x|+|y|, then return to the starting point",
}

FIELDNAMES = [
    "run_id",
    "task_id",
    "task_type",
    "instruction",
    "method_type",
    "model_name",
    "temperature",
    "adjust_policy",
    "repeat_id",
    "generated_code",
    "semantic_parse_correct",
    "decomposition_correct",
    "code_executable",
    "api_call_count",
    "adjust_count",
    "success_at_1",
    "success_at_2",
    "success_at_3",
    "final_result",
    "total_time_s",
    "nav_call_count",
    "pickup_call_count",
    "putdown_call_count",
    "nav_success_count",
    "pickup_success_count",
    "putdown_success_count",
    "final_position_error",
    "failure_reason",
    "vlm_summary",
    "human_note",
]


@dataclass
class NavRun:
    nav_call_count: int = 0
    nav_success_count: int = 0
    errors: List[float] = None

    def __post_init__(self) -> None:
        if self.errors is None:
            self.errors = []

    @property
    def final_position_error(self) -> float:
        return self.errors[-1] if self.errors else float("nan")

    @property
    def max_position_error(self) -> float:
        return max(self.errors) if self.errors else float("nan")


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


def goto(
    x: float,
    y: float,
    yaw: Optional[float],
    label: str,
    run: Optional[NavRun] = None,
) -> float:
    print(f"[goto] {label}: ({x:.3f}, {y:.3f})")
    if run is not None:
        run.nav_call_count += 1
    ros_bridge.move_to_goal(x, y, yaw)
    if run is not None:
        run.nav_success_count += 1
    pos = ros_bridge.get_current_pos()
    error = dist(pos, (x, y))
    print(f"[odom] ({pos[0]:.3f}, {pos[1]:.3f}), error={error:.3f}m")
    if run is not None:
        run.errors.append(error)
    return error


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def reset_home(home_x: float, home_y: float, settle_s: float) -> None:
    """Hard reset robot model pose in Gazebo.

    This is experiment setup, not a navigation action, so it is not counted in
    total_time_s or nav_call_count. It also fixes yaw to 0 so every repeat has
    the same initial heading.
    """
    print(f"\n[reset] teleporting robot to home ({home_x:.3f}, {home_y:.3f}, yaw=0)")

    rospy.wait_for_service("/gazebo/set_model_state", timeout=10.0)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    msg = ModelState()
    msg.model_name = ROBOT_MODEL_NAME
    msg.reference_frame = "world"
    msg.pose.position.x = home_x
    msg.pose.position.y = home_y
    msg.pose.position.z = HOME_Z
    qx, qy, qz, qw = yaw_to_quat(0.0)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    msg.twist.linear.x = 0.0
    msg.twist.linear.y = 0.0
    msg.twist.linear.z = 0.0
    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = 0.0

    resp = set_state(msg)
    if not resp.success:
        raise RuntimeError(f"Gazebo reset failed: {resp.status_message}")

    deadline = time.time() + 5.0
    while time.time() < deadline:
        pos = ros_bridge.get_current_pos()
        yaw = ros_bridge.get_current_orientation()
        if dist(pos, (home_x, home_y)) < 0.05 and abs(yaw) < 0.10:
            break
        time.sleep(0.1)

    pos = ros_bridge.get_current_pos()
    yaw = ros_bridge.get_current_orientation()
    print(f"[reset] odom=({pos[0]:.3f}, {pos[1]:.3f}), yaw={yaw:.3f}")
    time.sleep(settle_s)


def manual_code_for(task_id: str) -> str:
    if task_id == "N1":
        return "move_to_xy(1.0, 2.0)"
    if task_id == "N2":
        return "trash = min(trash_objects, key=lambda o: abs(x)+abs(y)); move_to_obj_by_offset(trash, 0, 0)"
    if task_id == "N3":
        return "sofa = parse_obj_name('sofa'); follow 5 waypoints around sofa center with half-size 2.0m"
    if task_id == "N4":
        return "start = get_robot_pos(); trash = min(trash_objects, key=lambda o: abs(x)+abs(y)); move to trash approach; return start"
    raise KeyError(task_id)


def run_n1(_: Dict[str, dict], run: NavRun) -> None:
    goto(1.0, 2.0, None, "N1 target (1.0, 2.0)", run)


def run_n2(scene: Dict[str, dict], run: NavRun) -> None:
    trash_names = names_containing(scene, "trash", "bin")
    target = choose_min_abs_sum(scene, trash_names)
    ax, ay, ayaw = approach_pose(scene, target)
    print(f"[N2] selected trash: {target}")
    goto(ax, ay, ayaw, f"N2 approach {target}", run)


def run_n3(scene: Dict[str, dict], run: NavRun) -> None:
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
    for x, y, yaw, label in points:
        goto(x, y, yaw, label, run)


def run_n4(scene: Dict[str, dict], run: NavRun) -> None:
    start = ros_bridge.get_current_pos()
    trash_names = names_containing(scene, "trash", "bin")
    target = choose_min_abs_sum(scene, trash_names)
    tx, ty = obj_xy(scene, target)
    ax, ay, ayaw = approach_pose(scene, target)
    print(f"[N4] start=({start[0]:.3f}, {start[1]:.3f})")
    print(f"[N4] selected trash: {target}, object=({tx:.3f}, {ty:.3f})")
    goto(ax, ay, ayaw, f"N4 approach {target}", run)
    goto(start[0], start[1], None, "N4 return to start", run)


TASK_RUNNERS = {
    "N1": run_n1,
    "N2": run_n2,
    "N3": run_n3,
    "N4": run_n4,
}


def build_record(
    run_index: int,
    task_id: str,
    repeat_id: int,
    run: NavRun,
    final_result: str,
    total_time_s: float,
    failure_reason: str,
    tolerance: float,
) -> dict:
    if task_id == "N3":
        final_position_error = run.max_position_error
        note = f"E4 manual baseline; no LLM; no VLM; tolerance={tolerance:.2f}m; N3 error=max waypoint error"
    else:
        final_position_error = run.final_position_error
        note = f"E4 manual baseline; no LLM; no VLM; tolerance={tolerance:.2f}m"

    return {
        "run_id": f"E4_{run_index:03d}",
        "task_id": task_id,
        "task_type": "Navigation",
        "instruction": TASK_INSTRUCTIONS[task_id],
        "method_type": "Manual",
        "model_name": "None",
        "temperature": "None",
        "adjust_policy": 0,
        "repeat_id": repeat_id,
        "generated_code": manual_code_for(task_id),
        "semantic_parse_correct": "yes",
        "decomposition_correct": "yes",
        "code_executable": "yes",
        "api_call_count": run.nav_call_count,
        "adjust_count": 0,
        "success_at_1": "yes" if final_result == "Success" else "no",
        "success_at_2": "",
        "success_at_3": "",
        "final_result": final_result,
        "total_time_s": f"{total_time_s:.1f}",
        "nav_call_count": run.nav_call_count,
        "pickup_call_count": 0,
        "putdown_call_count": 0,
        "nav_success_count": run.nav_success_count,
        "pickup_success_count": 0,
        "putdown_success_count": 0,
        "final_position_error": f"{final_position_error:.3f}" if not math.isnan(final_position_error) else "",
        "failure_reason": failure_reason,
        "vlm_summary": "not used",
        "human_note": note,
    }


def run_one(
    run_index: int,
    task_id: str,
    repeat_id: int,
    scene: Dict[str, dict],
    tolerance: float,
    home_x: float,
    home_y: float,
    reset_settle_s: float,
) -> dict:
    print(f"\n========== E4 {task_id} repeat {repeat_id} ==========")
    reset_home(home_x, home_y, reset_settle_s)

    run = NavRun()
    t0 = time.time()
    failure_reason = ""
    try:
        TASK_RUNNERS[task_id](scene, run)
        measured_error = run.max_position_error if task_id == "N3" else run.final_position_error
        if measured_error <= tolerance:
            final_result = "Success"
        else:
            final_result = "Fail"
            failure_reason = f"position_error {measured_error:.3f}m > tolerance {tolerance:.3f}m"
    except Exception as e:
        final_result = "Fail"
        failure_reason = str(e)
    total_time_s = time.time() - t0

    print(
        f"[result] {task_id} repeat {repeat_id}: {final_result}, "
        f"time={total_time_s:.1f}s, nav={run.nav_success_count}/{run.nav_call_count}, "
        f"reason={failure_reason or 'OK'}"
    )
    return build_record(
        run_index,
        task_id,
        repeat_id,
        run,
        final_result,
        total_time_s,
        failure_reason,
        tolerance,
    )


def write_csv(path: str, records: List[dict], append: bool) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    file_exists = os.path.exists(path)
    mode = "a" if append else "w"
    with open(path, mode, encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        if not append or not file_exists:
            writer.writeheader()
        writer.writerows(records)
    print(f"\n[csv] wrote {len(records)} records to {path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tasks", nargs="+", choices=list(TASK_RUNNERS), default=None)
    parser.add_argument("--all", action="store_true", help="Run N1 N2 N3 N4.")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--tolerance", type=float, default=0.30)
    parser.add_argument("--home-x", type=float, default=0.0)
    parser.add_argument("--home-y", type=float, default=0.0)
    parser.add_argument("--reset-settle-s", type=float, default=1.0)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--append", action="store_true")
    args = parser.parse_args()

    if args.all:
        tasks = ["N1", "N2", "N3", "N4"]
    elif args.tasks:
        tasks = args.tasks
    else:
        parser.error("Use --all or --tasks N1 N2 ...")

    scene = load_scene()
    ros_bridge.init()

    records: List[dict] = []
    run_index = 1
    for task_id in tasks:
        for repeat_id in range(1, args.repeats + 1):
            record = run_one(
                run_index=run_index,
                task_id=task_id,
                repeat_id=repeat_id,
                scene=scene,
                tolerance=args.tolerance,
                home_x=args.home_x,
                home_y=args.home_y,
                reset_settle_s=args.reset_settle_s,
            )
            records.append(record)
            run_index += 1

    write_csv(args.output, records, append=args.append)

    failed = [r for r in records if r["final_result"] != "Success"]
    print("\n========== summary ==========")
    for record in records:
        print(
            f"{record['task_id']} repeat {record['repeat_id']}: "
            f"{record['final_result']} error={record['final_position_error']}m "
            f"time={record['total_time_s']}s"
        )
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
