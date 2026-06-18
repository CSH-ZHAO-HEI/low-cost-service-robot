#!/usr/bin/env python3
"""
Smoke-test E2 for G1-G4 with real LLM + VLM Judge + replan.

E2 in this project means:
  1. Reset robot/red_block to the task initial state.
  2. Ask BigBrain/LLM to generate code.
  3. Execute the code with JudgeLLM enabled.
  4. JudgeLLM may call VLM and may replan once.
  5. Use Gazebo truth as the final external result.

Default is intentionally small and cheap:
  python3 G1-G4/test_e2_vlm_manip.py --tasks G1 --start-writer

Before running, Gazebo, navigation, arm_task_server, and ROS env must be ready.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
import time
from typing import Dict, Iterable, List, Tuple


HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
BIG_BRAIN_DIR = os.path.join(PROJECT_ROOT, "big_brain")
OUT_DIR = os.path.join(HERE, "outputs")
DEFAULT_CSV = os.path.join(OUT_DIR, "E2_vlm_smoke_results.csv")
DEFAULT_VLM_TOPIC = "/judge_camera/rgb/image_raw"

if HERE not in sys.path:
    sys.path.insert(0, HERE)
if BIG_BRAIN_DIR not in sys.path:
    sys.path.insert(0, BIG_BRAIN_DIR)

import manip_capability_test as cap  # noqa: E402
from big_brain import BigBrain  # noqa: E402
from config import JUDGE_LOG_DIR, REPLAN_EXECUTE_CODE, TARGET_LLM_MODEL, TARGET_VLM_MODEL  # noqa: E402
from model.llm import JudgeLLM  # noqa: E402


TASK_INSTRUCTIONS = {
    "G1": "Pick up the red block",
    "G2": "Navigate to the red block on the ground and pick it up",
    "G3": "Pick up the red block and put it on the CoffeeTable",
    "G4": "Pick up the red block and put it between blue_block and yellow_block",
}

FIELDS = [
    "run_id",
    "task_id",
    "instruction",
    "model_name",
    "vlm_model",
    "replan_execute_code",
    "repeat_id",
    "e2_final_result",
    "big_brain_returned",
    "gazebo_final_result",
    "judge_disagreement",
    "disagreement_type",
    "final_red_pose_xyz",
    "target_pose_xyz",
    "xy_error_m",
    "z_error_m",
    "judge_event_count",
    "vlm_call_count",
    "vlm_pass_count",
    "last_vlm_pass",
    "last_judge_final_pass",
    "replan_generated_count",
    "replan_executed_count",
    "replan_failed_count",
    "last_vlm_reason",
    "last_failure_reason",
    "image_age_before_s",
    "image_age_after_s",
    "total_time_s",
]


def fmt_pose(values: Tuple[float, ...]) -> str:
    return "(" + ", ".join(f"{v:.3f}" for v in values) + ")"


def image_path() -> str:
    return os.path.join(BIG_BRAIN_DIR, "image", "image.jpg")


def image_age_s() -> float:
    path = image_path()
    if not os.path.exists(path):
        return float("inf")
    return time.time() - os.path.getmtime(path)


def start_vlm_writer(topic: str) -> subprocess.Popen:
    cmd = [sys.executable, os.path.join(BIG_BRAIN_DIR, "vlm_image_writer.py"), topic]
    print(f"[writer] starting: {' '.join(cmd)}")
    return subprocess.Popen(cmd, cwd=BIG_BRAIN_DIR)


def wait_fresh_image(timeout_s: float = 8.0, max_age_s: float = 3.0) -> float:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        age = image_age_s()
        if age <= max_age_s:
            return age
        time.sleep(0.2)
    return image_age_s()


def reset_red_pose_for(task_id: str) -> Tuple[float, float, float]:
    return cap.G1_RED_POSE if task_id == "G1" else cap.G2_RED_POSE


def target_for(task_id: str, tester: cap.ManipTester) -> Tuple[str, Tuple[float, float, float]]:
    if task_id == "G1":
        return "red_block picked above ground", reset_red_pose_for(task_id)
    if task_id == "G2":
        return "red_block picked above ground", reset_red_pose_for(task_id)
    if task_id == "G3":
        scene = cap.load_scene()
        info = scene[cap.TABLE]
        return (
            cap.TABLE,
            (
                float(info.get("place_x", info["object_x"])),
                float(info.get("place_y", info["object_y"])),
                float(info.get("surface_z", 0.0)) + cap.BLOCK_HALF,
            ),
        )
    bx, by, bz = cap.model_pose(tester.get_model_state, cap.BLUE)
    yx, yy, yz = cap.model_pose(tester.get_model_state, cap.YELLOW)
    target_z = max(0.0, ((bz - cap.BLOCK_HALF) + (yz - cap.BLOCK_HALF)) / 2.0) + cap.BLOCK_HALF
    return "midpoint(blue_block,yellow_block)", ((bx + yx) / 2.0, (by + yy) / 2.0, target_z)


def gazebo_truth(task_id: str, tester: cap.ManipTester) -> Tuple[bool, Tuple[float, float, float], Tuple[float, float, float], str, str]:
    target_desc, target = target_for(task_id, tester)
    red_pose = cap.model_pose(tester.get_model_state, cap.RED)
    if task_id in {"G1", "G2"}:
        ok = red_pose[2] > cap.PICK_Z_THRESHOLD
        final_error = "" if ok else f"red_block z <= {cap.PICK_Z_THRESHOLD:.3f}"
        return ok, red_pose, target, "", final_error
    xy = cap.dist_xy((red_pose[0], red_pose[1]), (target[0], target[1]))
    zz = abs(red_pose[2] - target[2])
    if task_id == "G3":
        ok = tester.check_red_on_table(cap.TABLE)
    elif task_id == "G4":
        ok = tester.check_red_between_objs(cap.BLUE, cap.YELLOW)
    else:
        ok = xy <= cap.PLACE_XY_TOL and zz <= cap.PLACE_Z_TOL
    final_error = "" if ok else f"{target_desc} placement outside semantic tolerance"
    return ok, red_pose, target, f"{xy:.3f}", final_error


def read_new_judge_events(log_path: str, start_offset: int) -> List[Dict[str, object]]:
    if not os.path.exists(log_path):
        return []
    with open(log_path, "r", encoding="utf-8") as f:
        f.seek(start_offset)
        rows = []
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                rows.append({"event": "unparsed", "raw": line})
        return rows


def summarize_events(events: List[Dict[str, object]]) -> Dict[str, object]:
    judge_events = [e for e in events if e.get("event") == "judge"]
    vlm_events = [e for e in judge_events if e.get("vlm_needed")]
    replan_generated = [e for e in events if e.get("event") == "replan_generated"]
    replan_executed = [e for e in events if e.get("event") == "replan_executed"]
    replan_failed = [e for e in events if e.get("event") == "replan_failed"]
    last_vlm_reason = ""
    last_vlm_pass = ""
    for event in reversed(vlm_events):
        vlm_result = event.get("vlm_result") or {}
        if last_vlm_pass == "" and "pass" in vlm_result:
            last_vlm_pass = str(vlm_result.get("pass"))
        last_vlm_reason = vlm_result.get("reason", "")
        if last_vlm_reason:
            break
    last_judge_final_pass = ""
    for event in reversed(judge_events):
        if "final_pass" in event:
            last_judge_final_pass = str(event.get("final_pass"))
            break
    last_failure_reason = ""
    for event in reversed(replan_failed):
        last_failure_reason = str(event.get("reason", ""))
        if last_failure_reason:
            break
    return {
        "judge_event_count": len(judge_events),
        "vlm_call_count": len(vlm_events),
        "vlm_pass_count": sum(1 for e in vlm_events if (e.get("vlm_result") or {}).get("pass") is True),
        "last_vlm_pass": last_vlm_pass,
        "last_judge_final_pass": last_judge_final_pass,
        "replan_generated_count": len(replan_generated),
        "replan_executed_count": len(replan_executed),
        "replan_failed_count": len(replan_failed),
        "last_vlm_reason": last_vlm_reason,
        "last_failure_reason": last_failure_reason,
    }


def reenable_real_judge() -> None:
    # ManipTester disables JudgeLLM for E4 capability tests. E2 must restore it.
    cap.robot_api.judge_llm = JudgeLLM()


def run_one(task_id: str, repeat_id: int, tester: cap.ManipTester, brain: BigBrain) -> Dict[str, object]:
    instruction = TASK_INSTRUCTIONS[task_id]
    print(f"\n========== E2 VLM smoke {task_id} repeat {repeat_id} ==========")
    print(f"[instruction] {instruction}")

    tester.reset_scene(red_pose=reset_red_pose_for(task_id))
    reenable_real_judge()

    age_before = wait_fresh_image(timeout_s=5.0)
    print(f"[image] age before task: {age_before:.1f}s")

    log_path = os.path.join(JUDGE_LOG_DIR, "judge_events.jsonl")
    start_offset = os.path.getsize(log_path) if os.path.exists(log_path) else 0

    t0 = time.time()
    big_brain_ok = brain.run_once(instruction)
    total_time_s = time.time() - t0

    gazebo_ok, red_pose, target_pose, xy_error, failure_reason = gazebo_truth(task_id, tester)
    events = read_new_judge_events(log_path, start_offset)
    summary = summarize_events(events)
    age_after = image_age_s()
    z_error = ""
    if task_id in {"G3", "G4"}:
        z_error = f"{abs(red_pose[2] - target_pose[2]):.3f}"

    row = {
        "run_id": f"E2_VLM_{task_id}_r{repeat_id:02d}",
        "task_id": task_id,
        "instruction": instruction,
        "model_name": TARGET_LLM_MODEL,
        "vlm_model": TARGET_VLM_MODEL,
        "replan_execute_code": str(REPLAN_EXECUTE_CODE),
        "repeat_id": repeat_id,
        "e2_final_result": "",
        "big_brain_returned": "Success" if big_brain_ok else "Fail",
        "gazebo_final_result": "Success" if gazebo_ok else "Fail",
        "judge_disagreement": "False",
        "disagreement_type": "",
        "final_red_pose_xyz": fmt_pose(red_pose),
        "target_pose_xyz": fmt_pose(target_pose),
        "xy_error_m": xy_error,
        "z_error_m": z_error,
        "image_age_before_s": f"{age_before:.1f}",
        "image_age_after_s": f"{age_after:.1f}",
        "total_time_s": f"{total_time_s:.1f}",
        **summary,
    }
    if failure_reason and not row["last_failure_reason"]:
        row["last_failure_reason"] = failure_reason

    if row["last_vlm_pass"] == "False" and gazebo_ok:
        row["judge_disagreement"] = "True"
        row["disagreement_type"] = "vlm_false_negative"
    elif row["last_vlm_pass"] == "True" and not gazebo_ok:
        row["judge_disagreement"] = "True"
        row["disagreement_type"] = "vlm_false_positive"

    e2_ok = bool(big_brain_ok and gazebo_ok)
    if row["last_judge_final_pass"] == "False" or row["last_vlm_pass"] == "False":
        e2_ok = False
    row["e2_final_result"] = "Success" if e2_ok else "Fail"

    print(
        f"[result] E2={row['e2_final_result']} BigBrain={row['big_brain_returned']} Gazebo={row['gazebo_final_result']} "
        f"VLM calls={row['vlm_call_count']} replan={row['replan_generated_count']}/{row['replan_executed_count']} "
        f"time={row['total_time_s']}s"
    )
    if row["last_vlm_reason"]:
        print(f"[last VLM] {row['last_vlm_reason']}")
    if row["last_failure_reason"]:
        print(f"[failure] {row['last_failure_reason']}")
    return row


def write_csv(path: str, rows: List[Dict[str, object]], append: bool) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    mode = "a" if append else "w"
    file_exists = os.path.exists(path)
    with open(path, mode, encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        if not append or not file_exists:
            writer.writeheader()
        writer.writerows(rows)
    print(f"[csv] wrote {len(rows)} rows: {path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tasks", nargs="+", choices=["G1", "G2", "G3", "G4"], default=["G1"])
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--output", default=DEFAULT_CSV)
    parser.add_argument("--append", action="store_true")
    parser.add_argument("--start-writer", action="store_true", help="Start vlm_image_writer.py during this test.")
    parser.add_argument("--vlm-topic", default=DEFAULT_VLM_TOPIC, help="RGB image topic for VLM judge.")
    parser.add_argument("--allow-stale-image", action="store_true", help="Run even if image/image.png is stale.")
    parser.add_argument("--save-history", action="store_true", help="Allow BigBrain to write successful plans to RAG history.")
    args = parser.parse_args()

    writer_proc = None
    if args.start_writer:
        writer_proc = start_vlm_writer(args.vlm_topic)
        time.sleep(1.0)

    try:
        age = wait_fresh_image(timeout_s=5.0)
        if age > 3.0 and not args.allow_stale_image:
            print(f"[ERROR] VLM image is stale: age={age:.1f}s")
            print("        Start the writer or rerun with --start-writer.")
            return 2
        print(f"[image] current age={age:.1f}s")

        tester = cap.ManipTester()
        reenable_real_judge()
        brain = BigBrain()
        if not args.save_history:
            brain._save_history = lambda path: None

        rows = []
        for task_id in args.tasks:
            for repeat_id in range(1, args.repeats + 1):
                rows.append(run_one(task_id, repeat_id, tester, brain))

        write_csv(args.output, rows, args.append)
        return 0 if all(row["e2_final_result"] == "Success" for row in rows) else 1
    finally:
        if writer_proc is not None:
            writer_proc.terminate()
            try:
                writer_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                writer_proc.kill()


if __name__ == "__main__":
    raise SystemExit(main())
