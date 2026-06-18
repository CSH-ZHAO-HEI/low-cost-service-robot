#!/usr/bin/env python3
"""
Collect E4 manual-baseline data for manipulation tasks G1-G4.

E4 means:
  - manual small-brain API sequence
  - Gazebo execution
  - no LLM
  - no VLM as the main judge

Reset time is not counted in total_time_s. Each repeat starts by resetting the
robot and red_block through manip_capability_test.ManipTester.

Prerequisites:
  1. ./run_gazebo.sh
  2. ./run_rtab.sh
  3. ./run_teb.sh or ./run_teb_compare.sh
  4. rosrun small_brain_sim arm_task_server.py
  5. python3 get_scene.py

Usage:
  python3 G1-G4/run_e4_manip.py --all
  python3 G1-G4/run_e4_manip.py --tasks G1 G2 G3 G4 --repeats 3
  python3 G1-G4/run_e4_manip.py --tasks G2 --repeats 1

Outputs:
  G1-G4/E4_G_original.csv
  G1-G4/E4_G_appendix.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Tuple


HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
DEFAULT_ORIGINAL = os.path.join(HERE, "E4_G_original.csv")
DEFAULT_APPENDIX = os.path.join(HERE, "E4_G_appendix.csv")

if HERE not in sys.path:
    sys.path.insert(0, HERE)

import manip_capability_test as cap  # noqa: E402


ORIGINAL_FIELDS = [
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

APPENDIX_FIELDS = [
    "run_id",
    "task_id",
    "task_type",
    "instruction",
    "experiment",
    "repeat_id",
    "model_name",
    "temperature",
    "adjust_policy",
    "reset_robot_pose_xyz_yaw",
    "reset_red_pose_xyz",
    "red_block_size_xyz",
    "blue_pose_xyz",
    "yellow_pose_xyz",
    "expected_target",
    "expected_target_pose_xyz",
    "final_robot_pose_xyz_yaw",
    "final_red_pose_xyz",
    "xy_error_m",
    "z_error_m",
    "picked_height_pass",
    "placement_pass",
    "gazebo_judge_result",
    "vlm_judge_result_optional",
    "failure_stage",
    "failure_type",
    "nav_goal_sequence",
    "arm_services_used",
    "api_trace_summary",
    "attempt_trace_summary",
    "model_call_latency_s",
    "model_input_tokens",
    "model_output_tokens",
    "estimated_cost_cny",
    "object_reset_method",
    "why_useful",
    "manual_api_sequence",
    "service_success_sequence",
    "gazebo_before_state",
    "gazebo_after_state",
    "judge_rule",
]

TASK_INSTRUCTIONS = {
    "G1": "Pick up the red block",
    "G2": "Navigate to the red block on the ground and pick it up",
    "G3": "Put the held object on the CoffeeTable",
    "G4": "Put the held object between blue_block and yellow_block",
}

RED_SIZE = "(0.03, 0.025, 0.05)"


@dataclass
class Trace:
    nav_call_count: int = 0
    nav_success_count: int = 0
    pickup_call_count: int = 0
    pickup_success_count: int = 0
    putdown_call_count: int = 0
    putdown_success_count: int = 0
    nav_goals: List[str] = field(default_factory=list)
    services: List[str] = field(default_factory=list)

    @property
    def api_call_count(self) -> int:
        return self.nav_call_count + len(self.services)


def fmt_pose(values: Tuple[float, ...]) -> str:
    return "(" + ", ".join(f"{v:.3f}" for v in values) + ")"


def reset_red_pose_for(task_id: str) -> Tuple[float, float, float]:
    return cap.G1_RED_POSE if task_id == "G1" else cap.G2_RED_POSE


def expected_target(task_id: str) -> str:
    return {
        "G1": "red_block picked above ground",
        "G2": "red_block picked above ground after navigation",
        "G3": "red_block placed on CoffeeTable_01_001 surface bbox",
        "G4": "red_block placed between blue_block and yellow_block",
    }[task_id]


def manual_code_for(task_id: str) -> str:
    return {
        "G1": "reset; /arm/pick(red_block); judge red_block.z > threshold",
        "G2": "reset; pick_up_obj(red_block); judge red_block.z > threshold",
        "G3": "reset; pick_up_obj(red_block); put_down_obj_by_offset(CoffeeTable_01_001, 0, 0); judge table placement",
        "G4": "reset; pick_up_obj(red_block); put_down_between_objs(blue_block, yellow_block); judge between-segment placement; record midpoint error",
    }[task_id]


def judge_rule_for(task_id: str) -> str:
    if task_id in {"G1", "G2"}:
        return f"picked if red_block.z > {cap.PICK_Z_THRESHOLD:.3f}"
    if task_id == "G3":
        return f"placed if red_block center is inside CoffeeTable bbox and z_error <= {cap.PLACE_Z_TOL:.3f}"
    return (
        f"placed if red_block projection lies between blue/yellow and lateral_error <= "
        f"{cap.BETWEEN_Y_TOL:.3f}; midpoint_error is still recorded"
    )


def failure_stage(trace: Trace, failure_reason: str, task_id: str) -> str:
    if not failure_reason:
        return ""
    if trace.nav_call_count > trace.nav_success_count:
        return "nav"
    if any(":fail" in item and "/arm/pick" in item for item in trace.services):
        return "pick"
    if any(":fail" in item and "/arm/prepare_put" in item for item in trace.services):
        return "prepare_put"
    if any(":fail" in item and ("/arm/drop" in item or "/arm/put" in item) for item in trace.services):
        return "drop"
    if "did not" in failure_reason or "judge" in failure_reason or task_id in {"G3", "G4"}:
        return "judge"
    return "execution"


def failure_type(failure_reason: str) -> str:
    lower = failure_reason.lower()
    if not failure_reason:
        return ""
    if "timeout" in lower:
        return "service_timeout"
    if "navigation" in lower or "move_base" in lower:
        return "navigation_failed"
    if "pick" in lower:
        return "pick_failed"
    if "drop" in lower or "put" in lower:
        return "place_failed"
    if "error" in lower or "tolerance" in lower:
        return "judge_fail"
    return "execution_failed"


def install_tracers(tester: cap.ManipTester, trace: Trace):
    original_move_to_goal = cap.ros_bridge.move_to_goal
    original_call_arm_service = cap.robot_api._call_arm_service
    original_call_arm_pick_only = tester.call_arm_pick_only

    def traced_move_to_goal(x, y, yaw=None, *args, **kwargs):
        trace.nav_call_count += 1
        label = f"({float(x):.3f},{float(y):.3f},{'' if yaw is None else f'{float(yaw):.3f}'})"
        try:
            result = original_move_to_goal(x, y, yaw, *args, **kwargs)
            trace.nav_success_count += 1
            trace.nav_goals.append(label + ":success")
            return result
        except Exception:
            trace.nav_goals.append(label + ":fail")
            raise

    def traced_call_arm_service(service_name: str, target_name: str):
        is_pick = service_name == "/arm/pick"
        is_put = service_name in {"/arm/drop", "/arm/put"}
        if is_pick:
            trace.pickup_call_count += 1
        if is_put:
            trace.putdown_call_count += 1
        try:
            result = original_call_arm_service(service_name, target_name)
            if is_pick:
                trace.pickup_success_count += 1
            if is_put:
                trace.putdown_success_count += 1
            trace.services.append(f"{service_name}({target_name}):success")
            return result
        except Exception:
            trace.services.append(f"{service_name}({target_name}):fail")
            raise

    def traced_call_arm_pick_only(target_name: str):
        trace.pickup_call_count += 1
        try:
            result = original_call_arm_pick_only(target_name)
            trace.pickup_success_count += 1
            trace.services.append(f"/arm/pick({target_name}):success")
            return result
        except Exception:
            trace.services.append(f"/arm/pick({target_name}):fail")
            raise

    cap.ros_bridge.move_to_goal = traced_move_to_goal
    cap.robot_api._call_arm_service = traced_call_arm_service
    tester.call_arm_pick_only = traced_call_arm_pick_only

    def restore():
        cap.ros_bridge.move_to_goal = original_move_to_goal
        cap.robot_api._call_arm_service = original_call_arm_service
        tester.call_arm_pick_only = original_call_arm_pick_only

    return restore


def target_for(task_id: str, tester: cap.ManipTester) -> Tuple[str, Tuple[float, float, float]]:
    if task_id == "G1":
        return "red_block picked above ground", cap.G1_RED_POSE
    if task_id == "G2":
        return "red_block picked above ground", cap.G2_RED_POSE
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


def run_task_body(task_id: str, tester: cap.ManipTester) -> Tuple[bool, str, Tuple[float, float, float]]:
    if task_id == "G1":
        tester.call_arm_pick_only(cap.RED)
        ok = tester.check_picked()
        target_desc, target = target_for(task_id, tester)
        return ok, target_desc, target
    if task_id == "G2":
        cap.robot_api.pick_up_obj(cap.RED)
        ok = tester.check_picked()
        target_desc, target = target_for(task_id, tester)
        return ok, target_desc, target
    if task_id == "G3":
        tester.ensure_red_held()
        cap.robot_api.put_down_obj_by_offset(cap.TABLE, 0.0, 0.0)
        target_desc, target = target_for(task_id, tester)
        ok = tester.check_red_on_table(cap.TABLE)
        return ok, target_desc, target
    tester.ensure_red_held()
    cap.robot_api.put_down_between_objs(cap.BLUE, cap.YELLOW)
    target_desc, target = target_for(task_id, tester)
    ok = tester.check_red_between_objs(cap.BLUE, cap.YELLOW)
    return ok, target_desc, target


def build_original_record(
    run_id: str,
    task_id: str,
    repeat_id: int,
    trace: Trace,
    final_result: str,
    total_time_s: float,
    final_position_error: str,
    failure_reason: str,
) -> Dict[str, object]:
    return {
        "run_id": run_id,
        "task_id": task_id,
        "task_type": "Manipulation",
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
        "api_call_count": trace.api_call_count,
        "adjust_count": 0,
        "success_at_1": "yes" if final_result == "Success" else "no",
        "success_at_2": "",
        "success_at_3": "",
        "final_result": final_result,
        "total_time_s": f"{total_time_s:.1f}",
        "nav_call_count": trace.nav_call_count,
        "pickup_call_count": trace.pickup_call_count,
        "putdown_call_count": trace.putdown_call_count,
        "nav_success_count": trace.nav_success_count,
        "pickup_success_count": trace.pickup_success_count,
        "putdown_success_count": trace.putdown_success_count,
        "final_position_error": final_position_error,
        "failure_reason": failure_reason,
        "vlm_summary": "not used",
        "human_note": "E4 G manual baseline; no LLM; no VLM; Gazebo judge.",
    }


def build_appendix_record(
    run_id: str,
    task_id: str,
    repeat_id: int,
    trace: Trace,
    final_result: str,
    failure_reason: str,
    reset_red_pose: Tuple[float, float, float],
    reset_state: str,
    target_desc: str,
    target_pose: Tuple[float, float, float],
    final_robot_pose: Tuple[float, float, float, float],
    final_red_pose: Tuple[float, float, float],
    xy_error: str,
    z_error: str,
) -> Dict[str, object]:
    picked_pass = "yes" if final_red_pose[2] > cap.PICK_Z_THRESHOLD else "no"
    placed_pass = ""
    if task_id in {"G3", "G4"}:
        placed_pass = "yes" if final_result == "Success" else "no"
    return {
        "run_id": run_id,
        "task_id": task_id,
        "task_type": "Manipulation",
        "instruction": TASK_INSTRUCTIONS[task_id],
        "experiment": "E4",
        "repeat_id": repeat_id,
        "model_name": "None",
        "temperature": "None",
        "adjust_policy": 0,
        "reset_robot_pose_xyz_yaw": fmt_pose(cap.HOME_POSE),
        "reset_red_pose_xyz": fmt_pose(reset_red_pose),
        "red_block_size_xyz": "(0.030, 0.025, 0.050)",
        "blue_pose_xyz": fmt_pose(cap.model_pose(cap.rospy.ServiceProxy("/gazebo/get_model_state", cap.GetModelState), cap.BLUE))
        if task_id == "G4"
        else "",
        "yellow_pose_xyz": fmt_pose(cap.model_pose(cap.rospy.ServiceProxy("/gazebo/get_model_state", cap.GetModelState), cap.YELLOW))
        if task_id == "G4"
        else "",
        "expected_target": expected_target(task_id),
        "expected_target_pose_xyz": fmt_pose(target_pose),
        "final_robot_pose_xyz_yaw": fmt_pose(final_robot_pose),
        "final_red_pose_xyz": fmt_pose(final_red_pose),
        "xy_error_m": xy_error,
        "z_error_m": z_error,
        "picked_height_pass": picked_pass if task_id in {"G1", "G2"} else "",
        "placement_pass": placed_pass,
        "gazebo_judge_result": "pass" if final_result == "Success" else "fail",
        "vlm_judge_result_optional": "not used",
        "failure_stage": failure_stage(trace, failure_reason, task_id),
        "failure_type": failure_type(failure_reason),
        "nav_goal_sequence": " | ".join(trace.nav_goals),
        "arm_services_used": " | ".join(trace.services),
        "api_trace_summary": f"nav={trace.nav_success_count}/{trace.nav_call_count}; services={'; '.join(trace.services)}",
        "attempt_trace_summary": "single manual attempt",
        "model_call_latency_s": "",
        "model_input_tokens": "",
        "model_output_tokens": "",
        "estimated_cost_cny": "0",
        "object_reset_method": "ManipTester.reset_scene; red reset to task pose; blue/yellow unchanged",
        "why_useful": "Objective Gazebo ground-truth row for G-series manual baseline; G3 uses table bbox, G4 uses between-segment and keeps midpoint error.",
        "manual_api_sequence": manual_code_for(task_id),
        "service_success_sequence": " | ".join(trace.services),
        "gazebo_before_state": reset_state,
        "gazebo_after_state": f"robot={fmt_pose(final_robot_pose)}; red={fmt_pose(final_red_pose)}",
        "judge_rule": judge_rule_for(task_id),
    }


def run_one(run_index: int, task_id: str, repeat_id: int, tester: cap.ManipTester):
    print(f"\n========== E4 {task_id} repeat {repeat_id} ==========")
    reset_red_pose = reset_red_pose_for(task_id)
    tester.reset_scene(red_pose=reset_red_pose)
    reset_robot_pose = cap.model_pose_yaw(tester.get_model_state, cap.ROBOT)
    reset_red_actual = cap.model_pose(tester.get_model_state, cap.RED)
    reset_state = f"robot={fmt_pose(reset_robot_pose)}; red={fmt_pose(reset_red_actual)}"

    trace = Trace()
    restore = install_tracers(tester, trace)
    t0 = time.time()
    failure_reason = ""
    target_desc = expected_target(task_id)
    target_pose = reset_red_pose
    try:
        ok, target_desc, target_pose = run_task_body(task_id, tester)
        final_result = "Success" if ok else "Fail"
        if not ok:
            failure_reason = "gazebo judge failed"
    except Exception as exc:
        final_result = "Fail"
        failure_reason = str(exc)
    finally:
        restore()
    total_time_s = time.time() - t0

    final_red_pose = cap.model_pose(tester.get_model_state, cap.RED)
    final_robot_pose = cap.model_pose_yaw(tester.get_model_state, cap.ROBOT)

    if task_id in {"G1", "G2"}:
        xy_error = ""
        z_error = ""
        final_position_error = "0.000" if final_red_pose[2] > cap.PICK_Z_THRESHOLD else f"{cap.PICK_Z_THRESHOLD - final_red_pose[2]:.3f}"
    else:
        xy = cap.dist_xy((final_red_pose[0], final_red_pose[1]), (target_pose[0], target_pose[1]))
        zz = abs(final_red_pose[2] - target_pose[2])
        xy_error = f"{xy:.3f}"
        z_error = f"{zz:.3f}"
        final_position_error = xy_error

    run_id = f"E4_G{run_index:03d}"
    original = build_original_record(
        run_id,
        task_id,
        repeat_id,
        trace,
        final_result,
        total_time_s,
        final_position_error,
        failure_reason,
    )
    appendix = build_appendix_record(
        run_id,
        task_id,
        repeat_id,
        trace,
        final_result,
        failure_reason,
        reset_red_pose,
        reset_state,
        target_desc,
        target_pose,
        final_robot_pose,
        final_red_pose,
        xy_error,
        z_error,
    )
    print(
        f"[result] {task_id} repeat {repeat_id}: {final_result}, "
        f"time={total_time_s:.1f}s, reason={failure_reason or 'OK'}"
    )
    return original, appendix


def write_csv(path: str, fields: List[str], rows: List[Dict[str, object]], append: bool) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    mode = "a" if append else "w"
    file_exists = os.path.exists(path)
    with open(path, mode, encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        if not append or not file_exists:
            writer.writeheader()
        writer.writerows(rows)
    print(f"[csv] wrote {len(rows)} rows: {path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tasks", nargs="+", choices=["G1", "G2", "G3", "G4"], default=None)
    parser.add_argument("--all", action="store_true", help="Run G1 G2 G3 G4.")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--output-original", default=DEFAULT_ORIGINAL)
    parser.add_argument("--output-appendix", default=DEFAULT_APPENDIX)
    parser.add_argument("--append", action="store_true")
    parser.add_argument("--pause-after-reset", type=float, default=0.0)
    args = parser.parse_args()

    if args.all:
        tasks = ["G1", "G2", "G3", "G4"]
    elif args.tasks:
        tasks = args.tasks
    else:
        parser.error("Use --all or specify --tasks G1 G2 ...")

    tester = cap.ManipTester(pause_after_reset=args.pause_after_reset)
    originals: List[Dict[str, object]] = []
    appendices: List[Dict[str, object]] = []
    run_index = 1
    for task_id in tasks:
        for repeat_id in range(1, args.repeats + 1):
            original, appendix = run_one(run_index, task_id, repeat_id, tester)
            originals.append(original)
            appendices.append(appendix)
            run_index += 1

    write_csv(args.output_original, ORIGINAL_FIELDS, originals, args.append)
    write_csv(args.output_appendix, APPENDIX_FIELDS, appendices, args.append)

    ok_count = sum(1 for row in originals if row["final_result"] == "Success")
    print(f"[summary] Success {ok_count}/{len(originals)}")
    return 0 if ok_count == len(originals) else 1


if __name__ == "__main__":
    raise SystemExit(main())
