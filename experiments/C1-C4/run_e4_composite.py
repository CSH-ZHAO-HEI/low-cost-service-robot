#!/usr/bin/env python3
"""
E4 manual API baseline data collection for composite tasks C1-C4.

E4 (per plan.md):
  - manual small-brain API sequence
  - Gazebo execution
  - no LLM, no VLM, no adjust/replan
  - method_type = Manual, model_name = None, temperature = None, adjust_policy = 0

This script lives in C1-C4/ and imports the LOCAL composite_capability_test.py
(not G1-G4/composite_capability_test.py). Outputs go to C1-C4/outputs/.

Prerequisites:
  1. ./run_gazebo.sh
  2. ./run_rtab.sh
  3. ./run_teb.sh
  4. rosrun small_brain_sim arm_task_server.py
  5. python3 get_scene.py

Usage:
  python3 C1-C4/run_e4_composite.py --all --repeats 3
  python3 C1-C4/run_e4_composite.py --tasks C1 C2 C3 C4 --repeats 3
  python3 C1-C4/run_e4_composite.py --tasks C4 --repeats 1
  python3 C1-C4/run_e4_composite.py --all --repeats 3 --append

Outputs:
  C1-C4/outputs/C-E4.csv
  C1-C4/outputs/C-E4-appendix.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
import traceback
from dataclasses import dataclass, field
from typing import Dict, List, Tuple


HERE = os.path.dirname(os.path.abspath(__file__))
# Insert C1-C4 dir FIRST so local composite_capability_test wins over any G1-G4 copy
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import composite_capability_test as comp     # noqa: E402
# After composite imports, G1-G4 and Big Brain are on sys.path
import manip_capability_test as cap          # noqa: E402  (base ManipTester + helpers)
import ros_bridge                            # noqa: E402  (move_to_goal monkey-patch target)
from action import robot_api                 # noqa: E402  (_call_arm_service monkey-patch target)

# Tolerance constants from manip_capability_test
try:
    from manip_capability_test import BETWEEN_Y_TOL, PLACE_Z_TOL
except ImportError:
    BETWEEN_Y_TOL = 0.08
    PLACE_Z_TOL   = 0.10


# ── Output paths ──────────────────────────────────────────────────
OUTPUT_DIR       = os.path.join(HERE, "outputs")
DEFAULT_ORIGINAL = os.path.join(OUTPUT_DIR, "C-E4.csv")
DEFAULT_APPENDIX = os.path.join(OUTPUT_DIR, "C-E4-appendix.csv")


# ── CSV fields ────────────────────────────────────────────────────

# Main table — strictly aligned with original author's E4.csv (30 cols)
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

# Appendix table — detailed E4 info, including C4-specific fields
APPENDIX_FIELDS = [
    "run_id",
    "task_id",
    "experiment",
    "repeat_id",
    "instruction",
    "method_type",
    "model_name",
    "temperature",
    "adjust_policy",
    "reset_robot_pose_xyz_yaw",
    "reset_object_poses",
    "expected_target",
    "expected_target_pose_xyz",
    "final_robot_pose_xyz_yaw",
    "final_object_poses",
    "xy_error_m",
    "z_error_m",
    "picked_height_pass",
    "placement_pass",
    "gazebo_judge_result",
    "vlm_judge_result_optional",
    "manual_api_sequence",
    "service_success_sequence",
    "api_trace_summary",
    "object_reset_method",
    "judge_rule",
    "total_time_s",
    "failure_stage",
    "failure_type",
    "human_note",
    # C4-specific
    "c4_completed_waypoints",
    "c4_detected_red",
    "c4_resumed_after_drop",
    "c4_patrol_completed",
    "c4_red_to_trash_error_m",
]


# ── Task instructions (current C1-C4 behavior) ────────────────────

TASK_INSTRUCTIONS = {
    "C1": "Pick up the Coke can from BalconyTable_01_001 and put it on NightStand_01_001",
    "C2": "Pick up the red block from NightStand_01_002 and put it between blue_block and yellow_block",
    "C3": "Pick all ground blocks red/blue/yellow and put them into Trash_01_001",
    "C4": ("Follow the N3-style SofaC_01_001 square route; when red_block is detected, "
           "pick it up, put it into Trash_01_001, return to the interrupted patrol pose, "
           "and complete the square route"),
}


# ── Manual API sequence text (recorded in generated_code) ─────────

MANUAL_SEQ = {
    "C1": ("manual sequence: reset_c1; nav→BalconyTable approach; arm→BALCONY_PICK_POSE; "
           "snap Coke + close gripper + attach block_follower::link7; "
           "rotate→NightStand_01_001 + ARM_CLAMP + re-snap; "
           "nav→NightStand_01_001 approach; arm→ARM_PUT; detach + open gripper + teleport Coke onto surface"),
    "C2": ("manual sequence: reset_table_pick; nav→NightStand_01_002 south approach; "
           "arm→NIGHTSTAND_PICK_POSE; snap red_block + close + attach block_follower::link7; "
           "rotate→drop area + ARM_CLAMP + re-snap; "
           "nav→blue/yellow midpoint approach; arm→ARM_PUT; detach + open + teleport red between blue/yellow"),
    "C3": ("manual sequence: reset_c3 (red/blue/yellow on ground); "
           "for each block in [red, blue, yellow]: robot_api.pick_up_obj(block); "
           "self._drop_to_trash(block); drive_forward(-0.5) + arm→ARM_CLAMP"),
    "C4": ("manual sequence: reset_c4 (red on SofaC patrol path); "
           "follow N3-style waypoints (corner1..corner4 + close square) around SofaC_01_001 with half=2.0; "
           "after reaching corner1: robot_api.pick_up_obj(red); self._drop_to_trash(red, force_side='E'); "
           "back from trash; arm→ARM_CLAMP; nav to segment1 midpoint (resume); "
           "complete remaining corners and close square"),
}


# ── Trace (count API calls via monkey-patch) ──────────────────────

@dataclass
class Trace:
    nav_call_count:        int = 0
    nav_success_count:     int = 0
    pickup_call_count:     int = 0
    pickup_success_count:  int = 0
    putdown_call_count:    int = 0
    putdown_success_count: int = 0
    nav_goals: List[str] = field(default_factory=list)
    services:  List[str] = field(default_factory=list)

    @property
    def api_call_count(self) -> int:
        return self.nav_call_count + len(self.services)


def install_tracers(trace: Trace):
    original_move_to_goal     = ros_bridge.move_to_goal
    original_call_arm_service = robot_api._call_arm_service

    def traced_move_to_goal(x, y, yaw=None, *args, **kwargs):
        trace.nav_call_count += 1
        label = f"({float(x):.3f},{float(y):.3f})"
        try:
            result = original_move_to_goal(x, y, yaw, *args, **kwargs)
            if result:
                trace.nav_success_count += 1
                trace.nav_goals.append(label + ":success")
            else:
                trace.nav_goals.append(label + ":fail")
            return result
        except Exception:
            trace.nav_goals.append(label + ":exception")
            raise

    def traced_call_arm_service(service_name: str, target_name: str):
        is_pick = service_name == "/arm/pick"
        is_put  = service_name in {"/arm/drop", "/arm/put"}
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

    ros_bridge.move_to_goal      = traced_move_to_goal
    robot_api._call_arm_service  = traced_call_arm_service

    def restore():
        ros_bridge.move_to_goal     = original_move_to_goal
        robot_api._call_arm_service = original_call_arm_service

    return restore


# ── Helpers ───────────────────────────────────────────────────────

def fmt_pose(values) -> str:
    if values is None:
        return "N/A"
    return "(" + ", ".join(f"{float(v):.3f}" for v in values) + ")"


def expected_target_desc(task_id: str) -> str:
    return {
        "C1": f"Coke center inside {comp.NIGHTSTAND} bbox; z >= NightStand surface_z - tolerance",
        "C2": (f"red_block projection between blue_block and yellow_block; "
               f"lateral_error <= {BETWEEN_Y_TOL:.3f}; z near BLOCK_HALF"),
        "C3": (f"red/blue/yellow all within {comp.TRASH_NEAR_TOL:.2f}m XY of {comp.TRASH}"),
        "C4": (f"red_block within {comp.TRASH_NEAR_TOL:.2f}m XY of {comp.TRASH}; "
               f"N3-style SofaC patrol completed (half={comp.C4_SQUARE_HALF}); "
               f"resumed_after_drop if detected"),
    }[task_id]


def expected_target_pose(task_id: str, tester: comp.CompositeTester) -> Tuple[float, float, float]:
    if task_id == "C1":
        return (comp.NIGHTSTAND_XY[0], comp.NIGHTSTAND_XY[1],
                comp.NIGHTSTAND_SURFACE_Z + comp.BLOCK_HALF)
    if task_id == "C2":
        bx, by, _ = cap.model_pose(tester.get_model_state, comp.BLUE)
        yx, yy, _ = cap.model_pose(tester.get_model_state, comp.YELLOW)
        return ((bx + yx) / 2.0, (by + yy) / 2.0, comp.BLOCK_HALF)
    if task_id in {"C3", "C4"}:
        return (comp.TRASH_XY[0], comp.TRASH_XY[1],
                comp.TRASH_SURFACE_Z + comp.BLOCK_HALF)
    return (0.0, 0.0, 0.0)


def judge_rule_for(task_id: str) -> str:
    if task_id == "C1":
        return ("C1 judge: Coke center inside NightStand_01_001 bbox "
                f"(hx={comp.NIGHTSTAND_BBOX[0]:.3f}, hy={comp.NIGHTSTAND_BBOX[1]:.3f}) "
                "and Coke z >= NightStand surface_z - 0.05")
    if task_id == "C2":
        return ("C2 judge: red_block projection lies between blue/yellow segment "
                f"(lateral_error <= {BETWEEN_Y_TOL:.3f}) and z_error <= {PLACE_Z_TOL:.3f}")
    if task_id == "C3":
        return ("C3 judge: red/blue/yellow all have xy distance to Trash_01_001 "
                f"<= {comp.TRASH_NEAR_TOL:.2f}m")
    return ("C4 judge: red_block within "
            f"{comp.TRASH_NEAR_TOL:.2f}m of Trash_01_001 AND patrol_completed AND "
            "all N3-style waypoints (corner1..4 + close square) covered AND "
            "if detected_red then resumed_after_drop")


def reset_initial_pose_for(task_id: str) -> Tuple[float, float, float]:
    return {
        "C1": comp.C1_CUP_POSE,
        "C2": comp.C2_RED_POSE,
        "C3": comp.C3_RED_POSE,
        "C4": comp.C4_RED_POSE,
    }[task_id]


def reset_object_poses_str(task_id: str) -> str:
    """human-readable reset state for appendix."""
    if task_id == "C1":
        return f"Coke={fmt_pose(comp.C1_CUP_POSE)}"
    if task_id == "C2":
        return (f"red={fmt_pose(comp.C2_RED_POSE)}; "
                f"blue={fmt_pose(comp.BLUE_POSE)}; "
                f"yellow={fmt_pose(comp.YELLOW_POSE)}")
    if task_id == "C3":
        return (f"red={fmt_pose(comp.C3_RED_POSE)}; "
                f"blue={fmt_pose(comp.C3_BLUE_POSE)}; "
                f"yellow={fmt_pose(comp.C3_YELLOW_POSE)}")
    if task_id == "C4":
        return (f"red={fmt_pose(comp.C4_RED_POSE)}; "
                f"sofa_center={fmt_pose(comp.SOFA_C_XY)}; "
                f"half={comp.C4_SQUARE_HALF}")
    return "N/A"


def final_object_poses_str(task_id: str, tester: comp.CompositeTester) -> str:
    """Read current Gazebo poses of relevant objects."""
    try:
        if task_id == "C1":
            return f"Coke={fmt_pose(cap.model_pose(tester.get_model_state, comp.CUP))}"
        if task_id == "C2":
            r = cap.model_pose(tester.get_model_state, comp.RED)
            b = cap.model_pose(tester.get_model_state, comp.BLUE)
            y = cap.model_pose(tester.get_model_state, comp.YELLOW)
            return f"red={fmt_pose(r)}; blue={fmt_pose(b)}; yellow={fmt_pose(y)}"
        if task_id == "C3":
            r = cap.model_pose(tester.get_model_state, comp.RED)
            b = cap.model_pose(tester.get_model_state, comp.BLUE)
            y = cap.model_pose(tester.get_model_state, comp.YELLOW)
            return f"red={fmt_pose(r)}; blue={fmt_pose(b)}; yellow={fmt_pose(y)}"
        if task_id == "C4":
            return f"red={fmt_pose(cap.model_pose(tester.get_model_state, comp.RED)) }"
    except Exception as e:
        return f"error: {e}"
    return "N/A"


def compute_errors(task_id: str, tester: comp.CompositeTester,
                   target_pose: Tuple[float, float, float]) -> Tuple[str, str]:
    """Per-task xy_error_m / z_error_m strings.
    C3 uses MAX of red/blue/yellow→trash distances (strictest)."""
    try:
        if task_id == "C1":
            p = cap.model_pose(tester.get_model_state, comp.CUP)
            xy = cap.dist_xy((p[0], p[1]), (target_pose[0], target_pose[1]))
            zz = abs(p[2] - target_pose[2])
            return f"{xy:.3f}", f"{zz:.3f}"
        if task_id == "C2":
            p = cap.model_pose(tester.get_model_state, comp.RED)
            xy = cap.dist_xy((p[0], p[1]), (target_pose[0], target_pose[1]))
            zz = abs(p[2] - comp.BLOCK_HALF)
            return f"{xy:.3f}", f"{zz:.3f}"
        if task_id == "C3":
            max_xy = 0.0
            max_z  = 0.0
            for nm in (comp.RED, comp.BLUE, comp.YELLOW):
                p = cap.model_pose(tester.get_model_state, nm)
                xy = cap.dist_xy((p[0], p[1]), comp.TRASH_XY)
                zz = abs(p[2] - target_pose[2])
                if xy > max_xy: max_xy = xy
                if zz > max_z:  max_z  = zz
            return f"{max_xy:.3f}", f"{max_z:.3f}"
        if task_id == "C4":
            p = cap.model_pose(tester.get_model_state, comp.RED)
            xy = cap.dist_xy((p[0], p[1]), comp.TRASH_XY)
            zz = abs(p[2] - target_pose[2])
            return f"{xy:.3f}", f"{zz:.3f}"
    except Exception as e:
        return f"err:{e}", "N/A"
    return "N/A", "N/A"


def failure_stage_for(trace: Trace, failure_reason: str) -> str:
    if not failure_reason or failure_reason == "N/A":
        return "N/A"
    if trace.nav_call_count > trace.nav_success_count:
        return "nav"
    if any(":fail" in s and "/arm/pick" in s for s in trace.services):
        return "pick"
    if any(":fail" in s and "/arm/prepare_put" in s for s in trace.services):
        return "prepare_put"
    if any(":fail" in s and ("/arm/drop" in s or "/arm/put" in s) for s in trace.services):
        return "drop"
    return "judge"


def failure_type_for(failure_reason: str) -> str:
    if not failure_reason or failure_reason == "N/A":
        return "N/A"
    lower = failure_reason.lower()
    if "timeout" in lower:        return "service_timeout"
    if "navigation" in lower:     return "navigation_failed"
    if "move_base" in lower:      return "navigation_failed"
    if "pick" in lower:           return "pick_failed"
    if "drop" in lower or "put" in lower: return "place_failed"
    if "judge" in lower or "tolerance" in lower or "error" in lower:
        return "judge_fail"
    return "execution_failed"


# ── Task body ─────────────────────────────────────────────────────

def run_task_body(task_id: str, tester: comp.CompositeTester) -> bool:
    """Each tester.run_c* already calls its own reset internally."""
    runners = {
        "C1": tester.run_c1,
        "C2": tester.run_c2,
        "C3": tester.run_c3,
        "C4": tester.run_c4,
    }
    return runners[task_id]()


# ── Record builders ───────────────────────────────────────────────

def build_original_record(
    run_id: str, task_id: str, repeat_id: int, trace: Trace,
    final_result: str, total_time_s: float,
    final_position_error: str, failure_reason: str,
) -> Dict[str, object]:
    return {
        "run_id":                 run_id,
        "task_id":                task_id,
        "task_type":              "Composite",
        "instruction":            TASK_INSTRUCTIONS[task_id],
        "method_type":            "Manual",
        "model_name":             "None",
        "temperature":            "None",
        "adjust_policy":          0,
        "repeat_id":              repeat_id,
        "generated_code":         MANUAL_SEQ[task_id],
        "semantic_parse_correct": "yes",
        "decomposition_correct":  "yes",
        "code_executable":        "yes",
        "api_call_count":         trace.api_call_count,
        "adjust_count":           0,
        "success_at_1":           "yes" if final_result == "Success" else "no",
        "success_at_2":           "N/A",
        "success_at_3":           "N/A",
        "final_result":           final_result,
        "total_time_s":           f"{total_time_s:.1f}",
        "nav_call_count":         trace.nav_call_count,
        "pickup_call_count":      trace.pickup_call_count,
        "putdown_call_count":     trace.putdown_call_count,
        "nav_success_count":      trace.nav_success_count,
        "pickup_success_count":   trace.pickup_success_count,
        "putdown_success_count":  trace.putdown_success_count,
        "final_position_error":   final_position_error,
        "failure_reason":         failure_reason if failure_reason else "N/A",
        "vlm_summary":            "not used",
        "human_note":             "E4 manual API baseline; no LLM/VLM; total_time_s includes internal reset",
    }


def build_appendix_record(
    run_id: str, task_id: str, repeat_id: int, trace: Trace,
    final_result: str, failure_reason: str,
    reset_robot_pose: Tuple[float, float, float, float],
    reset_objects: str,
    target_desc: str, target_pose: Tuple[float, float, float],
    final_robot_pose: Tuple[float, float, float, float],
    final_objects: str,
    xy_error: str, z_error: str,
    total_time_s: float,
    tester: comp.CompositeTester,
) -> Dict[str, object]:
    is_success = (final_result == "Success")
    placement_pass     = "yes" if is_success else "no"
    picked_height_pass = "yes" if is_success else "no"

    # C4 specific state
    c4_completed     = "N/A"
    c4_detected_red  = "N/A"
    c4_resumed       = "N/A"
    c4_patrol_done   = "N/A"
    c4_red_err       = "N/A"
    if task_id == "C4":
        c4_completed = " | ".join(getattr(tester, "c4_completed_waypoints", []) or []) or "(empty)"
        c4_detected_red = "yes" if getattr(tester, "c4_detected_red", False) else "no"
        c4_resumed      = "yes" if getattr(tester, "c4_resumed_after_drop", False) else "no"
        c4_patrol_done  = "yes" if getattr(tester, "c4_patrol_completed", False) else "no"
        c4_red_err      = xy_error

    return {
        "run_id":                  run_id,
        "task_id":                 task_id,
        "experiment":              "E4",
        "repeat_id":               repeat_id,
        "instruction":             TASK_INSTRUCTIONS[task_id],
        "method_type":             "Manual",
        "model_name":              "None",
        "temperature":             "None",
        "adjust_policy":           0,
        "reset_robot_pose_xyz_yaw": fmt_pose(reset_robot_pose),
        "reset_object_poses":      reset_objects,
        "expected_target":         target_desc,
        "expected_target_pose_xyz": fmt_pose(target_pose),
        "final_robot_pose_xyz_yaw": fmt_pose(final_robot_pose),
        "final_object_poses":      final_objects,
        "xy_error_m":              xy_error,
        "z_error_m":               z_error,
        "picked_height_pass":      picked_height_pass,
        "placement_pass":          placement_pass,
        "gazebo_judge_result":     "pass" if is_success else "fail",
        "vlm_judge_result_optional": "not used",
        "manual_api_sequence":     MANUAL_SEQ[task_id],
        "service_success_sequence": " | ".join(trace.services) if trace.services else "(none)",
        "api_trace_summary": (
            f"nav={trace.nav_success_count}/{trace.nav_call_count}; "
            f"pick={trace.pickup_success_count}/{trace.pickup_call_count}; "
            f"put={trace.putdown_success_count}/{trace.putdown_call_count}"
        ),
        "object_reset_method": (
            "CompositeTester internal reset (HOME robot + teleport objects); "
            "C2 also teleports blue/yellow reference positions; "
            "C4 only teleports red"
        ),
        "judge_rule":              judge_rule_for(task_id),
        "total_time_s":            f"{total_time_s:.1f}",
        "failure_stage":           failure_stage_for(trace, failure_reason),
        "failure_type":            failure_type_for(failure_reason),
        "human_note": (
            "E4 manual API baseline; no LLM/VLM; "
            "total_time_s includes the internal reset performed by tester.run_c*."
        ),
        # C4-specific
        "c4_completed_waypoints":  c4_completed,
        "c4_detected_red":         c4_detected_red,
        "c4_resumed_after_drop":   c4_resumed,
        "c4_patrol_completed":     c4_patrol_done,
        "c4_red_to_trash_error_m": c4_red_err,
    }


# ── Per-run runner ────────────────────────────────────────────────

def run_one(run_index: int, task_id: str, repeat_id: int,
            tester: comp.CompositeTester) -> Tuple[Dict, Dict]:
    print(f"\n========== E4 {task_id} repeat {repeat_id} ==========")

    # Reset-time pre-snapshot (robot at HOME after previous task, before this run_c* resets)
    reset_robot_pose_snapshot = cap.model_pose_yaw(tester.get_model_state, comp.ROBOT)
    reset_objects_str         = reset_object_poses_str(task_id)

    trace   = Trace()
    restore = install_tracers(trace)
    target_desc = expected_target_desc(task_id)
    target_pose = expected_target_pose(task_id, tester)
    failure_reason = ""

    t0 = time.time()
    try:
        ok = run_task_body(task_id, tester)
        final_result = "Success" if ok else "Fail"
        if not ok:
            failure_reason = "gazebo judge failed (tester.check_c* returned False)"
    except Exception as exc:
        final_result   = "Fail"
        failure_reason = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        restore()
    total_time_s = time.time() - t0

    # target_pose may have changed (blue/yellow could move in C2). recompute.
    target_pose = expected_target_pose(task_id, tester)

    final_robot_pose = cap.model_pose_yaw(tester.get_model_state, comp.ROBOT)
    final_objects    = final_object_poses_str(task_id, tester)
    xy_error, z_error = compute_errors(task_id, tester, target_pose)

    run_id = f"E4_C{run_index:03d}"
    original = build_original_record(
        run_id, task_id, repeat_id, trace,
        final_result, total_time_s, xy_error, failure_reason,
    )
    appendix = build_appendix_record(
        run_id, task_id, repeat_id, trace,
        final_result, failure_reason,
        reset_robot_pose_snapshot, reset_objects_str,
        target_desc, target_pose,
        final_robot_pose, final_objects,
        xy_error, z_error,
        total_time_s, tester,
    )
    print(
        f"[result] {task_id} repeat {repeat_id}: {final_result}, "
        f"time={total_time_s:.1f}s, xy_err={xy_error}m, "
        f"reason={failure_reason or 'OK'}"
    )
    return original, appendix


def write_csv(path: str, fields: List[str], rows: List[Dict],
              append: bool) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    mode = "a" if append else "w"
    file_exists = os.path.exists(path)
    with open(path, mode, encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        if not append or not file_exists:
            writer.writeheader()
        writer.writerows(rows)
    print(f"[csv] wrote {len(rows)} rows → {path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tasks", nargs="+", choices=["C1", "C2", "C3", "C4"], default=None)
    parser.add_argument("--all", action="store_true", help="Run C1 C2 C3 C4.")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--output-original", default=DEFAULT_ORIGINAL)
    parser.add_argument("--output-appendix", default=DEFAULT_APPENDIX)
    parser.add_argument("--append", action="store_true",
                        help="Append to CSVs instead of overwriting.")
    parser.add_argument("--continue-on-error", action="store_true",
                        help="(default behavior already; flag kept for compatibility)")
    parser.add_argument("--pause-after-reset", type=float, default=0.0)
    args = parser.parse_args()

    if args.all:
        tasks = ["C1", "C2", "C3", "C4"]
    elif args.tasks:
        tasks = args.tasks
    else:
        parser.error("Use --all or specify --tasks C1 C2 ...")

    # Sanity: ensure we're using the local C1-C4 composite
    comp_path = os.path.abspath(comp.__file__)
    expected_dir = os.path.abspath(HERE)
    if not comp_path.startswith(expected_dir):
        print(f"[WARNING] composite_capability_test imported from {comp_path}, "
              f"expected under {expected_dir}")
    else:
        print(f"[import] composite_capability_test = {comp_path}")

    tester = comp.CompositeTester(pause_after_reset=args.pause_after_reset)
    originals: List[Dict] = []
    appendices: List[Dict] = []
    run_index = 1

    for task_id in tasks:
        for repeat_id in range(1, args.repeats + 1):
            try:
                original, appendix = run_one(run_index, task_id, repeat_id, tester)
            except Exception as exc:
                # Outer safety net: never stop the batch
                print(f"[run_one] outer exception for {task_id} r{repeat_id}: {exc}")
                traceback.print_exc()
                original = {f: "N/A" for f in ORIGINAL_FIELDS}
                original.update({
                    "run_id":      f"E4_C{run_index:03d}",
                    "task_id":     task_id,
                    "task_type":   "Composite",
                    "instruction": TASK_INSTRUCTIONS[task_id],
                    "method_type": "Manual",
                    "model_name":  "None",
                    "temperature": "None",
                    "adjust_policy": 0,
                    "repeat_id":   repeat_id,
                    "final_result": "Fail",
                    "failure_reason": f"outer:{exc}",
                    "vlm_summary":  "not used",
                    "human_note":   "outer exception",
                })
                appendix = {f: "N/A" for f in APPENDIX_FIELDS}
                appendix.update({
                    "run_id":  f"E4_C{run_index:03d}",
                    "task_id": task_id,
                    "experiment": "E4",
                    "repeat_id":  repeat_id,
                    "instruction": TASK_INSTRUCTIONS[task_id],
                    "judge_rule":  judge_rule_for(task_id),
                    "failure_stage": "outer",
                    "failure_type":  "execution_failed",
                    "human_note":    f"outer exception: {exc}",
                })
            originals.append(original)
            appendices.append(appendix)
            run_index += 1

    write_csv(args.output_original, ORIGINAL_FIELDS, originals, args.append)
    write_csv(args.output_appendix, APPENDIX_FIELDS, appendices, args.append)

    ok_count = sum(1 for r in originals if r.get("final_result") == "Success")
    print(f"\n[summary] Success {ok_count}/{len(originals)}")
    return 0 if ok_count == len(originals) else 1


if __name__ == "__main__":
    raise SystemExit(main())
