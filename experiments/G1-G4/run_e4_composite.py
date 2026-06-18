#!/usr/bin/env python3
"""
Collect E4 manual-baseline data for composite tasks C1-C4.

E4 means:
  - manual small-brain API sequence
  - Gazebo execution
  - no LLM
  - no VLM as the main judge

Reset time is not counted in total_time_s.

Prerequisites:
  1. ./run_gazebo.sh
  2. ./run_rtab.sh
  3. ./run_teb.sh
  4. rosrun small_brain_sim arm_task_server.py
  5. python3 get_scene.py

Usage:
  python3 G1-G4/run_e4_composite.py --all
  python3 G1-G4/run_e4_composite.py --tasks C1 C2 --repeats 3
  python3 G1-G4/run_e4_composite.py --tasks C2 --repeats 1

Outputs:
  G1-G4/E4_C_original.csv
  G1-G4/E4_C_appendix.csv
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


HERE         = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
DEFAULT_ORIGINAL = os.path.join(HERE, "E4_C_original.csv")
DEFAULT_APPENDIX = os.path.join(HERE, "E4_C_appendix.csv")

if HERE not in sys.path:
    sys.path.insert(0, HERE)

import manip_capability_test as cap           # noqa: E402
import composite_capability_test as comp      # noqa: E402


# ── CSV 欄位（與 G-series E4 保持相同 30-column 格式）──────────────

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

# ── 任務描述 ──────────────────────────────────────────────────────

TASK_INSTRUCTIONS = {
    "C1": "Pick up the Coke can from BalconyTable_01_001 and put it on NightStand_01_001",
    "C2": "Pick up the red block from the CoffeeTable and put it between blue_block and yellow_block",
    "C3": "Pick all non-furniture objects on the ground and put them in the trash can",
    "C4": "Navigate in a 1m x 1m square around CoffeeTable; pick up the red block when found and put it in the trash",
}

RED_SIZE = "(0.030, 0.025, 0.050)"


# ── Trace（追蹤 API 呼叫）────────────────────────────────────────

@dataclass
class Trace:
    nav_call_count:       int = 0
    nav_success_count:    int = 0
    pickup_call_count:    int = 0
    pickup_success_count: int = 0
    putdown_call_count:   int = 0
    putdown_success_count: int = 0
    nav_goals:  List[str] = field(default_factory=list)
    services:   List[str] = field(default_factory=list)

    @property
    def api_call_count(self) -> int:
        return self.nav_call_count + len(self.services)


# ── 工具函式 ──────────────────────────────────────────────────────

def fmt_pose(values: Tuple[float, ...]) -> str:
    return "(" + ", ".join(f"{v:.3f}" for v in values) + ")"


def reset_pose_for(task_id: str) -> Tuple[float, float, float]:
    return {
        "C1": comp.C1_CUP_POSE,    # cup on CoffeeTable near DeskPortraitA_02
        "C2": comp.C2_RED_POSE,
        "C3": comp.C3_RED_POSE,
        "C4": comp.C4_RED_POSE,
    }[task_id]


def expected_target(task_id: str) -> str:
    return {
        "C1": f"{comp.CUP} placed on {comp.NIGHTSTAND} surface",
        "C2": "red_block placed between blue_block and yellow_block",
        "C3": f"red/blue/yellow blocks near {comp.TRASH} (all within {comp.TRASH_NEAR_TOL:.2f}m)",
        "C4": f"red_block placed near {comp.TRASH} after square patrol",
    }[task_id]


def expected_target_pose(task_id: str, tester: comp.CompositeTester) -> Tuple[float, float, float]:
    if task_id == "C1":
        return (comp.NIGHTSTAND_XY[0], comp.NIGHTSTAND_XY[1],
                comp.NIGHTSTAND_SURFACE_Z + comp.BLOCK_HALF)
    if task_id == "C2":
        bx, by, bz = cap.model_pose(tester.get_model_state, comp.BLUE)
        yx, yy, yz = cap.model_pose(tester.get_model_state, comp.YELLOW)
        target_z = max(0.0, ((bz - comp.BLOCK_HALF) + (yz - comp.BLOCK_HALF)) / 2.0) + comp.BLOCK_HALF
        return ((bx + yx) / 2.0, (by + yy) / 2.0, target_z)
    if task_id in {"C3", "C4"}:
        return (comp.TRASH_XY[0], comp.TRASH_XY[1], comp.TRASH_SURFACE_Z + comp.BLOCK_HALF)
    return (0.0, 0.0, 0.0)


def manual_code_for(task_id: str) -> str:
    return {
        "C1": (
            "reset_c1(); "
            "pick_up_obj(Coke); "   # 從 Gazebo 拿即時座標，導航到罐子前，high pick
            "put_down_obj_by_offset(NightStand_01_001, 0, 0); "
            "judge NightStand bbox + surface_z"
        ),
        "C2": (
            "reset_table_pick(C2_RED_POSE); "
            "pick_up_from_coffeetable(red_block); "
            "put_down_between_objs(blue_block, yellow_block); "
            "judge between-segment placement"
        ),
        "C3": (
            "reset_c3; "
            "for block in [red, blue, yellow]: pick_up_obj(block); put_down_obj_by_offset(Trash_01_001, 0, 0); "
            "judge all blocks near trash"
        ),
        "C4": (
            "reset_c4; "
            "for corner in square(±0.5m around CoffeeTable): move_to_xy(corner); "
            "if red_block within 0.9m: pick_up_obj(red_block); put_down_obj_by_offset(Trash_01_001, 0, 0); "
            "judge red_block near trash"
        ),
    }[task_id]


def judge_rule_for(task_id: str) -> str:
    if task_id == "C1":
        info = cap.load_scene().get(comp.NIGHTSTAND, {})
        return (
            f"placed if plastic_cup inside {comp.NIGHTSTAND} bbox "
            f"(hx={info.get('bbox_half_x', comp.NIGHTSTAND_BBOX[0]):.3f}, "
            f"hy={info.get('bbox_half_y', comp.NIGHTSTAND_BBOX[1]):.3f}) "
            f"and cup z > nightstand surface_z - 0.05"
        )
    if task_id == "C2":
        return (
            f"placed if red_block projection lies between blue/yellow segment "
            f"(lateral_error <= {cap.BETWEEN_Y_TOL:.3f}) and z_error <= {cap.PLACE_Z_TOL:.3f}"
        )
    if task_id == "C3":
        return (
            f"success if red/blue/yellow all within {comp.TRASH_NEAR_TOL:.2f}m XY of {comp.TRASH}"
        )
    return (
        f"success if red_block within {comp.TRASH_NEAR_TOL:.2f}m XY of {comp.TRASH} "
        f"after square patrol"
    )


def failure_stage_for(trace: Trace, failure_reason: str) -> str:
    if not failure_reason:
        return ""
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


# ── Tracer 安裝（monkey-patch ros_bridge / robot_api）────────────

def install_tracers(tester: comp.CompositeTester, trace: Trace):
    original_move_to_goal    = cap.ros_bridge.move_to_goal
    original_call_arm_service = cap.robot_api._call_arm_service

    def traced_move_to_goal(x, y, yaw=None, *args, **kwargs):
        trace.nav_call_count += 1
        label = f"({float(x):.3f},{float(y):.3f})"
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

    cap.ros_bridge.move_to_goal      = traced_move_to_goal
    cap.robot_api._call_arm_service  = traced_call_arm_service

    def restore():
        cap.ros_bridge.move_to_goal     = original_move_to_goal
        cap.robot_api._call_arm_service = original_call_arm_service

    return restore


# ── 主執行函式 ────────────────────────────────────────────────────

def run_task_body(
    task_id: str,
    tester: comp.CompositeTester,
) -> Tuple[bool, str, Tuple[float, float, float]]:
    target_desc = expected_target(task_id)
    target_pose = expected_target_pose(task_id, tester)

    if task_id == "C1":
        # 完整流程委託給 CompositeTester.run_c1()（put_on_table_test.py 模式）
        ok = tester.run_c1()
        # run_c1 內已執行 reset_c1 → 不重複 reset

    elif task_id == "C2":
        ok = tester.run_c2()

    elif task_id == "C3":
        for name in [comp.RED, comp.BLUE, comp.YELLOW]:
            cap.robot_api.pick_up_obj(name)
            cap.robot_api.put_down_obj_by_offset(comp.TRASH, 0.0, 0.0)
        ok = tester.check_c3()

    elif task_id == "C4":
        tx, ty = comp.COFFEETABLE_XY
        half   = comp.C4_SQUARE_HALF
        corners = [
            (tx + half, ty + half),
            (tx + half, ty - half),
            (tx - half, ty - half),
            (tx - half, ty + half),
        ]
        picked = False
        for cx, cy in corners:
            cap.robot_api.move_to_xy(cx, cy)
            if not picked:
                rx, ry, rz = cap.model_pose(tester.get_model_state, comp.RED)
                robot_dist = cap.dist_xy((rx, ry), (cx, cy))
                if rz < comp.PICK_Z_THRESHOLD and robot_dist < comp.C4_DETECT_RANGE:
                    cap.robot_api.pick_up_obj(comp.RED)
                    cap.robot_api.put_down_obj_by_offset(comp.TRASH, 0.0, 0.0)
                    picked = True
        ok = tester.check_c4()

    else:
        raise ValueError(f"Unknown task_id: {task_id}")

    return ok, target_desc, target_pose


def _reset_for_task(task_id: str, tester: comp.CompositeTester) -> None:
    if task_id == "C1":
        tester.reset_c1()
    elif task_id == "C2":
        tester.reset_table_pick(comp.C2_RED_POSE)
    elif task_id == "C3":
        tester.reset_c3()
    elif task_id == "C4":
        tester.reset_c4()


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
        "run_id":              run_id,
        "task_id":             task_id,
        "task_type":           "Composite",
        "instruction":         TASK_INSTRUCTIONS[task_id],
        "method_type":         "Manual",
        "model_name":          "None",
        "temperature":         "None",
        "adjust_policy":       0,
        "repeat_id":           repeat_id,
        "generated_code":      manual_code_for(task_id),
        "semantic_parse_correct": "yes",
        "decomposition_correct":  "yes",
        "code_executable":        "yes",
        "api_call_count":      trace.api_call_count,
        "adjust_count":        0,
        "success_at_1":        "yes" if final_result == "Success" else "no",
        "success_at_2":        "",
        "success_at_3":        "",
        "final_result":        final_result,
        "total_time_s":        f"{total_time_s:.1f}",
        "nav_call_count":      trace.nav_call_count,
        "pickup_call_count":   trace.pickup_call_count,
        "putdown_call_count":  trace.putdown_call_count,
        "nav_success_count":   trace.nav_success_count,
        "pickup_success_count":  trace.pickup_success_count,
        "putdown_success_count": trace.putdown_success_count,
        "final_position_error":  final_position_error,
        "failure_reason":      failure_reason,
        "vlm_summary":         "not used",
        "human_note":          "E4 C manual baseline; no LLM; no VLM; Gazebo judge.",
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
    tester: comp.CompositeTester,
) -> Dict[str, object]:
    placement_pass = "yes" if final_result == "Success" else "no"

    # C3 額外：記錄藍黃最終位置
    if task_id == "C3":
        bp = cap.model_pose(tester.get_model_state, comp.BLUE)
        yp = cap.model_pose(tester.get_model_state, comp.YELLOW)
        after_extra = (
            f"; blue={fmt_pose(bp)}; yellow={fmt_pose(yp)}"
        )
        blue_pose_xyz   = fmt_pose(comp.C3_BLUE_POSE)
        yellow_pose_xyz = fmt_pose(comp.C3_YELLOW_POSE)
    elif task_id in {"C1", "C2", "C4"}:
        after_extra     = ""
        blue_pose_xyz   = fmt_pose(cap.model_pose(tester.get_model_state, comp.BLUE)) if task_id == "C2" else ""
        yellow_pose_xyz = fmt_pose(cap.model_pose(tester.get_model_state, comp.YELLOW)) if task_id == "C2" else ""
    else:
        after_extra = ""
        blue_pose_xyz = yellow_pose_xyz = ""

    return {
        "run_id":              run_id,
        "task_id":             task_id,
        "task_type":           "Composite",
        "instruction":         TASK_INSTRUCTIONS[task_id],
        "experiment":          "E4",
        "repeat_id":           repeat_id,
        "model_name":          "None",
        "temperature":         "None",
        "adjust_policy":       0,
        "reset_robot_pose_xyz_yaw": fmt_pose(cap.HOME_POSE),
        "reset_red_pose_xyz":       fmt_pose(reset_red_pose),
        "red_block_size_xyz":       RED_SIZE,
        "blue_pose_xyz":            blue_pose_xyz,
        "yellow_pose_xyz":          yellow_pose_xyz,
        "expected_target":          target_desc,
        "expected_target_pose_xyz": fmt_pose(target_pose),
        "final_robot_pose_xyz_yaw": fmt_pose(final_robot_pose),
        "final_red_pose_xyz":       fmt_pose(final_red_pose),
        "xy_error_m":              xy_error,
        "z_error_m":               z_error,
        "picked_height_pass":      "",
        "placement_pass":          placement_pass,
        "gazebo_judge_result":     "pass" if final_result == "Success" else "fail",
        "vlm_judge_result_optional": "not used",
        "failure_stage":           failure_stage_for(trace, failure_reason),
        "failure_type":            failure_type_for(failure_reason),
        "nav_goal_sequence":       " | ".join(trace.nav_goals),
        "arm_services_used":       " | ".join(trace.services),
        "api_trace_summary":       (
            f"nav={trace.nav_success_count}/{trace.nav_call_count}; "
            f"services={'; '.join(trace.services)}"
        ),
        "attempt_trace_summary":   "single manual attempt",
        "model_call_latency_s":    "",
        "model_input_tokens":      "",
        "model_output_tokens":     "",
        "estimated_cost_cny":      "0",
        "object_reset_method":     (
            "CompositeTester.reset_* ; red recreated/teleported per task; "
            "blue/yellow teleported for C3, unchanged for C1/C2/C4"
        ),
        "why_useful": (
            "Objective Gazebo ground-truth row for C-series manual baseline; "
            "C2 uses between-segment; C3 checks all 3 blocks; C4 uses square patrol."
        ),
        "manual_api_sequence":     manual_code_for(task_id),
        "service_success_sequence": " | ".join(trace.services),
        "gazebo_before_state":     reset_state,
        "gazebo_after_state": (
            f"robot={fmt_pose(final_robot_pose)}; red={fmt_pose(final_red_pose)}"
            + after_extra
        ),
        "judge_rule": judge_rule_for(task_id),
    }


def run_one(
    run_index: int,
    task_id: str,
    repeat_id: int,
    tester: comp.CompositeTester,
):
    print(f"\n========== E4 {task_id} repeat {repeat_id} ==========")
    reset_red = reset_pose_for(task_id)
    _reset_for_task(task_id, tester)

    # 記錄 reset 後狀態（C1 用杯子位置，其餘用紅塊）
    reset_robot = cap.model_pose_yaw(tester.get_model_state, comp.ROBOT)
    _obj_for_state = comp.CUP if task_id == "C1" else comp.RED
    reset_obj_actual = cap.model_pose(tester.get_model_state, _obj_for_state)
    reset_state = f"robot={fmt_pose(reset_robot)}; {_obj_for_state}={fmt_pose(reset_obj_actual)}"

    trace   = Trace()
    restore = install_tracers(tester, trace)
    t0      = time.time()
    failure_reason = ""
    target_desc    = expected_target(task_id)
    target_pose    = expected_target_pose(task_id, tester)

    try:
        ok, target_desc, target_pose = run_task_body(task_id, tester)
        final_result = "Success" if ok else "Fail"
        if not ok:
            failure_reason = "gazebo judge failed"
    except Exception as exc:
        final_result   = "Fail"
        failure_reason = str(exc)
    finally:
        restore()

    total_time_s     = time.time() - t0
    # C1 追蹤杯子位置；其餘追蹤紅塊
    _tracked = comp.CUP if task_id == "C1" else comp.RED
    final_red_pose   = cap.model_pose(tester.get_model_state, _tracked)
    final_robot_pose = cap.model_pose_yaw(tester.get_model_state, comp.ROBOT)

    # 計算誤差（受追蹤物件到目標位置）
    xy = cap.dist_xy(
        (final_red_pose[0], final_red_pose[1]),
        (target_pose[0],    target_pose[1]),
    )
    zz = abs(final_red_pose[2] - target_pose[2])
    xy_error = f"{xy:.3f}"
    z_error  = f"{zz:.3f}"
    final_position_error = xy_error

    run_id   = f"E4_C{run_index:03d}"
    original = build_original_record(
        run_id, task_id, repeat_id, trace,
        final_result, total_time_s, final_position_error, failure_reason,
    )
    appendix = build_appendix_record(
        run_id, task_id, repeat_id, trace,
        final_result, failure_reason,
        reset_red, reset_state,
        target_desc, target_pose,
        final_robot_pose, final_red_pose,
        xy_error, z_error,
        tester,
    )
    print(
        f"[result] {task_id} repeat {repeat_id}: {final_result}, "
        f"time={total_time_s:.1f}s, xy_err={xy_error}m, reason={failure_reason or 'OK'}"
    )
    return original, appendix


def write_csv(
    path: str,
    fields: List[str],
    rows: List[Dict[str, object]],
    append: bool,
) -> None:
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
    parser.add_argument("--append", action="store_true")
    parser.add_argument("--pause-after-reset", type=float, default=0.0)
    args = parser.parse_args()

    if args.all:
        tasks = ["C1", "C2", "C3", "C4"]
    elif args.tasks:
        tasks = args.tasks
    else:
        parser.error("Use --all or specify --tasks C1 C2 ...")

    tester     = comp.CompositeTester(pause_after_reset=args.pause_after_reset)
    originals: List[Dict[str, object]] = []
    appendices: List[Dict[str, object]] = []
    run_index  = 1

    for task_id in tasks:
        for repeat_id in range(1, args.repeats + 1):
            original, appendix = run_one(run_index, task_id, repeat_id, tester)
            originals.append(original)
            appendices.append(appendix)
            run_index += 1

    write_csv(args.output_original, ORIGINAL_FIELDS, originals, args.append)
    write_csv(args.output_appendix, APPENDIX_FIELDS, appendices, args.append)

    ok_count = sum(1 for r in originals if r["final_result"] == "Success")
    print(f"[summary] Success {ok_count}/{len(originals)}")
    return 0 if ok_count == len(originals) else 1


if __name__ == "__main__":
    raise SystemExit(main())
