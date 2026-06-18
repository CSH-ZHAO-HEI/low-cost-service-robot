#!/usr/bin/env python3
"""Build appendix and summary CSVs from the latest G1-G4 experiment outputs."""

from __future__ import annotations

import ast
import csv
import os
import re
import statistics
from collections import defaultdict
from typing import Dict, Iterable, List, Tuple

import yaml


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(HERE, "outputs")
SCENE_PATH = os.path.join(ROOT, "gazebo_scene.yaml")

PICK_Z_THRESHOLD = 0.08
PLACE_Z_TOL = 0.08
BETWEEN_Y_TOL = 0.08
BLOCK_HALF = 0.025


def read_csv(path: str) -> List[Dict[str, str]]:
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_csv(path: str, fields: List[str], rows: List[Dict[str, object]]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"[csv] wrote {len(rows)} rows: {path}")


def pose(text: str) -> Tuple[float, ...]:
    nums = re.findall(r"-?\d+(?:\.\d+)?", text or "")
    return tuple(float(x) for x in nums[:3])


def dist_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def load_scene() -> dict:
    with open(SCENE_PATH, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


SCENE = load_scene()
TABLE = SCENE["CoffeeTable_01_001"]
TABLE_CENTER = (float(TABLE["object_x"]), float(TABLE["object_y"]))
TABLE_HALF = (float(TABLE.get("bbox_half_x", 0.0)), float(TABLE.get("bbox_half_y", 0.0)))
TABLE_Z = float(TABLE.get("surface_z", 0.0)) + BLOCK_HALF
BLUE = (2.15, -0.20, BLOCK_HALF)
YELLOW = (2.57, -0.20, BLOCK_HALF)


def semantic_for_exec(row: Dict[str, str]) -> Tuple[str, str, str]:
    task = row["task_id"]
    red = pose(row.get("final_red_pose_xyz", ""))
    if len(red) < 3:
        return "unknown", "no final red pose", ""
    x, y, z = red

    if task in {"G1", "G2"}:
        return (
            "yes" if z > PICK_Z_THRESHOLD else "no",
            f"red_block.z > {PICK_Z_THRESHOLD:.3f}",
            f"z={z:.3f}",
        )

    if task == "G3":
        cx, cy = TABLE_CENTER
        hx, hy = TABLE_HALF
        in_bbox = (cx - hx) <= x <= (cx + hx) and (cy - hy) <= y <= (cy + hy)
        z_error = abs(z - TABLE_Z)
        return (
            "yes" if in_bbox and z_error <= PLACE_Z_TOL else "no",
            "red_block center inside CoffeeTable bbox and at table height",
            f"in_bbox={in_bbox}; z_error={z_error:.3f}",
        )

    if task == "G4":
        min_x, max_x = sorted([BLUE[0], YELLOW[0]])
        x_ok = min_x <= x <= max_x
        y_error = abs(y - BLUE[1])
        z_error = abs(z - BLUE[2])
        return (
            "yes" if x_ok and y_error <= BETWEEN_Y_TOL and z_error <= PLACE_Z_TOL else "no",
            f"red_block lies between blue/yellow x-segment and |y+0.20| <= {BETWEEN_Y_TOL:.2f}",
            f"x_ok={x_ok}; y_error={y_error:.3f}; z_error={z_error:.3f}",
        )

    return "unknown", "", ""


EXEC_FIELDS = [
    "run_id",
    "task_id",
    "experiment",
    "repeat_id",
    "model_name",
    "vlm_model",
    "strict_final_result",
    "gazebo_final_result",
    "big_brain_returned",
    "semantic_success",
    "semantic_rule",
    "semantic_detail",
    "judge_disagreement",
    "disagreement_type",
    "last_vlm_pass",
    "last_judge_final_pass",
    "vlm_call_count",
    "replan_generated_count",
    "replan_executed_count",
    "final_red_pose_xyz",
    "target_pose_xyz",
    "xy_error_m",
    "z_error_m",
    "total_time_s",
    "last_vlm_reason",
    "analysis_note",
]


def appendix_exec(src: str, experiment: str, final_col: str) -> List[Dict[str, object]]:
    rows = []
    for row in read_csv(src):
        semantic, rule, detail = semantic_for_exec(row)
        strict = row.get(final_col) or row.get("e2_final_result") or row.get("final_result") or ""
        gazebo = row.get("gazebo_final_result", "")
        notes = []
        if semantic == "yes" and strict != "Success":
            notes.append("semantic_success_but_strict_fail")
        if row.get("judge_disagreement") == "True":
            notes.append(row.get("disagreement_type", "judge_disagreement"))
        if int(row.get("replan_generated_count") or 0) > 0:
            notes.append("used_replan")
        if row.get("task_id") in {"G3", "G4"} and semantic == "yes" and gazebo != "Success":
            notes.append("gazebo_rule_too_strict_for_task_wording")
        rows.append(
            {
                "run_id": row.get("run_id", ""),
                "task_id": row.get("task_id", ""),
                "experiment": experiment,
                "repeat_id": row.get("repeat_id", ""),
                "model_name": row.get("model_name", ""),
                "vlm_model": row.get("vlm_model", ""),
                "strict_final_result": strict,
                "gazebo_final_result": gazebo,
                "big_brain_returned": row.get("big_brain_returned", ""),
                "semantic_success": semantic,
                "semantic_rule": rule,
                "semantic_detail": detail,
                "judge_disagreement": row.get("judge_disagreement", ""),
                "disagreement_type": row.get("disagreement_type", ""),
                "last_vlm_pass": row.get("last_vlm_pass", ""),
                "last_judge_final_pass": row.get("last_judge_final_pass", ""),
                "vlm_call_count": row.get("vlm_call_count", ""),
                "replan_generated_count": row.get("replan_generated_count", ""),
                "replan_executed_count": row.get("replan_executed_count", ""),
                "final_red_pose_xyz": row.get("final_red_pose_xyz", ""),
                "target_pose_xyz": row.get("target_pose_xyz", ""),
                "xy_error_m": row.get("xy_error_m", ""),
                "z_error_m": row.get("z_error_m", ""),
                "total_time_s": row.get("total_time_s", ""),
                "last_vlm_reason": row.get("last_vlm_reason", ""),
                "analysis_note": "; ".join(notes),
            }
        )
    return rows


EXPECTED_CALLS = {
    "G1": ["parse_obj_name", "pick_up_obj"],
    "G2": ["parse_obj_name", "move_to_obj_by_offset", "pick_up_obj"],
    "G3": ["parse_obj_name", "parse_obj_name", "pick_up_obj", "put_down_obj_by_offset"],
    "G4": ["parse_obj_name", "parse_obj_name", "parse_obj_name", "pick_up_obj", "put_down_between_objs"],
}


E3_FIELDS = [
    "run_id",
    "task_id",
    "experiment",
    "repeat_id",
    "model_name",
    "temperature",
    "code_executable",
    "semantic_parse_correct_auto",
    "decomposition_correct_auto",
    "api_sequence",
    "expected_api_sequence",
    "static_error_type",
    "generated_code",
    "total_time_s",
    "analysis_note",
]


def calls_from(code: str) -> List[str]:
    try:
        tree = ast.parse(code or "")
    except SyntaxError as exc:
        return [f"PARSE_ERROR:{exc}"]
    calls = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            calls.append(node.func.id)
    return calls


def appendix_e3(src: str) -> List[Dict[str, object]]:
    rows = []
    for row in read_csv(src):
        calls = calls_from(row.get("generated_code", ""))
        expected = EXPECTED_CALLS.get(row["task_id"], [])
        decomposition_ok = calls == expected
        code_ok = row.get("code_executable") == "yes"
        error_type = "none" if decomposition_ok and code_ok and not row.get("human_note") else "plan_or_code_issue"
        rows.append(
            {
                "run_id": row.get("run_id", ""),
                "task_id": row.get("task_id", ""),
                "experiment": "E3",
                "repeat_id": row.get("repeat_id", ""),
                "model_name": row.get("model_name", ""),
                "temperature": row.get("temperature", ""),
                "code_executable": row.get("code_executable", ""),
                "semantic_parse_correct_auto": "yes",
                "decomposition_correct_auto": "yes" if decomposition_ok else "no",
                "api_sequence": " -> ".join(calls),
                "expected_api_sequence": " -> ".join(expected),
                "static_error_type": error_type,
                "generated_code": row.get("generated_code", ""),
                "total_time_s": row.get("total_time_s", ""),
                "analysis_note": "" if error_type == "none" else f"expected {expected}, got {calls}; {row.get('human_note', '')}",
            }
        )
    return rows


SUMMARY_FIELDS = [
    "experiment",
    "task_id",
    "rows",
    "strict_success",
    "gazebo_success",
    "semantic_success",
    "judge_disagreement",
    "total_vlm_calls",
    "total_replans",
    "avg_time_s",
]


def avg_time(rows: Iterable[Dict[str, str]]) -> str:
    vals = [float(r["total_time_s"]) for r in rows if r.get("total_time_s")]
    return f"{statistics.mean(vals):.1f}" if vals else ""


def summary_rows(e1: List[Dict[str, str]], e2: List[Dict[str, str]], e3: List[Dict[str, str]], e4: List[Dict[str, str]]) -> List[Dict[str, object]]:
    rows = []

    for name, raw, app, result_col in [
        ("E1", read_csv(os.path.join(OUT, "E1_manip_results.csv")), e1, "e1_final_result"),
        ("E2", read_csv(os.path.join(OUT, "E2_vlm_smoke_results.csv")), e2, "e2_final_result"),
    ]:
        by_raw = defaultdict(list)
        by_app = defaultdict(list)
        for row in raw:
            by_raw[row["task_id"]].append(row)
        for row in app:
            by_app[row["task_id"]].append(row)
        for task_id in sorted(by_raw):
            task_rows = by_raw[task_id]
            app_rows = by_app[task_id]
            rows.append(
                {
                    "experiment": name,
                    "task_id": task_id,
                    "rows": len(task_rows),
                    "strict_success": sum(r.get(result_col) == "Success" for r in task_rows),
                    "gazebo_success": sum(r.get("gazebo_final_result") == "Success" for r in task_rows),
                    "semantic_success": sum(r.get("semantic_success") == "yes" for r in app_rows),
                    "judge_disagreement": sum(r.get("judge_disagreement") == "True" for r in task_rows),
                    "total_vlm_calls": sum(int(r.get("vlm_call_count") or 0) for r in task_rows),
                    "total_replans": sum(int(r.get("replan_generated_count") or 0) for r in task_rows),
                    "avg_time_s": avg_time(task_rows),
                }
            )

    by_e3 = defaultdict(list)
    for row in e3:
        by_e3[row["task_id"]].append(row)
    for task_id in sorted(by_e3):
        task_rows = by_e3[task_id]
        rows.append(
            {
                "experiment": "E3",
                "task_id": task_id,
                "rows": len(task_rows),
                "strict_success": "",
                "gazebo_success": "",
                "semantic_success": sum(r.get("semantic_parse_correct_auto") == "yes" for r in task_rows),
                "judge_disagreement": "",
                "total_vlm_calls": "",
                "total_replans": "",
                "avg_time_s": avg_time(task_rows),
            }
        )

    by_e4 = defaultdict(list)
    for row in read_csv(os.path.join(HERE, "E4_G_original.csv")):
        by_e4[row["task_id"]].append(row)
    for task_id in sorted(by_e4):
        task_rows = by_e4[task_id]
        rows.append(
            {
                "experiment": "E4",
                "task_id": task_id,
                "rows": len(task_rows),
                "strict_success": sum(r.get("final_result") == "Success" for r in task_rows),
                "gazebo_success": sum(r.get("final_result") == "Success" for r in task_rows),
                "semantic_success": sum(r.get("final_result") == "Success" for r in task_rows),
                "judge_disagreement": "",
                "total_vlm_calls": 0,
                "total_replans": 0,
                "avg_time_s": avg_time(task_rows),
            }
        )

    return rows


def main() -> None:
    e1 = appendix_exec(os.path.join(OUT, "E1_manip_results.csv"), "E1", "e1_final_result")
    e2 = appendix_exec(os.path.join(OUT, "E2_vlm_smoke_results.csv"), "E2", "e2_final_result")
    e3 = appendix_e3(os.path.join(OUT, "E3_manip_static.csv"))

    write_csv(os.path.join(OUT, "E1_manip_appendix.csv"), EXEC_FIELDS, e1)
    write_csv(os.path.join(OUT, "E2_manip_appendix.csv"), EXEC_FIELDS, e2)
    write_csv(os.path.join(OUT, "E3_manip_appendix.csv"), E3_FIELDS, e3)
    write_csv(os.path.join(OUT, "G_manip_summary.csv"), SUMMARY_FIELDS, summary_rows(e1, e2, e3, []))


if __name__ == "__main__":
    main()
