#!/usr/bin/env python3
"""
Create G1-G4 E1/E2/E3/E4 data pages by strictly referencing the author's CSVs.

Outputs:
  CSV pairs:
    pages_csv/E1_original.csv, pages_csv/E1_appendix.csv, ...
  One workbook with real sheets:
    G1_G4_E_pages.xlsx

The original pages preserve the author's headers and row structure for G1-G4.
The appendix pages add optional fields keyed by run_id.
"""

from __future__ import annotations

import csv
import os
import zipfile
from typing import Dict, Iterable, List, Sequence, Tuple
from xml.sax.saxutils import escape


PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
BIG_BRAIN_DATA = os.path.join(PROJECT_ROOT, "big_brain", "data")
HERE = os.path.dirname(os.path.abspath(__file__))
CSV_DIR = os.path.join(HERE, "pages_csv")
XLSX_OUT = os.path.join(HERE, "G1_G4_E_pages.xlsx")

TASK_IDS = {"G1", "G2", "G3", "G4"}

APPENDIX_FIELDS_COMMON = [
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
]

E_SPECIFIC_EXTRA = {
    "E1": [
        "first_attempt_code",
        "adjusted_code",
        "first_failure_reason",
        "adjust_success_delta",
    ],
    "E2": [
        "feedback_setting",
        "feedback_content",
        "no_feedback_result",
        "with_feedback_result",
        "feedback_helped",
    ],
    "E3": [
        "static_semantic_error_type",
        "static_decomposition_error_type",
        "invalid_api_name",
        "missing_required_step",
        "unsafe_or_irrelevant_step",
    ],
    "E4": [
        "manual_api_sequence",
        "service_success_sequence",
        "gazebo_before_state",
        "gazebo_after_state",
        "judge_rule",
    ],
}

RESET_ROBOT = "(0.0, 0.0, 0.05, 0.0)"
RED_NEAR = "(-0.35, -0.18, 0.025)"
RED_FAR = "(-2.20, -0.85, 0.025)"
RED_SIZE = "(0.03, 0.025, 0.05)"
BLUE_POSE = "(2.15, -0.20, 0.025)"
YELLOW_POSE = "(2.57, -0.20, 0.025)"
RESET_METHOD = "delete old red_block and spawn fresh red_block; blue/yellow unchanged"

TASK_INSTRUCTIONS = {
    "G1": "Pick up the red block",
    "G2": "Navigate to the red block on the ground and pick it up",
    "G3": "Put the held object on the CoffeeTable",
    "G4": "Put the held object between blue_block and yellow_block",
}


def read_author_rows(experiment: str) -> Tuple[List[str], List[Dict[str, str]]]:
    path = os.path.join(BIG_BRAIN_DATA, f"{experiment}.csv")
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = [row for row in reader if row.get("task_id") in TASK_IDS]
        fields = list(reader.fieldnames or [])
    if experiment == "E4" and not rows:
        rows = make_e4_g_rows(fields)
    return fields, rows


def make_e4_g_rows(fields: Sequence[str]) -> List[Dict[str, str]]:
    rows = []
    idx = 1
    for task_id in ["G1", "G2", "G3", "G4"]:
        for repeat_id in range(1, 4):
            row = {field: "" for field in fields}
            row.update(
                {
                    "run_id": f"E4_G{idx:03d}",
                    "task_id": task_id,
                    "task_type": "Manipulation",
                    "instruction": TASK_INSTRUCTIONS[task_id],
                    "method_type": "Manual",
                    "model_name": "None",
                    "temperature": "None",
                    "adjust_policy": "0",
                    "repeat_id": str(repeat_id),
                    "generated_code": "",
                    "human_note": "G1-G4 manual baseline generated from E4 original 30-column format.",
                }
            )
            rows.append(row)
            idx += 1
    return rows


def appendix_fields(experiment: str) -> List[str]:
    return APPENDIX_FIELDS_COMMON + E_SPECIFIC_EXTRA[experiment]


def task_expected_target(task_id: str) -> str:
    return {
        "G1": "red_block picked above ground",
        "G2": "red_block picked above ground after navigation",
        "G3": "red_block placed on CoffeeTable_01_001",
        "G4": "red_block placed at midpoint(blue_block,yellow_block)",
    }[task_id]


def task_target_pose(task_id: str) -> str:
    if task_id == "G1":
        return RED_NEAR
    if task_id == "G2":
        return RED_FAR
    if task_id == "G3":
        return "CoffeeTable_01_001 place point from gazebo_scene.yaml"
    return "midpoint of current blue_block and yellow_block poses"


def task_arm_services(task_id: str) -> str:
    return {
        "G1": "/arm/pick",
        "G2": "/arm/pick",
        "G3": "/arm/pick -> /arm/prepare_put -> /arm/drop",
        "G4": "/arm/pick -> /arm/prepare_put -> /arm/drop",
    }[task_id]


def task_nav_sequence(task_id: str) -> str:
    return {
        "G1": "none",
        "G2": "move_to_obj(red_block)",
        "G3": "move_to_obj(red_block) -> move_to_obj(CoffeeTable_01_001)",
        "G4": "move_to_obj(red_block) -> move_to_xy(midpoint blue/yellow approach)",
    }[task_id]


def why_useful(experiment: str) -> str:
    return {
        "E1": "Shows whether adjustment/replan rescued an initially failed manipulation plan.",
        "E2": "Separates no-feedback vs feedback behavior for the same manipulation task.",
        "E3": "Explains static plan/code errors without needing Gazebo execution.",
        "E4": "Provides objective Gazebo ground truth for manual/small-brain baseline.",
    }[experiment]


def make_appendix_row(experiment: str, original: Dict[str, str]) -> Dict[str, str]:
    task_id = original["task_id"]
    row = {field: "" for field in appendix_fields(experiment)}
    row.update(
        {
            "run_id": original.get("run_id", ""),
            "task_id": task_id,
            "task_type": original.get("task_type", "Manipulation"),
            "instruction": original.get("instruction", ""),
            "experiment": experiment,
            "repeat_id": original.get("repeat_id", ""),
            "model_name": original.get("model_name", ""),
            "temperature": original.get("temperature", ""),
            "adjust_policy": original.get("adjust_policy", ""),
            "reset_robot_pose_xyz_yaw": RESET_ROBOT,
            "reset_red_pose_xyz": RED_NEAR if task_id == "G1" else RED_FAR,
            "red_block_size_xyz": RED_SIZE,
            "blue_pose_xyz": BLUE_POSE if task_id == "G4" else "",
            "yellow_pose_xyz": YELLOW_POSE if task_id == "G4" else "",
            "expected_target": task_expected_target(task_id),
            "expected_target_pose_xyz": task_target_pose(task_id),
            "nav_goal_sequence": task_nav_sequence(task_id),
            "arm_services_used": task_arm_services(task_id),
            "object_reset_method": RESET_METHOD,
            "why_useful": why_useful(experiment),
        }
    )

    if experiment == "E1":
        row.update(
            {
                "first_attempt_code": "copy from generated_code attempt 1 if available",
                "adjusted_code": "copy from generated_code after adjustment if available",
                "first_failure_reason": "navigation / pick / put / judge / code_error",
                "adjust_success_delta": "success_after_adjust - success_before_adjust",
            }
        )
    elif experiment == "E2":
        row.update(
            {
                "feedback_setting": "adjust_policy 0=no feedback, 1=with feedback",
                "feedback_content": "short text/image/state feedback given to model",
                "feedback_helped": "yes/no/unclear",
            }
        )
    elif experiment == "E3":
        row.update(
            {
                "static_semantic_error_type": "wrong object / wrong relation / wrong coordinate / none",
                "static_decomposition_error_type": "missing step / wrong order / extra step / none",
                "invalid_api_name": "API name if generated code calls unsupported function",
                "missing_required_step": "e.g. missing pick before put",
                "unsafe_or_irrelevant_step": "e.g. unrelated navigation target",
            }
        )
    elif experiment == "E4":
        row.update(
            {
                "manual_api_sequence": task_nav_sequence(task_id) + " + " + task_arm_services(task_id),
                "service_success_sequence": "record each ROS service success/failure",
                "gazebo_before_state": "robot/red/blue/yellow poses after reset",
                "gazebo_after_state": "robot/red pose after task",
                "judge_rule": "G1/G2 red z > threshold; G3/G4 xy/z error within tolerance",
            }
        )
    return row


def write_csv(path: str, fields: Sequence[str], rows: Sequence[Dict[str, str]]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def table_to_rows(fields: Sequence[str], rows: Sequence[Dict[str, str]]) -> List[List[str]]:
    return [list(fields)] + [[str(row.get(field, "")) for field in fields] for row in rows]


def col_name(index: int) -> str:
    name = ""
    while index:
        index, rem = divmod(index - 1, 26)
        name = chr(65 + rem) + name
    return name


def sheet_xml(rows: List[List[str]]) -> str:
    out = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>']
    out.append('<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">')
    out.append("<sheetData>")
    for r_idx, row in enumerate(rows, start=1):
        out.append(f'<row r="{r_idx}">')
        for c_idx, value in enumerate(row, start=1):
            ref = f"{col_name(c_idx)}{r_idx}"
            text = escape(value)
            out.append(f'<c r="{ref}" t="inlineStr"><is><t>{text}</t></is></c>')
        out.append("</row>")
    out.append("</sheetData></worksheet>")
    return "".join(out)


def write_xlsx(path: str, sheets: List[Tuple[str, List[List[str]]]]) -> None:
    content_types = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>']
    content_types.append('<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">')
    content_types.append('<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>')
    content_types.append('<Default Extension="xml" ContentType="application/xml"/>')
    content_types.append('<Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>')
    for idx, _ in enumerate(sheets, start=1):
        content_types.append(
            f'<Override PartName="/xl/worksheets/sheet{idx}.xml" '
            'ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>'
        )
    content_types.append("</Types>")

    rels = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>'
        "</Relationships>"
    )

    workbook = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>']
    workbook.append(
        '<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
        'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"><sheets>'
    )
    wb_rels = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>']
    wb_rels.append('<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">')
    for idx, (name, _) in enumerate(sheets, start=1):
        workbook.append(f'<sheet name="{escape(name)}" sheetId="{idx}" r:id="rId{idx}"/>')
        wb_rels.append(
            f'<Relationship Id="rId{idx}" '
            'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" '
            f'Target="worksheets/sheet{idx}.xml"/>'
        )
    workbook.append("</sheets></workbook>")
    wb_rels.append("</Relationships>")

    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("[Content_Types].xml", "".join(content_types))
        zf.writestr("_rels/.rels", rels)
        zf.writestr("xl/workbook.xml", "".join(workbook))
        zf.writestr("xl/_rels/workbook.xml.rels", "".join(wb_rels))
        for idx, (_, rows) in enumerate(sheets, start=1):
            zf.writestr(f"xl/worksheets/sheet{idx}.xml", sheet_xml(rows))


def main() -> int:
    os.makedirs(CSV_DIR, exist_ok=True)
    sheets: List[Tuple[str, List[List[str]]]] = []

    for experiment in ["E1", "E2", "E3", "E4"]:
        original_fields, original_rows = read_author_rows(experiment)
        appendix = [make_appendix_row(experiment, row) for row in original_rows]
        app_fields = appendix_fields(experiment)

        original_csv = os.path.join(CSV_DIR, f"{experiment}_original.csv")
        appendix_csv = os.path.join(CSV_DIR, f"{experiment}_appendix.csv")
        write_csv(original_csv, original_fields, original_rows)
        write_csv(appendix_csv, app_fields, appendix)

        sheets.append((f"{experiment}_original", table_to_rows(original_fields, original_rows)))
        sheets.append((f"{experiment}_appendix", table_to_rows(app_fields, appendix)))
        print(f"{experiment}: {len(original_rows)} original rows, {len(appendix)} appendix rows")

    write_xlsx(XLSX_OUT, sheets)
    print(f"wrote workbook: {XLSX_OUT}")
    print(f"wrote csv pages: {CSV_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
