#!/usr/bin/env python3
"""
DeepSeek V4 Pro E3-only static planning data for N1-N4.

This script only collects E3 static planning records:
  - no Gazebo
  - no VLM
  - no execution

Default repeat count follows the original E3 design: 3 repeats.

Usage:
  python3 N1-N4/run_pro_e3_nav.py
  python3 N1-N4/run_pro_e3_nav.py --repeats 3

Output:
  N1-N4/outputs/E3_pro.csv
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import sys
import time
from typing import Dict, List

from openai import OpenAI


PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
BIG_BRAIN_DIR = os.path.join(PROJECT_ROOT, "big_brain")
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "N1-N4", "outputs")

if BIG_BRAIN_DIR not in sys.path:
    sys.path.insert(0, BIG_BRAIN_DIR)

import config as cfg  # noqa: E402


MODEL_LABEL = "deepseek-v4-pro"
MODEL_NAME = "deepseek-v4-pro"
TEMPERATURE = 0.0
DEFAULT_REPEATS = 3
LLM_TIMEOUT_S = 60
LLM_RETRIES = 2

TASKS = [
    ("N1", "Navigation", "Navigate to (1.0, 2.0)"),
    ("N2", "Navigation", "Navigate next to the trash can"),
    ("N3", "Navigation", "Move around the sofa in a 4x4 meter square"),
    ("N4", "Navigation", "Go to the trash can with the smallest |x|+|y|, then return to the starting point"),
]

FIELDS = [
    "run_id",
    "task_id",
    "task_type",
    "instruction",
    "model_name",
    "temperature",
    "repeat_id",
    "generated_code",
    "semantic_parse_correct",
    "decomposition_correct",
    "code_executable",
    "human_note",
]

PROMPT = """
You generate Python code for a mobile robot navigation API.
Return ONLY executable Python code, no markdown and no explanation.

Available functions:
- move_to_xy(x, y)
- move_to_obj_by_offset(obj, dx, dy)
- parse_obj_name(text, objects)  # returns a STRING object name. Do NOT index it with [0].
- get_obj_xy(obj)
- get_robot_pos()

Available objects dictionary:
objects = {{
    "trash": ["Trash_01_001", "Trash_01_002"],
    "sofa": ["SofaC_01_001"]
}}

Rules:
- All coordinates and distances are meters.
- Do not import modules.
- Do not define functions without calling them.
- Only solve the current task, do not include other tasks.
- N1 must call move_to_xy(1.0, 2.0).
- N2 should select a trash/bin object and move next to it.
- N3 should move around the sofa in a 4x4 meter square: use sofa center, half-size 2.0m, and visit 5 waypoints to close the square.
- N4 should record the start position, select the trash object with the smallest abs(x)+abs(y), move next to it, then return to the start position.

Task:
{instruction}
"""


def extract_code(text: str) -> str:
    match = re.search(r"```(?:python)?(.*?)```", text, re.DOTALL)
    return match.group(1).strip() if match else text.strip()


def call_deepseek(instruction: str) -> str:
    client = OpenAI(
        api_key=cfg.DEEPSEEK_API_KEY,
        base_url=cfg.DEEPSEEK_BASE_URL,
        timeout=LLM_TIMEOUT_S,
        max_retries=0,
    )
    last_error = None
    for attempt in range(1, LLM_RETRIES + 1):
        try:
            print(f"[llm] {MODEL_LABEL} request attempt {attempt}/{LLM_RETRIES}", flush=True)
            response = client.chat.completions.create(
                model=MODEL_NAME,
                messages=[
                    {"role": "system", "content": "Return only Python code."},
                    {"role": "user", "content": PROMPT.format(instruction=instruction)},
                ],
                temperature=TEMPERATURE,
            )
            print(f"[llm] {MODEL_LABEL} response received", flush=True)
            return extract_code(response.choices[0].message.content)
        except Exception as e:
            last_error = e
            print(f"[llm] {MODEL_LABEL} request failed: {e}", flush=True)
            if attempt < LLM_RETRIES:
                time.sleep(2.0)
    raise RuntimeError(f"{MODEL_LABEL} failed after {LLM_RETRIES} attempts: {last_error}")


def static_check(task_id: str, code: str) -> Dict[str, str]:
    c = code.lower()
    syntax_ok = True
    try:
        compile(code, "<generated_code>", "exec")
    except SyntaxError:
        syntax_ok = False

    semantic = False
    decomposition = False
    note = ""
    exact_n1_move = bool(re.search(r"move_to_xy\s*\(\s*1(?:\.0)?\s*,\s*2(?:\.0)?\s*\)", c))
    nav_count = c.count("move_to_xy") + c.count("move_to_obj_by_offset")
    has_unrelated_n1_move = task_id != "N1" and exact_n1_move

    if task_id == "N1":
        semantic = exact_n1_move
        decomposition = semantic and nav_count == 1
    elif task_id == "N2":
        semantic = ("trash" in c or "bin" in c) and ("table" not in c or "trash" in c)
        navigates_to_trash = "move_to_obj_by_offset" in c or ("get_obj_xy" in c and "move_to_xy" in c)
        decomposition = semantic and navigates_to_trash and not has_unrelated_n1_move
    elif task_id == "N3":
        semantic = "sofa" in c
        has_square_size = (
            "2.0" in c or "4x4" in c or "4" in c
            or "cx-2" in c or "cx + 2" in c or "cx+2" in c
            or "half" in c
        )
        follows_square = "move_to_xy" in c and (
            c.count("move_to_xy") >= 4 or "for " in c
        ) and has_square_size
        decomposition = semantic and follows_square and not has_unrelated_n1_move
    elif task_id == "N4":
        semantic = ("trash" in c or "bin" in c) and "abs" in c
        returns_to_start = (
            re.search(r"move_to_xy\s*\(\s*start(?:_pos)?\s*\[\s*0\s*\]", c)
            or re.search(r"move_to_xy\s*\(\s*start_x\s*,\s*start_y\s*\)", c)
        )
        no_circle_plan = "linspace" not in c and "for angle" not in c and "cos(" not in c and "sin(" not in c
        decomposition = (
            semantic
            and ("start" in c or "get_robot_pos" in c)
            and nav_count >= 2
            and returns_to_start
            and no_circle_plan
            and not has_unrelated_n1_move
        )

    if not semantic:
        note = "semantic rule failed"
    elif has_unrelated_n1_move:
        note = "decomposition rule failed: contains unrelated N1 move"
    elif not decomposition:
        note = "decomposition rule failed"
    elif not syntax_ok:
        note = "syntax check failed"

    return {
        "semantic_parse_correct": "yes" if semantic else "no",
        "decomposition_correct": "yes" if decomposition else "no",
        "code_executable": "yes" if syntax_ok else "no",
        "human_note": note,
    }


def write_csv(path: str, rows: List[dict]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    print(f"[csv] wrote {len(rows)} rows: {path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repeats", type=int, default=DEFAULT_REPEATS)
    args = parser.parse_args()

    rows = []
    run_index = 1
    for task_id, task_type, instruction in TASKS:
        for repeat_id in range(1, args.repeats + 1):
            print(f"[E3 pro] {task_id} repeat {repeat_id}")
            try:
                code = call_deepseek(instruction)
                checks = static_check(task_id, code)
            except Exception as e:
                code = ""
                checks = {
                    "semantic_parse_correct": "no",
                    "decomposition_correct": "no",
                    "code_executable": "no",
                    "human_note": f"LLM call failed: {e}",
                }
            rows.append(
                {
                    "run_id": f"E3_{run_index:03d}",
                    "task_id": task_id,
                    "task_type": task_type,
                    "instruction": instruction,
                    "model_name": MODEL_LABEL,
                    "temperature": TEMPERATURE,
                    "repeat_id": repeat_id,
                    "generated_code": code,
                    **checks,
                }
            )
            run_index += 1

    write_csv(os.path.join(OUTPUT_DIR, "E3_pro.csv"), rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
