#!/usr/bin/env python3
"""Collect E3 static planning data for manipulation tasks G1-G4.

E3 only asks the planner LLM to generate code. It does not move the robot,
does not need Gazebo, and does not call VLM.
"""

from __future__ import annotations

import argparse
import ast
import csv
import json
import os
import sys
import time
from typing import Dict, List

from openai import OpenAI


HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
BIG_BRAIN_DIR = os.path.join(PROJECT_ROOT, "big_brain")
OUT_DIR = os.path.join(HERE, "outputs")
DEFAULT_CSV = os.path.join(OUT_DIR, "E3_manip_static.csv")
DEFAULT_JSON = os.path.join(OUT_DIR, "E3_manip_static.json")

if BIG_BRAIN_DIR not in sys.path:
    sys.path.insert(0, BIG_BRAIN_DIR)

import config as cfg  # noqa: E402
from model.rag import RAGManager  # noqa: E402
from prompt.task_prompt import BASE_PROMPT  # noqa: E402
from utils.utils import extract_code  # noqa: E402


TASKS = {
    "G1": ("Manipulation", "Pick up the red block"),
    "G2": ("Manipulation", "Navigate to the red block on the ground and pick it up"),
    "G3": ("Manipulation", "Pick up the red block and put it on the CoffeeTable"),
    "G4": ("Manipulation", "Pick up the red block and put it between blue_block and yellow_block"),
}

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
    "total_time_s",
    "human_note",
]


def model_runtime(model_label: str) -> tuple[str, str, str]:
    label = model_label.lower()
    if label in {"flash", "deepseek-flash", "deepseek-chat"}:
        return cfg.DEEPSEEK_API_KEY, cfg.DEEPSEEK_BASE_URL, cfg.DEEPSEEK_LLM_MODEL
    if label in {"pro", "deepseek-pro", "deepseek-v4-pro"}:
        return cfg.DEEPSEEK_API_KEY, cfg.DEEPSEEK_BASE_URL, "deepseek-v4-pro"
    if label in {"task", "default"}:
        return cfg.TASK_LLM_API_KEY, cfg.TASK_LLM_BASE_URL, cfg.TASK_LLM_MODEL
    return cfg.TASK_LLM_API_KEY, cfg.TASK_LLM_BASE_URL, model_label


def code_executable(code: str) -> str:
    if not code.strip():
        return "no"
    try:
        ast.parse(code)
        return "yes"
    except SyntaxError:
        return "no"


def build_prompt(instruction: str, rag: RAGManager | None) -> tuple[str, str]:
    rag_context = rag.retrieve(instruction) if rag is not None else ""
    final_prompt = BASE_PROMPT + "\n"
    if rag_context:
        final_prompt += (
            "\n# === Reference: a past similar task (FOR INSPIRATION ONLY) ===\n"
            "# IMPORTANT: variables defined in the reference are NOT available in the current scope.\n"
            "# You must define all needed variables yourself.\n"
            + rag_context.strip()
            + "\n# === End of reference. Now solve the FOLLOWING NEW task: ===\n"
        )
    final_prompt += f"# {instruction}\n?"
    return final_prompt, rag_context


def run_one(task_id: str, model_label: str, temperature: float, repeat_id: int, rag: RAGManager | None) -> Dict[str, object]:
    task_type, instruction = TASKS[task_id]
    api_key, base_url, model_name = model_runtime(model_label)
    prompt, rag_context = build_prompt(instruction, rag)

    print(f"\n========== E3 {task_id} {model_name} temp={temperature} repeat {repeat_id} ==========")
    if rag_context:
        print("[rag] matched reference")

    t0 = time.time()
    human_note = ""
    raw_text = ""
    generated_code = ""
    try:
        client = OpenAI(api_key=api_key, base_url=base_url)
        response = client.chat.completions.create(
            model=model_name,
            messages=[
                {"role": "system", "content": "you only need to use code to answer the ? part and nothing else"},
                {"role": "user", "content": prompt},
            ],
            temperature=float(temperature),
            top_p=None,
        )
        raw_text = response.choices[0].message.content or ""
        last_line = rag_context.strip().splitlines()[-1] if rag_context.strip() else ""
        generated_code = extract_code(raw_text, last_line)
    except Exception as exc:
        human_note = f"generation_error: {type(exc).__name__}: {exc}"
    total_time = time.time() - t0

    print(generated_code or "<empty>")
    if human_note:
        print(f"[note] {human_note}")

    return {
        "run_id": f"E3_G_{task_id}_{model_name}_t{temperature}_r{repeat_id:02d}",
        "task_id": task_id,
        "task_type": task_type,
        "instruction": instruction,
        "model_name": model_name,
        "temperature": temperature,
        "repeat_id": repeat_id,
        "generated_code": generated_code,
        "semantic_parse_correct": "",
        "decomposition_correct": "",
        "code_executable": code_executable(generated_code),
        "total_time_s": f"{total_time:.1f}",
        "human_note": human_note,
    }


def write_outputs(csv_path: str, json_path: str, rows: List[Dict[str, object]], append: bool) -> None:
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    existing: List[Dict[str, object]] = []
    if append and os.path.exists(csv_path):
        with open(csv_path, newline="", encoding="utf-8") as f:
            existing = list(csv.DictReader(f))
    all_rows = existing + rows
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(all_rows)
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump({"records": all_rows}, f, ensure_ascii=False, indent=2)
    print(f"[csv] wrote {len(all_rows)} rows: {csv_path}")
    print(f"[json] wrote {len(all_rows)} rows: {json_path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tasks", nargs="+", choices=["G1", "G2", "G3", "G4"], default=["G1", "G2", "G3", "G4"])
    parser.add_argument("--models", nargs="+", default=["flash"], help="flash, pro, task, or an explicit model name.")
    parser.add_argument("--temperatures", nargs="+", type=float, default=[0.0])
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--output", default=DEFAULT_CSV)
    parser.add_argument("--json-output", default=DEFAULT_JSON)
    parser.add_argument("--append", action="store_true")
    parser.add_argument("--no-rag", action="store_true")
    args = parser.parse_args()

    rag = None
    if not args.no_rag:
        history_path = os.path.join(BIG_BRAIN_DIR, "memory", "rag_history.json")
        with open(history_path, "r", encoding="utf-8") as f:
            rag = RAGManager(json.load(f))

    rows: List[Dict[str, object]] = []
    for task_id in args.tasks:
        for model_label in args.models:
            for temperature in args.temperatures:
                for repeat_id in range(1, args.repeats + 1):
                    rows.append(run_one(task_id, model_label, temperature, repeat_id, rag))

    write_outputs(args.output, args.json_output, rows, args.append)
    return 0 if all(not row["human_note"] for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
