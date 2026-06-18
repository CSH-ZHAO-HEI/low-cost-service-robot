#!/usr/bin/env python3
"""Collect E1 manipulation data for G1-G4.

E1 here is the main LLM+Judge/VLM execution run with adjust_policy=2.
It reuses the validated E2 runner plumbing, but installs JudgeLLM with two
allowed replans and writes a separate E1 CSV.
"""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import time
from typing import Dict, List


HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
BIG_BRAIN_DIR = os.path.join(PROJECT_ROOT, "big_brain")
OUT_DIR = os.path.join(HERE, "outputs")
DEFAULT_CSV = os.path.join(OUT_DIR, "E1_manip_results.csv")

if HERE not in sys.path:
    sys.path.insert(0, HERE)
if BIG_BRAIN_DIR not in sys.path:
    sys.path.insert(0, BIG_BRAIN_DIR)

import test_e2_vlm_manip as e2  # noqa: E402
from model.llm import JudgeLLM  # noqa: E402


FIELDS = [
    "experiment",
    "adjust_policy",
    "e1_final_result",
    *e2.FIELDS,
]


def install_e1_judge(max_replan_times: int) -> None:
    e2.cap.robot_api.judge_llm = JudgeLLM()
    e2.cap.robot_api.judge_llm.max_replan_times = max_replan_times


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
    parser.add_argument("--tasks", nargs="+", choices=["G1", "G2", "G3", "G4"], default=["G1", "G2", "G3", "G4"])
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--adjust-policy", type=int, default=2)
    parser.add_argument("--output", default=DEFAULT_CSV)
    parser.add_argument("--append", action="store_true")
    parser.add_argument("--start-writer", action="store_true")
    parser.add_argument("--vlm-topic", default=e2.DEFAULT_VLM_TOPIC)
    parser.add_argument("--allow-stale-image", action="store_true")
    parser.add_argument("--save-history", action="store_true")
    args = parser.parse_args()

    e2.reenable_real_judge = lambda: install_e1_judge(args.adjust_policy)

    writer_proc = None
    if args.start_writer:
        writer_proc = e2.start_vlm_writer(args.vlm_topic)
        time.sleep(1.0)

    try:
        age = e2.wait_fresh_image(timeout_s=5.0)
        if age > 3.0 and not args.allow_stale_image:
            print(f"[ERROR] VLM image is stale: age={age:.1f}s")
            print("        Start the writer or rerun with --start-writer.")
            return 2
        print(f"[image] current age={age:.1f}s")

        tester = e2.cap.ManipTester()
        install_e1_judge(args.adjust_policy)
        brain = e2.BigBrain()
        if not args.save_history:
            brain._save_history = lambda path: None

        rows: List[Dict[str, object]] = []
        for task_id in args.tasks:
            for repeat_id in range(1, args.repeats + 1):
                row = e2.run_one(task_id, repeat_id, tester, brain)
                row["experiment"] = "E1"
                row["adjust_policy"] = args.adjust_policy
                row["e1_final_result"] = row["e2_final_result"]
                row["run_id"] = f"E1_VLM_{task_id}_r{repeat_id:02d}"
                rows.append(row)

        write_csv(args.output, rows, args.append)
        return 0 if all(row["e1_final_result"] == "Success" for row in rows) else 1
    finally:
        if writer_proc is not None:
            writer_proc.terminate()
            try:
                writer_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                writer_proc.kill()


if __name__ == "__main__":
    raise SystemExit(main())
