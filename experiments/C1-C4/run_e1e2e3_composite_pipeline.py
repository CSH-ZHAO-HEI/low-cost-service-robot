#!/usr/bin/env python3
"""Run C-series E1/E2/E3 experiments sequentially.

Running composite experiments in parallel would contaminate Gazebo, arm state,
VLM image state, and JudgeLLM logs, so this wrapper always runs one step at a
time.

Example:
  python3 C1-C4/run_e1e2e3_composite_pipeline.py \
      --tasks C1 C2 C3 C4 \
      --order e1 e2 e3 \
      --e1-repeats 5 \
      --e2-repeats 3 \
      --e3-repeats 3 \
      --e3-models flash pro \
      --e3-temperatures 0.0 0.4 0.8 \
      --start-writer \
      --continue-on-error
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from typing import List


HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(HERE)


def run_step(name: str, cmd: List[str], continue_on_error: bool) -> int:
    print("\n" + "=" * 80)
    print(f"[pipeline] {name}")
    print("[pipeline] " + " ".join(cmd))
    print("=" * 80)
    result = subprocess.run(cmd, cwd=PROJECT_ROOT)
    if result.returncode != 0:
        print(f"[pipeline] {name} exited with code {result.returncode}")
        if not continue_on_error:
            raise SystemExit(result.returncode)
    return result.returncode


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tasks", nargs="+", choices=["C1", "C2", "C3", "C4"], default=["C1", "C2", "C3", "C4"])
    parser.add_argument("--order", nargs="+", choices=["e1", "e2", "e3"], default=["e1", "e2", "e3"])
    parser.add_argument("--e1-repeats", type=int, default=5)
    parser.add_argument("--e2-repeats", type=int, default=3)
    parser.add_argument("--e3-repeats", type=int, default=3)
    parser.add_argument("--e1-adjust-policy", type=int, default=2,
                        help="E1's adjust_policy (max_replan_times). Default 2.")
    parser.add_argument("--e2-adjust-policies", nargs="+", default=["0", "1"])
    parser.add_argument("--e3-models", nargs="+", default=["flash", "pro"])
    parser.add_argument("--e3-temperatures", nargs="+", default=["0.0", "0.4", "0.8"])
    parser.add_argument("--start-writer", action="store_true")
    parser.add_argument("--allow-stale-image", action="store_true")
    parser.add_argument("--save-history", action="store_true")
    parser.add_argument("--continue-on-error", action="store_true")
    parser.add_argument("--pause-after-reset", type=float, default=0.0)
    parser.add_argument("--no-helper-prompt", action="store_true",
                        help="Forward --no-helper-prompt to all selected sub-runners; "
                             "produces adjust/replan data by removing C-helper hints from BASE_PROMPT.")
    parser.add_argument("--c1-block-variant", action="store_true",
                        help="Forward --c1-block-variant to E1/E2 (uses red_block instead of Coke for C1).")
    args = parser.parse_args()

    py = sys.executable
    commands = {
        "e1": [
            py,
            os.path.join(HERE, "run_e1_composite.py"),
            "--tasks",
            *args.tasks,
            "--repeats",
            str(args.e1_repeats),
            "--adjust-policy",
            str(args.e1_adjust_policy),
            "--pause-after-reset",
            str(args.pause_after_reset),
        ],
        "e2": [
            py,
            os.path.join(HERE, "run_e2_composite.py"),
            "--tasks",
            *args.tasks,
            "--repeats",
            str(args.e2_repeats),
            "--adjust-policies",
            *args.e2_adjust_policies,
            "--pause-after-reset",
            str(args.pause_after_reset),
        ],
        "e3": [
            py,
            os.path.join(HERE, "run_e3_composite.py"),
            "--tasks",
            *args.tasks,
            "--models",
            *args.e3_models,
            "--temperatures",
            *args.e3_temperatures,
            "--repeats",
            str(args.e3_repeats),
        ],
    }
    names = {
        "e1": "C E1 LLM+VLM execution adjust=2",
        "e2": "C E2 adjust ablation adjust=0/1",
        "e3": "C E3 static planning",
    }

    for key in ("e1", "e2"):
        if key in args.order:
            if args.start_writer:
                commands[key].append("--start-writer")
            if args.allow_stale_image:
                commands[key].append("--allow-stale-image")
            if args.save_history:
                commands[key].append("--save-history")
            if args.continue_on_error:
                commands[key].append("--continue-on-error")

    if args.continue_on_error and "e3" in args.order:
        commands["e3"].append("--continue-on-error")

    if args.no_helper_prompt:
        for key in args.order:
            commands[key].append("--no-helper-prompt")

    if args.c1_block_variant:
        for key in ("e1", "e2"):
            if key in args.order:
                commands[key].append("--c1-block-variant")

    errors = 0
    for key in args.order:
        errors += 1 if run_step(names[key], commands[key], args.continue_on_error) else 0

    print("\n[pipeline] done")
    return 0 if errors == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
