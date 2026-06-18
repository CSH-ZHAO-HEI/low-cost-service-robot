#!/usr/bin/env python3
"""
E1 main LLM+Judge/VLM data collection for composite tasks C1-C4.

E1 (per plan.md):
  - LLM + Judge/VLM + Gazebo execution
  - adjust_policy = 2 (up to 2 replans)
  - Default repeats = 5

This reuses run_e2_composite.run_one(...) for the heavy lifting and only
forces adjust_policy=2, repeats=5, experiment label = E1, output paths.

Outputs:
  C1-C4/outputs/C-E1.csv
  C1-C4/outputs/C-E1-appendix.csv

Usage:
  python3 C1-C4/run_e1_composite.py --tasks C1 C2 C3 C4 --repeats 5
  python3 C1-C4/run_e1_composite.py --tasks C1 --repeats 5 --start-writer
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
import traceback
from typing import Dict, List


HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import run_e2_composite as e2c     # noqa: E402  (shared run_one + fields + judge)


OUTPUT_DIR       = os.path.join(HERE, "outputs")
DEFAULT_ORIGINAL = os.path.join(OUTPUT_DIR, "C-E1.csv")
DEFAULT_APPENDIX = os.path.join(OUTPUT_DIR, "C-E1-appendix.csv")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tasks", nargs="+",
                        choices=["C1", "C2", "C3", "C4"],
                        default=["C1", "C2", "C3", "C4"])
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--adjust-policy", type=int, default=2,
                        help="E1 default is 2 replans")
    parser.add_argument("--temperature", type=float, default=0.0)
    parser.add_argument("--output-original", default=DEFAULT_ORIGINAL)
    parser.add_argument("--output-appendix", default=DEFAULT_APPENDIX)
    parser.add_argument("--append", action="store_true")
    parser.add_argument("--start-writer", action="store_true")
    parser.add_argument("--vlm-topic", default=e2c.DEFAULT_VLM_TOPIC)
    parser.add_argument("--allow-stale-image", action="store_true")
    parser.add_argument("--save-history", action="store_true")
    parser.add_argument("--continue-on-error", action="store_true")
    parser.add_argument("--pause-after-reset", type=float, default=0.0)
    parser.add_argument("--no-helper-prompt", action="store_true",
                        help="Strip C1-C4 tuned-helper examples from BASE_PROMPT so the LLM "
                             "composes from primitives. Produces adjust/replan data like G-series.")
    parser.add_argument("--c1-block-variant", action="store_true",
                        help="Use red_block instead of Coke for C1.")
    args = parser.parse_args()

    # Apply no-helper patch BEFORE BigBrain is created
    if args.no_helper_prompt:
        e2c.patch_base_prompt_remove_c_helpers()
        if args.output_original == DEFAULT_ORIGINAL:
            args.output_original = os.path.join(OUTPUT_DIR, "C-E1-no-helper.csv")
        if args.output_appendix == DEFAULT_APPENDIX:
            args.output_appendix = os.path.join(OUTPUT_DIR, "C-E1-no-helper-appendix.csv")
        print(f"[no-helper] output_original={args.output_original}")
        print(f"[no-helper] output_appendix={args.output_appendix}")

    if args.c1_block_variant:
        e2c.TASK_INSTRUCTIONS["C1"] = e2c.C1_BLOCK_VARIANT_INSTRUCTION
        if args.output_original.endswith("C-E1.csv"):
            args.output_original = os.path.join(OUTPUT_DIR, "C-E1-c1block.csv")
        if args.output_appendix.endswith("C-E1-appendix.csv"):
            args.output_appendix = os.path.join(OUTPUT_DIR, "C-E1-c1block-appendix.csv")
        print(f"[c1-block-variant] C1 → red_block; outputs={args.output_original}")

    # Import-path sanity
    comp_path = os.path.abspath(e2c.comp.__file__)
    if not comp_path.startswith(os.path.abspath(HERE)):
        print(f"[WARNING] composite_capability_test imported from {comp_path}; "
              f"expected under {HERE}")
    else:
        print(f"[import] composite_capability_test = {comp_path}")

    writer_proc = None
    if args.start_writer:
        writer_proc = e2c.start_vlm_writer(args.vlm_topic)
        time.sleep(1.0)

    try:
        age = e2c.wait_fresh_image(timeout_s=5.0)
        if age > 3.0 and not args.allow_stale_image:
            print(f"[ERROR] VLM image is stale: age={age:.1f}s "
                  "(start the writer or use --allow-stale-image)")
            return 2
        print(f"[image] current age={age:.1f}s")

        tester = e2c.comp.CompositeTester(pause_after_reset=args.pause_after_reset)
        e2c.reenable_real_judge(max_replan_times=args.adjust_policy)
        brain = e2c.BigBrain()
        if not args.save_history:
            brain._save_history = lambda path: None

        originals: List[Dict] = []
        appendices: List[Dict] = []
        run_index = 1
        for task_id in args.tasks:
            for repeat_id in range(1, args.repeats + 1):
                run_id = f"E1_C{run_index:03d}"
                try:
                    o, a = e2c.run_one(run_id, task_id, repeat_id,
                                       args.adjust_policy, tester, brain,
                                       experiment="E1",
                                       temperature=args.temperature)
                except Exception as exc:
                    print(f"[run_one] outer exception for {task_id} r{repeat_id}: {exc}")
                    traceback.print_exc()
                    o = {f: "N/A" for f in e2c.ORIGINAL_FIELDS}
                    o.update({
                        "run_id": run_id, "task_id": task_id, "task_type": "Composite",
                        "instruction": e2c.TASK_INSTRUCTIONS[task_id],
                        "method_type": "LLM",
                        "model_name": e2c.TARGET_LLM_MODEL,
                        "temperature": f"{args.temperature:.2f}",
                        "adjust_policy": args.adjust_policy, "repeat_id": repeat_id,
                        "final_result": "Fail",
                        "failure_reason": f"outer:{exc}",
                        "vlm_summary": "outer exception",
                        "human_note": "outer exception",
                    })
                    a = {f: "N/A" for f in e2c.APPENDIX_FIELDS}
                    a.update({
                        "run_id": run_id, "task_id": task_id,
                        "experiment": "E1",
                        "repeat_id": repeat_id,
                        "instruction": e2c.TASK_INSTRUCTIONS[task_id],
                        "method_type": "LLM",
                        "model_name": e2c.TARGET_LLM_MODEL,
                        "temperature": f"{args.temperature:.2f}",
                        "adjust_policy": args.adjust_policy,
                        "judge_rule": e2c.judge_rule_for(task_id),
                        "failure_stage": "outer",
                        "failure_type": "execution_failed",
                        "human_note": f"outer exception: {exc}",
                    })
                originals.append(o)
                appendices.append(a)
                run_index += 1

        e2c.write_csv(args.output_original, e2c.ORIGINAL_FIELDS, originals, args.append)
        e2c.write_csv(args.output_appendix, e2c.APPENDIX_FIELDS, appendices, args.append)

        ok = sum(1 for r in originals if r.get("final_result") == "Success")
        print(f"\n[summary] Success {ok}/{len(originals)}")
        return 0 if ok == len(originals) else 1
    finally:
        if writer_proc is not None:
            writer_proc.terminate()
            try:
                writer_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                writer_proc.kill()


if __name__ == "__main__":
    raise SystemExit(main())
