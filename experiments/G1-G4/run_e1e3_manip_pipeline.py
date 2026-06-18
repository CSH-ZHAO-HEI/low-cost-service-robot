#!/usr/bin/env python3
"""Run G-series E1/E2/E3 experiments sequentially.

This is intentionally sequential. Running manipulation experiments in parallel
would make Gazebo, arm state, and VLM image state contaminate each other.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys


HERE = os.path.dirname(os.path.abspath(__file__))


def run_step(name: str, cmd: list[str], continue_on_error: bool) -> int:
    print("\n" + "=" * 80)
    print(f"[pipeline] {name}")
    print("[pipeline] " + " ".join(cmd))
    print("=" * 80)
    result = subprocess.run(cmd, cwd=os.path.dirname(HERE))
    if result.returncode != 0:
        print(f"[pipeline] {name} exited with code {result.returncode}")
        if not continue_on_error:
            raise SystemExit(result.returncode)
    return result.returncode


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tasks", nargs="+", choices=["G1", "G2", "G3", "G4"], default=["G1", "G2", "G3", "G4"])
    parser.add_argument("--e3-repeats", type=int, default=3)
    parser.add_argument("--e1-repeats", type=int, default=5)
    parser.add_argument("--e2-repeats", type=int, default=3)
    parser.add_argument("--e3-models", nargs="+", default=["flash"])
    parser.add_argument("--e3-temperatures", nargs="+", default=["0.0"])
    parser.add_argument("--start-writer", action="store_true")
    parser.add_argument("--include-e2", action="store_true")
    parser.add_argument(
        "--order",
        nargs="+",
        choices=["e1", "e2", "e3"],
        default=["e3", "e1"],
        help="Sequential experiment order. Example: --order e1 e2 e3",
    )
    parser.add_argument("--continue-on-error", action="store_true")
    args = parser.parse_args()

    py = sys.executable
    errors = 0

    commands = {
        "e1": [
            py,
            os.path.join(HERE, "run_e1_manip.py"),
            "--tasks",
            *args.tasks,
            "--repeats",
            str(args.e1_repeats),
        ],
        "e2": [
            py,
            os.path.join(HERE, "test_e2_vlm_manip.py"),
            "--tasks",
            *args.tasks,
            "--repeats",
            str(args.e2_repeats),
        ],
        "e3": [
            py,
            os.path.join(HERE, "run_e3_manip.py"),
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
        "e1": "E1 LLM+VLM execution",
        "e2": "E2 VLM smoke",
        "e3": "E3 static planning",
    }

    if args.include_e2 and "e2" not in args.order:
        args.order.append("e2")

    for key in ("e1", "e2"):
        if args.start_writer and key in args.order:
            commands[key].append("--start-writer")

    for key in args.order:
        errors += 1 if run_step(names[key], commands[key], args.continue_on_error) else 0

    print("\n[pipeline] done")
    return 0 if errors == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
