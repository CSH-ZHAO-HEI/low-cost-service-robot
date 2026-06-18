#!/usr/bin/env python3
"""
E2 adjust-ablation data collection for composite tasks C1-C4.

E2 in this project (per plan.md):
  - LLM + Judge/VLM + Gazebo execution
  - adjust_policy = 0 and 1 only (adjust_policy=2 data comes from E1)
  - Default repeats = 3

Outputs:
  C1-C4/outputs/C-E2.csv
  C1-C4/outputs/C-E2-appendix.csv

Usage:
  python3 C1-C4/run_e2_composite.py --tasks C1 C2 C3 C4 --repeats 3
  python3 C1-C4/run_e2_composite.py --tasks C1 --adjust-policies 0 1 --repeats 3
  python3 C1-C4/run_e2_composite.py --tasks C1 --adjust-policies 0 --start-writer

This module is also imported by run_e1_composite.py for shared run_one() logic.
"""

from __future__ import annotations

import argparse
import ast
import csv
import json
import os
import subprocess
import sys
import time
import traceback
from dataclasses import dataclass, field
from typing import Dict, List, Tuple


HERE = os.path.dirname(os.path.abspath(__file__))
# Insert C1-C4 dir FIRST so local composite_capability_test wins
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import composite_capability_test as comp     # noqa: E402  (local C1-C4 version)
import manip_capability_test as cap          # noqa: E402  (from G1-G4, added by comp)
import ros_bridge                            # noqa: E402
from action import robot_api                 # noqa: E402
from big_brain import BigBrain               # noqa: E402
from config import (                         # noqa: E402
    JUDGE_LOG_DIR, REPLAN_EXECUTE_CODE,
    TARGET_LLM_MODEL, TARGET_VLM_MODEL,
)
from model.llm import JudgeLLM               # noqa: E402

try:
    from manip_capability_test import BETWEEN_Y_TOL, PLACE_Z_TOL
except ImportError:
    BETWEEN_Y_TOL = 0.08
    PLACE_Z_TOL   = 0.10


# Output paths
OUTPUT_DIR       = os.path.join(HERE, "outputs")
DEFAULT_ORIGINAL = os.path.join(OUTPUT_DIR, "C-E2.csv")
DEFAULT_APPENDIX = os.path.join(OUTPUT_DIR, "C-E2-appendix.csv")
DEFAULT_VLM_TOPIC = "/judge_camera/rgb/image_raw"
BIG_BRAIN_DIR = os.path.join(os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))), "big_brain")


# ── BASE_PROMPT patching (no-helper mode) ─────────────────────────
# Removes the four "run_c*_*" example lines from prompt.task_prompt.BASE_PROMPT
# so the LLM has to compose C1-C4 from primitives. This is what makes
# Judge/VLM fire per step and produces adjust/replan data (matching G-series).

C_HELPER_BLOCK_PATTERNS = [
    "# pick up the Coke can from BalconyTable_01_001 and put it on NightStand_01_001",
    "# pick up the red block from NightStand_01_002 and put it between blue_block and yellow_block",
    "# pick all ground blocks red/blue/yellow and put them into Trash_01_001",
    "# follow the N3-style SofaC_01_001 square route",
    "# pick it up, put it into Trash_01_001, return/resume, and complete the route.",
    # C1/C2 prototype mini-helpers — these primed the LLM with Coke/NightStand/CoffeeTable patterns
    # even for unrelated tasks like C3. Treat them as task-specific helpers too.
    "# pick up the red block from the CoffeeTable, then put it between the blue and yellow blocks",
]
# Block-style strip: when a pattern matches, also drop subsequent code lines until
# a blank line or a new top-level comment / for / if / def appears.
C_HELPER_BLOCK_START_PATTERNS = [
    "# pick up the red block from the CoffeeTable, then put it between the blue and yellow blocks",
]
C_HELPER_IMPORT_LINES = [
    "from action.robot_api import run_c1_coke_to_nightstand",
    "from action.robot_api import run_c2_red_block_between_blue_yellow",
    "from action.robot_api import run_c3_all_ground_blocks_to_trash",
    "from action.robot_api import run_c4_sofa_patrol_red_to_trash",
    # Mini-helpers / task-specific primitives — strip in no-helper mode so the LLM
    # can't bind "red block → CoffeeTable" or "Coke → BalconyTable/NightStand" from prompt.
    "from action.robot_api import pick_up_from_coffeetable, put_down_between_objs",
    "from action.robot_api import pick_up_coke_from_balconytable, put_down_coke_on_nightstand",
]


def patch_base_prompt_remove_c_helpers() -> None:
    """Strip C1-C4 tuned-helper lines from BASE_PROMPT and module symbols.

    After this call, the LLM no longer sees the canonical helper examples and
    has to compose C-series tasks from primitives. The helper imports are also
    removed from BASE_PROMPT text so the LLM is not even told the helpers exist.
    """
    import prompt.task_prompt as tp
    os.environ["BIGBRAIN_DISABLE_C_HELPER_CANONICAL"] = "1"
    text = tp.BASE_PROMPT
    lines = text.split("\n")
    out: List[str] = []

    helper_call_tokens = (
        "run_c1_coke_to_nightstand",
        "run_c2_red_block_between_blue_yellow",
        "run_c3_all_ground_blocks_to_trash",
        "run_c4_sofa_patrol_red_to_trash",
    )
    helper_hint_tokens = (
        "tuned C1 helper",
        "tuned C2 helper",
        "tuned C3 helper",
        "tuned C4 helper",
        "patrol-and-interrupt task needs the tuned C4 helper",
    )

    in_block_strip = False
    for line in lines:
        stripped = line.strip()

        # If we're inside a multi-line strip block, consume until terminator.
        if in_block_strip:
            # terminator: blank line, or a new top-level comment / for / if / def / class
            if stripped == "" or stripped.startswith("# ") or \
               stripped.startswith("for ") or stripped.startswith("if ") or \
               stripped.startswith("def ") or stripped.startswith("class "):
                in_block_strip = False
                # fall through to normal handling for THIS line
            else:
                continue

        # start a multi-line strip if this line matches a block-start pattern
        if any(pat.lower() in stripped.lower() for pat in C_HELPER_BLOCK_START_PATTERNS):
            in_block_strip = True
            continue

        # remove explicit helper imports and helper call examples
        if any(imp in line for imp in C_HELPER_IMPORT_LINES):
            continue
        if any(tok in line for tok in helper_call_tokens):
            continue
        # remove helper-specific comments/hints so model won't be primed to call helper
        if any(tok in stripped for tok in helper_hint_tokens):
            continue
        if any(pat.lower() in stripped.lower() for pat in C_HELPER_BLOCK_PATTERNS):
            continue
        out.append(line)

    tp.BASE_PROMPT = "\n".join(out)
    print(f"[patch] BASE_PROMPT C-helper lines removed; "
          f"len {len(text)} → {len(tp.BASE_PROMPT)}")


# ── CSV fields (mirror C-E4 ORIGINAL_FIELDS layout) ───────────────

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
    "generated_code_source",
    "run_evidence_json_path",
    "human_note",
]

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
    "gazebo_judge_result",
    "vlm_judge_result_optional",
    "generated_code",
    "judge_event_count",
    "vlm_call_count",
    "vlm_pass_count",
    "last_vlm_pass",
    "last_judge_final_pass",
    "replan_generated_count",
    "replan_executed_count",
    "replan_failed_count",
    "last_vlm_reason",
    "last_failure_reason",
    "api_trace_summary",
    "service_success_sequence",
    "judge_rule",
    "failure_stage",
    "failure_type",
    "human_note",
    "c4_completed_waypoints",
    "c4_detected_red",
    "c4_resumed_after_drop",
    "c4_patrol_completed",
    "c4_red_to_trash_error_m",
    # 完整 VLM / replan trace（每次 call 一份），no-helper 模式特別有用
    "all_vlm_passes",
    "all_vlm_reasons",
    "all_judge_final_pass",
    "all_judge_targets",
    "all_replan_generated_code",
    "all_replan_executed_outcome",
    "all_replan_failed_reasons",
    "judge_events_jsonl_path",
    "run_evidence_json_path",
]


# ── Task descriptors ──────────────────────────────────────────────

TASK_INSTRUCTIONS = {
    "C1": "Pick up the Coke can from BalconyTable_01_001 and put it on NightStand_01_001",
    "C2": "Pick up the red block from NightStand_01_002 and put it between blue_block and yellow_block",
    "C3": "Pick all ground blocks red/blue/yellow and put them into Trash_01_001",
    "C4": ("Follow the N3-style SofaC_01_001 square route; when red_block is detected, "
           "pick it up, put it into Trash_01_001, return to the interrupted patrol pose, "
           "and complete the square route"),
}

# Alternative C1 wording using a normal physics block instead of the static Coke model.
# Useful in --c1-block-variant mode to isolate "small-brain edge-pickup pose" issues
# from "Coke physics / snap_down" issues.
C1_BLOCK_VARIANT_INSTRUCTION = (
    "Pick up the red block from BalconyTable_01_001 and put it on NightStand_01_001"
)


# Heuristic keywords per task (for static semantic/decomposition judge)
REQUIRED_KEYWORDS = {
    "C1": (["coke", "balcony", "nightstand"], ["pick", "put", "put_down", "place"]),
    "C2": (["red_block", "blue_block", "yellow_block"], ["pick", "put", "between"]),
    "C3": (["red_block", "blue_block", "yellow_block", "trash"], ["pick", "put", "drop", "for", "all"]),
    "C4": (["sofac", "sofa", "red_block", "trash"], ["pick", "put", "drop", "move", "navigate", "around", "square", "loop", "for"]),
}

TASK_HELPERS = {
    "C1": "run_c1_coke_to_nightstand",
    "C2": "run_c2_red_block_between_blue_yellow",
    "C3": "run_c3_all_ground_blocks_to_trash",
    "C4": "run_c4_sofa_patrol_red_to_trash",
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
        is_put  = service_name in {"/arm/drop", "/arm/put", "/arm/prepare_put"}
        if is_pick: trace.pickup_call_count += 1
        if is_put:  trace.putdown_call_count += 1
        try:
            result = original_call_arm_service(service_name, target_name)
            if is_pick: trace.pickup_success_count += 1
            if is_put:  trace.putdown_success_count += 1
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


def reset_for_task(task_id: str, tester: comp.CompositeTester) -> None:
    if task_id == "C1":
        tester.reset_c1()
    elif task_id == "C2":
        tester.reset_table_pick(comp.C2_RED_POSE)
    elif task_id == "C3":
        tester.reset_c3()
    elif task_id == "C4":
        tester.reset_c4()


def reset_object_poses_str(task_id: str) -> str:
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
    try:
        if task_id == "C1":
            return f"Coke={fmt_pose(cap.model_pose(tester.get_model_state, comp.CUP))}"
        if task_id in {"C2", "C3"}:
            r = cap.model_pose(tester.get_model_state, comp.RED)
            b = cap.model_pose(tester.get_model_state, comp.BLUE)
            y = cap.model_pose(tester.get_model_state, comp.YELLOW)
            return f"red={fmt_pose(r)}; blue={fmt_pose(b)}; yellow={fmt_pose(y)}"
        if task_id == "C4":
            return f"red={fmt_pose(cap.model_pose(tester.get_model_state, comp.RED))}"
    except Exception as e:
        return f"error: {e}"
    return "N/A"


def expected_target_desc(task_id: str) -> str:
    return {
        "C1": f"Coke center inside {comp.NIGHTSTAND} bbox; z near NightStand surface_z",
        "C2": (f"red_block projection between blue_block and yellow_block; "
               f"lateral_error <= {BETWEEN_Y_TOL:.3f}; z near BLOCK_HALF"),
        "C3": f"red/blue/yellow all within {comp.TRASH_NEAR_TOL:.2f}m XY of {comp.TRASH}",
        "C4": (f"red_block within {comp.TRASH_NEAR_TOL:.2f}m XY of {comp.TRASH}; "
               f"N3-style SofaC patrol completed"),
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
    return ("C4 judge (E1/E2 LLM mode): red_block within "
            f"{comp.TRASH_NEAR_TOL:.2f}m of Trash_01_001 (patrol_completed not enforced "
            "because LLM code does not set internal C4 state vars)")


def compute_errors(task_id: str, tester: comp.CompositeTester,
                   target_pose: Tuple[float, float, float]) -> Tuple[str, str]:
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


def gazebo_judge_for_e2(task_id: str, tester: comp.CompositeTester) -> bool:
    """Pure-Gazebo judge for E1/E2 (LLM mode).
    C1/C2/C3 use the tester's check_c* directly (they are Gazebo-only).
    C4 uses red-near-trash only because check_c4 depends on internal patrol
    state vars that LLM-generated code does not set."""
    try:
        if task_id == "C1":
            return tester.check_c1()
        if task_id == "C2":
            return tester.check_c2()
        if task_id == "C3":
            return tester.check_c3()
        if task_id == "C4":
            p = cap.model_pose(tester.get_model_state, comp.RED)
            return cap.dist_xy((p[0], p[1]), comp.TRASH_XY) <= comp.TRASH_NEAR_TOL
    except Exception:
        return False
    return False


def heuristic_judge(task_id: str, code: str) -> Tuple[str, str, str]:
    """Return (semantic_parse_correct, decomposition_correct, code_executable)."""
    if not code or not code.strip():
        return "no", "no", "no"
    try:
        ast.parse(code)
        code_pass = "yes"
    except SyntaxError:
        code_pass = "no"

    keywords, verbs = REQUIRED_KEYWORDS.get(task_id, ([], []))
    code_lower = code.lower()
    if TASK_HELPERS.get(task_id, "") in code_lower:
        return "yes", "yes", code_pass
    found_k = sum(1 for k in keywords if k.lower() in code_lower)
    found_v = sum(1 for v in verbs    if v.lower() in code_lower)

    threshold_k = max(1, len(keywords) - 1)
    sem_pass = "yes" if found_k >= threshold_k else "no"
    dec_pass = "yes" if found_v >= 2 and found_k >= 1 else "no"
    if task_id == "C3":
        has_all_blocks = (
            all(k in code_lower for k in ("red_block", "blue_block", "yellow_block"))
            or "all" in code_lower
            or "ground" in code_lower
        )
        has_trash = "trash" in code_lower
        has_pick = "pick" in code_lower
        has_drop = ("put" in code_lower) or ("drop" in code_lower)
        has_multi = any(k in code_lower for k in ("for", "loop", "all", "list", "multiple"))
        dec_pass = "yes" if (has_all_blocks and has_trash and has_pick and has_drop and has_multi) else "no"
    elif task_id == "C4":
        has_patrol = any(k in code_lower for k in ("sofac", "sofa", "square", "around", "patrol", "waypoint", "move_to"))
        has_red = "red_block" in code_lower
        has_trash = "trash" in code_lower
        has_pick = "pick" in code_lower
        has_drop = ("put" in code_lower) or ("drop" in code_lower)
        has_resume = any(k in code_lower for k in ("resume", "continue", "complete", "loop", "waypoint", "for"))
        dec_pass = "yes" if (has_patrol and has_red and has_trash and has_pick and has_drop and has_resume) else "no"
    return sem_pass, dec_pass, code_pass


def sanitize_row(row: Dict[str, object], fields: List[str]) -> Dict[str, object]:
    clean: Dict[str, object] = {}
    for field in fields:
        value = row.get(field, "N/A")
        if value is None:
            value = "N/A"
        elif isinstance(value, str) and value == "":
            value = "N/A"
        clean[field] = value
    return clean


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
    if "timeout" in lower:                       return "service_timeout"
    if "navigation" in lower or "move_base" in lower: return "navigation_failed"
    if "pick" in lower:                          return "pick_failed"
    if "drop" in lower or "put" in lower:        return "place_failed"
    if "judge" in lower or "tolerance" in lower: return "judge_fail"
    return "execution_failed"


# ── VLM / Judge log helpers ───────────────────────────────────────

def image_path() -> str:
    return os.path.join(BIG_BRAIN_DIR, "image", "image.jpg")


def image_age_s() -> float:
    p = image_path()
    if not os.path.exists(p):
        return float("inf")
    return time.time() - os.path.getmtime(p)


def wait_fresh_image(timeout_s: float = 8.0, max_age_s: float = 3.0) -> float:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        age = image_age_s()
        if age <= max_age_s:
            return age
        time.sleep(0.2)
    return image_age_s()


def start_vlm_writer(topic: str) -> subprocess.Popen:
    cmd = [sys.executable, os.path.join(BIG_BRAIN_DIR, "vlm_image_writer.py"), topic]
    print(f"[writer] starting: {' '.join(cmd)}")
    return subprocess.Popen(cmd, cwd=BIG_BRAIN_DIR)


def read_new_judge_events(log_path: str, start_offset: int) -> List[Dict[str, object]]:
    if not os.path.exists(log_path):
        return []
    with open(log_path, "r", encoding="utf-8") as f:
        f.seek(start_offset)
        rows: List[Dict[str, object]] = []
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                rows.append({"event": "unparsed", "raw": line})
        return rows


def summarize_events(events: List[Dict[str, object]]) -> Dict[str, object]:
    judge_events = [e for e in events if e.get("event") == "judge"]
    vlm_events   = [e for e in judge_events if e.get("vlm_needed")]
    replan_gen   = [e for e in events if e.get("event") == "replan_generated"]
    replan_exec  = [e for e in events if e.get("event") == "replan_executed"]
    replan_fail  = [e for e in events if e.get("event") == "replan_failed"]

    last_vlm_reason = ""
    last_vlm_pass = ""
    for event in reversed(vlm_events):
        vlm_result = event.get("vlm_result") or {}
        if last_vlm_pass == "" and "pass" in vlm_result:
            last_vlm_pass = str(vlm_result.get("pass"))
        last_vlm_reason = vlm_result.get("reason", "")
        if last_vlm_reason:
            break

    last_judge_final_pass = ""
    for event in reversed(judge_events):
        if "final_pass" in event:
            last_judge_final_pass = str(event.get("final_pass"))
            break

    last_failure_reason = ""
    for event in reversed(replan_fail):
        last_failure_reason = str(event.get("reason", ""))
        if last_failure_reason:
            break

    # ── Full traces (not just last). Useful for no-helper / failing runs. ──
    def _trim(s: str, n: int = 240) -> str:
        s = str(s or "").replace("\n", " ").replace("\r", " ").strip()
        return s[:n] + ("…" if len(s) > n else "")

    all_vlm_passes = " | ".join(
        f"#{i+1}:{str((e.get('vlm_result') or {}).get('pass', '?'))}"
        for i, e in enumerate(vlm_events)
    ) or "N/A"
    all_vlm_reasons = " | ".join(
        f"#{i+1}:{_trim((e.get('vlm_result') or {}).get('reason', ''))}"
        for i, e in enumerate(vlm_events)
    ) or "N/A"
    all_judge_final_pass = " | ".join(
        f"#{i+1}:{str(e.get('final_pass', '?'))}"
        for i, e in enumerate(judge_events) if "final_pass" in e
    ) or "N/A"
    # JudgeLLM.replan() 把代碼存在 "adjust_code" 欄位（不是 code / generated_code）
    # 截斷加大到 800 字 — 保留足夠 LLM 推理樣本供論文引用，仍不爆 CSV 單格
    all_replan_generated_code = " || ".join(
        f"#{i+1}:{_trim(e.get('adjust_code', e.get('code', e.get('generated_code', ''))), 800)}"
        for i, e in enumerate(replan_gen)
    ) or "N/A"
    all_replan_executed_outcome = " | ".join(
        f"#{i+1}:ok={e.get('ok','?')}"
        for i, e in enumerate(replan_exec)
    ) or "N/A"
    all_replan_failed_reasons = " | ".join(
        f"#{i+1}:{_trim(e.get('reason', ''))}"
        for i, e in enumerate(replan_fail)
    ) or "N/A"
    # Original judge target / task descriptions (per event) — useful to see what
    # VLM was actually asked about.
    all_judge_targets = " | ".join(
        f"#{i+1}:{_trim(e.get('task_text', e.get('target', '')), 120)}"
        for i, e in enumerate(judge_events)
    ) or "N/A"

    return {
        "judge_event_count":      len(judge_events),
        "vlm_call_count":         len(vlm_events),
        "vlm_pass_count":         sum(1 for e in vlm_events
                                      if (e.get("vlm_result") or {}).get("pass") is True),
        "last_vlm_pass":          last_vlm_pass or "N/A",
        "last_judge_final_pass":  last_judge_final_pass or "N/A",
        "replan_generated_count": len(replan_gen),
        "replan_executed_count":  len(replan_exec),
        "replan_failed_count":    len(replan_fail),
        "last_vlm_reason":        last_vlm_reason or "N/A",
        "last_failure_reason":    last_failure_reason or "N/A",
        # 新增：完整 trace（每次 VLM / replan 一份）
        "all_vlm_passes":             all_vlm_passes,
        "all_vlm_reasons":            all_vlm_reasons,
        "all_judge_final_pass":       all_judge_final_pass,
        "all_judge_targets":          all_judge_targets,
        "all_replan_generated_code":  all_replan_generated_code,
        "all_replan_executed_outcome": all_replan_executed_outcome,
        "all_replan_failed_reasons":  all_replan_failed_reasons,
    }


def reenable_real_judge(max_replan_times: int = 2) -> None:
    """Restore a real JudgeLLM (CompositeTester disables it for capability tests)."""
    robot_api.judge_llm = JudgeLLM()
    robot_api.judge_llm.max_replan_times = max_replan_times


def derive_success_at_x(final_success: bool, adjust_policy: int,
                        replan_executed_count: int) -> Tuple[str, str, str]:
    """Map (success, adjust_policy, replan_executed) → success_at_1/2/3 yes/no/N/A."""
    s1 = s2 = s3 = "N/A"
    if final_success:
        if replan_executed_count == 0:
            s1 = "yes"
        elif replan_executed_count == 1:
            s1 = "no"; s2 = "yes"
        else:  # >=2
            s1 = "no"; s2 = "no"; s3 = "yes"
        if adjust_policy >= 1 and s2 == "N/A":
            s2 = "yes" if (s1 == "yes") else s2
        if adjust_policy >= 2 and s3 == "N/A":
            s3 = "yes" if (s1 == "yes" or s2 == "yes") else s3
    else:
        s1 = "no"
        if adjust_policy >= 1:
            s2 = "no" if replan_executed_count >= 1 else "N/A"
        if adjust_policy >= 2:
            s3 = "no" if replan_executed_count >= 2 else "N/A"
    return s1, s2, s3


# ── Per-run runner ────────────────────────────────────────────────

def run_one(run_id: str, task_id: str, repeat_id: int,
            adjust_policy: int,
            tester: comp.CompositeTester, brain: BigBrain,
            experiment: str = "E2",
            temperature: float = 0.0) -> Tuple[Dict, Dict]:
    print(f"\n========== {experiment} {task_id} repeat {repeat_id} "
          f"adjust_policy={adjust_policy} ==========")
    instruction = TASK_INSTRUCTIONS[task_id]
    print(f"[instruction] {instruction}")

    # Reset (sets robot HOME + objects)
    reset_for_task(task_id, tester)
    reset_robot_pose = cap.model_pose_yaw(tester.get_model_state, comp.ROBOT)
    reset_objects    = reset_object_poses_str(task_id)

    # Set up real JudgeLLM with replan policy
    reenable_real_judge(max_replan_times=adjust_policy)

    age_before = wait_fresh_image(timeout_s=5.0)
    print(f"[image] age before task: {age_before:.1f}s")

    log_path = os.path.join(JUDGE_LOG_DIR, "judge_events.jsonl")
    start_offset = os.path.getsize(log_path) if os.path.exists(log_path) else 0

    trace   = Trace()
    restore = install_tracers(trace)

    generated_code = ""
    failure_reason = ""
    big_brain_ok   = False
    t0 = time.time()
    orig_generate_code = brain.planner.generate_code
    code_holder = {
        "code": "",
        "prompt": "",
        "rag_context": "",
        "helper_violation": False,
        "regen_attempts": 0,
    }

    def wrapped_generate_code(prompt, rag_context):
        strict_non_helper = os.environ.get("BIGBRAIN_DISABLE_C_HELPER_CANONICAL") == "1"
        helper_names = tuple(TASK_HELPERS.values())

        def _uses_helper(text: str) -> bool:
            lowered = (text or "").lower()
            return any(name.lower() in lowered for name in helper_names)

        code_holder["prompt"] = prompt or ""
        code_holder["rag_context"] = rag_context or ""
        local_prompt = prompt
        max_tries = 3 if strict_non_helper else 1
        final_code = ""

        for attempt in range(1, max_tries + 1):
            code = orig_generate_code(local_prompt, rag_context) or ""
            final_code = code
            if not (strict_non_helper and _uses_helper(code)):
                code_holder["regen_attempts"] = attempt - 1
                break
            print(f"[no-helper] helper call detected in generated code (attempt {attempt}); regenerating...")
            local_prompt = (
                (prompt or "")
                + "\n# HARD CONSTRAINT: DO NOT call any run_c* helper functions."
                + "\n# You MUST compose the task only with primitive APIs:"
                + " move_to_obj_by_offset, pick_up_obj, put_down_obj_by_offset,"
                + " pick_up_from_coffeetable, put_down_between_objs, loops, and conditionals."
            )
        else:
            code_holder["helper_violation"] = True
            code_holder["regen_attempts"] = max_tries
            final_code = (
                "raise RuntimeError("
                "'non-manual policy violation: generated helper call run_c* forbidden in no-helper mode'"
                ")"
            )

        code_holder["code"] = final_code
        brain.last_generated_code = code_holder["code"]
        return final_code

    brain.planner.generate_code = wrapped_generate_code
    try:
        big_brain_ok = brain.run_once(instruction)
        generated_code = getattr(brain, "last_generated_code", "") or code_holder["code"] or "N/A"
    except Exception as exc:
        failure_reason = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        brain.planner.generate_code = orig_generate_code
        restore()
    total_time_s = time.time() - t0

    events  = read_new_judge_events(log_path, start_offset)
    summary = summarize_events(events)

    # 把這次 run 的 raw judge events 全量轉存一份，便於事後深度分析
    per_run_log_dir = os.path.join(OUTPUT_DIR, "judge_events_per_run")
    os.makedirs(per_run_log_dir, exist_ok=True)
    per_run_log_path = os.path.join(per_run_log_dir, f"{run_id}.jsonl")
    try:
        with open(per_run_log_path, "w", encoding="utf-8") as _f:
            for _e in events:
                _f.write(json.dumps(_e, ensure_ascii=False) + "\n")
    except Exception as _ex:
        print(f"[warn] failed to dump per-run judge log: {_ex}")
        per_run_log_path = "N/A"

    helper_names = tuple(TASK_HELPERS.values())
    code_lower = (generated_code or "").strip().lower()
    uses_helper = any(name.lower() in code_lower for name in helper_names)
    if code_holder.get("helper_violation"):
        generated_code_source = "helper_forbidden_violation"
    else:
        generated_code_source = "helper" if uses_helper else "llm_composed"

    # 每个 run 的完整证据包：可直接用于对外说明“确实是模型生成+执行，不是手写脚本硬跑”
    evidence_dir = os.path.join(OUTPUT_DIR, "run_evidence")
    os.makedirs(evidence_dir, exist_ok=True)
    evidence_path = os.path.join(evidence_dir, f"{run_id}.json")
    evidence_payload = {
        "run_id": run_id,
        "experiment": experiment,
        "task_id": task_id,
        "instruction": instruction,
        "temperature": temperature,
        "adjust_policy": adjust_policy,
        "generated_code_source": generated_code_source,
        "generated_code": generated_code or "",
        "planner_prompt": code_holder.get("prompt", ""),
        "planner_rag_context": code_holder.get("rag_context", ""),
        "trace": {
            "nav_goals": trace.nav_goals,
            "services": trace.services,
            "nav_call_count": trace.nav_call_count,
            "nav_success_count": trace.nav_success_count,
            "pickup_call_count": trace.pickup_call_count,
            "pickup_success_count": trace.pickup_success_count,
            "putdown_call_count": trace.putdown_call_count,
            "putdown_success_count": trace.putdown_success_count,
        },
        "judge_events_jsonl_path": per_run_log_path,
        "judge_events": events,
        "judge_summary": summary,
        "helper_regen_attempts": code_holder.get("regen_attempts", 0),
        "helper_violation": code_holder.get("helper_violation", False),
    }
    try:
        with open(evidence_path, "w", encoding="utf-8") as f:
            json.dump(evidence_payload, f, ensure_ascii=False, indent=2)
    except Exception as ex:
        print(f"[warn] failed to write evidence json: {ex}")
        evidence_path = "N/A"

    gazebo_ok = gazebo_judge_for_e2(task_id, tester)

    # final result: external success is BigBrain return + Gazebo semantic judge.
    # VLM/Judge false negatives are recorded as disagreement, not as main-result failure.
    final_ok = bool(big_brain_ok and gazebo_ok)
    final_result = "Success" if final_ok else "Fail"

    target_desc = expected_target_desc(task_id)
    target_pose = expected_target_pose(task_id, tester)
    xy_error, z_error = compute_errors(task_id, tester, target_pose)
    final_objects   = final_object_poses_str(task_id, tester)
    final_robot     = cap.model_pose_yaw(tester.get_model_state, comp.ROBOT)

    s1, s2, s3 = derive_success_at_x(final_ok, adjust_policy,
                                     int(summary["replan_executed_count"]))
    sem_pass, dec_pass, code_pass = heuristic_judge(task_id, generated_code)

    if not failure_reason and not final_ok:
        if summary["last_failure_reason"] != "N/A":
            failure_reason = summary["last_failure_reason"]
        else:
            failure_reason = "gazebo judge failed"
    if code_holder.get("helper_violation") and not failure_reason:
        failure_reason = "non-manual policy violation: helper call generated in no-helper mode"

    judge_disagreement = "N/A"
    if summary["last_vlm_pass"] == "False" and gazebo_ok:
        judge_disagreement = "vlm_false_negative"
    elif summary["last_vlm_pass"] == "True" and not gazebo_ok:
        judge_disagreement = "vlm_false_positive"

    # C4 state vars (LLM mode normally leaves these at __init__ defaults)
    c4_completed = "N/A"
    c4_detected  = "N/A"
    c4_resumed   = "N/A"
    c4_patrol    = "N/A"
    c4_red_err   = "N/A"
    if task_id == "C4":
        wp = getattr(tester, "c4_completed_waypoints", []) or []
        c4_completed = " | ".join(wp) if wp else "(empty)"
        c4_detected  = "yes" if getattr(tester, "c4_detected_red", False) else "no"
        c4_resumed   = "yes" if getattr(tester, "c4_resumed_after_drop", False) else "no"
        c4_patrol    = "yes" if getattr(tester, "c4_patrol_completed", False) else "no"
        c4_red_err   = xy_error

    original = {
        "run_id":                 run_id,
        "task_id":                task_id,
        "task_type":              "Composite",
        "instruction":            instruction,
        "method_type":            "LLM",
        "model_name":             TARGET_LLM_MODEL,
        "temperature":            f"{temperature:.2f}",
        "adjust_policy":          adjust_policy,
        "repeat_id":              repeat_id,
        "generated_code":         generated_code or "N/A",
        "semantic_parse_correct": sem_pass,
        "decomposition_correct":  dec_pass,
        "code_executable":        code_pass,
        "api_call_count":         trace.api_call_count,
        "adjust_count":           int(summary["replan_executed_count"]),
        "success_at_1":           s1,
        "success_at_2":           s2,
        "success_at_3":           s3,
        "final_result":           final_result,
        "total_time_s":           f"{total_time_s:.1f}",
        "nav_call_count":         trace.nav_call_count,
        "pickup_call_count":      trace.pickup_call_count,
        "putdown_call_count":     trace.putdown_call_count,
        "nav_success_count":      trace.nav_success_count,
        "pickup_success_count":   trace.pickup_success_count,
        "putdown_success_count":  trace.putdown_success_count,
        "final_position_error":   xy_error,
        "failure_reason":         failure_reason if failure_reason else "N/A",
        "vlm_summary": (
            f"vlm_calls={summary['vlm_call_count']}; "
            f"pass={summary['vlm_pass_count']}/{summary['vlm_call_count']}; "
            f"last_pass={summary['last_vlm_pass']}; "
            f"replan_exec={summary['replan_executed_count']}"
        ),
        "generated_code_source":  generated_code_source,
        "run_evidence_json_path": evidence_path,
        "human_note": (
            "auto heuristic for semantic_parse_correct/decomposition_correct based on "
            "keyword presence; code_executable via ast.parse; total_time_s excludes "
            "Gazebo reset performed before brain.run_once; VLM false negatives are "
            "recorded in appendix and do not override Gazebo final_result"
        ),
    }

    appendix = {
        "run_id":                  run_id,
        "task_id":                 task_id,
        "experiment":              experiment,
        "repeat_id":               repeat_id,
        "instruction":             instruction,
        "method_type":             "LLM",
        "model_name":              TARGET_LLM_MODEL,
        "temperature":             f"{temperature:.2f}",
        "adjust_policy":           adjust_policy,
        "reset_robot_pose_xyz_yaw": fmt_pose(reset_robot_pose),
        "reset_object_poses":      reset_objects,
        "expected_target":         target_desc,
        "expected_target_pose_xyz": fmt_pose(target_pose),
        "final_robot_pose_xyz_yaw": fmt_pose(final_robot),
        "final_object_poses":      final_objects,
        "xy_error_m":              xy_error,
        "z_error_m":               z_error,
        "gazebo_judge_result":     "pass" if gazebo_ok else "fail",
        "vlm_judge_result_optional": (
            f"last_vlm_pass={summary['last_vlm_pass']}; "
            f"last_judge_final_pass={summary['last_judge_final_pass']}; "
            f"vlm_disagreement={judge_disagreement}"
        ),
        "generated_code":          generated_code or "N/A",
        "judge_event_count":       summary["judge_event_count"],
        "vlm_call_count":          summary["vlm_call_count"],
        "vlm_pass_count":          summary["vlm_pass_count"],
        "last_vlm_pass":           summary["last_vlm_pass"],
        "last_judge_final_pass":   summary["last_judge_final_pass"],
        "replan_generated_count":  summary["replan_generated_count"],
        "replan_executed_count":   summary["replan_executed_count"],
        "replan_failed_count":     summary["replan_failed_count"],
        "last_vlm_reason":         summary["last_vlm_reason"],
        "last_failure_reason":     summary["last_failure_reason"],
        "api_trace_summary": (
            f"nav={trace.nav_success_count}/{trace.nav_call_count}; "
            f"pick={trace.pickup_success_count}/{trace.pickup_call_count}; "
            f"put={trace.putdown_success_count}/{trace.putdown_call_count}"
        ),
        "service_success_sequence": " | ".join(trace.services) if trace.services else "(none)",
        "judge_rule":              judge_rule_for(task_id),
        "failure_stage":           failure_stage_for(trace, failure_reason),
        "failure_type":            failure_type_for(failure_reason),
        "human_note":              (
            "auto heuristic for static fields; experiment via BigBrain.run_once; "
            "C4 E1/E2 external judge checks red_block-to-trash only because patrol "
            "internal flags are only available in E4 manual tester"
        ),
        "c4_completed_waypoints":  c4_completed,
        "c4_detected_red":         c4_detected,
        "c4_resumed_after_drop":   c4_resumed,
        "c4_patrol_completed":     c4_patrol,
        "c4_red_to_trash_error_m": c4_red_err,
        "all_vlm_passes":              summary["all_vlm_passes"],
        "all_vlm_reasons":             summary["all_vlm_reasons"],
        "all_judge_final_pass":        summary["all_judge_final_pass"],
        "all_judge_targets":           summary["all_judge_targets"],
        "all_replan_generated_code":   summary["all_replan_generated_code"],
        "all_replan_executed_outcome": summary["all_replan_executed_outcome"],
        "all_replan_failed_reasons":   summary["all_replan_failed_reasons"],
        "judge_events_jsonl_path":     per_run_log_path,
        "run_evidence_json_path":      evidence_path,
    }

    print(
        f"[result] {experiment} {task_id} r{repeat_id} ap={adjust_policy}: "
        f"{final_result}, time={total_time_s:.1f}s, xy={xy_error}m, "
        f"replan_exec={summary['replan_executed_count']}, "
        f"reason={failure_reason or 'OK'}"
    )
    return original, appendix


def write_csv(path: str, fields: List[str], rows: List[Dict], append: bool) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    mode = "a" if append else "w"
    file_exists = os.path.exists(path)
    with open(path, mode, encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        if not append or not file_exists:
            writer.writeheader()
        writer.writerows(sanitize_row(row, fields) for row in rows)
    print(f"[csv] wrote {len(rows)} rows → {path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tasks", nargs="+",
                        choices=["C1", "C2", "C3", "C4"],
                        default=["C1", "C2", "C3", "C4"])
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--adjust-policies", nargs="+", type=int, default=[0, 1],
                        help="C-E2 default = 0 1 (adjust_policy=2 is collected by E1)")
    parser.add_argument("--temperature", type=float, default=0.0)
    parser.add_argument("--output-original", default=DEFAULT_ORIGINAL)
    parser.add_argument("--output-appendix", default=DEFAULT_APPENDIX)
    parser.add_argument("--append", action="store_true")
    parser.add_argument("--start-writer", action="store_true",
                        help="Start vlm_image_writer.py during this run.")
    parser.add_argument("--vlm-topic", default=DEFAULT_VLM_TOPIC)
    parser.add_argument("--allow-stale-image", action="store_true")
    parser.add_argument("--save-history", action="store_true")
    parser.add_argument("--continue-on-error", action="store_true",
                        help="(default; flag kept for compatibility)")
    parser.add_argument("--experiment-label", default="E2")
    parser.add_argument("--pause-after-reset", type=float, default=0.0)
    parser.add_argument("--no-helper-prompt", action="store_true",
                        help="Strip C1-C4 tuned-helper examples from BASE_PROMPT so the LLM "
                             "has to compose from primitives. This is what makes "
                             "Judge/VLM fire per step and generates adjust/replan data.")
    parser.add_argument("--c1-block-variant", action="store_true",
                        help="Use red_block (physics) instead of Coke (static, custom snap) "
                             "for C1. Lets you isolate edge-pickup geometry issues from "
                             "Coke-specific physics issues. Output CSV auto-renamed.")
    args = parser.parse_args()

    # Patch C1 instruction if block variant requested
    if args.c1_block_variant:
        TASK_INSTRUCTIONS["C1"] = C1_BLOCK_VARIANT_INSTRUCTION
        if args.output_original == DEFAULT_ORIGINAL:
            args.output_original = os.path.join(OUTPUT_DIR, "C-E2-c1block.csv")
        if args.output_appendix == DEFAULT_APPENDIX:
            args.output_appendix = os.path.join(OUTPUT_DIR, "C-E2-c1block-appendix.csv")
        print(f"[c1-block-variant] C1 now uses red_block; outputs → {args.output_original}")

    # Apply no-helper patch BEFORE BigBrain is created (so it reads the patched prompt)
    if args.no_helper_prompt:
        patch_base_prompt_remove_c_helpers()
        # Auto-route outputs to *-no-helper.csv unless user overrode
        c1block_original = os.path.join(OUTPUT_DIR, "C-E2-c1block.csv")
        c1block_appendix = os.path.join(OUTPUT_DIR, "C-E2-c1block-appendix.csv")
        if args.output_original in (DEFAULT_ORIGINAL, c1block_original):
            args.output_original = os.path.join(OUTPUT_DIR, "C-E2-no-helper.csv")
        if args.output_appendix in (DEFAULT_APPENDIX, c1block_appendix):
            args.output_appendix = os.path.join(OUTPUT_DIR, "C-E2-no-helper-appendix.csv")
        print(f"[no-helper] output_original={args.output_original}")
        print(f"[no-helper] output_appendix={args.output_appendix}")

    # Confirm composite import path
    comp_path = os.path.abspath(comp.__file__)
    if not comp_path.startswith(os.path.abspath(HERE)):
        print(f"[WARNING] composite_capability_test imported from {comp_path}; "
              f"expected under {HERE}")
    else:
        print(f"[import] composite_capability_test = {comp_path}")

    writer_proc = None
    if args.start_writer:
        writer_proc = start_vlm_writer(args.vlm_topic)
        time.sleep(1.0)

    try:
        age = wait_fresh_image(timeout_s=5.0)
        if age > 3.0 and not args.allow_stale_image:
            print(f"[ERROR] VLM image is stale: age={age:.1f}s "
                  "(start the writer or use --allow-stale-image)")
            return 2
        print(f"[image] current age={age:.1f}s")

        tester = comp.CompositeTester(pause_after_reset=args.pause_after_reset)
        reenable_real_judge(max_replan_times=max(args.adjust_policies))
        brain = BigBrain()
        if not args.save_history:
            brain._save_history = lambda path: None

        originals: List[Dict] = []
        appendices: List[Dict] = []
        run_index = 1
        for task_id in args.tasks:
            for adjust_policy in args.adjust_policies:
                for repeat_id in range(1, args.repeats + 1):
                    run_id = f"{args.experiment_label}_C{run_index:03d}"
                    try:
                        o, a = run_one(run_id, task_id, repeat_id,
                                       adjust_policy, tester, brain,
                                       experiment=args.experiment_label,
                                       temperature=args.temperature)
                    except Exception as exc:
                        print(f"[run_one] outer exception for {task_id} r{repeat_id}: {exc}")
                        traceback.print_exc()
                        o = {f: "N/A" for f in ORIGINAL_FIELDS}
                        o.update({
                            "run_id": run_id, "task_id": task_id, "task_type": "Composite",
                            "instruction": TASK_INSTRUCTIONS[task_id],
                            "method_type": "LLM", "model_name": TARGET_LLM_MODEL,
                            "temperature": f"{args.temperature:.2f}",
                            "adjust_policy": adjust_policy, "repeat_id": repeat_id,
                            "final_result": "Fail",
                            "failure_reason": f"outer:{exc}",
                            "vlm_summary": "outer exception",
                            "human_note": "outer exception",
                        })
                        a = {f: "N/A" for f in APPENDIX_FIELDS}
                        a.update({
                            "run_id": run_id, "task_id": task_id,
                            "experiment": args.experiment_label,
                            "repeat_id": repeat_id,
                            "instruction": TASK_INSTRUCTIONS[task_id],
                            "method_type": "LLM",
                            "model_name": TARGET_LLM_MODEL,
                            "temperature": f"{args.temperature:.2f}",
                            "adjust_policy": adjust_policy,
                            "judge_rule": judge_rule_for(task_id),
                            "failure_stage": "outer",
                            "failure_type": "execution_failed",
                            "human_note": f"outer exception: {exc}",
                        })
                    originals.append(o)
                    appendices.append(a)
                    run_index += 1

        write_csv(args.output_original, ORIGINAL_FIELDS, originals, args.append)
        write_csv(args.output_appendix, APPENDIX_FIELDS, appendices, args.append)

        ok_count = sum(1 for r in originals if r.get("final_result") == "Success")
        print(f"\n[summary] Success {ok_count}/{len(originals)}")
        return 0 if ok_count == len(originals) else 1
    finally:
        if writer_proc is not None:
            writer_proc.terminate()
            try:
                writer_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                writer_proc.kill()


if __name__ == "__main__":
    raise SystemExit(main())
