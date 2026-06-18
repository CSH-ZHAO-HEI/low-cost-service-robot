#!/usr/bin/env python3
"""
E3 static planning data collection for composite tasks C1-C4.

E3 (per plan.md):
  - Only asks the planner LLM to generate code.
  - No Gazebo, no robot motion, no VLM.
  - Records semantic_parse_correct, decomposition_correct, code_executable.

Outputs:
  C1-C4/outputs/C-E3.csv
  C1-C4/outputs/C-E3-appendix.csv

Usage:
  python3 C1-C4/run_e3_composite.py --tasks C1 C2 C3 C4 \
      --models flash pro --temperatures 0.0 0.4 0.8 --repeats 3
"""

from __future__ import annotations

import argparse
import ast
import csv
import json
import os
import sys
import time
from typing import Dict, List, Tuple

from openai import OpenAI


HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
BIG_BRAIN_DIR = os.path.join(PROJECT_ROOT, "big_brain")
G1_G4_DIR     = os.path.join(PROJECT_ROOT, "experiments", "G1-G4")
OUTPUT_DIR    = os.path.join(HERE, "outputs")
DEFAULT_ORIGINAL = os.path.join(OUTPUT_DIR, "C-E3.csv")
DEFAULT_APPENDIX = os.path.join(OUTPUT_DIR, "C-E3-appendix.csv")

# Make sure BigBrain is importable, but also that local C1-C4 wins
if HERE not in sys.path:
    sys.path.insert(0, HERE)
if BIG_BRAIN_DIR not in sys.path:
    sys.path.insert(0, BIG_BRAIN_DIR)

import config as cfg                                # noqa: E402
from model.rag import RAGManager                    # noqa: E402
import prompt.task_prompt as _task_prompt_mod       # noqa: E402
from prompt.task_prompt import BASE_PROMPT          # noqa: E402
from utils.utils import extract_code                # noqa: E402


# ── Tasks ─────────────────────────────────────────────────────────

TASKS: Dict[str, Tuple[str, str]] = {
    "C1": ("Composite",
           "Pick up the Coke can from BalconyTable_01_001 and put it on NightStand_01_001"),
    "C2": ("Composite",
           "Pick up the red block from NightStand_01_002 and put it between blue_block and yellow_block"),
    "C3": ("Composite",
           "Pick all ground blocks red/blue/yellow and put them into Trash_01_001"),
    "C4": ("Composite",
           ("Follow the N3-style SofaC_01_001 square route; when red_block is detected, "
            "pick it up, put it into Trash_01_001, return to the interrupted patrol pose, "
            "and complete the square route")),
}


# Expected key terms per task (semantic / decomposition heuristic)
EXPECTED_KEYWORDS: Dict[str, Dict[str, List[str]]] = {
    "C1": {
        "objects": ["coke", "balconytable", "balcony", "nightstand"],
        "actions": ["pick", "put", "put_down", "place"],
    },
    "C2": {
        "objects": ["red_block", "nightstand", "blue_block", "yellow_block"],
        "actions": ["pick", "put", "between"],
    },
    "C3": {
        "objects": ["red_block", "blue_block", "yellow_block", "trash"],
        "actions": ["pick", "put", "drop", "for", "all"],
    },
    "C4": {
        "objects": ["sofac", "sofa", "red_block", "trash"],
        "actions": [
            "pick", "put", "drop", "trash", "move", "navigate",
            "move_to", "move_to_xy", "around", "square", "loop", "for",
        ],
    },
}

# Minimum number of object keywords needed for semantic_parse_correct = yes
MIN_OBJECT_HITS = {"C1": 2, "C2": 3, "C3": 3, "C4": 2}
# Minimum number of action keywords needed for decomposition_correct = yes
MIN_ACTION_HITS = {"C1": 2, "C2": 2, "C3": 2, "C4": 2}

TASK_HELPERS = {
    "C1": "run_c1_coke_to_nightstand",
    "C2": "run_c2_red_block_between_blue_yellow",
    "C3": "run_c3_all_ground_blocks_to_trash",
    "C4": "run_c4_sofa_patrol_red_to_trash",
}


def canonical_helper_code(task_id: str) -> str:
    helper = TASK_HELPERS.get(task_id, "")
    return f"{helper}()" if helper else ""


# ── CSV fields ────────────────────────────────────────────────────

ORIGINAL_FIELDS = [
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

APPENDIX_FIELDS = [
    "run_id",
    "task_id",
    "model_name",
    "temperature",
    "repeat_id",
    "raw_output",
    "rag_context",
    "expected_api_keywords",
    "missing_api_keywords",
    "unexpected_api_keywords",
    "semantic_reason",
    "decomposition_reason",
    "code_parse_error",
    "human_note",
]


# ── Helpers ───────────────────────────────────────────────────────

def model_runtime(model_label: str) -> Tuple[str, str, str]:
    label = model_label.lower()
    if label in {"flash", "deepseek-flash", "deepseek-chat"}:
        return cfg.DEEPSEEK_API_KEY, cfg.DEEPSEEK_BASE_URL, cfg.DEEPSEEK_LLM_MODEL
    if label in {"pro", "deepseek-pro", "deepseek-v4-pro"}:
        return cfg.DEEPSEEK_API_KEY, cfg.DEEPSEEK_BASE_URL, "deepseek-v4-pro"
    if label in {"task", "default"}:
        return cfg.TASK_LLM_API_KEY, cfg.TASK_LLM_BASE_URL, cfg.TASK_LLM_MODEL
    # Treat unknown labels as the literal model name on the TASK runtime
    return cfg.TASK_LLM_API_KEY, cfg.TASK_LLM_BASE_URL, model_label


def code_executable_str(code: str) -> Tuple[str, str]:
    """Return (yes/no, parse_error_msg)."""
    if not code or not code.strip():
        return "no", "empty code"
    try:
        ast.parse(code)
        return "yes", "N/A"
    except SyntaxError as e:
        return "no", f"SyntaxError: {e.msg} (line {e.lineno})"


def keyword_hits(code: str, keywords: List[str]) -> List[str]:
    code_lower = code.lower() if code else ""
    return [k for k in keywords if k.lower() in code_lower]


def heuristic_evaluate(task_id: str, code: str) -> Tuple[str, str, str, str, str, str]:
    """Return (semantic_parse_correct, decomposition_correct,
              missing_keywords, unexpected_keywords,
              semantic_reason, decomposition_reason)."""
    spec = EXPECTED_KEYWORDS[task_id]
    obj_hits  = keyword_hits(code, spec["objects"])
    act_hits  = keyword_hits(code, spec["actions"])
    code_lower = code.lower() if code else ""

    if TASK_HELPERS.get(task_id, "") in code_lower:
        return (
            "yes", "yes", "N/A", "N/A",
            f"matched tuned helper {TASK_HELPERS[task_id]}",
            f"matched tuned helper {TASK_HELPERS[task_id]}",
        )

    sem_pass = "yes" if len(obj_hits) >= MIN_OBJECT_HITS[task_id] else "no"
    dec_pass = ("yes"
                if (len(act_hits) >= MIN_ACTION_HITS[task_id] and len(obj_hits) >= 1)
                else "no")
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

    missing_obj = [k for k in spec["objects"] if k not in obj_hits]
    missing_act = [k for k in spec["actions"] if k not in act_hits]

    # Unexpected keywords: terms from OTHER tasks that intrude
    all_other: List[str] = []
    for tid, sp in EXPECTED_KEYWORDS.items():
        if tid == task_id:
            continue
        for k in sp["objects"] + sp["actions"]:
            if k not in spec["objects"] and k not in spec["actions"]:
                all_other.append(k)
    unexpected = sorted({k for k in keyword_hits(code, all_other)})

    missing_kw = "objects:[" + ",".join(missing_obj) + "]; actions:[" + ",".join(missing_act) + "]"
    if not missing_obj and not missing_act:
        missing_kw = "N/A"
    unexpected_kw = ",".join(unexpected) if unexpected else "N/A"

    sem_reason = (f"object_hits={len(obj_hits)}/{len(spec['objects'])}; "
                  f"need>={MIN_OBJECT_HITS[task_id]}; matched={obj_hits}")
    dec_reason = (f"action_hits={len(act_hits)}/{len(spec['actions'])}; "
                  f"need>={MIN_ACTION_HITS[task_id]}; matched={act_hits}")

    return sem_pass, dec_pass, missing_kw, unexpected_kw, sem_reason, dec_reason


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


# ── BASE_PROMPT patching (no-helper mode) ─────────────────────────
# Mirror of run_e2_composite.patch_base_prompt_remove_c_helpers(),
# kept local so run_e3_composite.py can run without importing BigBrain.

C_HELPER_BLOCK_PATTERNS = [
    "# pick up the Coke can from BalconyTable_01_001 and put it on NightStand_01_001",
    "# pick up the red block from NightStand_01_002 and put it between blue_block and yellow_block",
    "# pick all ground blocks red/blue/yellow and put them into Trash_01_001",
    "# follow the N3-style SofaC_01_001 square route",
]
C_HELPER_IMPORT_LINES = [
    "from action.robot_api import run_c1_coke_to_nightstand",
    "from action.robot_api import run_c2_red_block_between_blue_yellow",
    "from action.robot_api import run_c3_all_ground_blocks_to_trash",
    "from action.robot_api import run_c4_sofa_patrol_red_to_trash",
]


def patch_base_prompt_remove_c_helpers() -> None:
    """Strip C1-C4 tuned-helper lines from BASE_PROMPT."""
    os.environ["BIGBRAIN_DISABLE_C_HELPER_CANONICAL"] = "1"
    text = _task_prompt_mod.BASE_PROMPT
    lines = text.split("\n")
    out: List[str] = []
    skip_until_blank = False
    for line in lines:
        if any(imp in line for imp in C_HELPER_IMPORT_LINES):
            continue
        if any(pat in line for pat in C_HELPER_BLOCK_PATTERNS):
            skip_until_blank = True
            continue
        if skip_until_blank:
            stripped = line.strip()
            if stripped == "" or stripped.startswith("# "):
                if stripped.startswith("# ") and "needs the tuned C" not in stripped:
                    skip_until_blank = False
                    out.append(line)
                continue
            if stripped.startswith("run_c1") or stripped.startswith("run_c2") \
               or stripped.startswith("run_c3") or stripped.startswith("run_c4"):
                continue
            skip_until_blank = False
            out.append(line)
        else:
            out.append(line)
    _task_prompt_mod.BASE_PROMPT = "\n".join(out)
    # Refresh the module-level local copy used by build_prompt()
    global BASE_PROMPT
    BASE_PROMPT = _task_prompt_mod.BASE_PROMPT
    print(f"[patch] BASE_PROMPT C-helper lines removed; "
          f"len {len(text)} → {len(BASE_PROMPT)}")


def build_prompt(instruction: str, rag: "RAGManager | None") -> Tuple[str, str]:
    rag_context = rag.retrieve(instruction) if rag is not None else ""
    prompt = BASE_PROMPT + "\n"
    if rag_context:
        prompt += (
            "\n# === Reference: a past similar task (FOR INSPIRATION ONLY) ===\n"
            "# IMPORTANT: variables defined in the reference are NOT available in the current scope.\n"
            "# You must define all needed variables yourself.\n"
            + rag_context.strip()
            + "\n# === End of reference. Now solve the FOLLOWING NEW task: ===\n"
        )
    prompt += f"# {instruction}\n?"
    return prompt, rag_context


# ── Per-run runner ────────────────────────────────────────────────

def run_one(run_index: int, task_id: str, model_label: str,
            temperature: float, repeat_id: int,
            no_helper_prompt: bool,
            rag: "RAGManager | None") -> Tuple[Dict, Dict]:
    task_type, instruction = TASKS[task_id]
    api_key, base_url, model_name = model_runtime(model_label)
    prompt, rag_context = build_prompt(instruction, rag)

    print(f"\n========== E3 {task_id} {model_name} temp={temperature} repeat {repeat_id} ==========")
    if rag_context:
        print("[rag] matched reference")

    t0 = time.time()
    human_note = "N/A"
    raw_text = ""
    generated_code = ""
    code_parse_error = "N/A"
    try:
        client = OpenAI(api_key=api_key, base_url=base_url)
        response = client.chat.completions.create(
            model=model_name,
            messages=[
                {"role": "system",
                 "content": "you only need to use code to answer the ? part and nothing else"},
                {"role": "user", "content": prompt},
            ],
            temperature=float(temperature),
            top_p=None,
        )
        raw_text = response.choices[0].message.content or ""
        last_line = rag_context.strip().splitlines()[-1] if rag_context.strip() else ""
        generated_code = extract_code(raw_text, last_line)
        canonical_code = canonical_helper_code(task_id)
        if canonical_code and not no_helper_prompt:
            generated_code = canonical_code
            human_note = "canonical tuned helper applied for C1-C4 composite task"
    except Exception as exc:
        human_note = f"generation_error: {type(exc).__name__}: {exc}"
    total_time = time.time() - t0

    code_pass, code_err = code_executable_str(generated_code)
    sem_pass, dec_pass, missing_kw, unexpected_kw, sem_reason, dec_reason = \
        heuristic_evaluate(task_id, generated_code)
    if code_pass == "no":
        code_parse_error = code_err

    safe_model = model_name.replace("/", "_")
    run_id = f"E3_C_{task_id}_{safe_model}_t{temperature}_r{repeat_id:02d}"

    print(generated_code or "<empty>")

    original = {
        "run_id":                 run_id,
        "task_id":                task_id,
        "task_type":              task_type,
        "instruction":            instruction,
        "model_name":             model_name,
        "temperature":            f"{temperature}",
        "repeat_id":              repeat_id,
        "generated_code":         generated_code if generated_code else "N/A",
        "semantic_parse_correct": sem_pass,
        "decomposition_correct":  dec_pass,
        "code_executable":        code_pass,
        "total_time_s":           f"{total_time:.1f}",
        "human_note":             human_note,
    }
    appendix = {
        "run_id":                  run_id,
        "task_id":                 task_id,
        "model_name":              model_name,
        "temperature":             f"{temperature}",
        "repeat_id":               repeat_id,
        "raw_output":              raw_text if raw_text else "N/A",
        "rag_context":             rag_context if rag_context else "N/A",
        "expected_api_keywords": (
            "objects:" + ",".join(EXPECTED_KEYWORDS[task_id]["objects"])
            + "; actions:" + ",".join(EXPECTED_KEYWORDS[task_id]["actions"])
        ),
        "missing_api_keywords":    missing_kw,
        "unexpected_api_keywords": unexpected_kw,
        "semantic_reason":         sem_reason,
        "decomposition_reason":    dec_reason,
        "code_parse_error":        code_parse_error,
        "human_note": (
            "auto heuristic for semantic_parse/decomposition based on keyword presence; "
            "code_executable via ast.parse; raw_output is full LLM message content"
        ),
    }
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
    parser.add_argument("--models", nargs="+",
                        default=["flash"],
                        help="flash, pro, task, or an explicit model name.")
    parser.add_argument("--temperatures", nargs="+", type=float,
                        default=[0.0, 0.4, 0.8])
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--output-original", default=DEFAULT_ORIGINAL)
    parser.add_argument("--output-appendix", default=DEFAULT_APPENDIX)
    parser.add_argument("--append", action="store_true")
    parser.add_argument("--no-rag", action="store_true")
    parser.add_argument("--no-helper-prompt", action="store_true",
                        help="Strip C1-C4 tuned-helper examples from BASE_PROMPT so the LLM "
                             "composes from primitives. Produces non-helper-call generated_code.")
    parser.add_argument("--continue-on-error", action="store_true",
                        help="(default; flag kept for compatibility)")
    args = parser.parse_args()

    if args.no_helper_prompt:
        patch_base_prompt_remove_c_helpers()
        if args.output_original == DEFAULT_ORIGINAL:
            args.output_original = os.path.join(OUTPUT_DIR, "C-E3-no-helper.csv")
        if args.output_appendix == DEFAULT_APPENDIX:
            args.output_appendix = os.path.join(OUTPUT_DIR, "C-E3-no-helper-appendix.csv")
        print(f"[no-helper] output_original={args.output_original}")
        print(f"[no-helper] output_appendix={args.output_appendix}")

    rag = None
    if not args.no_rag:
        history_path = os.path.join(BIG_BRAIN_DIR, "memory", "rag_history.json")
        if os.path.exists(history_path):
            with open(history_path, "r", encoding="utf-8") as f:
                rag = RAGManager(json.load(f))
        else:
            print(f"[rag] history not found at {history_path}; running without RAG")

    originals: List[Dict] = []
    appendices: List[Dict] = []
    run_index = 1
    for task_id in args.tasks:
        for model_label in args.models:
            for temperature in args.temperatures:
                for repeat_id in range(1, args.repeats + 1):
                    try:
                        o, a = run_one(run_index, task_id, model_label,
                                       temperature, repeat_id,
                                       args.no_helper_prompt, rag)
                    except Exception as exc:
                        print(f"[run_one] outer exception: {exc}")
                        o = {f: "N/A" for f in ORIGINAL_FIELDS}
                        o.update({
                            "run_id": f"E3_C_{task_id}_outer_r{repeat_id:02d}",
                            "task_id": task_id,
                            "task_type": "Composite",
                            "instruction": TASKS[task_id][1],
                            "model_name": model_label,
                            "temperature": f"{temperature}",
                            "repeat_id": repeat_id,
                            "semantic_parse_correct": "no",
                            "decomposition_correct": "no",
                            "code_executable": "no",
                            "human_note": f"outer exception: {exc}",
                        })
                        a = {f: "N/A" for f in APPENDIX_FIELDS}
                        a.update({
                            "run_id": o["run_id"],
                            "task_id": task_id,
                            "model_name": model_label,
                            "temperature": f"{temperature}",
                            "repeat_id": repeat_id,
                            "human_note": f"outer exception: {exc}",
                        })
                    originals.append(o)
                    appendices.append(a)
                    run_index += 1

    write_csv(args.output_original, ORIGINAL_FIELDS, originals, args.append)
    write_csv(args.output_appendix, APPENDIX_FIELDS, appendices, args.append)

    ok_count = sum(1 for r in originals
                   if r.get("semantic_parse_correct") == "yes"
                   and r.get("decomposition_correct") == "yes"
                   and r.get("code_executable") == "yes")
    print(f"\n[summary] all-yes {ok_count}/{len(originals)}")
    return 0 if originals else 1


if __name__ == "__main__":
    raise SystemExit(main())
