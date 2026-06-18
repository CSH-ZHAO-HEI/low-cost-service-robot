#!/usr/bin/env python3
"""
DeepSeek V4 Flash data collection for N1-N4.

This script collects:
  - E3 static planning data (LLM code generation only)
  - E1 execution data with adjust_policy=2
  - E2 execution data with adjust_policy=0 and adjust_policy=1

For N1-N4, no VLM is used. Execution success is judged by odometry error.
Default repeat counts follow the original experiment design:
  - E1: 5 repeats
  - E2: 3 repeats for adjust_policy=0, 3 repeats for adjust_policy=1
  - E3: 3 repeats

Prerequisites for execution stages:
  1. ./run_gazebo.sh
  2. ./run_rtab.sh
  3. ./run_teb.sh or ./run_teb_compare.sh
  4. python3 get_scene.py

Usage:
  python3 N1-N4/run_flash_e1e2e3_nav.py --e3-only
  python3 N1-N4/run_flash_e1e2e3_nav.py --execute
  python3 N1-N4/run_flash_e1e2e3_nav.py --execute --e1-repeats 5 --e2-repeats 3 --e3-repeats 3

Output:
  N1-N4/outputs/E1_flash.csv
  N1-N4/outputs/E2_flash.csv
  N1-N4/outputs/E3_flash.csv
"""

from __future__ import annotations

import argparse
import builtins
import csv
import math
import os
import re
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import yaml
from openai import OpenAI


PROJECT_ROOT = os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
BIG_BRAIN_DIR = os.path.join(PROJECT_ROOT, "big_brain")
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "N1-N4", "outputs")
SCENE_PATH = os.path.join(PROJECT_ROOT, "gazebo_scene.yaml")

if BIG_BRAIN_DIR not in sys.path:
    sys.path.insert(0, BIG_BRAIN_DIR)

import config as cfg  # noqa: E402


MODEL_LABEL = "deepseek-v4-flash"
MODEL_NAME = "deepseek-v4-flash"
TEMPERATURE = 0.0
DEFAULT_E1_REPEATS = 5
DEFAULT_E2_REPEATS = 3
DEFAULT_E3_REPEATS = 3
LLM_TIMEOUT_S = 60
LLM_RETRIES = 2
TOLERANCE_M = 0.30

TASKS = [
    ("N1", "Navigation", "Navigate to (1.0, 2.0)"),
    ("N2", "Navigation", "Navigate next to the trash can"),
    ("N3", "Navigation", "Move around the sofa in a 4x4 meter square"),
    ("N4", "Navigation", "Go to the trash can with the smallest |x|+|y|, then return to the starting point"),
]

E3_FIELDS = [
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

EXEC_FIELDS = [
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

ADJUST_PROMPT = """
You generate recovery Python code for a mobile robot navigation API.
Return ONLY executable Python code, no markdown and no explanation.

Available functions:
- move_to_xy(x, y)
- move_to_obj_by_offset(obj, dx, dy)
- parse_obj_name(text, objects)  # returns a STRING object name. Do NOT index it with [0].
- get_obj_xy(obj)
- get_robot_pos()

Available objects dictionary:
objects = {{
    "trash": ["Trash_01_001"],
    "sofa": ["SofaC_01_001"]
}}

Rules:
- All coordinates and distances are meters.
- Do not import modules.
- Do not define functions without calling them.
- Only solve the current task, do not include other tasks.
- For N2, use move_to_obj_by_offset(trash_obj, 0.0, 0.0), avoid extra offsets.
- For N3, move around the sofa in a 4x4 meter square using 5 waypoint calls.
- For N4, record current/start position if needed, move to the min-|x|+|y| trash, then return to the start.

Current task id: {task_id}
Instruction: {instruction}

Previous code failed:
{failed_code}

Failure reason:
{failure_reason}

Generate corrected recovery code now.
"""


def load_scene() -> Dict[str, dict]:
    with open(SCENE_PATH, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def names_containing(scene: Dict[str, dict], *keywords: str) -> List[str]:
    result = []
    for name in scene:
        lower = name.lower()
        if any(key.lower() in lower for key in keywords):
            result.append(name)
    return sorted(result)


def obj_xy(scene: Dict[str, dict], name: str) -> Tuple[float, float]:
    info = scene[name]
    return float(info["object_x"]), float(info["object_y"])


def approach_pose(scene: Dict[str, dict], name: str) -> Tuple[float, float, float]:
    info = scene[name]
    return (
        float(info["approach_x"]),
        float(info["approach_y"]),
        float(info.get("approach_yaw", 0.0)),
    )


def choose_min_abs_sum(scene: Dict[str, dict], names: Iterable[str]) -> str:
    candidates = list(names)
    if not candidates:
        raise RuntimeError("No candidate objects found.")
    return min(candidates, key=lambda n: abs(obj_xy(scene, n)[0]) + abs(obj_xy(scene, n)[1]))


def dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def extract_code(text: str) -> str:
    match = re.search(r"```(?:python)?(.*?)```", text, re.DOTALL)
    return match.group(1).strip() if match else text.strip()


def call_deepseek_prompt(prompt: str) -> str:
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
                    {"role": "user", "content": prompt},
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


def call_deepseek(instruction: str) -> str:
    return call_deepseek_prompt(PROMPT.format(instruction=instruction))


def call_deepseek_adjust(task_id: str, instruction: str, failed_code: str, failure_reason: str) -> str:
    return call_deepseek_prompt(
        ADJUST_PROMPT.format(
            task_id=task_id,
            instruction=instruction,
            failed_code=failed_code,
            failure_reason=failure_reason,
        )
    )


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


@dataclass
class ExecState:
    scene: Dict[str, dict]
    nav_call_count: int = 0
    nav_success_count: int = 0
    errors: List[float] = None

    def __post_init__(self) -> None:
        if self.errors is None:
            self.errors = []

    @property
    def final_error(self) -> float:
        return self.errors[-1] if self.errors else float("nan")

    @property
    def max_error(self) -> float:
        return max(self.errors) if self.errors else float("nan")


def run_e3(repeats: int) -> List[dict]:
    records = []
    run_index = 1
    for task_id, task_type, instruction in TASKS:
        for repeat_id in range(1, repeats + 1):
            print(f"[E3 flash] {task_id} repeat {repeat_id}")
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
            records.append(
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
    return records


def lazy_ros_imports():
    import rospy
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import SetModelState
    import ros_bridge

    return rospy, ModelState, SetModelState, ros_bridge


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def reset_home(ros_bridge, rospy, ModelState, SetModelState) -> None:
    rospy.wait_for_service("/gazebo/set_model_state", timeout=10.0)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    msg = ModelState()
    msg.model_name = "mini_mec_six_arm"
    msg.reference_frame = "world"
    msg.pose.position.x = 0.0
    msg.pose.position.y = 0.0
    msg.pose.position.z = 0.05
    qx, qy, qz, qw = yaw_to_quat(0.0)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    msg.twist.linear.x = 0.0
    msg.twist.linear.y = 0.0
    msg.twist.linear.z = 0.0
    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = 0.0
    resp = set_state(msg)
    if not resp.success:
        raise RuntimeError(f"Gazebo reset failed: {resp.status_message}")

    deadline = time.time() + 5.0
    while time.time() < deadline:
        pos = ros_bridge.get_current_pos()
        yaw = ros_bridge.get_current_orientation()
        if dist(pos, (0.0, 0.0)) < 0.05 and abs(yaw) < 0.10:
            break
        time.sleep(0.1)
    time.sleep(1.0)


class ObjName(str):
    """Object name string that also tolerates the common parse_obj_name(...)[0] mistake."""

    def __getitem__(self, key):
        if key == 0:
            return self
        return super().__getitem__(key)


def deterministic_parse_obj_name(text: str, objects: Dict[str, List[str]]) -> str:
    lower = text.lower()
    if "trash" in lower or "bin" in lower:
        return ObjName(objects["trash"][0])
    if "sofa" in lower:
        return ObjName(objects["sofa"][0])
    raise ValueError(f"Cannot parse object from text: {text}")


def execute_code(task_id: str, code: str, scene: Dict[str, dict], ros_bridge, state: Optional[ExecState] = None) -> ExecState:
    if state is None:
        state = ExecState(scene=scene)
    objects = {
        "trash": names_containing(scene, "trash", "bin"),
        "sofa": names_containing(scene, "sofa"),
    }

    # Put min-|x|+|y| candidates first so simple parse_obj_name remains deterministic.
    objects["trash"] = [choose_min_abs_sum(scene, objects["trash"])]
    objects["sofa"] = [choose_min_abs_sum(scene, objects["sofa"])]

    def move_to_xy(x: float, y: float) -> None:
        state.nav_call_count += 1
        ros_bridge.move_to_goal(float(x), float(y))
        state.nav_success_count += 1
        pos = ros_bridge.get_current_pos()
        state.errors.append(dist(pos, (float(x), float(y))))

    def move_to_obj_by_offset(obj: str, dx: float, dy: float) -> None:
        state.nav_call_count += 1
        ax, ay, ayaw = approach_pose(scene, obj)
        target = (ax + float(dx), ay + float(dy))
        ros_bridge.move_to_goal(target[0], target[1], ayaw)
        state.nav_success_count += 1
        pos = ros_bridge.get_current_pos()
        state.errors.append(dist(pos, target))

    def get_obj_xy(obj: str) -> Tuple[float, float]:
        return obj_xy(scene, obj)

    def get_robot_pos() -> Tuple[float, float]:
        return ros_bridge.get_current_pos()

    allowed_builtins = {
        "__import__": builtins.__import__,
        "abs": abs,
        "min": min,
        "max": max,
        "range": range,
        "len": len,
        "enumerate": enumerate,
        "float": float,
        "int": int,
        "list": list,
        "tuple": tuple,
        "dict": dict,
        "print": print,
        "sum": sum,
        "sorted": sorted,
        "zip": zip,
        "reversed": reversed,
    }
    env = {
        "__name__": "__generated__",
        "__builtins__": allowed_builtins,
        "objects": objects,
        "move_to_xy": move_to_xy,
        "move_to_obj_by_offset": move_to_obj_by_offset,
        "parse_obj_name": deterministic_parse_obj_name,
        "get_obj_xy": get_obj_xy,
        "get_robot_pos": get_robot_pos,
        "math": math,
    }
    exec(code, env, env)
    return state


def run_execution(repeats: int, adjust_policy: int, stage: str, start_index: int = 1) -> List[dict]:
    rospy, ModelState, SetModelState, ros_bridge = lazy_ros_imports()
    ros_bridge.init()
    scene = load_scene()

    records = []
    run_index = start_index
    for task_id, task_type, instruction in TASKS:
        for repeat_id in range(1, repeats + 1):
            print(f"[{stage} flash] {task_id} repeat {repeat_id} adjust={adjust_policy}")
            reset_home(ros_bridge, rospy, ModelState, SetModelState)
            t0 = time.time()
            failure_reason = ""
            total_state = ExecState(scene=scene)
            last_state = ExecState(scene=scene)
            attempt_codes: List[str] = []
            checks = {
                "semantic_parse_correct": "no",
                "decomposition_correct": "no",
                "code_executable": "no",
                "human_note": "",
            }
            final_result = "Fail"
            success_attempt: Optional[int] = None
            adjust_count = 0
            last_code = ""
            max_attempts = 1 + max(0, adjust_policy)

            for attempt_idx in range(max_attempts):
                attempt_state: Optional[ExecState] = None
                try:
                    if attempt_idx == 0:
                        code = call_deepseek(instruction)
                    else:
                        adjust_count += 1
                        code = call_deepseek_adjust(task_id, instruction, last_code, failure_reason)
                    last_code = code
                    attempt_codes.append(f"# === attempt {attempt_idx + 1} ===\n{code}")

                    attempt_checks = static_check(task_id, code)
                    if attempt_idx == 0:
                        checks = attempt_checks
                    if attempt_checks["code_executable"] != "yes":
                        raise RuntimeError(attempt_checks["human_note"] or "code not executable")

                    attempt_state = ExecState(scene=scene)
                    execute_code(task_id, code, scene, ros_bridge, state=attempt_state)
                    total_state.nav_call_count += attempt_state.nav_call_count
                    total_state.nav_success_count += attempt_state.nav_success_count
                    total_state.errors.extend(attempt_state.errors)
                    last_state = attempt_state

                    measured_error = attempt_state.max_error if task_id == "N3" else attempt_state.final_error
                    if measured_error <= TOLERANCE_M:
                        final_result = "Success"
                        success_attempt = attempt_idx + 1
                        failure_reason = ""
                        break
                    failure_reason = f"position_error {measured_error:.3f}m > tolerance {TOLERANCE_M:.3f}m"
                except Exception as e:
                    # If a navigation call failed inside execute_code, count the call from the
                    # partially mutated attempt_state when it exists.
                    if attempt_state is not None:
                        total_state.nav_call_count += attempt_state.nav_call_count
                        total_state.nav_success_count += attempt_state.nav_success_count
                        total_state.errors.extend(attempt_state.errors)
                        last_state = attempt_state
                    failure_reason = str(e)
                    final_result = "Fail"
                    if attempt_idx >= max_attempts - 1:
                        break

            total_time_s = time.time() - t0
            final_error = last_state.max_error if task_id == "N3" else last_state.final_error
            code = "\n\n".join(attempt_codes)
            success_at_1 = "yes" if success_attempt is not None and success_attempt <= 1 else "no"
            success_at_2 = ""
            success_at_3 = ""
            if adjust_policy >= 1:
                success_at_2 = "yes" if success_attempt is not None and success_attempt <= 2 else "no"
            if adjust_policy >= 2:
                success_at_3 = "yes" if success_attempt is not None and success_attempt <= 3 else "no"
            records.append(
                {
                    "run_id": f"{stage}_{run_index:03d}",
                    "task_id": task_id,
                    "task_type": task_type,
                    "instruction": instruction,
                    "method_type": "LLM",
                    "model_name": MODEL_LABEL,
                    "temperature": TEMPERATURE,
                    "adjust_policy": adjust_policy,
                    "repeat_id": repeat_id,
                    "generated_code": code,
                    "semantic_parse_correct": checks["semantic_parse_correct"],
                    "decomposition_correct": checks["decomposition_correct"],
                    "code_executable": checks["code_executable"],
                    "api_call_count": total_state.nav_call_count,
                    "adjust_count": adjust_count,
                    "success_at_1": success_at_1,
                    "success_at_2": success_at_2,
                    "success_at_3": success_at_3,
                    "final_result": final_result,
                    "total_time_s": f"{total_time_s:.1f}",
                    "nav_call_count": total_state.nav_call_count,
                    "pickup_call_count": 0,
                    "putdown_call_count": 0,
                    "nav_success_count": total_state.nav_success_count,
                    "pickup_success_count": 0,
                    "putdown_success_count": 0,
                    "final_position_error": f"{final_error:.3f}" if not math.isnan(final_error) else "",
                    "failure_reason": failure_reason,
                    "vlm_summary": "not used",
                    "human_note": checks["human_note"],
                }
            )
            run_index += 1
    return records


def write_csv(path: str, fields: List[str], rows: List[dict]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"[csv] wrote {len(rows)} rows: {path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--repeats",
        type=int,
        default=None,
        help="Compatibility option: use the same repeat count for E1/E2/E3.",
    )
    parser.add_argument("--e1-repeats", type=int, default=DEFAULT_E1_REPEATS)
    parser.add_argument("--e2-repeats", type=int, default=DEFAULT_E2_REPEATS)
    parser.add_argument("--e3-repeats", type=int, default=DEFAULT_E3_REPEATS)
    parser.add_argument("--e3-only", action="store_true")
    parser.add_argument("--execute", action="store_true", help="Run E1/E2 execution stages in Gazebo.")
    args = parser.parse_args()

    if args.repeats is not None:
        e1_repeats = args.repeats
        e2_repeats = args.repeats
        e3_repeats = args.repeats
    else:
        e1_repeats = args.e1_repeats
        e2_repeats = args.e2_repeats
        e3_repeats = args.e3_repeats

    e3_rows = run_e3(e3_repeats)
    write_csv(os.path.join(OUTPUT_DIR, "E3_flash.csv"), E3_FIELDS, e3_rows)

    if args.execute and not args.e3_only:
        e1_rows = run_execution(e1_repeats, adjust_policy=2, stage="E1")
        write_csv(os.path.join(OUTPUT_DIR, "E1_flash.csv"), EXEC_FIELDS, e1_rows)

        e2_all_rows: List[dict] = []
        next_index = 1
        for policy in (0, 1):
            e2_rows = run_execution(e2_repeats, adjust_policy=policy, stage="E2", start_index=next_index)
            e2_all_rows.extend(e2_rows)
            next_index += len(e2_rows)
        write_csv(os.path.join(OUTPUT_DIR, "E2_flash.csv"), EXEC_FIELDS, e2_all_rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
