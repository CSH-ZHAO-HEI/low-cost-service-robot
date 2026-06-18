#!/usr/bin/env python3
"""
E3 static evaluation runner.

This script only generates high-level plans and saves them to CSV/JSON.
It does not execute any robot actions.
"""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from openai import OpenAI

import config as cfg
from model.rag import RAGManager
from prompt.task_prompt import BASE_PROMPT
from utils.utils import extract_code


# ============================================================================
# Manual config block
# Edit this section before running the script.
# ============================================================================
RUN_MODE = "batch"  # "single" or "batch"
# RUN_MODE = "single"
EXISTING_RECORD_POLICY = "skip"  # "skip" or "overwrite"
EXISTING_RECORD_POLICY = "overwrite"

print(f"RUN_MODE: {RUN_MODE}")
print(f"EXISTING_RECORD_POLICY: {EXISTING_RECORD_POLICY}")

SELECTED_TASK_IDS = [
    # "N1",
    # "N2",
    # "N3",
    "N4",
]

SELECTED_MODELS = [
    "GPT-4o",
    # "OLLAMA",
]

SELECTED_TEMPERATURES = [
    0.0,
    # 0.4,
    # 0.8,
]

REPEATS = 1
RUN_ID = "E3_static_run_001"
ENABLE_RAG = True

OUTPUT_DIR = Path(__file__).resolve().parent / "outputs" / "e3_static"
OUTPUT_JSON = OUTPUT_DIR / "e3_static_eval.json"
OUTPUT_CSV = OUTPUT_DIR / "e3_static_eval.csv"

SINGLE_TEST_CASE = {
    "task_id": "N1",
    "model": "OLLAMA",
    "temperature": 0.0,
    "repeat_id": 1,
}


FIELDNAMES = [
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

TASKS = [
    {
        "task_id": "N1",
        "task_type": "Navigation",
        "instruction": "Navigate to (100,200)",
    },
    {
        "task_id": "N2",
        "task_type": "Navigation",
        "instruction": "Navigate next to the trash can",
    },
    {
        "task_id": "N3",
        "task_type": "Navigation",
        "instruction": "Move in a 50 by 50 square around the trash can",
    },
    {
        "task_id": "N4",
        "task_type": "Navigation",
        "instruction": "Go to the trash can with the smallest sum of absolute coordinate values, circle around it with a radius of 50, then return to the starting position",
    },
    {
        "task_id": "G1",
        "task_type": "Manipulation",
        "instruction": "Pick up the red cube",
    },
    {
        "task_id": "G2",
        "task_type": "Manipulation",
        "instruction": "Pick up the red cube on the ground",
    },
    {
        "task_id": "G3",
        "task_type": "Manipulation",
        "instruction": "Put the object on the table",
    },
    {
        "task_id": "G4",
        "task_type": "Manipulation",
        "instruction": "Put the object between the blue cube and the yellow cube",
    },
    {
        "task_id": "C1",
        "task_type": "Composite",
        "instruction": "Put the water cup on the sofa onto the chair",
    },
    {
        "task_id": "C2",
        "task_type": "Composite",
        "instruction": "Pick up the red cube from the table and place it between the blue cube and the yellow cube",
    },
    {
        "task_id": "C3",
        "task_type": "Composite",
        "instruction": "Put all non-furniture items on the ground into the trash can",
    },
    {
        "task_id": "C4",
        "task_type": "Composite",
        "instruction": "Move around the table in a square pattern with side length 100; During the moving, when a red cube appears, pick it up and put it into the trash can",
    },
]

TASKS_ZH = [
    {
        "task_id": "N1",
        "task_type": "Navigation",
        "instruction": "导航到(100,200)",
    },
    {
        "task_id": "N2",
        "task_type": "Navigation",
        "instruction": "导航到垃圾桶旁边",
    },
    {
        "task_id": "N3",
        "task_type": "Navigation",
        "instruction": "绕垃圾桶做长50，宽50的方形运动",
    },
    {
        "task_id": "N4",
        "task_type": "Navigation",
        "instruction": "前往坐标绝对值之和最小的垃圾桶旁边，以50为半径绕行一圈，随后返回出发位置",
    },
    {
        "task_id": "G1",
        "task_type": "Manipulation",
        "instruction": "夹起红色方块",
    },
    {
        "task_id": "G2",
        "task_type": "Manipulation",
        "instruction": "夹起位于地面的红色方块",
    },
    {
        "task_id": "G3",
        "task_type": "Manipulation",
        "instruction": "把物体放到桌子上",
    },
    {
        "task_id": "G4",
        "task_type": "Manipulation",
        "instruction": "把物体放到蓝色方块和黄色方块中间",
    },
    {
        "task_id": "C1",
        "task_type": "Composite",
        "instruction": "把沙发上的水杯放到椅子上",
    },
    {
        "task_id": "C2",
        "task_type": "Composite",
        "instruction": "把红色方块从桌子上拿起，放到蓝色方块和黄色方块中间",
    },
    {
        "task_id": "C3",
        "task_type": "Composite",
        "instruction": "把地上所有的非家具丢到垃圾桶中",
    },
    {
        "task_id": "C4",
        "task_type": "Composite",
        "instruction": "以长度为100的方形方式绕着桌子运动，当有红色方块出现时，夹起他，放到垃圾桶中",
    },
]


MODEL_REGISTRY = {
    "GPT-4o": {
        "api_key": cfg.GPT_API_KEY,
        "base_url": cfg.GPT_BASE_URL,
        "model_name": cfg.GPT_LLM_MODEL,
    },
    "DeepSeek-V3.2": {
        "api_key": cfg.SILICONFLOW_API_KEY,
        "base_url": cfg.SILICONFLOW_BASE_URL,
        "model_name": cfg.SILICONFLOW_LLM_MODEL,
    },
    "OLLAMA": {
        "api_key": cfg.OLLAMA_API_KEY,
        "base_url": cfg.OLLAMA_BASE_URL,
        "model_name": cfg.OLLAMA_LLM_MODEL,
    },
}


@dataclass(frozen=True)
class TaskSpec:
    task_id: str
    task_type: str
    instruction: str


class E3StaticEvaluator:
    def __init__(self) -> None:
        self.script_dir = Path(__file__).resolve().parent
        self.history_path = self.script_dir / "memory" / "rag_history.json"
        self.output_dir = OUTPUT_DIR
        self.output_json = OUTPUT_JSON
        self.output_csv = OUTPUT_CSV
        self.tasks = {item["task_id"]: TaskSpec(**item) for item in TASKS}
        self.history_data = self._load_history(self.history_path)
        self.rag_manager = RAGManager(self.history_data) if ENABLE_RAG else None

    def _load_history(self, path: Path) -> List[dict]:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)

    def _load_existing_payload(self) -> dict:
        if not self.output_json.exists():
            return {"meta": {}, "records": []}
        with self.output_json.open("r", encoding="utf-8") as f:
            payload = json.load(f)
        records = payload.get("records", [])
        if not isinstance(records, list):
            raise ValueError(f"Invalid records format in {self.output_json}")
        return {
            "meta": payload.get("meta", {}),
            "records": records,
        }

    def _write_outputs(self, records: Sequence[dict]) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        payload = {
            "meta": self._build_meta(),
            "records": list(records),
        }
        with self.output_json.open("w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)

        with self.output_csv.open("w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
            writer.writeheader()
            writer.writerows(records)

    def _build_meta(self) -> dict:
        return {
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "run_mode": RUN_MODE,
            "existing_record_policy": EXISTING_RECORD_POLICY,
            "run_id": RUN_ID,
            "enable_rag": ENABLE_RAG,
            "selected_task_ids": SELECTED_TASK_IDS,
            "selected_models": SELECTED_MODELS,
            "selected_temperatures": SELECTED_TEMPERATURES,
            "repeats": REPEATS,
            "single_test_case": SINGLE_TEST_CASE,
            "fieldnames": FIELDNAMES,
            "output_json": str(self.output_json),
            "output_csv": str(self.output_csv),
        }

    def _record_key(self, record: dict) -> Tuple[str, str, float, int]:
        return (
            str(record["task_id"]),
            str(record["model_name"]),
            float(record["temperature"]),
            int(record["repeat_id"]),
        )

    def _existing_keys(self, records: Iterable[dict]) -> set[Tuple[str, str, float, int]]:
        return {self._record_key(record) for record in records}

    def _validate_selection(self, task_ids: Sequence[str], models: Sequence[str], temperatures: Sequence[float]) -> None:
        unknown_tasks = sorted(set(task_ids) - set(self.tasks))
        unknown_models = sorted(set(models) - set(MODEL_REGISTRY))
        if unknown_tasks:
            raise ValueError(f"Unknown task ids: {unknown_tasks}")
        if unknown_models:
            raise ValueError(f"Unknown models: {unknown_models}")
        if not temperatures:
            raise ValueError("SELECTED_TEMPERATURES cannot be empty")
        if EXISTING_RECORD_POLICY not in {"skip", "overwrite"}:
            raise ValueError(
                f"EXISTING_RECORD_POLICY must be 'skip' or 'overwrite', got: {EXISTING_RECORD_POLICY}"
            )

    def _find_record_index(
        self,
        records: Sequence[dict],
        record_key: Tuple[str, str, float, int],
    ) -> Optional[int]:
        for idx, record in enumerate(records):
            if self._record_key(record) == record_key:
                return idx
        return None

    def _build_prompt(self, instruction: str) -> Tuple[str, str]:
        rag_context = ""
        if self.rag_manager is not None:
            rag_context = self.rag_manager.retrieve(instruction)
        final_prompt = BASE_PROMPT + "\n"
        if rag_context:
            final_prompt += rag_context + "\n"
        final_prompt += f"# {instruction}\n?"
        return final_prompt, rag_context

    def _call_planner_llm(self, prompt: str, model_label: str, temperature: float) -> str:
        model_config = MODEL_REGISTRY[model_label]
        client = OpenAI(
            api_key=model_config["api_key"],
            base_url=model_config["base_url"],
        )
        response = client.chat.completions.create(
            model=model_config["model_name"],
            messages=[
                {"role": "system", "content": "you only need to use code to answer the ? part and nothing else"},
                {"role": "user", "content": prompt},
            ],
            temperature=temperature,
            top_p=None,
        )
        return response.choices[0].message.content or ""

    def _build_record(
        self,
        task: TaskSpec,
        model_label: str,
        temperature: float,
        repeat_id: int,
        generated_code: str,
        human_note: str = "",
    ) -> dict:
        return {
            "run_id": RUN_ID,
            "task_id": task.task_id,
            "task_type": task.task_type,
            "instruction": task.instruction,
            "model_name": model_label,
            "temperature": temperature,
            "repeat_id": repeat_id,
            "generated_code": generated_code,
            "semantic_parse_correct": "",
            "decomposition_correct": "",
            "code_executable": "",
            "human_note": human_note,
        }

    def run_one_case(self, task_id: str, model_label: str, temperature: float, repeat_id: int) -> dict:
        self._validate_selection([task_id], [model_label], [temperature])
        task = self.tasks[task_id]
        prompt, rag_context = self._build_prompt(task.instruction)

        print("=" * 80)
        print("Running single static evaluation case")
        print(f"task_id      : {task.task_id}")
        print(f"task_type    : {task.task_type}")
        print(f"instruction  : {task.instruction}")
        print(f"model        : {model_label}")
        print(f"temperature  : {temperature}")
        print(f"repeat_id    : {repeat_id}")
        print(f"RAG matched  : {'yes' if rag_context else 'no'}")
        if rag_context:
            print("RAG context:")
            print(rag_context)
        print("-" * 80)
        print("Prompt tail:")
        print("\n".join(prompt.splitlines()[-12:]))
        print("-" * 80)

        human_note = ""
        try:
            raw_text = self._call_planner_llm(prompt, model_label, temperature)
            last_line = rag_context.strip().splitlines()[-1] if rag_context.strip() else ""
            generated_code = extract_code(raw_text, last_line)
        except Exception as exc:
            raw_text = ""
            generated_code = ""
            human_note = f"generation_error: {type(exc).__name__}: {exc}"

        print("Raw model output:")
        print(raw_text or "<empty>")
        print("-" * 80)
        print("\033[1;31mExtracted generated_code:\033[0m")
        print(generated_code or "<empty>")
        if human_note:
            print("-" * 80)
            print(f"Note: {human_note}")
        print("=" * 80)

        return self._build_record(
            task=task,
            model_label=model_label,
            temperature=temperature,
            repeat_id=repeat_id,
            generated_code=generated_code,
            human_note=human_note,
        )

    def run_single_test(self) -> dict:
        record = self.run_one_case(
            task_id=SINGLE_TEST_CASE["task_id"],
            model_label=SINGLE_TEST_CASE["model"],
            temperature=float(SINGLE_TEST_CASE["temperature"]),
            repeat_id=int(SINGLE_TEST_CASE["repeat_id"]),
        )
        self._write_outputs([record])
        print(f"Saved single-test JSON to: {self.output_json}")
        print(f"Saved single-test CSV  to: {self.output_csv}")
        return record

    def run_batch(self) -> List[dict]:
        self._validate_selection(SELECTED_TASK_IDS, SELECTED_MODELS, SELECTED_TEMPERATURES)
        existing_payload = self._load_existing_payload()
        records = existing_payload["records"]
        completed_keys = self._existing_keys(records)

        total = len(SELECTED_TASK_IDS) * len(SELECTED_MODELS) * len(SELECTED_TEMPERATURES) * REPEATS
        processed = 0
        skipped = 0

        print(f"Loaded existing records: {len(records)}")
        print(f"Planned combinations    : {total}")

        for task_id in SELECTED_TASK_IDS:
            task = self.tasks[task_id]
            for model_label in SELECTED_MODELS:
                for temperature in SELECTED_TEMPERATURES:
                    for repeat_id in range(1, REPEATS + 1):
                        probe_record = self._build_record(
                            task=task,
                            model_label=model_label,
                            temperature=float(temperature),
                            repeat_id=repeat_id,
                            generated_code="",
                        )
                        record_key = self._record_key(probe_record)
                        if record_key in completed_keys:
                            if EXISTING_RECORD_POLICY == "skip":
                                skipped += 1
                                print(f"[SKIP] {record_key} already exists")
                                continue
                            print(f"[OVERWRITE] {record_key} already exists and will be replaced")

                        print(f"[RUN ] {record_key}")
                        record = self.run_one_case(
                            task_id=task_id,
                            model_label=model_label,
                            temperature=float(temperature),
                            repeat_id=repeat_id,
                        )
                        existing_index = self._find_record_index(records, record_key)
                        if existing_index is None:
                            records.append(record)
                        else:
                            records[existing_index] = record
                        completed_keys.add(record_key)
                        processed += 1
                        self._write_outputs(records)
                        print(f"[SAVE] progress={processed} new / {skipped} skipped / {len(records)} total saved")

        print("=" * 80)
        print("Batch run finished")
        print(f"new records : {processed}")
        print(f"skipped     : {skipped}")
        print(f"total saved : {len(records)}")
        print(f"json output : {self.output_json}")
        print(f"csv output  : {self.output_csv}")
        print("=" * 80)
        return records


def main() -> None:
    evaluator = E3StaticEvaluator()
    if RUN_MODE == "single":
        evaluator.run_single_test()
        return
    if RUN_MODE == "batch":
        evaluator.run_batch()
        return
    raise ValueError(f"RUN_MODE must be 'single' or 'batch', got: {RUN_MODE}")


if __name__ == "__main__":
    main()
