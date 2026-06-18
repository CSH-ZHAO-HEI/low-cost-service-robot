#!/usr/bin/env python3
"""
C3 sub-task: 只跑紅塊 → NightStand_01_002（1/3 of C3）。

用途：錄屏 / demo。流程完整 — 走 BigBrain → LLM 規劃 → VLM 判斷 → AdjustLLM 微調。
但只指示 LLM 處理紅塊，不碰藍/黃，跑得短乾淨。

用法：
  python3 C1-C4/run_c3_red_only.py
  python3 C1-C4/run_c3_red_only.py --adjust-policies 2     # 多給微調機會
  python3 C1-C4/run_c3_red_only.py --no-helper-prompt      # 強制 primitive 組合（預設）
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(HERE)
BIG_BRAIN_DIR = os.path.join(PROJECT_ROOT, "big_brain")
sys.path.insert(0, BIG_BRAIN_DIR)
sys.path.insert(0, HERE)

# 隱藏的 code 區塊（start_marker → end_marker）
HIDE_BLOCKS = [
    ("========== 生成的执行计划 ==========", "==================================== (模型:"),
    ("========== AdjustLLM 生成的微调/恢复代码 ==========", "===================================================="),
]
# LLM raw output 區塊：由 "原始输出检查" 開始，下兩條 "====" 圍欄夾住內容
RAW_OUT_MARKER = "原始输出检查"


def _replay_logs_without_generated_plan(text: str) -> None:
    """Print captured logs but hide ALL code blocks (initial plan, adjust code,
    raw LLM output). Keep judge/VLM/robot_api/ros_bridge process visible.
    """
    in_hide_end = None        # 等待這個 marker 結束隱藏
    in_raw = False            # 是否在 LLM raw output 區塊內
    raw_fence_seen = 0        # 已看到幾個 "====" 圍欄
    for line in (text or "").splitlines():
        # 處理「等待結束 marker」的隱藏狀態
        if in_hide_end is not None:
            if in_hide_end in line:
                in_hide_end = None
            continue
        # 處理 raw output（兩個 ==== 圍欄）
        if in_raw:
            stripped = line.strip()
            if stripped.startswith("=") and set(stripped) == {"="}:
                raw_fence_seen += 1
                if raw_fence_seen >= 2:
                    in_raw = False
                    raw_fence_seen = 0
            continue
        # 偵測 start markers
        matched = False
        for start, end in HIDE_BLOCKS:
            if start in line:
                print(f"[filtered] {start.strip('= ')} (隱藏)")
                in_hide_end = end
                matched = True
                break
        if matched:
            continue
        if line.strip() == RAW_OUT_MARKER:
            print("[filtered] LLM raw output (隱藏)")
            in_raw = True
            raw_fence_seen = 0
            continue
        print(line)

def _read_new_judge_events(log_path: str, start_offset: int) -> list[dict]:
    if not os.path.exists(log_path):
        return []
    rows: list[dict] = []
    with open(log_path, "r", encoding="utf-8") as f:
        f.seek(start_offset)
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except Exception:
                rows.append({"event": "unparsed", "raw": line})
    return rows

def _print_judge_event_summary(events: list[dict]) -> None:
    print("\n[judge-events] --- new events this run ---")
    if not events:
        print("[judge-events] (none)")
        return
    for e in events:
        ev = e.get("event", "?")
        if ev == "judge":
            vr = e.get("vlm_result") or {}
            print(
                f"[judge] final_pass={e.get('final_pass')} "
                f"rule_pass={e.get('rule_pass')} vlm_needed={e.get('vlm_needed')} "
                f"vlm_pass={vr.get('pass')} reason={str(vr.get('reason', ''))[:180]}"
            )
        elif ev == "replan_generated":
            code = str(e.get("code", e.get("generated_code", ""))).replace("\n", " ")
            print(f"[replan_generated] code={code[:180]}")
        elif ev == "replan_executed":
            print(f"[replan_executed] ok={e.get('ok')} reason={e.get('reason', '')}")
        elif ev == "replan_failed":
            print(f"[replan_failed] reason={e.get('reason', '')}")
        else:
            print(f"[{ev}] {str(e)[:220]}")
    print("[judge-events] --- end ---")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--adjust-policies", type=int, default=1,
                    help="max_replan_times for AdjustLLM (0=disabled, 1=default, 2=more)")
    ap.add_argument("--no-helper-prompt", action="store_true", default=True,
                    help="Strip run_c*_* helpers from BASE_PROMPT (forces primitive composition)")
    ap.add_argument("--reset-block", action="store_true", default=True,
                    help="Spawn red_block at C3 default position before run")
    ap.add_argument("--start-writer", action="store_true",
                    help="Auto-start vlm_image_writer in background")
    args = ap.parse_args()

    # 強制 no-helper（跟 batch 一致）
    if args.no_helper_prompt:
        import run_e2_composite
        run_e2_composite.patch_base_prompt_remove_c_helpers()
        os.environ["BIGBRAIN_DISABLE_C_HELPER_CANONICAL"] = "1"

    # 起 vlm_image_writer（雙 topic）
    writer_proc = None
    if args.start_writer:
        import subprocess
        cmd = [sys.executable, os.path.join(BIG_BRAIN_DIR, "vlm_image_writer.py"),
               "/judge_camera/rgb/image_raw"]
        print(f"[writer] starting: {' '.join(cmd)}")
        writer_proc = subprocess.Popen(cmd, cwd=BIG_BRAIN_DIR)
        time.sleep(1.0)

    try:
        # 設 max_replan_times
        from config import MAX_REPLAN_TIMES  # noqa
        import config
        config.MAX_REPLAN_TIMES = args.adjust_policies
        judge_log_path = os.path.join(config.JUDGE_LOG_DIR, "judge_events.jsonl")
        judge_start_offset = os.path.getsize(judge_log_path) if os.path.exists(judge_log_path) else 0

        # 用 CompositeTester 把紅塊放回 C3 預設位置（-2.005, -4.75）
        if args.reset_block:
            import composite_capability_test as comp
            tester = comp.CompositeTester(pause_after_reset=0.0)
            print("[reset] spawning red_block at C3 default position")
            tester.reset_c3()
            # tester.run_c3 預設會 reset 紅藍黃 — 這裡完整 reset 但只跑紅
            # 重要：reset 後手動把 judge 還原成真 JudgeLLM
            from action import robot_api
            from model.llm import JudgeLLM
            # 重新實例化（reset_c3 不會動 judge_llm，但 CompositeTester.__init__ 會 mock）
            robot_api.judge_llm = JudgeLLM()
            robot_api.judge_llm.max_replan_times = args.adjust_policies
            print(f"[reset] real JudgeLLM restored, max_replan_times={args.adjust_policies}")

        # 跑 BigBrain
        from big_brain import BigBrain
        brain = BigBrain()
        brain._save_history = lambda path: None  # 不污染 RAG history

        instruction = "Pick up the red_block and put it on NightStand_01_002"
        print()
        print("=" * 72)
        print(f"[task] {instruction}")
        print(f"[config] adjust_policies={args.adjust_policies}, no_helper={args.no_helper_prompt}")
        print("=" * 72)

        t0 = time.time()
        captured = io.StringIO()
        with contextlib.redirect_stdout(captured):
            ok = brain.run_once(instruction)
        _replay_logs_without_generated_plan(captured.getvalue())
        new_events = _read_new_judge_events(judge_log_path, judge_start_offset)
        _print_judge_event_summary(new_events)
        elapsed = time.time() - t0

        print()
        print("=" * 72)
        print(f"[result] success={ok}, elapsed={elapsed:.1f}s")
        # 檢查紅塊最終位置（NightStand_01_002）
        try:
            import ros_bridge
            info = ros_bridge.get_obj_info("red_block")
            target = ros_bridge.get_obj_info("NightStand_01_002")
            import math
            tx = float(target.get("place_x", target["object_x"]))
            ty = float(target.get("place_y", target["object_y"]))
            dx = info["object_x"] - tx
            dy = info["object_y"] - ty
            dist = math.hypot(dx, dy)
            print(f"[check] red_block final pos = ({info['object_x']:.2f}, {info['object_y']:.2f})")
            print(f"[check] distance to NightStand_01_002 target = {dist:.3f}m  "
                  f"({'ON TARGET' if dist < 0.25 else 'MISSED'})")
        except Exception as e:
            print(f"[check] could not verify red_block position: {e}")
        print("=" * 72)

    finally:
        if writer_proc is not None:
            print("[writer] stopping vlm_image_writer")
            writer_proc.terminate()
            try:
                writer_proc.wait(timeout=2)
            except Exception:
                writer_proc.kill()


if __name__ == "__main__":
    main()
