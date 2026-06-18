#!/usr/bin/env python3
"""
Demo：红色方块（地上）→ CoffeeTable 桌面 (纯展示，跑一次)

特点：
  1. 互动输入界面 — 支持中文 / 英文指令，例如：
       "把红色方块放到茶几上" 或 "Put the red block on the coffee table"
  2. 完整输出 — LLM 生成计划、规则层 Judge、VLM Judge 原始输出、AdjustLLM
     微调代码全部即时打印，不隐藏任何区块。
  3. 红块起始位置在地面、离 CoffeeTable_01_001 较远，放置时容易因定位/手臂
     误差让方块没有真正落在桌面上 → VLM Judge 判定失败 →
     AdjustLLM 重新拾取 → 再次放上桌面。

用法：
  python3 C1-C4/red_to_coffeetable.py
"""
from __future__ import annotations

import contextlib
import io
import math
import os
import re
import subprocess
import sys
import time

# 抑制 HuggingFace/transformers 的下载警告与进度条（"Loading weights: ...%",
# "Batches: ...%", "MPNetModel LOAD REPORT", HF_TOKEN 提醒等）— 这些是写到
# stderr 的 tqdm/log，跟输出的简繁体无关，但同样是 demo 不需要的杂讯。
os.environ.setdefault("HF_HUB_DISABLE_PROGRESS_BARS", "1")
os.environ.setdefault("TRANSFORMERS_VERBOSITY", "error")
os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")
os.environ.setdefault("HF_HUB_OFFLINE", "0")

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(HERE)
BIG_BRAIN_DIR = os.path.join(PROJECT_ROOT, "big_brain")
sys.path.insert(0, BIG_BRAIN_DIR)
sys.path.insert(0, HERE)


RED_KEYWORDS = ["红色方块", "紅色方塊", "红方块", "紅方塊", "red block", "red_block", "red cube"]
TABLE_KEYWORDS = ["茶几", "桌子", "桌面", "咖啡桌", "coffeetable", "coffee table", "table", "desk"]

CANONICAL_INSTRUCTION = "Pick up the red_block and put it on CoffeeTable_01_001"

# 红块起始位置：NightStand_01_002 旁，朝 C3_BLUE_POSE 方向偏移，并再往南移 0.5m
# (NightStand_01_002 object = (-4.407, 2.86)；偏移方向指向 C3_BLUE_POSE (3.296, 1.50)；南 = -Y)
# 再往南多移 0.3m + 0.15m + 0.15m（用户反馈：方块位置南多一点）
RED_START_POSE = (-3.915, 1.673, 0.025)

ADJUST_POLICY = 2


def parse_instruction(raw: str) -> str:
    low = raw.strip().lower()
    has_red = any(k.lower() in low for k in RED_KEYWORDS)
    has_table = any(k.lower() in low for k in TABLE_KEYWORDS)
    if has_red and has_table:
        return CANONICAL_INSTRUCTION
    return raw.strip()


def _quiet(fn, *args, **kwargs):
    """执行 fn，吞掉内部冗长的 print 与 stderr 杂讯（exception 仍会抛出）。"""
    out_buf = io.StringIO()
    err_buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(out_buf), contextlib.redirect_stderr(err_buf):
            return fn(*args, **kwargs)
    except Exception:
        sys.stdout.write(out_buf.getvalue())
        sys.stderr.write(err_buf.getvalue())
        raise


_ACTION_ICONS = {
    "导航中...": "🧭",
    "抓取方块中...": "🧲",
    "放置方块中...": "📦",
}


class _OutputFilter(io.TextIOBase):
    """过滤掉 RAG 分数表、生成计划原始码、L2 memory、[robot_api]/[BigBrain] 内部
    log 等技术杂讯，把导航/抓取/放置动作、Judge、VLM、AdjustLLM 重新包装成
    带 emoji / box-drawing 的精简输出，只给用户看重点。"""

    _DROP_LINES = {
        "planning……", "finish planning", "开始微调",
        "Task not completed. Initiating local replanning...",
    }

    def __init__(self, real):
        self.real = real
        self.buf = ""
        # None | 'rag_scores' | 'rag_ref' | 'raw_check' | 'l2_memory' | 'plan_code'
        # | 'vlm_raw' | 'adjust_code'
        self.suppress = None
        self._eq_count = 0
        self._last_action_status = None  # 上一個已立即印出的動作狀態（避免重複）
        self._vlm_active = False
        self._vlm_raw_lines = []
        self._adjust_code_lines = []

    def write(self, s):
        self.buf += s
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            self._handle_line(line)
        return len(s)

    def flush(self):
        self.real.flush()

    def close(self):
        if self.buf:
            self._handle_line(self.buf)
            self.buf = ""

    def _emit_vlm_raw_box(self):
        self.real.write("┌─── [JudgeLLM] Raw VLM Output ───┐\n")
        for l in self._vlm_raw_lines:
            self.real.write(f"│ {l}\n")
        self.real.write("└──────────────────────────────────┘\n")
        self._vlm_raw_lines = []

    def _emit_judge_fail_box(self, result_text):
        m = re.match(r"^FAIL\s*\((.*)\)$", result_text)
        reason = m.group(1) if m else result_text
        self.real.write("┌─── [Judge] ❌ FAIL ───┐\n")
        self.real.write(f"│ {reason}\n")
        self.real.write("└──────────────────────┘\n")

    def _emit_adjust_code_box(self):
        self.real.write("┌─── 🛠️ 恢复代码 ───┐\n")
        for l in self._adjust_code_lines:
            self.real.write(f"│ {l}\n")
        self.real.write("└──────────────────────┘\n")
        self._adjust_code_lines = []

    def _handle_line(self, line):
        stripped = line.strip()

        if self.suppress == 'rag_scores':
            if re.match(r"^\d+ : ", stripped):
                return
            self.suppress = None
        elif self.suppress == 'rag_ref':
            if not stripped.startswith("任务："):
                return
            self.suppress = None
        elif self.suppress == 'raw_check':
            if stripped and set(stripped) == {"="}:
                self._eq_count += 1
                if self._eq_count >= 2:
                    self.suppress = None
            return
        elif self.suppress == 'l2_memory':
            if line.startswith("  "):
                return
            self.suppress = None
        elif self.suppress == 'plan_code':
            if stripped.startswith("====================================") and "模型" in stripped:
                self.suppress = None
            return
        elif self.suppress == 'vlm_raw':
            if stripped and set(stripped) == {"-"}:
                self.suppress = None
                self._emit_vlm_raw_box()
            elif stripped:
                self._vlm_raw_lines.append(stripped)
            return
        elif self.suppress == 'adjust_code':
            if stripped.startswith("===") and set(stripped) == {"="}:
                self.suppress = None
                self._emit_adjust_code_box()
            else:
                self._adjust_code_lines.append(line)
            return

        if stripped in self._DROP_LINES or stripped.startswith("Last Line to Delete from answer:"):
            return
        if stripped.startswith("rag分数"):
            self.suppress = 'rag_scores'
            return
        if stripped.startswith("找到相似历史任务"):
            self.suppress = 'rag_ref'
            return
        if stripped == "原始输出检查":
            self.suppress = 'raw_check'
            self._eq_count = 0
            return
        if stripped.startswith("[utils] L2 memory loaded"):
            self.suppress = 'l2_memory'
            return
        if stripped.startswith("========== 生成的执行计划"):
            self.suppress = 'plan_code'
            return

        # [BigBrain] 内部准备动作 — 不显示
        if stripped.startswith("[BigBrain]"):
            return

        # parse_obj_name 的 LLM debug echo，例如「# Trash_01_001?」
        if stripped.startswith("#") and stripped.endswith("?"):
            return

        # 每个 action 的 rule-judge 细节（task 描述 / 感测观测 / action_info /
        # rule_result 原始字典）— 太技术性，不显示给用户
        if stripped.startswith("Judging task completion for task:"):
            return
        if stripped.startswith("Observation:"):
            return
        if stripped.startswith("{") and stripped.endswith("}"):
            return

        # [robot_api] 内部 log → 立即印出精简的动作状态（车已经在动了）
        if stripped.startswith("[robot_api]"):
            low = stripped.lower()
            status = None
            if "pick_up_obj" in low:
                status = "抓取方块中..."
            elif "put_down_obj_by_offset" in low or "put_down_xy" in low \
                    or "put_down_between_objs" in low:
                status = "放置方块中..."
            elif "moving to" in low or "approach" in low:
                status = "导航中..."
            if status and status != self._last_action_status:
                icon = _ACTION_ICONS.get(status, "")
                self.real.write(f"[Action] {icon} {status}\n")
                self._last_action_status = status
            return

        # VLM 开始评判 → 打印 VLM 状态行
        if stripped.startswith("send messages to VLM to judge"):
            self._vlm_active = True
            self.real.write("[VLM] 👁️ 正在发送图像至 VLM 进行评判...\n")
            return

        # VLM 原始输出 → 收集进 box
        if stripped == "[JudgeLLM] --- Raw VLM Output ---":
            self.suppress = 'vlm_raw'
            self._vlm_raw_lines = []
            return

        # Judge 结果：动作状态已经在动作开始时立即印出，这里独立打印结果
        m = re.match(r"^Judge result: (PASS.*|FAIL.*)$", stripped)
        if m:
            result_text = m.group(1)
            if result_text.startswith("FAIL"):
                self._emit_judge_fail_box(result_text)
            else:
                self.real.write(f"[Judge] {result_text}\n")
            self._vlm_active = False
            self._last_action_status = None
            return

        # JudgeLLM 追踪最后拾取的物件
        m = re.match(r"^\[JudgeLLM\] tracking last_picked_obj = (.*)$", stripped)
        if m:
            self.real.write(f"[JudgeLLM] 📌 正在追踪 last_picked_obj = {m.group(1)}\n")
            return

        # AdjustLLM：开始局部重规划
        m = re.match(
            r"^\[AdjustLLM\] Initiating Replanning\.\.\. "
            r"\(Attempt (\d+)/(\d+), block=(.*?), action=(.*)\)$",
            stripped,
        )
        if m:
            self._last_action_status = None
            attempt, max_attempt, _block, action_desc = m.groups()
            self.real.write("═" * 60 + "\n")
            self.real.write(
                f"[AdjustLLM] 🔄 触发局部重规划... "
                f"(尝试 {attempt}/{max_attempt}, 动作: {action_desc})\n"
            )
            return

        if stripped == "[AdjustLLM] Calling Task LLM to generate recovery code...":
            self.real.write("[AdjustLLM] 🧠 调用 Task LLM 生成恢复代码...\n")
            return

        if stripped.startswith("========== AdjustLLM 生成的微调/恢复代码"):
            self.suppress = 'adjust_code'
            self._adjust_code_lines = []
            return

        if stripped.startswith("[AdjustLLM] Max replan times"):
            self.real.write(f"[AdjustLLM] ⚠️ {stripped[len('[AdjustLLM] '):]}\n")
            return

        if stripped.startswith("[AdjustLLM] 微调动作执行完毕"):
            self.real.write("[AdjustLLM] ✅ 微调动作执行完毕\n")
            return

        if stripped.startswith("[AdjustLLM] 微调代码执行时发生异常"):
            self.real.write(f"[AdjustLLM] ⚠️ 微调代码执行时发生异常{stripped.split('异常', 1)[1]}\n")
            return

        if stripped.startswith("[AdjustLLM] 调用微调 LLM 失败"):
            self.real.write(f"[AdjustLLM] ⚠️ 调用微调 LLM 失败{stripped.split('失败', 1)[1]}\n")
            return

        if stripped.startswith("[AdjustLLM] 生成的代码为空"):
            self.real.write("[AdjustLLM] ⚠️ 生成的代码为空，无法微调。\n")
            return

        if stripped.startswith("[AdjustLLM] REPLAN_EXECUTE_CODE=False"):
            self.real.write("[AdjustLLM] REPLAN_EXECUTE_CODE=False，仅记录微调代码，不实际执行。\n")
            return

        # 任务最终结果加上 emoji
        if stripped.startswith("✓ 任务执行成功"):
            self.real.write("✅ 任务执行成功！\n")
            return
        if stripped.startswith("✗ 执行异常"):
            self.real.write(f"❌ 执行异常{stripped.split('异常', 1)[1]}\n")
            return

        # PlannerLLM 呼叫前的最後一行 — 接下來 LLM 生成計劃要等幾秒，
        # 加一行提示避免使用者以為卡住
        if stripped.startswith("任务："):
            self.real.write(line + "\n")
            self.real.write("[demo] ⏳ 正在生成计划，请稍候...\n")
            return

        self.real.write(line + "\n")


def setup():
    """所有模型/场景加载 — 一启动就做，只打印精简的初始化进度，最后打印就绪信息。"""
    print("💡 [INFO] 正在初始化语意模型与场景资料...")

    # demo 专用：用 action/robot_api_demo.py（robot_api.py 的副本，加了拾取/放置
    # 微调）取代 action.robot_api，不动到原本共用的 robot_api.py。
    import action
    from action import robot_api_demo
    sys.modules['action.robot_api'] = robot_api_demo
    action.robot_api = robot_api_demo

    import run_e2_composite as e2c
    _quiet(e2c.patch_base_prompt_remove_c_helpers)
    os.environ["BIGBRAIN_DISABLE_C_HELPER_CANONICAL"] = "1"
    # CoffeeTable_01_001 是有明确正面的桌子，用 yaml 里的 canonical approach。
    os.environ.pop("BIGBRAIN_AUTO_PICK_SIDE", None)

    import config
    config.MAX_REPLAN_TIMES = ADJUST_POLICY
    judge_log_path = os.path.join(config.JUDGE_LOG_DIR, "judge_events.jsonl")
    judge_start_offset = os.path.getsize(judge_log_path) if os.path.exists(judge_log_path) else 0

    print("   • 启动 VLM 图像串流节点")
    writer_proc = subprocess.Popen(
        [sys.executable, os.path.join(BIG_BRAIN_DIR, "vlm_image_writer.py"), e2c.DEFAULT_VLM_TOPIC],
        cwd=BIG_BRAIN_DIR, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(1.0)
    _quiet(e2c.wait_fresh_image, timeout_s=5.0)

    print("   • 重置 Gazebo 场景与机器人姿态")
    import composite_capability_test as comp
    tester = _quiet(comp.CompositeTester, pause_after_reset=0.0)
    _quiet(tester._base_reset)
    _quiet(tester._teleport_block_to_stable, comp.RED, RED_START_POSE)
    time.sleep(1.0)
    _quiet(tester.arm_home)

    from action import robot_api
    from model.llm import JudgeLLM
    robot_api.judge_llm = JudgeLLM()
    robot_api.judge_llm.max_replan_times = ADJUST_POLICY

    print("   • 加载语意嵌入模型与 RAG 任务记忆库")
    import importlib
    big_brain = _quiet(importlib.import_module, "big_brain")
    BigBrain = big_brain.BigBrain
    brain = _quiet(BigBrain)
    brain._save_history = lambda path: None

    # 关闭 sentence-transformers 编码进度条（tqdm "Batches: ..."）
    embedder = getattr(brain.rag_manager, "embedder", None)
    if embedder is not None:
        _orig_encode = embedder.encode
        embedder.encode = lambda *a, **k: _orig_encode(*a, **{**k, "show_progress_bar": False})

    print("\n🤖 [Ready] 系统初始化完成\n")
    return e2c, comp, tester, brain, writer_proc, judge_log_path, judge_start_offset


def _reset_scene(tester, comp):
    """重置场景、方块位置、机械臂，并清掉 demo 调整用的重试计数，
    让下一轮任务可以重复执行。"""
    _quiet(tester._base_reset)
    _quiet(tester._teleport_block_to_stable, comp.RED, RED_START_POSE)
    time.sleep(1.0)
    _quiet(tester.arm_home)

    from action import robot_api
    robot_api._trash_drop_attempts.clear()
    robot_api._trash_drop_yaw.clear()
    robot_api._pick_attempts.clear()
    judge_llm = getattr(robot_api, "judge_llm", None)
    if judge_llm is not None:
        judge_llm.replan_retry_counter = {}
        judge_llm.dead_blocks = set()


def main() -> int:
    e2c, comp, tester, brain, writer_proc, judge_log_path, judge_start_offset = setup()

    try:
        first_round = True
        while True:
            raw = input(
                "请输入任务指令 (中文或英文 / Chinese or English，输入 q 退出)\n"
                "例如：把红色方块放到茶几上 / Put the red block on the coffee table\n> "
            ).strip()
            if raw.lower() in ("q", "quit", "exit"):
                break
            if not raw:
                raw = "把红色方块放到茶几上"
                print(f"(未输入，使用默认指令: {raw})")

            # ---- 重置场景，准备这一轮 ----
            # demo 調整：不在上一輪結束時自動重置，等使用者輸入下一个任务指令后才重置，
            # 方便使用者在輸入前先觀察上一輪的最終狀態。
            if not first_round:
                print("\n[demo] 🔁 重置场景，准备本轮任务...\n")
                _reset_scene(tester, comp)
            first_round = False

            instruction = parse_instruction(raw)
            print("\n" + "─" * 60)
            print("[demo] 🚀 任务规划中...")
            print(f"任务目标：{instruction}")
            print("─" * 60 + "\n")

            t0 = time.time()
            out_filter = _OutputFilter(sys.stdout)
            with contextlib.redirect_stdout(out_filter):
                ok = brain.run_once(instruction)
            out_filter.close()
            elapsed = time.time() - t0

            # ---- judge / VLM / AdjustLLM 事件摘要 ----
            new_events = e2c.read_new_judge_events(judge_log_path, judge_start_offset)
            judge_start_offset = os.path.getsize(judge_log_path) if os.path.exists(judge_log_path) else judge_start_offset
            print("\n📊 [Summary] 历史事件总览 (Judge-Events)")
            if not new_events:
                print("  (无)")
            for e in new_events:
                ev = e.get("event", "?")
                if ev == "judge":
                    vr = e.get("vlm_result") or {}
                    tag = "PASS" if e.get("final_pass") else "FAIL"
                    print(
                        f"  ● [{tag}] rule_pass={e.get('rule_pass')} vlm_needed={e.get('vlm_needed')} "
                        f"vlm_pass={vr.get('pass')} reason={str(vr.get('reason', ''))[:120]}"
                    )
                elif ev == "replan_generated":
                    code = str(e.get("code", e.get("generated_code", ""))).replace("\n", " ")
                    print(f"  ● [AdjustLLM] 生成恢复代码: {code[:120]}")
                elif ev == "replan_executed":
                    tag = "PASS" if e.get("ok") else "FAIL"
                    print(f"  ● [{tag}] AdjustLLM 执行结果: {e.get('reason', '')}")
                elif ev == "replan_failed":
                    print(f"  ● [DEAD] AdjustLLM 失败: {e.get('reason', '')}")
                else:
                    print(f"  ● [{ev}] {str(e)[:160]}")

            # ---- 最终结果检查 ----
            status_text = "🎉 SUCCESS" if ok else "⚠️ FAIL"
            dist = None
            check_text = "无法验证"
            try:
                import ros_bridge
                info = ros_bridge.get_obj_info("red_block")
                table = ros_bridge.get_obj_info("CoffeeTable_01_001")
                dx = info["object_x"] - table["object_x"]
                dy = info["object_y"] - table["object_y"]
                dist = math.hypot(dx, dy)
                hx = float(table.get("bbox_half_x", 0.5)) + 0.10
                hy = float(table.get("bbox_half_y", 0.5)) + 0.10
                z  = float(info.get("surface_z", info.get("object_z", 0.0)))
                on_table = (abs(dx) <= hx and abs(dy) <= hy
                            and z > float(table.get("surface_z", 0.27)) - 0.06)
                check_text = "已放上茶几 ON TABLE" if on_table else "MISSED"
            except Exception as exc:
                check_text = f"无法验证 ({exc})"

            print()
            print("┏━━━ 🏁 最终执行结果 ━━━┓")
            print(f"┃ ● 任务状态: {status_text} ({'ok' if ok else 'failed'})")
            print(f"┃ ● 总耗时: {elapsed:.1f} 秒")
            print(f"┃ ● 物理检查: {check_text}")
            if dist is not None:
                print(f"┃ ● 最终误差: {dist:.3f} m")
            else:
                print("┃ ● 最终误差: -")
            print("┗━━━━━━━━━━━━━━━━━━━━━━┛")

        return 0

    finally:
        writer_proc.terminate()
        try:
            writer_proc.wait(timeout=2)
        except Exception:
            writer_proc.kill()


if __name__ == "__main__":
    sys.exit(main())
