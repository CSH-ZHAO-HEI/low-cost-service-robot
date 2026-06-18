# 存放与大模型交互的接口

import time
import base64
import re
import json
import math
import os
from datetime import datetime, timezone

from openai import OpenAI
import numpy as np

from config import VLM_API_KEY,VLM_API_BASE_URL,VLM_MODEL
from config import (
    MAX_REPLAN_TIMES,
    MOVE_ERROR_THRESHOLD,
    APPROACH_ERROR_THRESHOLD,
    PLACE_ERROR_THRESHOLD,
    CATCH_ERROR_THRESHOLD,
    JUDGE_LOG_DIR,
    REPLAN_EXECUTE_CODE,
)
from utils.utils import get_obj_xy,get_obj_z,get_obj_size
from utils.utils import get_robot_pos,get_robot_orientation,get_robot_arm
from utils.utils import extract_code,call_LLM,encode_image
from prompt.vlm_prompt import VLM_SYSTEM_PROMPT
from prompt.adjust_prompt import ADJUST_PROMPT

class PlannerLLM:
    # 负责根据用户指令生成计划
    def __init__(self):
        pass

    def generate_code(self, prompt:str,rag_context:str)->str:
        print("planning……")
        # 询问LLM
        try:
            raw_text = call_LLM(prompt)
            print("finish planning")
            last_line = ""
            rag_lines = rag_context.strip().splitlines() if rag_context else []
            if rag_lines:
                last_line = rag_lines[-1]
                print(f"Last Line to Delete from answer: {last_line}")
            return extract_code(raw_text,last_line)
        except Exception as e:
            print(f"Planner LLM 调用失败：{e}")
            return ""

class JudgeLLM:
    # 负责判断任务是否完成，并决定是否要进行重规划
    def __init__(self):
        self.max_replan_times = MAX_REPLAN_TIMES
        self.replan_retry_counter = {}
        # 全任務尺度的失敗歷史：list of dicts，按時間順序
        # 每筆：{action_desc, robot_pos_before, robot_pos_after, vlm_error_type,
        #        adjust_code (str, 截斷), nav_failed (bool)}
        # 注入 AdjustLLM prompt 讓 LLM 知道剛才試了什麼、結果如何
        self.failure_history = []
        # 追蹤最近一次成功 pick 的物件 — 給 put_down VLM judge 用，
        # 避免 VLM 看到桶裡之前放的物件就誤判當前 put 成功
        self.last_picked_obj = None
        # 已經用光 adjust 配額的 block。當 LLM plan 後續還想對它做動作時，
        # judge() 直接放行（PASS, skip）讓 plan 進到下一個 block，不再消耗時間。
        self.dead_blocks = set()
        # Sticky block focus：動作裡有 *_block 名 → 更新；沒有（Move to (x,y) / Put down on Trash）
        # → 沿用最近一個 block 名。這樣 red 的 pick+move-to-trash+put 全部歸到 red 的配額。
        self.current_block_focus = ""
        os.makedirs(JUDGE_LOG_DIR, exist_ok=True)
        self.log_path = os.path.join(JUDGE_LOG_DIR, "judge_events.jsonl")
        self.vlm_client = OpenAI(
            api_key=VLM_API_KEY,
            base_url=VLM_API_BASE_URL,
        )

    def _format_failure_history(self, max_entries: int = 4) -> str:
        """把最近 max_entries 筆 failure 渲染成 prompt 可讀的字串。"""
        if not self.failure_history:
            return "(no prior failures in this task)"
        recent = self.failure_history[-max_entries:]
        lines = []
        for i, h in enumerate(recent, 1):
            code_snippet = (h.get("adjust_code") or "").strip().replace("\n", " ; ")
            if len(code_snippet) > 140:
                code_snippet = code_snippet[:137] + "..."
            lines.append(
                f"  #{i} action={h.get('action_desc','?')} | "
                f"vlm={h.get('vlm_error_type','?')} | "
                f"strategy_tried={code_snippet or '(none yet)'} | "
                f"robot_after=({h.get('robot_after',[0,0])[0]:.2f},{h.get('robot_after',[0,0])[1]:.2f})"
            )
        return "\n".join(lines)

    def _log_event(self, event_type: str, payload: dict):
        event = {
            "ts": datetime.now(timezone.utc).isoformat(),
            "event": event_type,
            **payload,
        }
        try:
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(event, ensure_ascii=False, default=str) + "\n")
        except Exception as e:
            print(f"[JudgeLLM] failed to write judge log: {e}")

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("true", "yes", "1", "pass", "passed")
        return bool(value)

    @staticmethod
    def _xy_error(target, observation: dict) -> float:
        return math.hypot(
            float(target[0]) - float(observation["robot_x"]),
            float(target[1]) - float(observation["robot_y"]),
        )

    def _block_focus(self, action_info: dict) -> str:
        """Return the block name (red/blue/yellow) this action is about, or '' if none.

        Sticky: actions that explicitly name a block update current_block_focus;
        actions that don't (e.g. "Move to (x,y)" or "Put down on Trash") inherit
        the most recent focus. This makes red's pick + move-to-trash + put all
        count against red's budget; only when a new block name appears does the
        focus flip and a fresh budget kick in.
        """
        import re
        text_fields = [
            str(action_info.get("raw", "")),
            str(action_info.get("target", "")),
        ]
        params = action_info.get("params") or {}
        if isinstance(params, dict):
            text_fields.append(str(params.get("target", "")))
            text_fields.append(str(params.get("task_text", "")))
        haystack = " | ".join(text_fields).lower()
        m = re.search(r"(red|blue|yellow)_?block", haystack)
        if m:
            self.current_block_focus = f"{m.group(1)}_block"
            return self.current_block_focus
        # No block name → fall back to sticky focus, then to last_picked_obj
        if self.current_block_focus:
            return self.current_block_focus
        if self.last_picked_obj and "block" in self.last_picked_obj.lower():
            return self.last_picked_obj
        return ""

    def _expected_base_target(self, action_info: dict):
        """Return the expected robot-base XY for object-relative actions."""
        obj = action_info["target"]
        dx, dy = action_info.get("offset", (0.0, 0.0))
        try:
            import ros_bridge
            ax, ay, _ = ros_bridge.get_obj_approach_pos(obj)
            return (float(ax) + float(dx), float(ay) + float(dy), "approach")
        except Exception:
            obj_x, obj_y = get_obj_xy(obj)
            return (float(obj_x) + float(dx), float(obj_y) + float(dy), "object_center")


    def judge(self, task:str = None, action_id: int = None, task_desc: str = None, **kwargs):
        # 使用结构化动作(action_id + params)
        # 解析动作信息
        action_info = self._parse_atomic_task(task=task, action_id=action_id, task_desc=task_desc, params=kwargs)
        task_desc = task_desc or kwargs.get("task_text") or task or action_info.get("raw") or action_info["action"]
        action_info["raw"] = task_desc
        print(f"Judging task completion for task: {task_desc}")

        # 如果這個 action 的目標 block 已經被標記為 dead（adjust 配額用光），
        # 直接 PASS 跳過，讓 LLM plan 自然進到下一個 block（紅死 → 進藍 → 進黃）
        focus = self._block_focus(action_info)
        if focus and focus in self.dead_blocks:
            print(f"[JudgeLLM] '{focus}' already dead (replan budget exhausted) — auto-PASS this action to skip")
            self._log_event("judge_auto_pass_dead_block", {
                "action_info": action_info,
                "block": focus,
            })
            return True

        time.sleep(0.2)

        # 收集传感器信息
        observation = self._collect_observation()

        print(action_info)
        # 初步规则判断
        rule_result = self._rule_judge(action_info, observation)
        print(rule_result)

        # VLM判断
        vlm_result = {"pass": True, "reason": "VLM skipped", "skipped": True}
        # 1，2失败判断; 3，4需要检测是否夹对 ; 5,6需要检测放的位置是否符合语义(instruction 如between关系)
        vlm_needed = (not self._as_bool(rule_result["pass"])) or action_id in [3,4,5,6]
        if vlm_needed:
            raw_result = self.judge_VLM(action_info,observation,rule_result)
            vlm_result = self.extract_VLM_answer(raw_result)

        rule_pass = self._as_bool(rule_result.get("pass"))
        vlm_pass = self._as_bool(vlm_result.get("pass"))
        vlm_override = False
        result = rule_pass and (not vlm_needed or vlm_pass)

        # pick/put 目前没有可靠夹爪传感器，允许 VLM 对语义动作覆盖规则初判。
        if not result and vlm_needed and vlm_pass and action_id in [3, 4, 5, 6]:
            vlm_override = True
            result = True

        self._log_event("judge", {
            "action_info": action_info,
            "observation": observation,
            "rule_result": rule_result,
            "vlm_needed": vlm_needed,
            "vlm_result": vlm_result,
            "vlm_override": vlm_override,
            "final_pass": result,
        })

        if result:
            if vlm_override:
                print("Judge result: PASS (by VLM override)")
            else:
                print("Judge result: PASS")
            # 記住最近一次成功 pick 的物件 — 給後續 put_down VLM 判斷用
            # 避免 VLM 看桶內舊物件就誤判當前 put 成功
            if action_id in [3, 4]:  # pick_up_xy / pick_up_obj
                tgt = action_info.get("target")
                if isinstance(tgt, str):
                    self.last_picked_obj = tgt
                    print(f"[JudgeLLM] tracking last_picked_obj = '{tgt}'")
            elif action_id in [5, 6]:  # put_down_*
                # 放下成功後清除（下次 pick 才會更新）
                self.last_picked_obj = None
            return True

        # 判断失败，需要呼唤LLM进行微调
        failure_reason = vlm_result.get("reason") or rule_result.get("failure_code") or "unknown"
        print(f"Judge result: FAIL ({failure_reason})")
        print("Task not completed. Initiating local replanning...")
        return self.replan(action_info, vlm_result, observation=observation)

    def _parse_atomic_task(self, task: str = None, action_id: int = None, task_desc: str = None, params: dict = None) -> dict:
        # 结构化解析
        if action_id is not None:
            info = self._parse_by_action_id(action_id, params or {})
            info["raw"] = task_desc
            return info
        else:
            raise ValueError("No action_id provided for structured task parsing")

    def _parse_by_action_id(self, action_id: int, params: dict) -> dict:
        if action_id == 1:
            return {
                "action": "move_to_xy",
                "action_id": action_id,
                "target": (float(params["x"]), float(params["y"])),
                "params": params,
            }

        if action_id == 2:
            return {
                "action": "move_to_obj_by_offset",
                "action_id": action_id,
                "target": params["target"],
                "offset": (float(params["dx"]), float(params["dy"])),
                "params": params,
            }

        if action_id == 3:
            return {
                "action": "pick_up_xy",
                "action_id": action_id,
                "target": (float(params["x"]), float(params["y"])),
                "params": params,
            }

        if action_id == 4:
            return {
                "action": "pick_up_obj",
                "action_id": action_id,
                "target": params["target"],
                "params": params,
            }

        if action_id == 5:
            return {
                "action": "put_down_xy",
                "action_id": action_id,
                "target": (float(params["x"]), float(params["y"])),
                "params": params,
            }

        if action_id == 6:
            return {
                "action": "put_down_obj_by_offset",
                "action_id": action_id,
                "target": params["target"],
                "offset": (float(params["dx"]), float(params["dy"])),
                "params": params,
            }

        return {
            "action": "unknown",
            "action_id": action_id,
            "params": params,
        }

    def _collect_observation(self) -> dict:
        # 传感器
        robot_x,robot_y = get_robot_pos()
        robot_orientation = get_robot_orientation()
        holding = get_robot_arm()
        obs = {
            "robot_x": robot_x,
            "robot_y": robot_y,
            "robot_orientation": robot_orientation,
            "holding": holding,
        }
        print(f"Observation: {obs}")
        return obs

    def _rule_judge(self, action_info: dict, observation: dict) -> dict:
        action_id = action_info["action_id"]

        if action_id == 1:
            error = self._xy_error(action_info["target"], observation)
            passed = error <= MOVE_ERROR_THRESHOLD
            return {
                "pass": passed,
                "failure_code": "localization_error" if not passed else "",
                "error_m": round(error, 4),
                "threshold_m": MOVE_ERROR_THRESHOLD,
                "metric": "euclidean_xy",
            } 
        elif action_id == 2:
            # move_to_obj_by_offset 的目标是 base 的 approach 点，不是物体中心。
            target_x, target_y, target_kind = self._expected_base_target(action_info)
            error = self._xy_error((target_x, target_y), observation)
            passed = error <= APPROACH_ERROR_THRESHOLD
            return {
                "pass":passed,
                "failure_code": "localization_error" if not passed else "",
                "error_m": round(error, 4),
                "threshold_m": APPROACH_ERROR_THRESHOLD,
                "expected_base_xy": (round(target_x, 4), round(target_y, 4)),
                "expected_base_kind": target_kind,
                "metric": "euclidean_xy",
            }
        elif action_id == 3:
            # pick_up_xy
            holding = observation["holding"]
            error = self._xy_error(action_info["target"], observation)
            placed = error <= CATCH_ERROR_THRESHOLD
            passed = bool(holding) and placed
            failure_code = ""
            if not placed:
                failure_code = "localization_error"
            elif not holding:
                failure_code = "grasp_failure"    
            return {
                "pass": passed,
                "failure_code": failure_code,
                "error_m": round(error, 4),
                "threshold_m": CATCH_ERROR_THRESHOLD,
                "holding": bool(holding),
                "metric": "euclidean_xy",
            }
        elif action_id == 4:
            # pick_up_obj
            # holding 并且小车和obj原来的位置在误差范围内
            holding = observation["holding"]
            obj_x,obj_y = get_obj_xy(action_info["target"])
            error = self._xy_error((obj_x, obj_y), observation)
            placed = error <= CATCH_ERROR_THRESHOLD
            failure_code = ""
            if not placed:
                failure_code = "localization_error"
            elif not holding:
                failure_code = "grasp_failure"
            return {
                "pass": holding and placed,
                "failure_code": failure_code,
                "error_m": round(error, 4),
                "threshold_m": CATCH_ERROR_THRESHOLD,
                "holding": bool(holding),
                "metric": "euclidean_xy",
            }
        elif action_id == 5:
            # put_down_xy 机器人不在目标位置，或者还拿着东西都算失败
            holding = observation["holding"]
            error = self._xy_error(action_info["target"], observation)
            placed = error <= CATCH_ERROR_THRESHOLD
            failure_code = ""
            if not placed:
                failure_code = "localization_error"
            elif holding:
                failure_code = "placement_error"
            return {
                "pass": not holding and placed,
                "failure_code": failure_code,
                "error_m": round(error, 4),
                "threshold_m": CATCH_ERROR_THRESHOLD,
                "holding": bool(holding),
                "metric": "euclidean_xy",
            }
        elif action_id == 6:
            # put_down_obj_by_offset 机器人不在目标位置，或者还拿着东西都算失败
            holding = observation["holding"]
            obj_x,obj_y = get_obj_xy(action_info["target"])
            target_x = obj_x + action_info["offset"][0]
            target_y = obj_y + action_info["offset"][1]
            error = self._xy_error((target_x, target_y), observation)
            placed = error <= CATCH_ERROR_THRESHOLD
            failure_code = ""
            if not placed:
                failure_code = "localization_error"
            elif holding:
                failure_code = "placement_error"
            return {
                "pass": not holding and placed,
                "failure_code": failure_code,
                "error_m": round(error, 4),
                "threshold_m": CATCH_ERROR_THRESHOLD,
                "holding": bool(holding),
                "metric": "euclidean_xy",
            }

        # 未知动作
        print("UNKNOWN ACTION!")
        return {"pass": True, "failure_code": ""}

    def judge_VLM(self, action_info: dict, observation: dict, rule_result: dict) -> dict:
        # 构建VLM的prompt，调用VLM进行判断
        # 获取全局用户指令
        from runtime_context import CTX
        global_instruction = CTX.get("instruction")
        if not global_instruction:
            print("can not gain the user instruction")
            time.sleep(10)
        current_action = action_info["raw"]
        # 拆 rule_result：只給 VLM 原始數字事實，不丟預先貼好的 failure_code 標籤
        # （label 會 anchor VLM 跟著貼，讓它不獨立看圖；只給 raw numerics 讓 VLM 自行判斷類型）
        rule_facts = {k: v for k, v in rule_result.items()
                      if k not in ('failure_code', 'metric')}
        # 對 put_down 動作：明確告訴 VLM 現在該放的是哪個物件，
        # 避免 VLM 看到桶內舊物件就誤判當前 put 成功（false positive）
        currently_placing_str = ""
        if action_info.get("action_id") in [5, 6] and self.last_picked_obj:
            currently_placing_str = (
                f"\n        Currently Placing: '{self.last_picked_obj}' (this is the SPECIFIC object\n"
                f"        you must verify is newly inside the bin/on the target — do NOT pass just\n"
                f"        because some other previously-placed object is visible there)"
            )
        user_text = f"""
        User Input:
        Global Instruction: "{global_instruction}"
        Current Action: {current_action}{currently_placing_str}
        Observation: {observation}
        Rule Check Numerics: {rule_facts}
        """
        # 用絕對路徑（避免被 CWD 影響）— 對應 vlm_image_writer.py 寫入的位置
        import os as _os
        _img_dir = _os.path.join(
            _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))),
            "image"
        )
        image_path       = _os.path.join(_img_dir, "image.jpg")          # 車載側拍
        trash_image_path = _os.path.join(_img_dir, "trash_inside.jpg")   # 桶內俯拍
        try:
            base64_image = encode_image(image_path)
        except Exception as e:
            print(f"[Warning] Failed to read image at {image_path}: {e}. "
                  f"Proceeding without visual (Hallucination risk high).")
            base64_image = ""
        # 桶內視角永遠嘗試讀。沒有就跳過（不是 trash 任務時 VLM 也不會被誤導，
        # 因為它看到桶內空就是空，不影響對 pick/put-on-table 的判斷）。
        try:
            base64_trash = encode_image(trash_image_path)
        except Exception:
            base64_trash = ""

        # 拼装多模态 Message
        messages =[
            {"role": "system", "content": VLM_SYSTEM_PROMPT},
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": user_text},
                ]
            }
        ]

        # 如果图片读取成功，加入图片体
        if base64_image:
            messages[1]["content"].append({
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}
            })
        # 第二張：桶內俯拍。VLM 自決是否作為證據。
        if base64_trash:
            messages[1]["content"].append({
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{base64_trash}"}
            })
        print("send messages to VLM to judge……")
        try:
            response = self.vlm_client.chat.completions.create(
                model=VLM_MODEL,
                messages=messages,
                temperature=0.1, # 保持低温度以输出稳定 JSON
                top_p=0.9
            )
            raw_text = response.choices[0].message.content
            return raw_text
        except Exception as e:
            print(f"[JudgeLLM] VLM API Call Failed: {e}")
            return '{"pass": false, "error_type": "vlm_api_error", "reason": "API call failed", "suggested_correction": "Retry previous action."}'

    def extract_VLM_answer(self,raw_result:dict) -> dict:
        print("\n[JudgeLLM] --- Raw VLM Output ---")
        print(raw_result)
        print("---------------------------------\n")
        try:
            # 使用正则提取 JSON 代码块中的内容
            json_pattern = re.search(r'\{.*\}', raw_result, re.DOTALL)
            if json_pattern:
                json_str = json_pattern.group(0)
                parsed = json.loads(json_str)
            else:
                # 尝试直接解析
                parsed = json.loads(raw_result)
            parsed.setdefault("raw_output", raw_result)
            return parsed
        except json.JSONDecodeError as e:
            print(f"[JudgeLLM] Failed to parse VLM JSON: {e}")
            # 严重解析错误时的兜底保护
            return {
                "pass": False, 
                "error_type": "json_parse_error", 
                "reason": f"Failed to parse VLM output. Raw text: {raw_result[:50]}...",
                "suggested_correction": "Re-evaluate the action.",
                "raw_output": raw_result,
            }
 
    def replan(self, action_info, vlm_result, observation=None):
        print("开始微调")
        action_desc = action_info.get("raw", "unknown action")
        from runtime_context import CTX
        global_instruction = CTX.get("instruction")
        # Per-block budget: red 的 pick+move+put 共用同一 counter；藍/黃 各自獨立。
        # 當一個 block 用光配額 → 標記 dead_blocks，judge() 之後對它的動作直接 PASS
        # 讓 LLM plan 自然進到下一個 block，不再消耗 adjust 也不卡住整個 task。
        block_focus = self._block_focus(action_info)
        counter_key = block_focus if block_focus else f"_no_block|{action_desc}"
        if counter_key not in self.replan_retry_counter:
            self.replan_retry_counter[counter_key] = 0

        if self.replan_retry_counter[counter_key] >= self.max_replan_times:
            print(f"[AdjustLLM] Max replan times ({self.max_replan_times}) reached for block={counter_key} "
                  f"(action: {action_desc}). Marking dead; subsequent actions on it will be skipped.")
            if block_focus:
                self.dead_blocks.add(block_focus)
            self._log_event("replan_skipped", {
                "action_info": action_info,
                "vlm_result": vlm_result,
                "reason": "max_replan_times_reached",
                "max_replan_times": self.max_replan_times,
                "counter_key": counter_key,
                "block_focus": block_focus,
            })
            return False

        self.replan_retry_counter[counter_key] += 1
        print(f"\n[AdjustLLM] Initiating Replanning... "
              f"(Attempt {self.replan_retry_counter[counter_key]}/{self.max_replan_times}, "
              f"block={counter_key}, action={action_desc})")

        # 将 VLM 的核心输出压缩为单行字符串
        import json
        vlm_feedback_str = json.dumps({
            "error_type": vlm_result.get("error_type", "unknown"),
            "suggested_correction": vlm_result.get("suggested_correction", "Retry the action.")
        }, ensure_ascii=False)

        # 最小注入：robot state（一律需要）+ 工具清單 + 失敗歷史（讓 LLM 知道剛試了什麼）
        # 不預先塞「附近障礙物清單」— 由 LLM 自行呼叫 list_obstacles_near 決定範圍
        if observation:
            robot_state_str = (f"pos=({observation.get('robot_x', 0):.2f}, "
                               f"{observation.get('robot_y', 0):.2f}), "
                               f"yaw={observation.get('robot_orientation', 0):.2f}, "
                               f"holding={observation.get('holding', False)}")
            robot_pos_now = (float(observation.get('robot_x', 0)),
                             float(observation.get('robot_y', 0)))
        else:
            robot_state_str = "unknown"
            robot_pos_now = (0.0, 0.0)
        failure_history_str = self._format_failure_history(max_entries=4)

        tools_help = (
            "# ── Query tools (read-only, no motion) ──\n"
            "#   get_current_pos() -> (x, y)\n"
            "#   get_obj_xy(name) -> (x, y)\n"
            "#   get_obj_size(name) -> (width, height, surface_z)\n"
            "#   is_reachable(x, y) -> bool\n"
            "#       Asks the global path planner (NavFn). True ≠ TEB will actually\n"
            "#       execute — global and local costmaps can disagree.\n"
            "#   list_obstacles_near(x, y, radius=3.0)\n"
            "#       Returns [{name, x, y, hx, hy, dist}, ...] sorted nearest-first.\n"
            "#   find_reachable_approach_to(obj_name, dist=0.55, num_angles=8)\n"
            "#       Tries 360° around obj at given dist; returns the nearest\n"
            "#       reachable point to robot, or None. Use for symmetric targets\n"
            "#       (trash, ball) when the canonical approach side is blocked —\n"
            "#       there's no rule that says you must approach from one specific\n"
            "#       direction; any side that's reachable will work.\n"
            "#\n"
            "# ── Motion primitives — two layers, you choose ──\n"
            "# Layer 1 (TEB-based, global path + obstacle avoidance):\n"
            "#   move_to_xy(x, y)\n"
            "#   move_to_obj_by_offset(obj, dx, dy)\n"
            "#       Pros: avoids obstacles, can travel meters across rooms.\n"
            "#       Cons: optimises rotate-then-forward; in tight spaces TEB may\n"
            "#             try to spin in place and abort (state=4) when the\n"
            "#             chassis would sweep into an adjacent obstacle.\n"
            "# Layer 2 (raw /cmd_vel, bypasses TEB, no path planning):\n"
            "#   drive_forward(distance, speed=0.15)   # NEGATIVE = backward\n"
            "#   rotate_to_face(target_x, target_y)    # rotate in place toward a point\n"
            "#   rotate_by(angle_rad)                  # pure relative rotation, no point needed\n"
            "#                                         # +CCW, -CW; e.g. 1.57=90°left, 3.14=180°\n"
            "#       Pros: guaranteed open-loop motion, no abort, no rotate-stuck.\n"
            "#       Cons: BLIND — no obstacle check. Safe distance is < 1 m and\n"
            "#             only when you already know that direction is clear (e.g.\n"
            "#             you just observed the obstacle is in front, so behind\n"
            "#             must be relatively free).\n"
            "# You decide which layer fits the situation."
        )

        # 按照纯 CAP 风格拼接最终的 Prompt
        from prompt.adjust_prompt import ADJUST_PROMPT
        final_prompt = ADJUST_PROMPT + "\n"
        final_prompt += f"# Global Instruction: \"{global_instruction}\"\n"
        # Robot State 強化：明確說明這就是 recovery 的起點
        final_prompt += (
            f"# Robot Current State: {robot_state_str}\n"
            f"#   ^ This is the robot's actual stuck/failure position. ALL recovery\n"
            f"#     waypoints, backup distances, and rotations should be computed\n"
            f"#     RELATIVE TO this position — not relative to the failed target.\n"
        )
        final_prompt += f"# Failed Action: {action_desc}\n"
        final_prompt += f"# VLM Feedback: {vlm_feedback_str}\n"
        # 失敗歷史：告知 LLM 剛才試了什麼、結果如何（純事實，不下指導語）
        final_prompt += (
            f"# Recent failure history (latest at bottom; you may repeat or change strategy):\n"
            f"{failure_history_str}\n"
        )
        final_prompt += tools_help + "\n"
        final_prompt += "?"

        print(f"[AdjustLLM] Calling Task LLM to generate recovery code...")
        
        # 调用基础的 call_LLM
        from utils.utils import call_LLM, extract_code, load_L2_memory
        try:
            raw_text = call_LLM(final_prompt)
        except Exception as e:
            self._log_event("replan_failed", {
                "action_info": action_info,
                "vlm_result": vlm_result,
                "attempt": self.replan_retry_counter[counter_key],
                "reason": f"adjust_llm_call_failed: {e}",
            })
            print(f"[AdjustLLM] 调用微调 LLM 失败: {e}")
            return False
        
        # 提取真正的代码
        adjust_code = extract_code(raw_text, "?")
        adjust_code = self._add_extra_forward_for_close_retry(
            adjust_code, action_info, vlm_result
        )
        print("========== AdjustLLM 生成的微调/恢复代码 ==========")
        print(adjust_code)
        print("===================================================")
        self._log_event("replan_generated", {
            "action_info": action_info,
            "vlm_result": vlm_result,
            "attempt": self.replan_retry_counter[counter_key],
            "raw_llm_output": raw_text,
            "adjust_code": adjust_code,
            "execute_code": REPLAN_EXECUTE_CODE,
        })
        
        if not adjust_code.strip():
            print("[AdjustLLM] 生成的代码为空，无法微调。")
            self._log_event("replan_failed", {
                "action_info": action_info,
                "attempt": self.replan_retry_counter[counter_key],
                "reason": "empty_adjust_code",
            })
            return False

        if not REPLAN_EXECUTE_CODE:
            print("[AdjustLLM] REPLAN_EXECUTE_CODE=False，仅记录微调代码，不实际执行。")
            self._log_event("replan_not_executed", {
                "action_info": action_info,
                "attempt": self.replan_retry_counter[counter_key],
                "adjust_code": adjust_code,
            })
            return False
            
        # 动态执行这段代码
        try:
            # 引入全局变量和依赖
            objects = load_L2_memory()
            from action import robot_api
            import ros_bridge as _ros_bridge_for_adjust
            from utils.utils import get_obj_xy, get_obj_z, get_obj_size, parse_obj_name
            local_env = {
                "objects": objects,
                "instruction": global_instruction,
                "move_to_xy": robot_api.move_to_xy,
                "move_to_obj_by_offset": robot_api.move_to_obj_by_offset,
                "pick_up_xy": robot_api.pick_up_xy,
                "pick_up_obj": robot_api.pick_up_obj,
                "pick_up_from_coffeetable": robot_api.pick_up_from_coffeetable,
                "pick_up_coke_from_balconytable": robot_api.pick_up_coke_from_balconytable,
                "put_down_xy": robot_api.put_down_xy,
                "put_down_obj_by_offset": robot_api.put_down_obj_by_offset,
                "put_down_between_objs": robot_api.put_down_between_objs,
                "put_down_coke_on_nightstand": robot_api.put_down_coke_on_nightstand,
                "run_c1_coke_to_nightstand": robot_api.run_c1_coke_to_nightstand,
                "run_c2_red_block_between_blue_yellow": robot_api.run_c2_red_block_between_blue_yellow,
                "run_c3_all_ground_blocks_to_trash": robot_api.run_c3_all_ground_blocks_to_trash,
                "run_c4_sofa_patrol_red_to_trash": robot_api.run_c4_sofa_patrol_red_to_trash,
                "get_obj_xy": get_obj_xy,
                "get_obj_z": get_obj_z,
                "get_obj_size": get_obj_size,
                "parse_obj_name": parse_obj_name,
                "load_L2_memory": load_L2_memory,
                "is_reachable": _ros_bridge_for_adjust.is_reachable,
                "get_current_pos": _ros_bridge_for_adjust.get_current_pos,
                "list_obstacles_near": _ros_bridge_for_adjust.list_obstacles_near,
                "find_reachable_approach_to": _ros_bridge_for_adjust.find_reachable_approach_to,
                # 低階開環動作（繞過 TEB）— 專給脫困用，距離不超過 1m
                "drive_forward": _ros_bridge_for_adjust.drive_forward,
                "rotate_to_face": _ros_bridge_for_adjust.rotate_to_face,
                "rotate_by": _ros_bridge_for_adjust.rotate_by,
                "np": np,
            }
            exec_globals = globals().copy()
            exec_globals.update(local_env)
            
            # 使用 exec 执行新生成的原语
            # 注意：新执行的原语里面又会调用 robot_api 的方法，从而再次触发 judge_llm.judge()
            # 这里有递归，依靠 max_replan_times 防止同一动作无限微调。
            exec(adjust_code, exec_globals)

            # 寫入失敗歷史（exec 完還能繼續就算 OK；exec raise 在 except 也會寫）
            try:
                post_obs = self._collect_observation()
                self.failure_history.append({
                    "action_desc": action_desc,
                    "vlm_error_type": vlm_result.get("error_type", "unknown"),
                    "robot_before": list(robot_pos_now),
                    "robot_after": [post_obs.get("robot_x", 0.0), post_obs.get("robot_y", 0.0)],
                    "adjust_code": adjust_code,
                    "exec_ok": True,
                })
            except Exception:
                pass

            print(f"[AdjustLLM] 微调动作执行完毕。")
            self._log_event("replan_executed", {
                "action_info": action_info,
                "attempt": self.replan_retry_counter[counter_key],
                "adjust_code": adjust_code,
                "success": True,
            })
            return True

        except Exception as e:
            # 失敗也寫入歷史 — LLM 下次能看到「上次寫的 code 引發了 X exception」
            try:
                post_obs = self._collect_observation()
                self.failure_history.append({
                    "action_desc": action_desc,
                    "vlm_error_type": vlm_result.get("error_type", "unknown"),
                    "robot_before": list(robot_pos_now),
                    "robot_after": [post_obs.get("robot_x", 0.0), post_obs.get("robot_y", 0.0)],
                    "adjust_code": adjust_code,
                    "exec_ok": False,
                    "exec_error": str(e)[:120],
                })
            except Exception:
                pass

            print(f"[AdjustLLM] 微调代码执行时发生异常: {e}")
            self._log_event("replan_failed", {
                "action_info": action_info,
                "attempt": self.replan_retry_counter[counter_key],
                "adjust_code": adjust_code,
                "reason": f"exec_failed: {e}",
            })
            return False

    def _add_extra_forward_for_close_retry(self, code: str, action_info: dict, vlm_result: dict) -> str:
        """Force close-range recovery when VLM says the robot stopped short.

        The LLM often repeats pick_up_obj/put_down_obj_by_offset without turning
        "move closer" into a physical nudge. Keep the generated structure, but
        add the optional extra_forward argument to the same primitive.
        """
        import ast

        feedback = " ".join([
            str(vlm_result.get("error_type", "")),
            str(vlm_result.get("reason", "")),
            str(vlm_result.get("suggested_correction", "")),
        ]).lower()
        needs_close_retry = any(token in feedback for token in (
            "move closer", "grasp_missed", "gripper is empty",
            "failed to pick", "floating", "next to", "placement_error",
        ))
        if not needs_close_retry or not code.strip():
            return code

        target_text = str(action_info.get("target", "")) + " " + feedback
        table_like = ("coke", "table", "desk", "nightstand")
        pick_step = 0.08 if any(k in target_text.lower() for k in table_like) else 0.12
        put_step = 0.12

        class ExtraForwardTransformer(ast.NodeTransformer):
            def visit_Call(self, node):
                self.generic_visit(node)
                if not isinstance(node.func, ast.Name):
                    return node
                if any(kw.arg == "extra_forward" for kw in node.keywords):
                    return node
                if node.func.id == "pick_up_obj":
                    node.keywords.append(ast.keyword(arg="extra_forward", value=ast.Constant(pick_step)))
                elif node.func.id == "put_down_obj_by_offset":
                    node.keywords.append(ast.keyword(arg="extra_forward", value=ast.Constant(put_step)))
                return node

        try:
            tree = ast.parse(code)
            tree = ExtraForwardTransformer().visit(tree)
            ast.fix_missing_locations(tree)
            return ast.unparse(tree)
        except Exception:
            return code
    
if __name__ == "__main__":
    # instruction = "pick up the bottle from the desk first and then put it between the apple and banana"
    instruction = "put the coke can on the desk"
    from runtime_context import CTX
    CTX["instruction"] = instruction
    CTX["step_idx"] = 0
    from action.robot_api import put_down_obj_by_offset
    put_down_obj_by_offset("desk",0,0)
