"""
human_brain.py — 無 LLM 版大腦，用人工預寫代碼替代規劃，用規則匹配替代 parse_obj_name。
用於 API 額度不足時測試機器人執行能力。

用法：
  cd <PROJECT_ROOT>
  python3 big_brain/human_brain.py
"""

import sys
import os
import math
import numpy as np

# 把 Big Brain 根目錄加入 path
sys.path.insert(0, os.path.dirname(__file__))

import ros_bridge
from action.robot_api import move_to_xy, move_to_obj_by_offset
from action.robot_api import pick_up_xy, pick_up_obj
from action.robot_api import put_down_xy, put_down_obj_by_offset
from action.robot_api import pick_up_from_coffeetable, put_down_between_objs
from utils.utils import load_L2_memory, get_obj_xy, get_obj_z, get_obj_size, get_obj_rgb, get_robot_pos


# ── 無 LLM 的 parse_obj_name ──────────────────────────────────────
def parse_obj_name(text: str, objects: dict):
    """
    規則版 parse_obj_name：不呼叫 LLM，靠關鍵字比對從 objects dict 選出目標物件。
    回傳格式與 LLM 版一致：list of object names，或單一 string。
    """
    text_lower = text.lower()

    # 顏色篩選輔助
    def filter_by_color(names, color):
        return [n for n in names if get_obj_rgb(n) == color]

    def filter_by_ground(names):
        return [n for n in names if get_obj_z(n) < 0.1]

    # 找最小座標絕對值之和
    def find_min_abs_coord(names):
        best, best_sum = None, float('inf')
        for n in names:
            x, y = get_obj_xy(n)
            s = abs(x) + abs(y)
            if s < best_sum:
                best, best_sum = n, s
        return best

    # 找最靠近機器人的
    def find_closest(names):
        rx, ry = get_robot_pos()
        best, best_d = None, float('inf')
        for n in names:
            x, y = get_obj_xy(n)
            d = math.hypot(x - rx, y - ry)
            if d < best_d:
                best, best_d = n, d
        return best

    # 先嘗試從 objects dict 直接找匹配類別
    _KEYWORD_MAP = {
        "trash":     ["trash", "bin", "garbage"],
        "table":     ["table", "desk"],
        "chair":     ["chair"],
        "red_block": ["red block", "red_block", "block"],
        "other":     ["bed", "cabinet", "sofa", "couch"],
    }
    matched_cat = None
    for cat, kws in _KEYWORD_MAP.items():
        if any(kw in text_lower for kw in kws):
            if cat in objects:
                matched_cat = cat
                break

    # 若有直接匹配類別
    if matched_cat:
        candidates = objects[matched_cat]

        # 顏色篩選
        for color in ["red", "blue", "yellow", "green"]:
            if color in text_lower:
                filtered = filter_by_color(candidates, color)
                if filtered:
                    candidates = filtered
                break

        # 地面篩選
        if "ground" in text_lower or "floor" in text_lower or "地面" in text_lower:
            filtered = filter_by_ground(candidates)
            if filtered:
                candidates = filtered

        # 空間篩選
        if "smallest sum" in text_lower or "坐标绝对值之和最小" in text_lower:
            result = find_min_abs_coord(candidates)
            return result if result else candidates[0]

        if "closest" in text_lower or "最近" in text_lower:
            result = find_closest(candidates)
            return result if result else candidates[0]

        # 回傳單個還是列表
        if len(candidates) == 1:
            return candidates[0]
        return candidates

    # fallback：回傳所有物件的展開列表
    print(f"[human_parse] 無法匹配 '{text}'，回傳空列表")
    return []


# ── 任務代碼字典 ───────────────────────────────────────────────────
# 每個任務對應一段 Python 代碼字符串，直接 exec 執行
TASK_CODE = {

"N1": """
move_to_xy(100, 200)
""",

"N2": """
trash_can_obj = parse_obj_name('trash can', objects)
move_to_obj_by_offset(trash_can_obj[0] if isinstance(trash_can_obj, list) else trash_can_obj, 0, 0)
""",

"N3": """
trash_can_obj = parse_obj_name('trash can', objects)
trash_name = trash_can_obj[0] if isinstance(trash_can_obj, list) else trash_can_obj
trash_can_x, trash_can_y = get_obj_xy(trash_name)
move_to_xy(trash_can_x + 25, trash_can_y + 25)
move_to_xy(trash_can_x + 25, trash_can_y - 25)
move_to_xy(trash_can_x - 25, trash_can_y - 25)
move_to_xy(trash_can_x - 25, trash_can_y + 25)
move_to_xy(trash_can_x + 25, trash_can_y + 25)
""",

"N4": """
original_x, original_y = get_robot_pos()
trash_can_obj = parse_obj_name('the trash can with the smallest sum of absolute coordinates', objects)
trash_name = trash_can_obj[0] if isinstance(trash_can_obj, list) else trash_can_obj
trash_can_x, trash_can_y = get_obj_xy(trash_name)
for angle in np.linspace(0, 2 * np.pi, 8, endpoint=False):
    offset_x = 50 * np.cos(angle)
    offset_y = 50 * np.sin(angle)
    move_to_xy(trash_can_x + offset_x, trash_can_y + offset_y)
move_to_xy(original_x, original_y)
""",

"G1": """
red_block_obj = parse_obj_name('red block', objects)
name = red_block_obj[0] if isinstance(red_block_obj, list) else red_block_obj
pick_up_obj(name)
""",

"G2": """
red_block_obj = parse_obj_name('red block on the ground', objects)
name = red_block_obj[0] if isinstance(red_block_obj, list) else red_block_obj
pick_up_obj(name)
""",

"G3": """
table_obj = parse_obj_name('table', objects)
name = table_obj[0] if isinstance(table_obj, list) else table_obj
put_down_obj_by_offset(name, 0, 0)
""",

"G4": """
blue_block_obj = parse_obj_name('blue block', objects)
yellow_block_obj = parse_obj_name('yellow block', objects)
blue_name = blue_block_obj[0] if isinstance(blue_block_obj, list) else blue_block_obj
yellow_name = yellow_block_obj[0] if isinstance(yellow_block_obj, list) else yellow_block_obj
put_down_between_objs(blue_name, yellow_name)
""",

"C1": """
cup_obj = parse_obj_name('cup on the sofa', objects)
chair_obj = parse_obj_name('chair', objects)
cup_name = cup_obj[0] if isinstance(cup_obj, list) else cup_obj
chair_name = chair_obj[0] if isinstance(chair_obj, list) else chair_obj
move_to_obj_by_offset(cup_name, 0, 0)
pick_up_obj(cup_name)
move_to_obj_by_offset(chair_name, 0, 0)
put_down_obj_by_offset(chair_name, 0, 0)
""",

"C2": """
red_block_obj = parse_obj_name('red block on the table', objects)
blue_block_obj = parse_obj_name('blue block', objects)
yellow_block_obj = parse_obj_name('yellow block', objects)
red_name = red_block_obj[0] if isinstance(red_block_obj, list) else red_block_obj
blue_name = blue_block_obj[0] if isinstance(blue_block_obj, list) else blue_block_obj
yellow_name = yellow_block_obj[0] if isinstance(yellow_block_obj, list) else yellow_block_obj
pick_up_from_coffeetable(red_name)
put_down_between_objs(blue_name, yellow_name)
""",

"C3": """
objects_on_ground = parse_obj_name('objects on the ground', objects)
trash_can_obj = parse_obj_name('trash can', objects)
trash_name = trash_can_obj[0] if isinstance(trash_can_obj, list) else trash_can_obj
if not isinstance(objects_on_ground, list):
    objects_on_ground = [objects_on_ground]
for obj in objects_on_ground:
    move_to_obj_by_offset(obj, 0, 0)
    pick_up_obj(obj)
    move_to_obj_by_offset(trash_name, 0, 0)
    put_down_obj_by_offset(trash_name, 0, 0)
""",

"C4": """
table_obj = parse_obj_name('table', objects)
table_name = table_obj[0] if isinstance(table_obj, list) else table_obj
table_x, table_y = get_obj_xy(table_name)
trash_can_obj = parse_obj_name('trash can', objects)
trash_name = trash_can_obj[0] if isinstance(trash_can_obj, list) else trash_can_obj
corners = [
    (table_x + 50, table_y + 50),
    (table_x + 50, table_y - 50),
    (table_x - 50, table_y - 50),
    (table_x - 50, table_y + 50),
]
for cx, cy in corners:
    move_to_xy(cx, cy)
    red_blocks = parse_obj_name('red block', objects)
    if not isinstance(red_blocks, list):
        red_blocks = [red_blocks] if red_blocks else []
    for rb in red_blocks:
        move_to_obj_by_offset(rb, 0, 0)
        pick_up_obj(rb)
        move_to_obj_by_offset(trash_name, 0, 0)
        put_down_obj_by_offset(trash_name, 0, 0)
""",
}

TASK_LIST = """
  Navigation:
   N1  导航到 (1.0, 2.0)
   N2  导航到垃圾桶旁边
   N3  绕沙发做 4m × 4m 方形运动
   N4  前往坐标绝对值之和最小的垃圾桶，然后返回起点

  Manipulation:
    G1  在机器人已位于红色方块旁边时，夹起 red_block
    G2  从起点出发，导航到地面 red_block 旁边并夹起它
    G3  把物體放到桌子上
    G4  把物體放到藍色方塊和黃色方塊中間

  Composite:
    C1  把桌上的水杯放到櫃子上
    C2  把桌上紅色方塊拿起，放到藍黃方塊中間
    C3  把地上所有非家具丟到垃圾桶
    C4  以 100×100 方形繞桌子，見紅色方塊就撿起丟到垃圾桶
"""


def run_task(task_id: str, objects: dict):
    task_id = task_id.upper()
    if task_id not in TASK_CODE:
        print(f"[HumanBrain] 未知任務 '{task_id}'，可用：{list(TASK_CODE.keys())}")
        return False

    code = TASK_CODE[task_id]
    print(f"\n========== 任務 {task_id} 執行代碼 ==========")
    print(code.strip())
    print("==========================================\n")

    exec_globals = {
        "np": np,
        "objects": objects,
        "parse_obj_name": parse_obj_name,
        "move_to_xy": move_to_xy,
        "move_to_obj_by_offset": move_to_obj_by_offset,
        "pick_up_xy": pick_up_xy,
        "pick_up_obj": pick_up_obj,
        "pick_up_from_coffeetable": pick_up_from_coffeetable,
        "put_down_xy": put_down_xy,
        "put_down_obj_by_offset": put_down_obj_by_offset,
        "put_down_between_objs": put_down_between_objs,
        "get_obj_xy": get_obj_xy,
        "get_obj_z": get_obj_z,
        "get_obj_size": get_obj_size,
        "get_obj_rgb": get_obj_rgb,
        "get_robot_pos": get_robot_pos,
        "load_L2_memory": load_L2_memory,
    }
    try:
        exec(code, exec_globals)
        print(f"✓ 任務 {task_id} 執行完成")
        return True
    except Exception as e:
        print(f"✗ 任務 {task_id} 執行異常：{e}")
        return False


def run_interactive():
    print("\n[HumanBrain] 就緒（無 LLM 模式）")
    print(TASK_LIST)
    objects = load_L2_memory()
    print("輸入任務編號（如 N1 / G1 / C2），list 重新列出，q 退出\n")

    while True:
        try:
            raw = input("任務> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not raw:
            continue
        if raw.lower() in ("q", "quit"):
            break
        if raw.lower() in ("list", "ls"):
            print(TASK_LIST)
            continue
        run_task(raw, objects)


if __name__ == "__main__":
    ros_bridge.init()
    run_interactive()
