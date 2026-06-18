# 辅助判断位置的工具函数
import re
import base64
import os

import numpy as np
import yaml
from openai import OpenAI

from prompt.position_prompt import POSITION_PROMPT
from prompt.object_prompt import OBJECT_PROMPT
from config import TASK_LLM_API_KEY, TASK_LLM_BASE_URL, TASK_LLM_MODEL
from config import SCENE_YAML_PATH as _SCENE_YAML_PATH

# ── 場景 YAML 路徑（由 config.PROJECT_ROOT / GAZEBO_SCENE_PATH env 决定）──
_SEMANTIC_MAP_PATH = os.path.expanduser("~/.ros/semantic_map.yaml")

# 非導航目標物件（前綴比對，跳過不加入 L2 memory）
_SKIP_PREFIXES = [
    "airconditioner", "ball", "board", "carpet", "chandelier", "curtain",
    "deskportrait",   # 相框，不是桌子
    "door", "fitnessequipment", "dumbbell", "foldingdoor",
    "floor", "handle", "housewall", "roomwall", "roomwindow",
    "light", "portrait",
    "rangehood", "refrigerator", "securitycamera", "shoerack",
    "kitchenutensils", "seasoning", "tableware", "tablet",
    "tv", "vase", "wardrobe",
]

# 物件名稱 → 語義類別（用精確前綴匹配，避免 DeskPortrait 被當桌子）
_CATEGORY_PATTERNS = [
    ("table", [
        "balconytable", "kitchentable", "coffeetable",
        "readingdesk", "nightstand", "cookingbench", "tvkitchen",
    ]),
    ("chair", ["chaira", "chaird", "sofac"]),
    ("trash", ["trash", "bin"]),
    ("red_block", ["red_block"]),
    ("yellow_block", ["yellow_block"]),
    ("blue_block", ["blue_block"]),
]

_scene_cache: dict = {}

def _load_scene_yaml() -> dict:
    global _scene_cache
    for path in [_SCENE_YAML_PATH, _SEMANTIC_MAP_PATH]:
        if os.path.exists(path):
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            if data:
                _scene_cache = data
                return data
    return {}

def _get_scene_info(obj: str) -> dict:
    scene = _scene_cache if _scene_cache else _load_scene_yaml()
    if obj not in scene:
        scene = _load_scene_yaml()  # 重新加載
    if obj not in scene:
        raise KeyError(f"[utils] Object '{obj}' not found in scene. Available: {list(scene.keys())}")
    return scene[obj]

def _get_obj_info_unified(obj: str) -> dict:
    """統一查詢：動態物件走 ros_bridge → Gazebo；靜態家具走 yaml 快照。

    動態物件清單由 ros_bridge.DYNAMIC_OBJECTS 決定（目前 = {'red_block'}）。
    沒接 ROS 時自動 fallback 到 yaml（不會炸）。
    """
    try:
        import ros_bridge
        if obj in ros_bridge.DYNAMIC_OBJECTS and ros_bridge._get_model_state is not None:
            return ros_bridge.get_obj_info(obj)
    except (ImportError, AttributeError):
        pass
    return _get_scene_info(obj)

def get_obj_xy(obj: str):
    """获取物体在地图中的真实坐标 (object_x, object_y)，单位米。"""
    info = _get_obj_info_unified(obj)
    return (float(info["object_x"]), float(info["object_y"]))

def get_obj_z(obj: str) -> float:
    """获取物体顶面高度（surface_z），用于判断是否在桌上。"""
    info = _get_obj_info_unified(obj)
    return float(info.get("surface_z", 0.0))

def get_obj_size(obj: str):
    """获取物体的长宽高（米）。"""
    info = _get_obj_info_unified(obj)
    w = float(info.get("bbox_half_x", info.get("size_w", 0.5) / 2)) * 2.0
    h = float(info.get("bbox_half_y", info.get("size_h", 0.5) / 2)) * 2.0
    z = float(info.get("surface_z", 0.5))
    return (w, h, z)

def get_obj_rgb(obj: str) -> str:
    """获取物体颜色（场景 YAML 暂无颜色信息，返回 unknown）"""
    info = _get_scene_info(obj)
    r = info.get("color_r", -1)
    g = info.get("color_g", -1)
    b = info.get("color_b", -1)
    if r == -1:
        return "unknown"
    if r > g and r > b:
        return "red"
    if g > r and g > b:
        return "green"
    if b > r and b > g:
        return "blue"
    return "unknown"

def get_robot_pos():
    """获取机器人当前坐标 (x, y)"""
    try:
        import ros_bridge
        return ros_bridge.get_current_pos()
    except Exception:
        return (0.0, 0.0)

def get_robot_orientation() -> float:
    """获取机器人当前朝向（yaw，弧度）"""
    try:
        import ros_bridge
        return ros_bridge.get_current_orientation()
    except Exception:
        return 0.0

def get_robot_arm() -> bool:
    """获取机械臂是否夹持物品（暂时返回 False，无传感器）"""
    return False

def parse_obj_name(text:str,objects:dict)->str:
    # 从文本中解析出物体名称（永遠回傳 string，不論 LLM 產生 list 還是 string）
    lowered = (text or "").lower()

    # C-series thesis reliability guard:
    # For explicit color-block references, use deterministic mapping first
    # instead of LLM semantic parsing, to avoid red/blue/yellow drift.
    deterministic_map = [
        ("red_block", ("red_block", "red block")),
        ("blue_block", ("blue_block", "blue block")),
        ("yellow_block", ("yellow_block", "yellow block")),
    ]
    for canonical, aliases in deterministic_map:
        if any(alias in lowered for alias in aliases):
            candidates = objects.get(canonical) or []
            if candidates:
                return candidates[0]
            # fallback to canonical token itself if memory misses
            return canonical

    # 构建prompt
    base_prompt = OBJECT_PROMPT
    # 构建objects的字符串表示
    objects_str = "objects = {\n"
    for category, obj_list in objects.items():
        objects_str += f'    "{category}": {obj_list},\n'
    objects_str += "}\n"
    # 补充问题
    final_prompt = base_prompt + "\n" + objects_str + f"# {text}\n?"
    print(f"# {text}?")

    # 调用大模型
    raw_text = call_LLM(final_prompt)

    code = extract_code(raw_text,text)
    local_env = {
        "objects": objects,
        "get_obj_xy": get_obj_xy,
        "get_obj_z": get_obj_z,
        "get_obj_size": get_obj_size,
        "np": np,
    }
    exec(code, {}, local_env)
    result = local_env.get("ret_val")

    # 標準化：list → 取第一個；其他保持原樣
    # （object_prompt 跟 BASE_PROMPT 對 ret_val 型別不一致，這裡統一吃掉差異）
    if isinstance(result, (list, tuple)):
        if not result:
            lowered = text.lower()
            for key in ("red_block", "blue_block", "yellow_block"):
                color = key.split("_", 1)[0]
                if color in lowered and "block" in lowered and objects.get(key):
                    result = objects[key][0]
                    break
        if not result:
            raise ValueError(f"parse_obj_name: LLM 對 '{text}' 回傳空 list")
        elif isinstance(result, (list, tuple)):
            result = result[0]
    if not isinstance(result, str):
        raise TypeError(f"parse_obj_name: 預期 str，得到 {type(result).__name__}: {result!r}")
    return result

def call_LLM(prompt:str):
    client = OpenAI(
            api_key=TASK_LLM_API_KEY,
            base_url=TASK_LLM_BASE_URL,
        )
    model_name = TASK_LLM_MODEL
    response = client.chat.completions.create(
        model=model_name,
        messages=[
            # system有些模型需要有些模型不需要，具体看temperature为0，top_p为None时具体的输出
            {"role": "system", "content": "you only need to use code to answer the ? part"},
            {"role": "user", "content": prompt}
        ],
        temperature=0.0,
        top_p = None,
    )
    raw_text = response.choices[0].message.content
    return raw_text

def extract_code(text: str, last_line: str) -> str:
    # ai可能直接输出结果，也可能输出```python```代码块
    print("原始输出检查")
    print("====================================")
    print(text)
    print("====================================")
    pattern = r"```python(.*?)```"
    match = re.search(pattern, text, re.DOTALL)
    if match:
        return match.group(1).strip()
    # 沒有 ```python``` 區塊 — 嘗試 last_line 之後的部分
    # 但 last_line 之後若為空（LLM 重複了 RAG 的最後一行），表示 LLM 真的就只給了那一行 → 回傳整段
    if last_line and last_line in text:
        after = text.split(last_line, 1)[-1].strip()
        if after:
            return after
        # 否則 fall through 回傳原始 text
    return text.strip()

def load_L2_memory() -> dict:
    """從 gazebo_scene.yaml 讀取場景，過濾非導航目標，按類別分組，回傳 {category: [names]}"""
    scene = _load_scene_yaml()
    if not scene:
        print("[utils] 警告：scene yaml 為空，請先執行 python3 get_scene.py")
        return {}

    objects: dict = {}
    for name in scene:
        lower = name.lower()

        # 過濾掉非導航目標
        if any(lower.startswith(skip) for skip in _SKIP_PREFIXES):
            continue

        # 精確前綴比對分類
        matched = False
        for cat, prefixes in _CATEGORY_PATTERNS:
            if any(lower.startswith(p) for p in prefixes):
                objects.setdefault(cat, []).append(name)
                matched = True
                break

        # 未匹配的放 other（保留，方便除錯）
        if not matched:
            objects.setdefault("other", []).append(name)

    print(f"[utils] L2 memory loaded:")
    for cat, names in objects.items():
        print(f"  {cat}: {names}")
    return objects

def encode_image(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')

def parse_obj_position(text:str):
    # 从文本中解析出物体位置
    client = OpenAI(
            api_key=TASK_LLM_API_KEY,
            base_url=TASK_LLM_BASE_URL,
        )
    return (1.0, 2.0)

if __name__ == "__main__":
    objects = {
        "desk": ["desk1", "desk2"],
        "chair" : ["chair1"],
        "bottle" : ['bottle1','bottle2'],
        "fruits" : ['apple', 'banana']
    }
    print(parse_obj_name("red desk",objects))
    # code = parse_obj_name("the bottle that is closest to the chair",objects)
    # print(parse_obj_name("the bottle that is between the fruits",objects))
