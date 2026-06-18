# 存放各项配置

# LLM
# 智谱API
ZHIPU_API_KEY = "<YOUR_ZHIPU_API_KEY>"
ZHIPU_BASE_URL = "https://open.bigmodel.cn/api/paas/v4/"
ZHIPU_LLM_MODEL = "glm-4.7-flash"
ZHIPU_VLM_MODEL = "glm-4.6v-flash"

# 硅基流动API
SILICONFLOW_API_KEY = "<YOUR_SILICONFLOW_API_KEY>"
SILICONFLOW_BASE_URL = "https://api.siliconflow.cn/v1"
# SILICONFLOW_LLM_MODEL = "Qwen/Qwen2.5-7B-Instruct"
SILICONFLOW_LLM_MODEL = "deepseek-ai/DeepSeek-V3.2"
SILICONFLOW_VLM_MODEL = "THUDM/GLM-4.1V-9B-Thinking"

# DeepSeek 官方 API（OpenAI 相容，比 SiliconFlow 直接，價格約 70%）
DEEPSEEK_API_KEY = "<YOUR_DEEPSEEK_API_KEY>"
DEEPSEEK_BASE_URL = "https://api.deepseek.com/v1"
DEEPSEEK_LLM_MODEL = "deepseek-chat"        # 自動路由到當前最新（V4-flash）
# DEEPSEEK_LLM_MODEL = "deepseek-reasoner"  # R1 推理版，慢但精準

# Google Gemini API（OpenAI 相容端點，原生多模態，可同時當 LLM 與 VLM）
# 免費額度：1500 req/day, 15 req/min（gemini-2.5-flash）
GEMINI_API_KEY = "<YOUR_GEMINI_API_KEY>"
GEMINI_BASE_URL = "https://generativelanguage.googleapis.com/v1beta/openai/"
GEMINI_LLM_MODEL = "gemini-2.5-flash"
GEMINI_VLM_MODEL = "gemini-3.1-flash-lite"   # E2 VLM judge: image input + text output, higher daily quota

# GPT4o API
GPT_API_KEY = "<YOUR_GPT_API_KEY>"
GPT_BASE_URL = "https://api.gptsapi.net/v1" 
GPT_LLM_MODEL = "gpt-4o"
GPT_VLM_MODEL = "gpt-4o"

# ollama API
OLLAMA_API_KEY = "ollama"  # 任意字符串即可
OLLAMA_BASE_URL = "http://localhost:11434/v1"
OLLAMA_LLM_MODEL = "deepseek-r1:7b"


# ── 路由設定（混合架構：各取所長）─────────────────────────────
# Planner / Judge LLM → DeepSeek 官方（中文 code-gen 強、便宜）
# VLM                → Gemini 2.5 Flash（免費 quota 大、多模態強）

# Planner LLM
TASK_LLM_API_KEY  = DEEPSEEK_API_KEY
TASK_LLM_BASE_URL = DEEPSEEK_BASE_URL
TASK_LLM_MODEL    = DEEPSEEK_LLM_MODEL

# Judge LLM（replan 用，目前 exec 被注釋故未實際呼叫）
JUDGE_LLM_API_KEY  = DEEPSEEK_API_KEY
JUDGE_LLM_BASE_URL = DEEPSEEK_BASE_URL
JUDGE_LLM_MODEL    = DEEPSEEK_LLM_MODEL

# VLM → Gemini（免費 1500 req/day）
VLM_API_KEY      = GEMINI_API_KEY
VLM_API_BASE_URL = GEMINI_BASE_URL
VLM_MODEL        = GEMINI_VLM_MODEL

# 舊的 TARGET_* 變數保留供其他地方引用（例如 big_brain.py 列印模型名）
TARGET_API_KEY   = DEEPSEEK_API_KEY
TARGET_BASE_URL  = DEEPSEEK_BASE_URL
TARGET_LLM_MODEL = DEEPSEEK_LLM_MODEL
TARGET_VLM_MODEL = GEMINI_VLM_MODEL

import os as _os

# ── 项目路径（可移植）─────────────────────────────────────────
# PROJECT_ROOT：优先读环境变量 PROJECT_ROOT；否则取 big_brain 的上级目录
#   （解压后 = code/）。这样换机器/换用户无需改代码。
PROJECT_ROOT = _os.environ.get("PROJECT_ROOT") or _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
# L2 语义-几何资料库（gazebo_scene.yaml）。可用 GAZEBO_SCENE_PATH 覆盖。
SCENE_YAML_PATH = _os.environ.get("GAZEBO_SCENE_PATH") or _os.path.join(PROJECT_ROOT, "gazebo_scene.yaml")

MAX_REPLAN_TIMES = 1
REPLAN_EXECUTE_CODE = True

# Judge / VLM logging
JUDGE_LOG_DIR = _os.path.join(_os.path.dirname(__file__), "outputs", "judge_logs")

# RAG
# Optional local sentence-transformers model path. If unset or missing, the
# standard HuggingFace model name is used.
_RAG_LOCAL = _os.environ.get("RAG_MODEL_PATH", "")
RAG_MODEL = _RAG_LOCAL if _RAG_LOCAL and _os.path.exists(_RAG_LOCAL) else "sentence-transformers/all-mpnet-base-v2"
RAG_SIMILARITY_THRESHOLD = 0.5
HISTORY_PATH = _os.path.join(_os.path.dirname(__file__), "memory", "rag_history.json")

# JUDGE 规则阈值，单位全部是米。
# move_base 的实际停止误差通常在 0.10-0.25m；抓取/放置用机械臂可达范围判断。
MOVE_ERROR_THRESHOLD = 0.25
APPROACH_ERROR_THRESHOLD = 0.35
PLACE_ERROR_THRESHOLD = 0.25
CATCH_ERROR_THRESHOLD = 0.55

# 物体默认长宽高，单位米
BOX_SIZE = (0.50, 0.50, 0.50)

# YOLO Detect Objects
YOLO_DETECT_OBJECTS = ["desk","chair","bottle","apple","banana"]
