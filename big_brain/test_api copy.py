#!/usr/bin/env python3
"""
Simple API connectivity checker for SiliconFlow and OpenAI.

Usage examples:
  python3 test_api_connectivity.py
  python3 test_api_connectivity.py --only siliconflow
  python3 test_api_connectivity.py --only openai --timeout 30
"""

import argparse
import time
from typing import Tuple

from openai import OpenAI
from openai import APITimeoutError, APIConnectionError, AuthenticationError, BadRequestError

import config as cfg


def resolve_siliconflow() -> Tuple[str, str, str]:
    api_key = getattr(cfg, "SILICONFLOW_API_KEY", "")
    base_url = getattr(cfg, "SILICONFLOW_BASE_URL", "https://api.siliconflow.cn/v1")
    model = getattr(cfg, "SILICONFLOW_LLM_MODEL", "Qwen/Qwen2.5-7B-Instruct")
    return api_key, base_url, model


def resolve_openai() -> Tuple[str, str, str]:
    # Prefer explicit OPENAI_* in config.py, fallback to common defaults.
    api_key = getattr(cfg, "GPT_API_KEY", "")
    base_url = getattr(cfg, "GPT_BASE_URL", "https://api.openai.com/v1")
    model = getattr(cfg, "GPT_LLM_MODEL", "gpt-4o")
    return api_key, base_url, model

def resolve_ollama() -> Tuple[str, str, str]:
    api_key = "ollama"  # 任意字符串即可
    base_url = "http://localhost:11434/v1"
    model = "deepseek-r1:7b"
    return api_key, base_url, model


def test_one(name: str, api_key: str, base_url: str, model: str, timeout_s: float) -> bool:
    print("=" * 70)
    print(f"[{name}] base_url={base_url}")
    print(f"[{name}] model={model}")

    if not api_key:
        print(f"[{name}] FAIL: api_key is empty in config.py")
        return False

    client = OpenAI(api_key=api_key, base_url=base_url, timeout=timeout_s)
    t0 = time.time()
    try:
        resp = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": "roleplaying as a master of computer science"},
                {"role": "user", "content": "Can you give me the code of 'Hello World' in Python?"},
            ],
            temperature=0.0,
        )
        elapsed = time.time() - t0
        text = (resp.choices[0].message.content or "").strip()
        print(f"[{name}] PASS: reachable, elapsed={elapsed:.2f}s, reply={text!r}")
        return True

    except APITimeoutError:
        elapsed = time.time() - t0
        print(f"[{name}] FAIL: request timeout after {elapsed:.2f}s")
        return False
    except APIConnectionError as e:
        print(f"[{name}] FAIL: connection error -> {e}")
        return False
    except AuthenticationError as e:
        print(f"[{name}] FAIL: auth error -> {e}")
        return False
    except BadRequestError as e:
        # Model name or request format issue.
        print(f"[{name}] FAIL: bad request -> {e}")
        return False
    except Exception as e:
        print(f"[{name}] FAIL: unexpected error -> {type(e).__name__}: {e}")
        return False


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Test SiliconFlow/OpenAI API connectivity")
    p.add_argument(
        "--only",
        choices=["siliconflow", "openai", "ollama", "both"],
        default="both",
        help="Test target",
    )
    p.add_argument("--timeout", type=float, default=20.0, help="Per-request timeout seconds")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    sf_ok = None
    oa_ok = None

    if args.only in ("siliconflow", "both"):
        sf_key, sf_url, sf_model = resolve_siliconflow()
        sf_ok = test_one("SiliconFlow", sf_key, sf_url, sf_model, args.timeout)

    if args.only in ("openai", "both"):
        oa_key, oa_url, oa_model = resolve_openai()
        oa_ok = test_one("OpenAI", oa_key, oa_url, oa_model, args.timeout)
    
    if args.only in ("ollama", "both"):
        ollama_key, ollama_url, ollama_model = resolve_ollama()
        ollama_ok = test_one("Ollama", ollama_key, ollama_url, ollama_model, args.timeout)
    print("=" * 70)
    if args.only == "siliconflow":
        ok = bool(sf_ok)
    elif args.only == "openai":
        ok = bool(oa_ok)
    elif args.only == "ollama":
        ok = bool(ollama_ok)
    else:
        ok = bool(sf_ok) and bool(oa_ok) and bool(ollama_ok)

    print(f"SUMMARY: {'PASS' if ok else 'FAIL'}")




if __name__ == "__main__":
    main()
    # from openai import OpenAI

    # client = OpenAI(
    #     base_url="http://localhost:11434/v1",
    #     api_key="ollama"  # 任意字符串即可
    # )

    # response = client.chat.completions.create(
    #     model="deepseek-r1:7b",
    #     messages=[
    #         {"role": "system", "content": "你是一个机器人控制助手"},
    #         {"role": "user", "content": "如何用ROS控制机械臂？"}
    #     ],
    #     temperature=0.7,
    #     stream=False
    # )

    # print(response.choices[0].message.content)

