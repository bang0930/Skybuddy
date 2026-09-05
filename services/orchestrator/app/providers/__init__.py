"""LLM provider factory.

LLM_PROVIDER 환경변수(기본값 "gemini")로 어떤 벤더의 LLM을 쓸지 고른다.
orchestrator의 나머지 코드는 이 함수가 반환하는 LLMProvider 인터페이스에만
의존하므로, 새 벤더를 추가할 때 app/llm.py는 건드릴 필요가 없다.
"""

import os

from app.providers.base import LLMProvider


def get_provider() -> LLMProvider:
    name = os.environ.get("LLM_PROVIDER", "gemini").lower()

    if name == "gemini":
        from app.providers.gemini import GeminiProvider

        return GeminiProvider()

    if name == "claude":
        from app.providers.claude import ClaudeProvider

        return ClaudeProvider()

    raise ValueError(f"Unknown LLM_PROVIDER: {name!r}")
