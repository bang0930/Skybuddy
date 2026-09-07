"""Common interface every LLM provider (Gemini, Claude, ...) implements.

MCP 도구 목록 조회와 실제 실행(session.call_tool)은 어떤 LLM을 쓰든 동일하다.
벤더마다 다른 건 "이 메시지 + 이 도구 목록을 보고 뭘 호출할지 판단하는 방법"과
"도구 실행 결과를 받아서 자연어 답변으로 마무리하는 방법" 뿐이라, 그 두 가지만
이 인터페이스로 분리한다.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any

from mcp.types import Tool as McpTool


@dataclass
class ToolCall:
    name: str
    args: dict


@dataclass
class Decision:
    text: str | None
    tool_calls: list[ToolCall] = field(default_factory=list)
    # 같은 provider의 finalize()만 해석하는 불투명한 상태
    # (예: Gemini는 대화 히스토리 Content 객체, Claude는 assistant 메시지 블록 등)
    provider_state: Any = None

    @property
    def is_final(self) -> bool:
        return not self.tool_calls


class LLMProvider(ABC):
    @abstractmethod
    async def decide(self, message: str, mcp_tools: list[McpTool]) -> Decision:
        """자연어 명령과 MCP 도구 목록을 보고 어떤 도구를 호출할지 판단한다."""

    @abstractmethod
    async def finalize(
        self,
        message: str,
        mcp_tools: list[McpTool],
        decision: Decision,
        tool_results: list[str],
    ) -> str:
        """MCP 도구 실행 결과를 받아 사용자에게 보여줄 최종 자연어 답변을 만든다."""
