"""Common interface every LLM provider (Gemini, Claude, ...) implements.

MCP 도구 목록 조회와 실제 실행(session.call_tool)은 어떤 LLM을 쓰든 동일하다.
벤더마다 다른 건 "이 메시지 + 이 도구 목록을 보고 뭘 호출할지 판단하는 방법"뿐이라,
그것만 이 인터페이스로 분리한다.

판단은 한 번으로 안 끝날 수 있다 — 도구 실행 결과를 보고 LLM이 또 다른 도구를
부르고 싶어할 수 있기 때문이다. 그래서 decide()/continue_with_results() 둘 다
Decision을 리턴하고, 호출부(llm.py)는 Decision.is_final이 True가 될 때까지
"도구 실행 -> continue_with_results" 루프를 돈다.
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
    # 같은 provider의 continue_with_results()만 해석하는 불투명한 상태
    # (예: Gemini는 대화 히스토리 Content 객체 누적, Claude는 assistant 메시지 블록 누적 등)
    provider_state: Any = None

    @property
    def is_final(self) -> bool:
        return not self.tool_calls


class LLMProvider(ABC):
    @abstractmethod
    async def decide(self, message: str, mcp_tools: list[McpTool]) -> Decision:
        """자연어 명령과 MCP 도구 목록을 보고 어떤 도구를 호출할지 판단한다."""

    @abstractmethod
    async def continue_with_results(
        self,
        mcp_tools: list[McpTool],
        decision: Decision,
        tool_results: list[str],
    ) -> Decision:
        """직전 도구 실행 결과를 보고 다시 판단한다.

        추가로 도구를 불러야 하면 tool_calls가 채워진 Decision을,
        더 이상 부를 게 없으면 text가 채워진 Decision(is_final=True)을 리턴한다.
        """
