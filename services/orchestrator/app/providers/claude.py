"""Claude(Anthropic) implementation of LLMProvider — 아직 미구현 스켈레톤.

ANTHROPIC_API_KEY와 `pip install anthropic`이 준비되면, GeminiProvider의
decide()/finalize() 구조를 그대로 참고해서 채우면 된다:
  - decide(): anthropic 클라이언트로 messages.create(tools=[...]) 호출.
    MCP 도구의 input_schema는 이미 JSON Schema라 Gemini 때와 달리
    Schema.from_json_schema 변환 없이 그대로 tool["input_schema"]에 넣을 수 있다.
    응답의 content 중 type == "tool_use" 블록들을 ToolCall로 변환.
  - finalize(): tool_use_id별로 {"type": "tool_result", ...} 블록을 만들어
    이전 assistant 메시지 뒤에 이어붙여 다시 messages.create 호출.

이 파일은 실제 API 호출/응답 파싱을 테스트하지 않은 상태이므로,
LLM_PROVIDER=claude로 전환하기 전에 반드시 동작 확인이 필요하다.
"""

from mcp.types import Tool as McpTool

from app.providers.base import Decision, LLMProvider


class ClaudeProvider(LLMProvider):
    def __init__(self) -> None:
        raise NotImplementedError(
            "ClaudeProvider는 아직 구현되지 않았습니다. anthropic SDK와 "
            "ANTHROPIC_API_KEY가 준비되면 이 클래스의 decide()/finalize()를 "
            "GeminiProvider를 참고해 구현해주세요."
        )

    async def decide(self, message: str, mcp_tools: list[McpTool]) -> Decision:
        raise NotImplementedError

    async def finalize(
        self,
        message: str,
        mcp_tools: list[McpTool],
        decision: Decision,
        tool_results: list[str],
    ) -> str:
        raise NotImplementedError
