"""Gemini implementation of LLMProvider."""

import os

from google import genai
from google.genai import types
from mcp.types import Tool as McpTool

from app.providers.base import Decision, LLMProvider, ToolCall

_MODEL = "models/gemini-3.6-flash"

# 우리는 MCP session.call_tool()로 도구를 직접 실행하므로, SDK가 제공하는
# "자동 함수 호출(AFC)" 기능은 쓰지 않는다. disable=True를 명시하지 않으면
# generate_content()가 도구 유무와 무관하게 AFC 래퍼 경로를 타면서
# "Direct use of AFC ... is not recommended" 경고를 찍는다.
_DISABLE_AFC = types.AutomaticFunctionCallingConfig(disable=True)


def _mcp_tool_to_gemini_tool(tool: McpTool) -> types.Tool:
    """MCP 도구 스키마를 Gemini function declaration으로 변환합니다."""
    return types.Tool(
        function_declarations=[
            types.FunctionDeclaration(
                name=tool.name,
                description=tool.description,
                parameters=types.Schema.from_json_schema(
                    json_schema=types.JSONSchema(
                        **getattr(tool, "input_schema", getattr(tool, "inputSchema", {}))
                    )
                ),
            )
        ]
    )


class GeminiProvider(LLMProvider):
    def __init__(self) -> None:
        self._client = genai.Client(api_key=os.environ["GOOGLE_API_KEY"])

    async def decide(self, message: str, mcp_tools: list[McpTool]) -> Decision:
        gemini_tools = [_mcp_tool_to_gemini_tool(t) for t in mcp_tools]
        contents = [types.Content(role="user", parts=[types.Part.from_text(text=message)])]
        return await self._step(contents, gemini_tools)

    async def continue_with_results(
        self,
        mcp_tools: list[McpTool],
        decision: Decision,
        tool_results: list[str],
    ) -> Decision:
        state = decision.provider_state
        contents = state["contents"]
        gemini_tools = state["gemini_tools"]

        function_response_parts = [
            types.Part.from_function_response(name=call.name, response={"result": result})
            for call, result in zip(decision.tool_calls, tool_results)
        ]
        contents = [*contents, types.Content(role="user", parts=function_response_parts)]

        return await self._step(contents, gemini_tools)

    async def _step(self, contents: list[types.Content], gemini_tools: list[types.Tool]) -> Decision:
        """한 번의 generate_content 호출과, 그 결과를 Decision으로 변환하는 공통 로직.

        decide()와 continue_with_results()는 둘 다 "지금까지의 대화(contents)를
        보고 다음 행동을 판단한다"는 점에서 동일하다 — 차이는 contents를 처음
        만드는지, 직전 도구 결과를 이어붙이는지뿐이라 이 메서드로 통합했다.
        """
        response = await self._client.aio.models.generate_content(
            model=_MODEL,
            contents=contents,
            config=types.GenerateContentConfig(
                tools=gemini_tools, automatic_function_calling=_DISABLE_AFC
            ),
        )

        function_calls = response.function_calls
        if not function_calls:
            # 텍스트 파트가 하나도 없으면 .text는 None을 리턴하므로 폴백한다.
            return Decision(text=response.text or "")

        model_content = response.candidates[0].content
        return Decision(
            text=None,
            tool_calls=[
                ToolCall(name=call.name, args=dict(call.args or {})) for call in function_calls
            ],
            provider_state={
                "contents": [*contents, model_content],
                "gemini_tools": gemini_tools,
            },
        )
