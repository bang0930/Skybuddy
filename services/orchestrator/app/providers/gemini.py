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
        user_content = types.Content(role="user", parts=[types.Part.from_text(text=message)])

        response = await self._client.aio.models.generate_content(
            model=_MODEL,
            contents=[user_content],
            config=types.GenerateContentConfig(
                tools=gemini_tools, automatic_function_calling=_DISABLE_AFC
            ),
        )

        function_calls = response.function_calls
        if not function_calls:
            return Decision(text=response.text)

        return Decision(
            text=None,
            tool_calls=[
                ToolCall(name=call.name, args=dict(call.args or {})) for call in function_calls
            ],
            provider_state={
                "user_content": user_content,
                "model_content": response.candidates[0].content,
            },
        )

    async def finalize(
        self,
        message: str,
        mcp_tools: list[McpTool],
        decision: Decision,
        tool_results: list[str],
    ) -> str:
        state = decision.provider_state
        function_response_parts = [
            types.Part.from_function_response(name=call.name, response={"result": result})
            for call, result in zip(decision.tool_calls, tool_results)
        ]

        # tools를 넘기지 않아 모델이 이 응답에서 또 다른 function_call을
        # 만들 수 없게 막는다. finalize()는 이미 실행된 결과를 자연어로
        # 요약하는 단계라 추가 도구 호출이 필요 없다.
        follow_up = await self._client.aio.models.generate_content(
            model=_MODEL,
            contents=[
                state["user_content"],
                state["model_content"],
                types.Content(role="user", parts=function_response_parts),
            ],
            config=types.GenerateContentConfig(automatic_function_calling=_DISABLE_AFC),
        )
        return follow_up.text
