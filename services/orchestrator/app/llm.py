"""자연어 드론 명령 오케스트레이션: MCP 클라이언트 + 교체 가능한 LLM provider.

이 파일은 특정 LLM SDK에 종속되지 않는다. "어떤 도구를 호출할지 판단"하고
"도구 실행 결과로 자연어 답변을 만드는" 부분은 app.providers.LLMProvider
구현체(GeminiProvider, ClaudeProvider, ...)에 위임하고, 여기서는
MCP 서버 접속과 실제 도구 실행(session.call_tool) 루프만 담당한다.
어떤 provider를 쓸지는 LLM_PROVIDER 환경변수로 정한다.
"""

from pathlib import Path

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

from app.providers import get_provider

# repo root: services/orchestrator/app/llm.py -> ../../../
_REPO_ROOT = Path(__file__).resolve().parents[3]
_MOCK_SERVER_PATH = _REPO_ROOT / "sandbox" / "mock-orchestrator" / "mock_drone_server.py"
_MOCK_SERVER_PYTHON = _REPO_ROOT / "sandbox" / "mock-orchestrator" / "venv" / "bin" / "python"

_server_params = StdioServerParameters(
    command=str(_MOCK_SERVER_PYTHON),
    args=[str(_MOCK_SERVER_PATH)],
)


async def run_command(user_message: str) -> dict:
    """자연어 명령을 받아 MCP 서버의 실제 도구를 통해 LLM이 드론을 제어하게 합니다."""
    provider = get_provider()

    async with stdio_client(_server_params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            mcp_tools = (await session.list_tools()).tools

            decision = await provider.decide(user_message, mcp_tools)
            if decision.is_final:
                return {"result": decision.text}

            tool_results = []
            for call in decision.tool_calls:
                tool_result = await session.call_tool(call.name, call.args)
                tool_results.append(
                    "".join(part.text for part in tool_result.content if hasattr(part, "text"))
                )

            final_text = await provider.finalize(user_message, mcp_tools, decision, tool_results)
            return {"result": final_text}
