"""자연어 드론 명령 오케스트레이션: MCP 클라이언트 + 교체 가능한 LLM provider.

이 파일은 특정 LLM SDK에 종속되지 않는다. "어떤 도구를 호출할지 판단"하는 부분은
app.providers.LLMProvider 구현체(GeminiProvider, ClaudeProvider, ...)에 위임하고,
여기서는 MCP 서버 접속과 실제 도구 실행(session.call_tool) 루프만 담당한다.
어떤 provider를 쓸지는 LLM_PROVIDER 환경변수로 정한다.

판단 -> 실행을 한 번만 하고 끝내지 않는다. LLM이 도구 실행 결과를 보고 또 다른
도구를 부르고 싶어할 수 있으므로(예: 상태 확인 후 계획 수립), Decision.is_final이
True가 될 때까지 "실행 -> continue_with_results로 재판단"을 반복한다.
무한 루프를 막기 위해 라운드 수를 _MAX_ROUNDS로 제한한다.
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

_MAX_ROUNDS = 5


async def run_command(user_message: str) -> dict:
    """자연어 명령을 받아 MCP 서버의 실제 도구를 통해 LLM이 드론을 제어하게 합니다."""
    provider = get_provider()

    async with stdio_client(_server_params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            mcp_tools = (await session.list_tools()).tools

            decision = await provider.decide(user_message, mcp_tools)

            rounds = 0
            while not decision.is_final and rounds < _MAX_ROUNDS:
                tool_results = []
                for call in decision.tool_calls:
                    tool_result = await session.call_tool(call.name, call.args)
                    tool_results.append(
                        "".join(part.text for part in tool_result.content if hasattr(part, "text"))
                    )

                decision = await provider.continue_with_results(mcp_tools, decision, tool_results)
                rounds += 1

            if decision.is_final:
                return {"result": decision.text}

            return {
                "result": (
                    f"요청을 처리하는 데 {_MAX_ROUNDS}번의 시도로도 부족했습니다. "
                    "명령을 더 구체적으로 나눠서 다시 시도해주세요."
                )
            }
