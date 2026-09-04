"""이 demo_server.py에 MCP 클라이언트로 접속해서 도구를 조회/실행해본다.
LLM은 등장하지 않는다 - MCP 자체의 동작만 확인하기 위한 예제."""

import asyncio #기다림이 있을 수 있는 함수를 선언하기 위해
import sys
from pathlib import Path

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

server_params = StdioServerParameters(
    command=sys.executable,  # 이 클라이언트를 실행 중인 파이썬(=fastmcp가 설치된 venv)을 그대로 씀
    args=[str(Path(__file__).parent / "demo_server.py")],
)


async def main():
    async with stdio_client(server_params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            print("--- 1) 핸드셰이크 완료 ---")

            tools = await session.list_tools()
            print("--- 2) 서버가 가진 도구 목록 ---")
            for t in tools.tools:
                print(f"  이름: {t.name} / 설명: {t.description} / 파라미터: {t.inputSchema}")

            print("--- 3) 실제로 altitude(3) 호출 ---")
            result = await session.call_tool("takeoff", {"altitude": 3})
            print("  결과:", result.content[0].text)


asyncio.run(main())
