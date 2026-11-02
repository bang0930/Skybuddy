"""HTTP로 실행 중인 mock 서버에 접속해 도구 호출이 되는지 확인한다. LLM은 쓰지 않는다.

서버를 먼저 HTTP로 실행해 둔다 (MCP_TRANSPORT=http, 기본 주소 http://127.0.0.1:9000/mcp).
이 스크립트는 오케스트레이터 환경의 Python으로 실행한다(HTTP 클라이언트가 그쪽에 있다).

사용법 (프로젝트 루트에서):
    services/orchestrator/venv/bin/python sandbox/mock-orchestrator/http_check.py
    # 윈도우: services\\orchestrator\\venv\\Scripts\\python.exe sandbox\\mock-orchestrator\\http_check.py

다른 주소를 확인하려면 MCP_SERVER_URL 환경변수에 지정한다.
"""

import asyncio
import os
import sys

from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client

# 윈도우 콘솔(cp949 등)에서도 한글이 깨지지 않게 한다.
sys.stdout.reconfigure(encoding="utf-8")

URL = os.environ.get("MCP_SERVER_URL", "http://127.0.0.1:9000/mcp")


async def main():
    print(f"접속 주소: {URL}")
    async with streamable_http_client(URL) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()

            tools = (await session.list_tools()).tools
            print("도구 목록:", [t.name for t in tools])

            result = await session.call_tool("get_status", {})
            print("get_status:", result.content[0].text)


asyncio.run(main())
