"""mock_drone_server.py의 move_to가 GeoPosition(latitude/longitude/altitude_m)
필드명을 쓰도록 잘 고쳐졌는지 확인하는 테스트 클라이언트. LLM은 등장하지 않고,
사람이 직접 정한 값으로 도구를 호출한다 (sandbox/mcp-101/demo_client.py와 같은 패턴).

사용법:
    cd sandbox/mock-orchestrator
    venv/bin/python test_client.py
"""

import asyncio
import sys
from pathlib import Path

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

server_params = StdioServerParameters(
    command=sys.executable,
    args=[str(Path(__file__).parent / "mock_drone_server.py")],
)


async def main():
    async with stdio_client(server_params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()

            tools = {t.name: t for t in (await session.list_tools()).tools}
            move_to_schema = tools["move_to"].inputSchema["properties"].keys()
            print("--- move_to 파라미터 이름 ---")
            print(" ", list(move_to_schema))

            expected = {"latitude", "longitude", "altitude_m"}
            if expected <= set(move_to_schema):
                print("  ✅ GeoPosition 필드명과 일치합니다.")
            else:
                print(f"  ❌ 아직 x/y/z로 되어 있거나 이름이 다릅니다. 기대값: {expected}")
                return

            print("--- takeoff -> move_to -> get_status 순서로 호출 ---")
            r1 = await session.call_tool("takeoff", {"altitude": 80})
            print("  takeoff:", r1.content[0].text)

            r2 = await session.call_tool(
                "move_to",
                {"latitude": 37.45, "longitude": 127.12, "altitude_m": 80},
            )
            print("  move_to:", r2.content[0].text)

            r3 = await session.call_tool("get_status", {})
            print("  get_status:", r3.content[0].text)


asyncio.run(main())
