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

            print("--- propose_mission_plan 스키마 확인 ---")
            plan_schema = tools["propose_mission_plan"].inputSchema["properties"].keys()
            print(" ", list(plan_schema))

            print("--- propose_mission_plan: 정상 케이스 ---")
            r4 = await session.call_tool(
                "propose_mission_plan",
                {
                    "mission_id": "mission-001",
                    "assignments": [
                        {"drone_id": "drone-1", "area_id": "area-a", "priority": 80},
                        {"drone_id": "drone-2", "area_id": "area-b", "priority": 80},
                    ],
                    "generated_at": "2026-09-06T10:00:00+09:00",
                },
            )
            print("  결과:", r4.content[0].text)

            print("--- propose_mission_plan(accepted) 이후 실제 실행 여부 확인 ---")
            r4_drone1 = await session.call_tool("get_status", {"drone_id": "drone-1"})
            r4_drone2 = await session.call_tool("get_status", {"drone_id": "drone-2"})
            print("  drone-1:", r4_drone1.content[0].text)
            print("  drone-2:", r4_drone2.content[0].text)
            if "latitude=37.45" in r4_drone1.content[0].text and "latitude=37.46" in r4_drone2.content[0].text:
                print("  ✅ 계획대로 drone-1은 area-a, drone-2는 area-b로 실제 이동했습니다.")
            else:
                print("  ❌ 계획이 accepted됐는데도 드론이 실제로 이동하지 않았습니다.")

            print("--- propose_mission_plan: 중복 구역 배정(검증 실패) 케이스 ---")
            r5 = await session.call_tool(
                "propose_mission_plan",
                {
                    "mission_id": "mission-002",
                    "assignments": [
                        {"drone_id": "drone-1", "area_id": "area-a", "priority": 80},
                        {"drone_id": "drone-2", "area_id": "area-a", "priority": 80},
                    ],
                    "generated_at": "2026-09-06T10:00:00+09:00",
                },
            )
            print("  결과:", r5.content[0].text)

            print("--- split_search_area: L자 구역(area-c)을 2조각으로 분할 ---")
            r7 = await session.call_tool(
                "split_search_area", {"area_id": "area-c", "n_pieces": 2}
            )
            print("  결과:", r7.content[0].text)

            print("--- split_search_area가 만든 area_id로 propose_mission_plan 실행 ---")
            r8 = await session.call_tool(
                "propose_mission_plan",
                {
                    "mission_id": "mission-003",
                    "assignments": [
                        {"drone_id": "drone-1", "area_id": "area-c-1", "priority": 80},
                        {"drone_id": "drone-2", "area_id": "area-c-2", "priority": 80},
                    ],
                    "generated_at": "2026-09-06T10:00:00+09:00",
                },
            )
            print("  결과:", r8.content[0].text)
            r8_drone1 = await session.call_tool("get_status", {"drone_id": "drone-1"})
            r8_drone2 = await session.call_tool("get_status", {"drone_id": "drone-2"})
            print("  drone-1:", r8_drone1.content[0].text)
            print("  drone-2:", r8_drone2.content[0].text)
            if "area-c-1" in r7.content[0].text and "area-c-2" in r7.content[0].text:
                print("  ✅ area-c가 area-c-1, area-c-2로 분할되었습니다.")
            else:
                print("  ❌ 새 area_id가 생성되지 않았습니다.")

            print("--- split_search_area: 기존 점 구역(area-a)도 분할 가능한지 확인 ---")
            r9 = await session.call_tool(
                "split_search_area", {"area_id": "area-a", "n_pieces": 2}
            )
            print("  결과:", r9.content[0].text)
            if "area-a-1" in r9.content[0].text and "area-a-2" in r9.content[0].text:
                print("  ✅ area-a(기존에 점으로만 있던 구역)도 분할되었습니다.")
            else:
                print("  ❌ area-a는 분할되지 않았습니다.")

            print("--- return_home: 홈(0, 0)으로 직선 복귀하는지 확인 ---")
            r6 = await session.call_tool("return_home", {"drone_id": "drone-1"})
            print("  return_home:", r6.content[0].text)
            r6_status = await session.call_tool("get_status", {"drone_id": "drone-1"})
            print("  drone-1 최종 상태:", r6_status.content[0].text)
            if "latitude=0.0" in r6_status.content[0].text and "longitude=0.0" in r6_status.content[0].text:
                print("  ✅ 홈 위치(0, 0)로 정상 복귀했습니다.")
            else:
                print("  ❌ 홈 위치로 복귀하지 못했습니다.")


asyncio.run(main())
