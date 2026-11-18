"""관제 화면용 상태 조회: LLM을 거치지 않고 미들웨어 MCP 도구를 직접 부른다.

화면은 몇 초마다 이 조회를 반복하므로 LLM을 호출하지 않는다(비용, 지연 없음).
MCP_SERVER_URL로 상주 중인 미들웨어에만 붙는다. 주소가 없으면 llm._connect()가
mock 서버를 매번 새로 띄우게 되므로 조회하지 않는다.
"""

import json
import os

from mcp import ClientSession
from mcp.types import CallToolResult

from app.llm import _connect

# 이 상태가 아닌 명령은 아직 진행 중이다 (미들웨어 dispatcher.TERMINAL_STATUSES와 같다).
_DONE_STATUSES = {"succeeded", "failed", "timed_out", "unsupported"}


class StatusUnavailable(RuntimeError):
    """미들웨어 상태를 조회할 수 없을 때. 메시지는 화면에 그대로 보여 준다."""


def is_connection_error(exc: BaseException) -> bool:
    """미들웨어가 꺼져 있어 접속하지 못한 경우인지.

    MCP 클라이언트는 실패를 예외 그룹으로 감싸 던지므로 안쪽까지 살핀다. httpx 계열 패키지
    이름이 mcp 버전마다 달라(httpx, httpx2) 클래스 이름으로 판단한다.
    """
    inner = getattr(exc, "exceptions", None)
    if inner:
        return any(is_connection_error(e) for e in inner)
    return isinstance(exc, ConnectionError) or type(exc).__name__ in {"ConnectError", "ConnectTimeout"}


async def read_status() -> dict:
    """불러온 모든 임무의 상태를 화면에 필요한 항목만 추려 돌려준다."""
    if not os.environ.get("MCP_SERVER_URL"):
        raise StatusUnavailable("MCP_SERVER_URL이 설정되지 않아 미들웨어 상태를 조회할 수 없습니다.")

    async with _connect() as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            missions = _payload(await session.call_tool("list_missions", {}))["missions"]
            summaries = []
            for mission in missions:
                status = _payload(
                    await session.call_tool(
                        "get_mission_status", {"mission_id": mission["mission_id"]}
                    )
                )
                summaries.append(_summarize(mission, status))
    return {"missions": summaries}


def _payload(result: CallToolResult) -> dict:
    """도구 결과를 dict로 꺼낸다. 도구 오류면 StatusUnavailable."""
    text = "".join(part.text for part in result.content if hasattr(part, "text"))
    if result.is_error:
        raise StatusUnavailable(text or "미들웨어 도구 호출이 실패했습니다.")
    if isinstance(result.structured_content, dict):
        return result.structured_content
    return json.loads(text)


def _summarize(mission: dict, status: dict) -> dict:
    return {
        "mission_id": mission["mission_id"],
        "state": status["state"],
        "area_ids": mission["area_ids"],
        "drones": [_drone(drone) for drone in status["drones"]],
        "tasks": [_task(task) for task in status["tasks"]],
    }


def _drone(drone: dict) -> dict:
    position = drone.get("position")
    return {
        "drone_id": drone["drone_id"],
        "protocol": drone["protocol"],
        "status": drone["status"],
        "connection": drone["connection_status"],
        "battery_percent": drone.get("battery_percent"),
        # 링크 품질 0~100(%). 시뮬레이션 장애 주입이 채우고, 값을 주지 않는 어댑터는 None.
        "link_quality_pct": drone.get("link_quality_pct"),
        "position": None
        if position is None
        else {
            "latitude": position["latitude"],
            "longitude": position["longitude"],
            "altitude_m": position["altitude_m"],
        },
    }


def _task(task: dict) -> dict:
    commands = task["commands"]
    current = next(
        (command for command in commands if command["status"] not in _DONE_STATUSES), None
    )
    return {
        "task_id": task["task_id"],
        "drone_id": task["drone_id"],
        "area_id": task["area_id"],
        "state": task["state"],
        "done_commands": sum(command["status"] == "succeeded" for command in commands),
        "total_commands": len(commands),
        "current_command": None if current is None else current["command_type"],
    }
