"""SkyBuddy middleware MCP server.

Exposes three tools to the LLM orchestrator:

- ``get_mission_context``: drones and search areas the plan must refer to
- ``submit_mission_plan``: validate a drone-to-area plan and start execution
- ``get_mission_status``: per-drone state and per-command results

Run from the repository root::

    PYTHONPATH=services/middleware python -m app.mcp_server                    # stdio
    MCP_TRANSPORT=http PYTHONPATH=services/middleware python -m app.mcp_server  # HTTP

Over stdio the client starts the server and the server stops when the client leaves, so
a flight in progress loses its Dispatcher thread (K5). Over HTTP the server stays up
between client sessions: flights continue after a client disconnects and later sessions
see the same mission state. Clients connect to ``http://<MCP_HOST>:<MCP_PORT>/mcp``.

Environment variables:

- ``MCP_TRANSPORT``: ``stdio`` (default) or ``http``; anything else is rejected at start
- ``MCP_HOST``: HTTP bind address (default ``127.0.0.1``). There is no authentication,
  so bind other addresses only on a trusted network.
- ``MCP_PORT``: HTTP port (default ``9000``)
- ``MIDDLEWARE_DRONES_CONFIG``: registry config (default ``config/drones.fake.json``)
- ``MIDDLEWARE_SCENARIOS_DIR``: directory of mission scenario JSON files
- ``MIDDLEWARE_EVENT_LOG``: JSONL event log path
"""

import os
import sys
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from fastmcp import FastMCP
from fastmcp.exceptions import ToolError
from typing_extensions import NotRequired, TypedDict

from app.runtime import (
    Dispatcher,
    DroneRegistry,
    EventLog,
    MissionScenario,
    MissionService,
    RegistryConfig,
    UnknownMissionError,
)

MIDDLEWARE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DRONES_CONFIG = MIDDLEWARE_ROOT / "config" / "drones.fake.json"
DEFAULT_SCENARIOS_DIR = MIDDLEWARE_ROOT / "config" / "scenarios"
DEFAULT_EVENT_LOG = MIDDLEWARE_ROOT / "logs" / "mission-events.jsonl"

# Same variable names and defaults as sandbox/mock-orchestrator/mock_drone_server.py, so
# the orchestrator swaps the mock for the middleware by MCP_SERVER_URL alone.
TRANSPORTS = ("stdio", "http")
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 9000
HTTP_PATH = "/mcp"


@dataclass(frozen=True)
class TransportSettings:
    """How the server talks to MCP clients. ``host``/``port`` are set for HTTP only."""

    transport: str
    host: str | None = None
    port: int | None = None

    @property
    def url(self) -> str | None:
        if self.transport != "http":
            return None
        return f"http://{self.host}:{self.port}{HTTP_PATH}"


def transport_from_env(environ: Mapping[str, str] = os.environ) -> TransportSettings:
    """Read ``MCP_TRANSPORT``/``MCP_HOST``/``MCP_PORT``; raise ValueError on bad values.

    Unlike the mock server, an unknown transport is an error rather than a silent
    fallback to stdio: an HTTP server that quietly ran over stdio would leave the
    orchestrator unable to connect without saying why.
    """
    transport = environ.get("MCP_TRANSPORT", "").strip() or "stdio"
    if transport not in TRANSPORTS:
        raise ValueError(
            f"MCP_TRANSPORT must be one of {', '.join(TRANSPORTS)}; got {transport!r}"
        )
    if transport == "stdio":
        return TransportSettings("stdio")

    host = environ.get("MCP_HOST", "").strip() or DEFAULT_HOST
    port_text = environ.get("MCP_PORT", "").strip() or str(DEFAULT_PORT)
    try:
        port = int(port_text)
    except ValueError:
        raise ValueError(f"MCP_PORT must be an integer; got {port_text!r}") from None
    if not 1 <= port <= 65_535:
        raise ValueError(f"MCP_PORT must be between 1 and 65535; got {port}")
    return TransportSettings("http", host, port)


class AssignmentInput(TypedDict):
    """One drone-to-area assignment proposed by the LLM."""

    drone_id: str
    area_id: str
    priority: NotRequired[int]


def create_mcp_server(service: MissionService) -> FastMCP:
    mcp = FastMCP("skybuddy-middleware")

    @mcp.tool()
    def get_mission_context(mission_id: str) -> dict:
        """현재 임무에 사용할 수 있는 드론 상태와 탐색 구역을 조회합니다.

        계획을 제출하기 전에 반드시 먼저 호출하세요. submit_mission_plan의 drone_id와
        area_id는 이 결과에 있는 값만 사용할 수 있습니다. status가 available이고
        connection_status가 connected인 드론만 배정할 수 있습니다.

        Args:
            mission_id: 임무 식별자 (예: "mission-001")
        """
        try:
            return service.mission_context(mission_id).model_dump(mode="json")
        except UnknownMissionError as exc:
            raise ToolError(str(exc.args[0])) from None

    @mcp.tool()
    def submit_mission_plan(
        mission_id: str,
        assignments: list[AssignmentInput],
        generated_at: str | None = None,
    ) -> dict:
        """드론별 탐색 구역 배정 계획을 제출합니다.

        좌표나 비행 명령은 만들지 마세요. 어떤 드론을 어떤 구역에 배정할지만 정하면
        미들웨어가 검증 후 이륙·구역 순회·귀환·착륙 명령을 생성해 실행합니다.
        결과는 즉시 반환되며 비행은 계속 진행되므로, 진행 상황은
        get_mission_status로 확인하세요.

        status가 "rejected"이면 errors의 code와 message를 보고 계획을 고쳐 다시
        제출하세요. 한 계획에서 같은 드론이나 같은 구역을 두 번 배정할 수 없습니다.

        Args:
            mission_id: 임무 식별자
            assignments: [{"drone_id": str, "area_id": str, "priority": 1~100}, ...]
                priority는 생략 가능하며 숫자가 클수록 먼저 시작합니다.
            generated_at: 생략하세요. 서버가 제출 시각으로 채웁니다. 직접 줄 때는 시간대
                포함 ISO 8601 (예: "2026-09-28T10:00:00+09:00")
        """
        raw_plan = {
            "mission_id": mission_id,
            "assignments": [dict(assignment) for assignment in assignments],
        }
        if generated_at:
            raw_plan["generated_at"] = generated_at
        return service.submit_plan(raw_plan).model_dump(mode="json")

    @mcp.tool()
    def get_mission_status(mission_id: str) -> dict:
        """임무 진행 상황을 조회합니다.

        state는 idle(계획 없음), running(비행 중), succeeded(모든 작업 완료),
        failed(하나 이상 실패) 중 하나입니다. tasks에는 드론별 명령과 각 명령의
        상태 이력이 들어 있습니다.

        Args:
            mission_id: 임무 식별자
        """
        try:
            return service.mission_status(mission_id).model_dump(mode="json")
        except UnknownMissionError as exc:
            raise ToolError(str(exc.args[0])) from None

    @mcp.tool()
    def list_missions() -> dict:
        """서버에 불러온 임무 목록을 조회합니다.

        사용자가 임무 ID를 말하지 않았으면 가장 먼저 호출하세요. 다른 도구의
        mission_id에는 이 목록에 있는 값만 사용할 수 있습니다. 임무가 여러 개이고
        어느 것인지 분명하지 않다면 임의로 고르지 말고 사용자에게 물어보세요.
        """
        return {
            "missions": [
                {
                    "mission_id": scenario.mission_id,
                    "instruction": scenario.instruction,
                    "area_ids": [area.area_id for area in scenario.search_areas],
                }
                for scenario in service.scenarios.values()
            ]
        }

    return mcp


def build_service_from_env(transport: TransportSettings | None = None) -> MissionService:
    transport = transport or TransportSettings("stdio")
    config_path = Path(os.environ.get("MIDDLEWARE_DRONES_CONFIG", DEFAULT_DRONES_CONFIG))
    scenarios_dir = Path(os.environ.get("MIDDLEWARE_SCENARIOS_DIR", DEFAULT_SCENARIOS_DIR))
    event_log_path = Path(os.environ.get("MIDDLEWARE_EVENT_LOG", DEFAULT_EVENT_LOG))

    config = RegistryConfig.load(config_path)
    registry = DroneRegistry.from_config(config)
    scenarios = [
        MissionScenario.load(path) for path in sorted(scenarios_dir.glob("*.json"))
    ]
    if not scenarios:
        raise RuntimeError(f"no mission scenario JSON found in {scenarios_dir}")
    event_log = EventLog(event_log_path)
    # Drones that fail to connect stay registered as disconnected; the server still starts.
    registry.connect_all()
    event_log.write(
        "server_started",
        transport=transport.transport,
        host=transport.host,
        port=transport.port,
        drones_config=str(config_path),
        missions=[scenario.mission_id for scenario in scenarios],
        dispatcher=config.dispatcher,
        waypoints={
            **config.waypoints.model_dump(mode="json"),
            "swath_width_m": {drone.drone_id: drone.swath_width_m for drone in config.drones},
        },
        drones=[handle.describe() for handle in registry.handles()],
    )
    dispatcher = Dispatcher(registry, event_log, config=config.dispatcher)
    return MissionService(
        registry,
        scenarios,
        dispatcher,
        event_log,
        waypoint_generator=config.waypoints.build_generator(),
    )


def stop_service(service: MissionService, transport: TransportSettings) -> list[dict]:
    """Record what is still flying, warn about it, then close every drone connection.

    Nothing is landed automatically: the Dispatcher threads end with the process, so a
    drone listed here keeps holding its last target until someone lands it (K5).
    ``close_all()`` waits for a blocking command (takeoff, land) that holds a client.
    """
    running = service.running_tasks()
    service.event_log.write(
        "server_stopping", transport=transport.transport, running_tasks=running
    )
    if running:
        lines = [
            (
                f"[mcp_server] 경고: 실행 중인 작업 {len(running)}개를 남기고 서버를 "
                "종료합니다. 자동 착륙은 하지 않으므로 아래 드론이 공중에 남아 있을 수 있습니다."
            )
        ]
        for task in running:
            command = task["current_command"] or {}
            lines.append(
                f"  - {task['drone_id']}: {task['task_id']} "
                f"(현재 명령 {command.get('command_type')} {command.get('status')})"
            )
        lines.append(
            "  수동 착륙 방법: docs/development/local-full-pipeline.md 의 '서버 중단 시 수동 착륙'"
        )
        print("\n".join(lines), file=sys.stderr, flush=True)
    service.registry.close_all()
    return running


def main() -> int:
    try:
        transport = transport_from_env()
    except ValueError as exc:
        # Reject before connecting to any drone.
        print(f"[mcp_server] {exc}", file=sys.stderr)
        return 2

    service = build_service_from_env(transport)
    server = create_mcp_server(service)
    try:
        if transport.transport == "http":
            print(f"[mcp_server] listening on {transport.url}", file=sys.stderr, flush=True)
            # Banner off: its box characters fail on non-UTF-8 consoles.
            server.run(
                transport="http",
                host=transport.host,
                port=transport.port,
                path=HTTP_PATH,
                show_banner=False,
            )
        else:
            server.run(transport="stdio", show_banner=False)
    except KeyboardInterrupt:
        pass  # Ctrl+C is the normal way to stop the HTTP server.
    finally:
        stop_service(service, transport)
    return 0


if __name__ == "__main__":
    sys.exit(main())
