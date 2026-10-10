"""SkyBuddy middleware MCP server.

Exposes three tools to the LLM orchestrator:

- ``get_mission_context``: drones and search areas the plan must refer to
- ``submit_mission_plan``: validate a drone-to-area plan and start execution
- ``get_mission_status``: per-drone state and per-command results

Run over stdio from the repository root::

    PYTHONPATH=services/middleware python -m app.mcp_server

Environment variables:

- ``MIDDLEWARE_DRONES_CONFIG``: registry config (default ``config/drones.fake.json``)
- ``MIDDLEWARE_SCENARIOS_DIR``: directory of mission scenario JSON files
- ``MIDDLEWARE_EVENT_LOG``: JSONL event log path
"""

import os
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

    return mcp


def build_service_from_env() -> MissionService:
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
    connect_errors = registry.connect_all()
    event_log.write(
        "server_started",
        drones_config=str(config_path),
        missions=[scenario.mission_id for scenario in scenarios],
        dispatcher=config.dispatcher,
        drones=[
            {
                "drone_id": handle.drone_id,
                "protocol": handle.protocol,
                "mode": handle.config.mode,
                "connected": handle.connected,
                "connect_error": connect_errors.get(handle.drone_id),
                "binding": handle.binding,
            }
            for handle in registry.handles()
        ],
    )
    dispatcher = Dispatcher(registry, event_log, config=config.dispatcher)
    return MissionService(registry, scenarios, dispatcher, event_log)


def main() -> None:
    service = build_service_from_env()
    try:
        create_mcp_server(service).run(show_banner=False)
    finally:
        service.registry.close_all()


if __name__ == "__main__":
    main()
