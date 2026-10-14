"""Tests for waypoint generator selection, path checks, and plan -> command reproducibility."""

import inspect
import json
import math
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest
from conftest import SWATH_WIDTH_M, fast_registry_config, plan, scenario
from test_mission_service import wait_for_completion

from app.mcp_server import build_service_from_env
from app.runtime import (
    CoverageGenerator,
    Dispatcher,
    DispatcherConfig,
    DroneRegistry,
    EventLog,
    MissionService,
    VertexTraversalGenerator,
    WaypointConfig,
    build_mission_task,
    build_mission_tasks,
    validate_plan,
)
from app.runtime.coverage import _to_xy, generate_coverage_waypoints
from app.runtime.waypoints import (
    DEFAULT_MAX_LEG_M,
    DEFAULT_SIDE_OVERLAP,
    WaypointCheckError,
    path_problems,
)
from app.schemas import (
    AltitudeReference,
    DroneCommandType,
    GeoCoordinate,
    GeoPosition,
    MissionContext,
    MissionPlan,
    SearchArea,
    SubmissionStatus,
)

MIDDLEWARE = Path(__file__).resolve().parents[1]
NOW = datetime(2026, 10, 14, 3, 0, tzinfo=timezone.utc)

# ~200 m x 200 m square, large enough that coverage legs are split at 100 m.
LARGE_AREA = SearchArea(
    area_id="area-large",
    boundary=[
        GeoCoordinate(latitude=-35.3633, longitude=149.1652),
        GeoCoordinate(latitude=-35.3633, longitude=149.1674),
        GeoCoordinate(latitude=-35.3615, longitude=149.1674),
        GeoCoordinate(latitude=-35.3615, longitude=149.1652),
    ],
    search_altitude_m=40,
    search_altitude_reference=AltitudeReference.HOME_RELATIVE,
)


def leg_lengths(points):
    return [
        math.hypot(*_to_xy(b.latitude, b.longitude, a.latitude, a.longitude))
        for a, b in zip(points, points[1:])
    ]


def at_altitude(points, altitude, reference=AltitudeReference.HOME_RELATIVE):
    return [
        GeoPosition(
            latitude=p.latitude,
            longitude=p.longitude,
            altitude_m=altitude,
            altitude_reference=reference,
        )
        for p in points
    ]


def context(registry) -> MissionContext:
    current = scenario()
    return MissionContext(
        mission_id=current.mission_id,
        instruction=current.instruction,
        drones=registry.states(),
        search_areas=current.search_areas,
        requested_at=NOW,
    )


# ------------------------------------------------------------------ generator settings


def test_defaults_match_the_coverage_function() -> None:
    defaults = inspect.signature(generate_coverage_waypoints).parameters
    assert DEFAULT_SIDE_OVERLAP == defaults["side_overlap"].default
    assert DEFAULT_MAX_LEG_M == defaults["max_leg_m"].default
    config = WaypointConfig()
    assert (config.generator, config.side_overlap, config.max_leg_m) == ("coverage", 0.2, 100)


def test_config_selects_generator() -> None:
    assert WaypointConfig().build_generator() == CoverageGenerator()
    assert WaypointConfig(side_overlap=0.5, max_leg_m=50).build_generator() == (
        CoverageGenerator(side_overlap=0.5, max_leg_m=50)
    )
    assert WaypointConfig(generator="vertex_traversal").build_generator() == (
        VertexTraversalGenerator()
    )


def test_coverage_generator_passes_settings_through() -> None:
    generator = CoverageGenerator(side_overlap=0.5, max_leg_m=60)
    assert generator.generate(LARGE_AREA, swath_width_m=40) == generate_coverage_waypoints(
        LARGE_AREA.boundary, 40, side_overlap=0.5, max_leg_m=60
    )
    assert max(leg_lengths(generator.generate(LARGE_AREA, swath_width_m=40))) <= 60


def test_coverage_generator_requires_swath() -> None:
    with pytest.raises(ValueError, match="swath_width_m"):
        CoverageGenerator().generate(LARGE_AREA, swath_width_m=None)


def test_vertex_traversal_keeps_the_old_path_and_splits_long_edges() -> None:
    small = scenario().search_areas[0]
    assert VertexTraversalGenerator().generate(small, swath_width_m=None) == [
        *small.boundary,
        small.boundary[0],
    ]

    points = VertexTraversalGenerator().generate(LARGE_AREA, swath_width_m=None)
    assert points[0] == points[-1] == LARGE_AREA.boundary[0]
    assert all(vertex in points for vertex in LARGE_AREA.boundary)
    assert max(leg_lengths(points)) <= DEFAULT_MAX_LEG_M


# ------------------------------------------------------------------------ path checks


@pytest.mark.parametrize(
    "generator",
    [CoverageGenerator(), VertexTraversalGenerator()],
    ids=lambda g: g.name,
)
@pytest.mark.parametrize("area", [*scenario().search_areas, LARGE_AREA], ids=lambda a: a.area_id)
def test_generated_paths_pass_the_checks(generator, area) -> None:
    points = generator.generate(area, swath_width_m=SWATH_WIDTH_M)
    targets = at_altitude(points, area.search_altitude_m)
    assert path_problems(area, targets, max_leg_m=generator.max_leg_m) == []


def test_path_check_reports_outside_point_wrong_altitude_and_long_leg() -> None:
    area = scenario().search_areas[0]
    inside = area.boundary[0]
    # About 111 m north of the area.
    outside = GeoCoordinate(latitude=inside.latitude + 0.001, longitude=inside.longitude)
    targets = [
        *at_altitude([inside], area.search_altitude_m),
        *at_altitude([outside], area.search_altitude_m),
        *at_altitude([inside], area.search_altitude_m + 5),
        *at_altitude([inside], area.search_altitude_m, AltitudeReference.MSL),
    ]

    problems = path_problems(area, targets, max_leg_m=100)

    assert any("waypoint 1" in p and "outside" in p for p in problems)
    assert any("waypoint 2 altitude 35.0m" in p for p in problems)
    assert any("waypoint 3" in p and "msl" in p for p in problems)
    assert any("leg to waypoint 1" in p and "longer than max_leg_m" in p for p in problems)
    assert path_problems(area, [], max_leg_m=100) == ["no waypoints were generated for area-north"]


def test_points_on_the_boundary_are_inside() -> None:
    area = scenario().search_areas[0]
    targets = at_altitude([*area.boundary, area.boundary[0]], area.search_altitude_m)
    assert path_problems(area, targets, max_leg_m=100) == []


class OffsetGenerator:
    """A faulty generator whose path drifts out of the area."""

    name = "offset"
    max_leg_m = 100.0

    def generate(self, area, *, swath_width_m):
        return [
            GeoCoordinate(latitude=p.latitude + 0.001, longitude=p.longitude)
            for p in area.boundary
        ]


def test_build_raises_when_the_path_fails_its_checks(registry) -> None:
    with pytest.raises(WaypointCheckError) as error:
        build_mission_task(
            task_id="task-001",
            mission_id="mission-001",
            drone=registry.state("drone-01"),
            area=scenario().search_areas[0],
            created_at=NOW,
            generator=OffsetGenerator(),
        )
    assert len(error.value.problems) == 3  # every vertex is outside


def test_failed_path_check_rejects_the_plan_without_flying(registry, event_log) -> None:
    dispatcher = Dispatcher(registry, event_log)
    service = MissionService(
        registry, [scenario()], dispatcher, event_log, waypoint_generator=OffsetGenerator()
    )

    result = service.submit_plan(plan(("drone-01", "area-north"), ("drone-02", "area-south")))

    assert result.status == SubmissionStatus.REJECTED
    assert [issue.code for issue in result.errors] == ["WAYPOINT_CHECK_FAILED"] * 2
    assert result.errors[0].location == "assignments.0.area_id"
    assert "outside area-north" in result.errors[0].message
    assert "command_transition" not in [event["event"] for event in event_log.events]
    assert service.mission_status("mission-001").tasks == []


def test_invalid_boundary_is_rejected_by_the_service_not_raised(registry, event_log) -> None:
    areas = [area.model_dump() for area in scenario().search_areas]
    a, b, c = areas[0]["boundary"]
    areas[0]["boundary"] = [a, c, b, {"latitude": c["latitude"], "longitude": a["longitude"]}]
    service = MissionService(
        registry, [scenario(search_areas=areas)], Dispatcher(registry, event_log), event_log
    )

    result = service.submit_plan(plan(("drone-01", "area-north")))

    assert result.status == SubmissionStatus.REJECTED
    assert [issue.code for issue in result.errors] == ["INVALID_AREA_BOUNDARY"]


def test_coverage_task_flies_rows_at_search_altitude(registry) -> None:
    drone = registry.state("drone-01")
    area = scenario().search_areas[1]

    task = build_mission_task(
        task_id="task-001",
        mission_id="mission-001",
        drone=drone,
        area=area,
        created_at=NOW,
        generator=CoverageGenerator(),
        swath_width_m=SWATH_WIDTH_M,
    )

    rows = CoverageGenerator().generate(area, swath_width_m=SWATH_WIDTH_M)
    types = [command.payload.command_type for command in task.commands]
    assert types == [
        DroneCommandType.TAKEOFF,
        *[DroneCommandType.GOTO] * (len(rows) + 1),  # rows, then goto(home)
        DroneCommandType.LAND,
    ]
    area_legs = [command.payload.target for command in task.commands[1 : 1 + len(rows)]]
    assert [(t.latitude, t.longitude) for t in area_legs] == [
        (p.latitude, p.longitude) for p in rows
    ]
    assert {(t.altitude_m, t.altitude_reference) for t in area_legs} == {
        (50, AltitudeReference.HOME_RELATIVE)
    }


# ---------------------------------------------------------------------- reproducibility


def without_times(tasks):
    """Task dumps with every timestamp removed, for comparing repeated builds."""
    return [
        (
            area_id,
            task.model_dump(
                mode="json",
                exclude={"created_at": True, "commands": {"__all__": {"created_at"}}},
            ),
        )
        for task, area_id in tasks
    ]


def test_same_context_and_plan_build_identical_tasks(registry) -> None:
    current = context(registry)
    accepted, issues = validate_plan(
        plan(("drone-01", "area-north"), ("drone-02", "area-south")), current
    )
    assert issues == []

    def build(created_at):
        tasks, problems = build_mission_tasks(
            accepted,
            current,
            generator=CoverageGenerator(),
            swath_widths={"drone-01": SWATH_WIDTH_M, "drone-02": SWATH_WIDTH_M},
            first_task_number=1,
            created_at=created_at,
        )
        assert problems == []
        return tasks

    runs = [build(NOW + timedelta(seconds=offset)) for offset in range(5)]

    assert all(without_times(run) == without_times(runs[0]) for run in runs)
    first = runs[0]
    assert [task.task_id for task, _ in first] == ["task-001-drone-01", "task-002-drone-02"]
    assert [c.command_id for c in first[0][0].commands] == [
        f"task-001-drone-01-c{index:02d}" for index in range(1, len(first[0][0].commands) + 1)
    ]
    # Rebuilding from a JSON round trip of the same inputs gives the same result too.
    replayed_context = MissionContext.model_validate_json(current.model_dump_json())
    replayed_plan = MissionPlan.model_validate_json(accepted.model_dump_json())
    tasks, _ = build_mission_tasks(
        replayed_plan,
        replayed_context,
        generator=CoverageGenerator(),
        swath_widths={"drone-01": SWATH_WIDTH_M, "drone-02": SWATH_WIDTH_M},
        first_task_number=1,
        created_at=NOW,
    )
    assert without_times(tasks) == without_times(first)


def test_priority_order_and_numbering_are_stable(registry) -> None:
    current = context(registry)
    raw = plan(("drone-01", "area-north"), ("drone-02", "area-south"))
    raw["assignments"][1]["priority"] = 5
    accepted, _ = validate_plan(raw, current)

    tasks, _ = build_mission_tasks(
        accepted,
        current,
        generator=CoverageGenerator(),
        swath_widths={"drone-01": SWATH_WIDTH_M, "drone-02": SWATH_WIDTH_M},
        first_task_number=7,
        created_at=NOW,
    )
    assert [task.task_id for task, _ in tasks] == ["task-007-drone-02", "task-008-drone-01"]


def test_fresh_servers_log_identical_commands_for_the_same_plan(tmp_path) -> None:
    def accepted_commands(run: int):
        registry = DroneRegistry.from_config(fast_registry_config())
        registry.connect_all()
        event_log = EventLog(tmp_path / f"events-{run}.jsonl")
        dispatcher = Dispatcher(
            registry,
            event_log,
            config=DispatcherConfig(waypoint_timeout_s=10),
            telemetry_poll_timeout_s=0.2,
        )
        service = MissionService(registry, [scenario()], dispatcher, event_log)
        try:
            service.submit_plan(plan(("drone-01", "area-north"), ("drone-02", "area-south")))
            wait_for_completion(service)
        finally:
            registry.close_all()
        event = next(e for e in event_log.events if e["event"] == "plan_accepted")
        for commands in event["commands"]:
            for command in commands:
                command.pop("created_at")
        return event["tasks"], event["commands"]

    assert accepted_commands(1) == accepted_commands(2)


# ------------------------------------------------------------------------ server start


def test_server_started_logs_waypoint_settings(tmp_path, monkeypatch) -> None:
    log_path = tmp_path / "events.jsonl"
    monkeypatch.setenv("MIDDLEWARE_EVENT_LOG", str(log_path))
    monkeypatch.delenv("MIDDLEWARE_DRONES_CONFIG", raising=False)
    monkeypatch.delenv("MIDDLEWARE_SCENARIOS_DIR", raising=False)

    service = build_service_from_env()
    service.registry.close_all()

    started = json.loads(log_path.read_text().splitlines()[0])
    assert started["event"] == "server_started"
    assert started["waypoints"] == {
        "generator": "coverage",
        "side_overlap": 0.2,
        "max_leg_m": 100.0,
        "swath_width_m": {"drone-01": 10.0, "drone-02": 10.0},  # FakeDrone temporary values
    }
    assert service.waypoint_generator == CoverageGenerator()
