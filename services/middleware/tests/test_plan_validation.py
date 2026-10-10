"""Tests for cross-validating Mission Plans against the Mission Context."""

from datetime import datetime, timezone

from conftest import plan, scenario

from app.adapters import AP_DDS_CAPABILITIES
from app.runtime import build_mission_task, validate_plan
from app.schemas import (
    DroneCapabilities,
    DroneCommandType,
    DroneStatus,
    MissionContext,
    ReturnHomePayload,
    TelemetryField,
)

NOW = datetime(2026, 9, 28, 3, 0, tzinfo=timezone.utc)


def context(registry, **scenario_overrides) -> MissionContext:
    current = scenario(**scenario_overrides)
    return MissionContext(
        mission_id=current.mission_id,
        instruction=current.instruction,
        drones=registry.states(),
        search_areas=current.search_areas,
        requested_at=NOW,
    )


def codes(issues):
    return [issue.code for issue in issues]


def test_accepts_plan_that_matches_context(registry) -> None:
    accepted, issues = validate_plan(
        plan(("drone-01", "area-north"), ("drone-02", "area-south")), context(registry)
    )

    assert issues == []
    assert [a.drone_id for a in accepted.assignments] == ["drone-01", "drone-02"]


def test_rejects_unknown_drone_and_area(registry) -> None:
    accepted, issues = validate_plan(plan(("drone-9", "area-west")), context(registry))

    assert accepted is None
    assert codes(issues) == ["UNKNOWN_DRONE", "UNKNOWN_AREA"]
    assert issues[0].location == "assignments.0.drone_id"
    assert "drone-01" in issues[0].message


def test_rejects_mission_id_mismatch(registry) -> None:
    _, issues = validate_plan(
        plan(("drone-01", "area-north"), mission_id="mission-999"), context(registry)
    )
    assert codes(issues) == ["MISSION_ID_MISMATCH"]


def test_contract_errors_are_reported_as_schema_issues(registry) -> None:
    _, issues = validate_plan(
        plan(("drone-01", "area-north"), ("drone-01", "area-south")), context(registry)
    )
    assert codes(issues) == ["SCHEMA_VALIDATION_ERROR"]
    assert "unique drone_id" in issues[0].message


def test_rejects_drone_that_is_not_available(registry) -> None:
    registry.set_status("drone-01", DroneStatus.ASSIGNED)

    _, issues = validate_plan(plan(("drone-01", "area-north")), context(registry))
    assert codes(issues) == ["DRONE_NOT_AVAILABLE"]


def test_rejects_disconnected_drone(registry) -> None:
    registry.get("drone-02").client.link_up = False
    registry._refresh_timeout_s = 0.01
    handle = registry.get("drone-02")
    handle._latest_at -= 10  # last sample is older than the timeout

    _, issues = validate_plan(plan(("drone-02", "area-south")), context(registry))
    assert codes(issues) == ["DRONE_DISCONNECTED"]


def test_rejects_drone_missing_required_commands(registry) -> None:
    registry.get("drone-02").capabilities = DroneCapabilities(
        commands=[DroneCommandType.TAKEOFF, DroneCommandType.LAND],
        telemetry_fields=AP_DDS_CAPABILITIES.telemetry_fields,
    )

    _, issues = validate_plan(plan(("drone-02", "area-south")), context(registry))
    assert codes(issues) == ["COMMAND_NOT_SUPPORTED"]
    assert "goto" in issues[0].message


def test_rejects_area_below_safe_altitude_or_with_msl_reference(registry) -> None:
    areas = [area.model_dump() for area in scenario().search_areas]
    areas[0]["search_altitude_m"] = 20
    areas[1]["search_altitude_reference"] = "msl"

    _, issues = validate_plan(
        plan(("drone-01", "area-north"), ("drone-02", "area-south")),
        context(registry, search_areas=areas),
    )
    assert codes(issues) == ["ALTITUDE_BELOW_MINIMUM", "UNSUPPORTED_ALTITUDE_REFERENCE"]


def test_builds_ordered_task_with_temporary_goto_home(registry) -> None:
    drone = registry.state("drone-01")
    area = scenario().search_areas[0]

    task = build_mission_task(
        task_id="task-001", mission_id="mission-001", drone=drone, area=area, created_at=NOW
    )

    types = [command.payload.command_type.value for command in task.commands]
    # takeoff, 3 vertices + closing vertex, goto(home) stand-in, land
    assert types == ["takeoff", "goto", "goto", "goto", "goto", "goto", "land"]
    home_leg = task.commands[-2].payload.target
    assert (home_leg.latitude, home_leg.longitude) == (
        drone.position.latitude,
        drone.position.longitude,
    )
    assert all(
        command.payload.target.altitude_m == 30
        for command in task.commands
        if command.payload.command_type == DroneCommandType.GOTO
    )


def test_uses_return_home_when_drone_supports_it(registry) -> None:
    drone = registry.state("drone-01")
    drone = drone.model_copy(
        update={
            "capabilities": DroneCapabilities(
                commands=[*drone.capabilities.commands, DroneCommandType.RETURN_HOME],
                telemetry_fields=list(TelemetryField),
            )
        }
    )

    task = build_mission_task(
        task_id="task-001",
        mission_id="mission-001",
        drone=drone,
        area=scenario().search_areas[0],
        created_at=NOW,
    )
    assert isinstance(task.commands[-2].payload, ReturnHomePayload)
