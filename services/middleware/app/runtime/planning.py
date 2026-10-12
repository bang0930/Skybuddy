"""Validate submitted Mission Plans and turn accepted assignments into Mission Tasks."""

from collections.abc import Mapping
from datetime import datetime
from typing import Any

from pydantic import ValidationError

from app.adapters import MIN_SAFE_ALTITUDE_M
from app.schemas import (
    AltitudeReference,
    ConnectionStatus,
    DroneCommand,
    DroneCommandType,
    DroneState,
    DroneStatus,
    GeoCoordinate,
    GeoPosition,
    GotoPayload,
    LandPayload,
    MissionContext,
    MissionPlan,
    MissionTask,
    ReturnHomePayload,
    SearchArea,
    TakeoffPayload,
    ValidationIssue,
)

# Commands every assigned drone must support to fly takeoff -> area -> home -> land.
REQUIRED_COMMANDS = (
    DroneCommandType.TAKEOFF,
    DroneCommandType.GOTO,
    DroneCommandType.LAND,
)


def validate_plan(
    raw_plan: Mapping[str, Any], context: MissionContext
) -> tuple[MissionPlan | None, list[ValidationIssue]]:
    """Check the plan contract, then cross-check it against the current context."""
    try:
        plan = MissionPlan.model_validate(raw_plan)
    except ValidationError as exc:
        return None, [_issue_from_pydantic(error) for error in exc.errors()]

    issues: list[ValidationIssue] = []
    if plan.mission_id != context.mission_id:
        issues.append(
            ValidationIssue(
                code="MISSION_ID_MISMATCH",
                message=f"plan targets {plan.mission_id}, expected {context.mission_id}",
                location="mission_id",
            )
        )

    drones = {drone.drone_id: drone for drone in context.drones}
    areas = {area.area_id: area for area in context.search_areas}
    for index, assignment in enumerate(plan.assignments):
        location = f"assignments.{index}"
        drone = drones.get(assignment.drone_id)
        if drone is None:
            issues.append(
                ValidationIssue(
                    code="UNKNOWN_DRONE",
                    message=(
                        f"drone_id {assignment.drone_id} is not in the mission context; "
                        f"use one of {sorted(drones)}"
                    ),
                    location=f"{location}.drone_id",
                )
            )
        else:
            issues.extend(_drone_issues(drone, location))

        area = areas.get(assignment.area_id)
        if area is None:
            issues.append(
                ValidationIssue(
                    code="UNKNOWN_AREA",
                    message=(
                        f"area_id {assignment.area_id} is not in the mission context; "
                        f"use one of {sorted(areas)}"
                    ),
                    location=f"{location}.area_id",
                )
            )
        else:
            issues.extend(_area_issues(area, location))

    return (plan if not issues else None), issues


def area_waypoints(area: SearchArea) -> list[GeoCoordinate]:
    """TEMPORARY: fly the polygon perimeter and close the loop at the first vertex.

    Replace with the simulation part's boundary -> waypoint function (coverage pattern)
    once it exists. This stand-in only visits the boundary vertices in order.
    """
    return [*area.boundary, area.boundary[0]]


def build_mission_task(
    *,
    task_id: str,
    mission_id: str,
    drone: DroneState,
    area: SearchArea,
    created_at: datetime,
) -> MissionTask:
    """Build takeoff -> goto x N -> return -> land for one drone and one area."""
    if drone.position is None:
        raise ValueError(f"{drone.drone_id} has no known home position")
    altitude = area.search_altitude_m
    home = drone.position

    payloads: list[Any] = [
        TakeoffPayload(
            command_type=DroneCommandType.TAKEOFF,
            altitude_m=altitude,
            altitude_reference=AltitudeReference.HOME_RELATIVE,
        )
    ]
    for waypoint in area_waypoints(area):
        payloads.append(_goto(waypoint.latitude, waypoint.longitude, altitude))
    if DroneCommandType.RETURN_HOME in drone.capabilities.commands:
        payloads.append(ReturnHomePayload(command_type=DroneCommandType.RETURN_HOME))
    else:
        # TEMPORARY: DroneInterface has no return_home() yet, so return via goto(home)
        # at cruise altitude. Switches automatically once the client provides it.
        payloads.append(_goto(home.latitude, home.longitude, altitude))
    payloads.append(LandPayload(command_type=DroneCommandType.LAND))

    commands = [
        DroneCommand(
            command_id=f"{task_id}-c{index:02d}",
            mission_id=mission_id,
            task_id=task_id,
            drone_id=drone.drone_id,
            protocol=drone.protocol,
            payload=payload,
            created_at=created_at,
        )
        for index, payload in enumerate(payloads, 1)
    ]
    return MissionTask(
        task_id=task_id,
        mission_id=mission_id,
        drone_id=drone.drone_id,
        commands=commands,
        created_at=created_at,
    )


def _goto(latitude: float, longitude: float, altitude: float) -> GotoPayload:
    return GotoPayload(
        command_type=DroneCommandType.GOTO,
        target=GeoPosition(
            latitude=latitude,
            longitude=longitude,
            altitude_m=altitude,
            altitude_reference=AltitudeReference.HOME_RELATIVE,
        ),
    )


def _drone_issues(drone: DroneState, location: str) -> list[ValidationIssue]:
    issues = []
    if drone.status != DroneStatus.AVAILABLE:
        issues.append(
            ValidationIssue(
                code="DRONE_NOT_AVAILABLE",
                message=f"{drone.drone_id} is {drone.status.value}",
                location=f"{location}.drone_id",
            )
        )
    if drone.connection_status != ConnectionStatus.CONNECTED:
        issues.append(
            ValidationIssue(
                code="DRONE_DISCONNECTED",
                message=f"{drone.drone_id} has no recent telemetry",
                location=f"{location}.drone_id",
            )
        )
    elif drone.position is None:
        issues.append(
            ValidationIssue(
                code="DRONE_POSITION_UNKNOWN",
                message=f"{drone.drone_id} has no position to return to",
                location=f"{location}.drone_id",
            )
        )
    missing = [
        command.value
        for command in REQUIRED_COMMANDS
        if command not in drone.capabilities.commands
    ]
    if missing:
        issues.append(
            ValidationIssue(
                code="COMMAND_NOT_SUPPORTED",
                message=f"{drone.drone_id} does not support {', '.join(missing)}",
                location=f"{location}.drone_id",
            )
        )
    return issues


def _area_issues(area: SearchArea, location: str) -> list[ValidationIssue]:
    issues = []
    if area.search_altitude_reference != AltitudeReference.HOME_RELATIVE:
        issues.append(
            ValidationIssue(
                code="UNSUPPORTED_ALTITUDE_REFERENCE",
                message=f"{area.area_id} must use a home_relative search altitude",
                location=f"{location}.area_id",
            )
        )
    if area.search_altitude_m < MIN_SAFE_ALTITUDE_M:
        issues.append(
            ValidationIssue(
                code="ALTITUDE_BELOW_MINIMUM",
                message=(
                    f"{area.area_id} search altitude {area.search_altitude_m}m is below "
                    f"the minimum safe altitude {MIN_SAFE_ALTITUDE_M}m"
                ),
                location=f"{location}.area_id",
            )
        )
    return issues


def _issue_from_pydantic(error: Mapping[str, Any]) -> ValidationIssue:
    location = ".".join(str(part) for part in error.get("loc", ())) or None
    return ValidationIssue(
        code="SCHEMA_VALIDATION_ERROR",
        message=str(error.get("msg", "invalid value"))[:2_000],
        location=location,
    )
