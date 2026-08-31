"""Tests for middleware mission data contracts."""

from datetime import datetime, timezone

import pytest
from pydantic import ValidationError

from app.schemas import (
    ConnectionStatus,
    DroneState,
    DroneStatus,
    GeoCoordinate,
    GeoPosition,
    MissionAssignment,
    MissionContext,
    MissionPlan,
    SearchArea,
)

NOW = datetime(2026, 8, 31, 12, 0, tzinfo=timezone.utc)


def make_drone(
    drone_id: str = "drone-01",
    *,
    status: DroneStatus = DroneStatus.AVAILABLE,
    connection_status: ConnectionStatus = ConnectionStatus.CONNECTED,
) -> DroneState:
    """Build a valid drone state for focused validation tests."""
    return DroneState(
        drone_id=drone_id,
        position=GeoPosition(latitude=37.45, longitude=127.12, altitude_m=220),
        battery_percent=82.5,
        status=status,
        connection_status=connection_status,
        observed_at=NOW,
    )


def make_area(area_id: str = "area-a") -> SearchArea:
    """Build a valid triangular search area."""
    return SearchArea(
        area_id=area_id,
        boundary=[
            GeoCoordinate(latitude=37.45, longitude=127.12),
            GeoCoordinate(latitude=37.46, longitude=127.12),
            GeoCoordinate(latitude=37.45, longitude=127.13),
        ],
        search_altitude_m=80,
    )


def test_valid_mission_context_normalizes_instruction() -> None:
    context = MissionContext(
        mission_id="mission-001",
        instruction="  북쪽 능선을 우선 수색해 줘  ",
        drones=[make_drone()],
        search_areas=[make_area()],
        requested_at=NOW,
    )

    assert context.instruction == "북쪽 능선을 우선 수색해 줘"
    assert context.model_dump(mode="json")["drones"][0]["status"] == "available"


def test_disconnected_drone_preserves_last_operational_status() -> None:
    drone = make_drone(
        status=DroneStatus.ASSIGNED,
        connection_status=ConnectionStatus.DISCONNECTED,
    )

    serialized = drone.model_dump(mode="json")
    assert serialized["status"] == "assigned"
    assert serialized["connection_status"] == "disconnected"


@pytest.mark.parametrize("battery_percent", [-0.1, 100.1])
def test_drone_rejects_battery_outside_percentage_range(
    battery_percent: float,
) -> None:
    with pytest.raises(ValidationError, match="battery_percent"):
        DroneState(
            drone_id="drone-01",
            position=GeoPosition(latitude=37.45, longitude=127.12, altitude_m=220),
            battery_percent=battery_percent,
            status=DroneStatus.AVAILABLE,
            connection_status=ConnectionStatus.CONNECTED,
            observed_at=NOW,
        )


def test_drone_rejects_out_of_range_coordinate() -> None:
    with pytest.raises(ValidationError, match="latitude"):
        GeoPosition(latitude=91, longitude=127.12, altitude_m=220)


def test_contract_rejects_unknown_fields() -> None:
    with pytest.raises(ValidationError, match="unexpected"):
        DroneState.model_validate(
            {
                **make_drone().model_dump(),
                "unexpected": "value",
            }
        )


def test_timestamp_requires_timezone() -> None:
    with pytest.raises(ValidationError, match="timezone offset"):
        DroneState.model_validate(
            {
                **make_drone().model_dump(),
                "observed_at": datetime(2026, 8, 31),
            }
        )


def test_search_area_requires_three_distinct_points() -> None:
    point = GeoCoordinate(latitude=37.45, longitude=127.12)

    with pytest.raises(ValidationError, match="three distinct points"):
        SearchArea(
            area_id="area-a",
            boundary=[point, point, point],
            search_altitude_m=80,
        )


def test_context_rejects_duplicate_drone_ids() -> None:
    with pytest.raises(ValidationError, match="unique drone_id"):
        MissionContext(
            mission_id="mission-001",
            instruction="수색 시작",
            drones=[make_drone(), make_drone()],
            search_areas=[make_area()],
            requested_at=NOW,
        )


@pytest.mark.parametrize(
    ("assignments", "expected_message"),
    [
        (
            [
                MissionAssignment(drone_id="drone-01", area_id="area-a"),
                MissionAssignment(drone_id="drone-01", area_id="area-b"),
            ],
            "unique drone_id",
        ),
        (
            [
                MissionAssignment(drone_id="drone-01", area_id="area-a"),
                MissionAssignment(drone_id="drone-02", area_id="area-a"),
            ],
            "unique area_id",
        ),
    ],
)
def test_plan_rejects_duplicate_assignments(
    assignments: list[MissionAssignment], expected_message: str
) -> None:
    with pytest.raises(ValidationError, match=expected_message):
        MissionPlan(
            mission_id="mission-001",
            assignments=assignments,
            generated_at=NOW,
        )


def test_mission_plan_exposes_json_schema() -> None:
    schema = MissionPlan.model_json_schema()

    assert schema["title"] == "MissionPlan"
    assert schema["additionalProperties"] is False
    assert set(schema["required"]) == {"mission_id", "assignments", "generated_at"}


def test_drone_state_json_schema_requires_connection_status() -> None:
    schema = DroneState.model_json_schema()

    assert "connection_status" in schema["required"]
    assert schema["$defs"]["ConnectionStatus"]["enum"] == [
        "connected",
        "disconnected",
    ]
