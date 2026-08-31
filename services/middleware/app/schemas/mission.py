"""Core contracts exchanged by the orchestrator, middleware, and simulator."""

from datetime import datetime
from enum import Enum
from typing import Annotated, Self

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    StringConstraints,
    field_validator,
    model_validator,
)

Identifier = Annotated[
    str,
    StringConstraints(
        strip_whitespace=True,
        min_length=1,
        max_length=64,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9_-]*$",
    ),
]


class ContractModel(BaseModel):
    """Base model that rejects fields outside the agreed contract."""

    model_config = ConfigDict(extra="forbid")


class GeoCoordinate(ContractModel):
    """A WGS84 latitude and longitude pair in decimal degrees."""

    latitude: float = Field(ge=-90, le=90)
    longitude: float = Field(ge=-180, le=180)


class GeoPosition(GeoCoordinate):
    """A drone position with altitude above mean sea level."""

    altitude_m: float = Field(ge=-500, le=10_000)


class DroneStatus(str, Enum):
    """Operational state used by mission allocation and fallback logic."""

    AVAILABLE = "available"
    ASSIGNED = "assigned"
    RETURNING = "returning"
    UNAVAILABLE = "unavailable"


class ConnectionStatus(str, Enum):
    """Communication state tracked independently from drone operation."""

    CONNECTED = "connected"
    DISCONNECTED = "disconnected"


class DroneState(ContractModel):
    """Normalized state reported for one drone."""

    drone_id: Identifier
    position: GeoPosition
    battery_percent: float = Field(ge=0, le=100)
    status: DroneStatus
    connection_status: ConnectionStatus
    observed_at: datetime

    @field_validator("observed_at")
    @classmethod
    def observed_at_must_include_timezone(cls, value: datetime) -> datetime:
        """Reject ambiguous timestamps at the service boundary."""
        if value.utcoffset() is None:
            raise ValueError("observed_at must include a timezone offset")
        return value


class SearchArea(ContractModel):
    """Polygonal search area assigned as one indivisible unit."""

    area_id: Identifier
    boundary: list[GeoCoordinate] = Field(min_length=3, max_length=100)
    search_altitude_m: float = Field(gt=0, le=500)

    @field_validator("boundary")
    @classmethod
    def boundary_must_have_three_distinct_points(
        cls, value: list[GeoCoordinate]
    ) -> list[GeoCoordinate]:
        """A polygon requires at least three distinct vertices."""
        points = {(point.latitude, point.longitude) for point in value}
        if len(points) < 3:
            raise ValueError("boundary must contain at least three distinct points")
        return value


class MissionContext(ContractModel):
    """Validated input supplied to the LLM orchestrator."""

    mission_id: Identifier
    instruction: str = Field(min_length=1, max_length=2_000)
    drones: list[DroneState] = Field(min_length=1)
    search_areas: list[SearchArea] = Field(min_length=1)
    requested_at: datetime

    @field_validator("instruction")
    @classmethod
    def instruction_must_not_be_blank(cls, value: str) -> str:
        """Trim natural-language input and reject whitespace-only commands."""
        normalized = value.strip()
        if not normalized:
            raise ValueError("instruction must not be blank")
        return normalized

    @field_validator("requested_at")
    @classmethod
    def requested_at_must_include_timezone(cls, value: datetime) -> datetime:
        """Require an unambiguous mission request time."""
        if value.utcoffset() is None:
            raise ValueError("requested_at must include a timezone offset")
        return value

    @model_validator(mode="after")
    def identifiers_must_be_unique(self) -> Self:
        """Reject state snapshots whose members cannot be addressed uniquely."""
        _require_unique(
            [drone.drone_id for drone in self.drones],
            "drones must have unique drone_id values",
        )
        _require_unique(
            [area.area_id for area in self.search_areas],
            "search_areas must have unique area_id values",
        )
        return self


class MissionAssignment(ContractModel):
    """High-level allocation of one drone to one search area."""

    drone_id: Identifier
    area_id: Identifier
    priority: int = Field(default=1, ge=1, le=100)


class MissionPlan(ContractModel):
    """Structured orchestrator output accepted by the middleware."""

    mission_id: Identifier
    assignments: list[MissionAssignment] = Field(min_length=1)
    generated_at: datetime

    @field_validator("generated_at")
    @classmethod
    def generated_at_must_include_timezone(cls, value: datetime) -> datetime:
        """Require an unambiguous plan generation time."""
        if value.utcoffset() is None:
            raise ValueError("generated_at must include a timezone offset")
        return value

    @model_validator(mode="after")
    def assignments_must_be_unique(self) -> Self:
        """Prevent one plan from allocating a drone or area more than once."""
        _require_unique(
            [assignment.drone_id for assignment in self.assignments],
            "assignments must have unique drone_id values",
        )
        _require_unique(
            [assignment.area_id for assignment in self.assignments],
            "assignments must have unique area_id values",
        )
        return self


def _require_unique(values: list[str], message: str) -> None:
    """Raise a validation error when a contract identifier is duplicated."""
    if len(values) != len(set(values)):
        raise ValueError(message)
