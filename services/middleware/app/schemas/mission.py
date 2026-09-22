"""Core contracts exchanged by the orchestrator, middleware, and simulator."""

from collections.abc import Hashable, Sequence
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


class AltitudeReference(str, Enum):
    """Reference datum used to interpret an altitude value."""

    MSL = "msl"
    HOME_RELATIVE = "home_relative"


class GeoPosition(GeoCoordinate):
    """A drone position whose altitude datum is explicit."""

    altitude_m: float = Field(ge=-500, le=10_000)
    altitude_reference: AltitudeReference


class ProtocolType(str, Enum):
    """Supported vehicle communication protocols."""

    MAVLINK = "mavlink"
    AP_DDS = "ap_dds"


class DroneCommandType(str, Enum):
    """Protocol-neutral commands understood by the middleware."""

    TAKEOFF = "takeoff"
    GOTO = "goto"
    LAND = "land"
    RETURN_HOME = "return_home"
    DISARM = "disarm"


class TelemetryField(str, Enum):
    """Optional telemetry groups a protocol may provide."""

    POSITION = "position"
    VELOCITY_NED = "velocity_ned"
    HEADING = "heading"
    GPS_FIX_TYPE = "gps_fix_type"
    SATELLITES_VISIBLE = "satellites_visible"
    GPS_EPH = "gps_eph"
    BATTERY_VOLTAGE = "battery_voltage"
    BATTERY_PERCENT = "battery_percent"
    VIBRATION = "vibration"
    CLIPPING = "clipping"


class TelemetryAvailability(str, Enum):
    """Why a normalized telemetry value is present or absent."""

    AVAILABLE = "available"
    TEMPORARILY_UNAVAILABLE = "temporarily_unavailable"
    UNSUPPORTED = "unsupported"


class DroneCapabilities(ContractModel):
    """Commands and telemetry groups supported by one drone adapter."""

    commands: list[DroneCommandType] = Field(min_length=1)
    telemetry_fields: list[TelemetryField] = Field(min_length=1)

    @model_validator(mode="after")
    def capabilities_must_be_unique(self) -> Self:
        """Reject duplicated capability entries at the service boundary."""
        _require_unique(self.commands, "commands must contain unique values")
        _require_unique(
            self.telemetry_fields,
            "telemetry_fields must contain unique values",
        )
        return self


class NedVelocity(ContractModel):
    """Velocity in the North-East-Down frame, measured in metres per second."""

    north_m_s: float
    east_m_s: float
    down_m_s: float


class Vibration(ContractModel):
    """Protocol-reported vibration values on the vehicle axes."""

    x: float
    y: float
    z: float


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
    protocol: ProtocolType
    capabilities: DroneCapabilities
    position: GeoPosition | None = None
    velocity_ned_m_s: NedVelocity | None = None
    heading_deg: float | None = Field(default=None, ge=0, lt=360)
    gps_fix_type: int | None = Field(default=None, ge=0, le=8)
    satellites_visible: int | None = Field(default=None, ge=0)
    gps_eph_m: float | None = Field(default=None, ge=0)
    battery_voltage_v: float | None = Field(default=None, ge=0)
    battery_percent: float | None = Field(default=None, ge=0, le=100)
    vibration: Vibration | None = None
    clipping_count: int | None = Field(default=None, ge=0)
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

    @model_validator(mode="after")
    def populated_telemetry_must_be_supported(self) -> Self:
        """Do not accept values that the adapter says it cannot provide."""
        supported = set(self.capabilities.telemetry_fields)
        for telemetry_field, attribute in _TELEMETRY_ATTRIBUTES.items():
            if getattr(self, attribute) is not None and telemetry_field not in supported:
                raise ValueError(
                    f"{attribute} is populated but {telemetry_field.value} is not "
                    "declared in capabilities"
                )
        return self

    def telemetry_availability(
        self, telemetry_field: TelemetryField
    ) -> TelemetryAvailability:
        """Distinguish unsupported telemetry from a temporary missing sample."""
        if telemetry_field not in self.capabilities.telemetry_fields:
            return TelemetryAvailability.UNSUPPORTED
        attribute = _TELEMETRY_ATTRIBUTES[telemetry_field]
        if getattr(self, attribute) is None:
            return TelemetryAvailability.TEMPORARILY_UNAVAILABLE
        return TelemetryAvailability.AVAILABLE


class SearchArea(ContractModel):
    """Polygonal search area assigned as one indivisible unit."""

    area_id: Identifier
    boundary: list[GeoCoordinate] = Field(min_length=3, max_length=100)
    search_altitude_m: float = Field(gt=0, le=500)
    search_altitude_reference: AltitudeReference

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
    priority: int = Field(
        default=1,
        ge=1,
        le=100,
        description="Execution priority from 1 (lowest) to 100 (highest).",
    )


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


def _require_unique(values: Sequence[Hashable], message: str) -> None:
    """Raise a validation error when a contract identifier is duplicated."""
    if len(values) != len(set(values)):
        raise ValueError(message)


_TELEMETRY_ATTRIBUTES: dict[TelemetryField, str] = {
    TelemetryField.POSITION: "position",
    TelemetryField.VELOCITY_NED: "velocity_ned_m_s",
    TelemetryField.HEADING: "heading_deg",
    TelemetryField.GPS_FIX_TYPE: "gps_fix_type",
    TelemetryField.SATELLITES_VISIBLE: "satellites_visible",
    TelemetryField.GPS_EPH: "gps_eph_m",
    TelemetryField.BATTERY_VOLTAGE: "battery_voltage_v",
    TelemetryField.BATTERY_PERCENT: "battery_percent",
    TelemetryField.VIBRATION: "vibration",
    TelemetryField.CLIPPING: "clipping_count",
}
