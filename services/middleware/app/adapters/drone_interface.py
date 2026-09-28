"""Translate common ``DroneCommand`` objects into ``DroneInterface`` method calls.

Both simulation adapters (``MavlinkDrone`` and ``Ros2Drone``) implement the same
``DroneInterface`` contract, so the MVP pipeline controls either protocol through one
translation. Protocol-specific request models (``to_mavlink_takeoff_request`` and
``to_ap_dds_takeoff_request``) stay available for lower-level observation hooks.
"""

from enum import Enum
from typing import Any, Protocol, Self

from pydantic import Field, model_validator

from app.schemas import (
    AltitudeReference,
    DroneCapabilities,
    DroneCommand,
    DroneCommandType,
    GotoPayload,
    ProtocolType,
    TakeoffPayload,
)
from app.schemas.mission import ContractModel, Identifier

# Flights below this home-relative altitude hit obstacles in the simulation. The
# simulation's run_mission() enforces the same floor; the middleware rejects earlier so
# an invalid plan never reaches a vehicle.
MIN_SAFE_ALTITUDE_M = 30.0


class DroneInterfaceClient(Protocol):
    """Subset of ``services/simulation/app/drone_interface.DroneInterface`` used here."""

    def takeoff(self, altitude_m: float) -> None: ...

    def goto(self, lat: float, lon: float, alt_m: float) -> None: ...

    def land(self, timeout: float = 60) -> bool: ...

    def disarm(self, timeout: float = 5) -> bool: ...

    def get_telemetry(self, timeout: float = 2) -> dict[str, Any] | None: ...


class InterfaceMethod(str, Enum):
    """``DroneInterface`` methods reachable from a common command."""

    TAKEOFF = "takeoff"
    GOTO = "goto"
    LAND = "land"
    DISARM = "disarm"
    RETURN_HOME = "return_home"


class UnsupportedDroneCommandError(ValueError):
    """Raised when the target drone cannot execute the requested command."""


class UnsafeAltitudeError(ValueError):
    """Raised when a command would fly below the minimum safe altitude."""


class DroneInterfaceRequest(ContractModel):
    """One validated ``DroneInterface`` call derived from a ``DroneCommand``."""

    command_id: Identifier
    drone_id: Identifier
    protocol: ProtocolType
    method: InterfaceMethod
    latitude: float | None = Field(default=None, ge=-90, le=90)
    longitude: float | None = Field(default=None, ge=-180, le=180)
    altitude_home_relative_m: float | None = Field(
        default=None, ge=MIN_SAFE_ALTITUDE_M, le=500
    )

    @model_validator(mode="after")
    def arguments_must_match_method(self) -> Self:
        needs_altitude = self.method in (InterfaceMethod.TAKEOFF, InterfaceMethod.GOTO)
        needs_position = self.method == InterfaceMethod.GOTO
        if needs_altitude != (self.altitude_home_relative_m is not None):
            requirement = "required" if needs_altitude else "not allowed"
            raise ValueError(f"altitude is {requirement} for {self.method.value}")
        has_position = self.latitude is not None or self.longitude is not None
        if needs_position and (self.latitude is None or self.longitude is None):
            raise ValueError("goto requires latitude and longitude")
        if not needs_position and has_position:
            raise ValueError(f"position is not allowed for {self.method.value}")
        return self

    def invoke(self, client: DroneInterfaceClient) -> Any:
        """Call the injected simulation adapter; its return value is passed through."""
        if self.method == InterfaceMethod.TAKEOFF:
            return client.takeoff(self.altitude_home_relative_m)
        if self.method == InterfaceMethod.GOTO:
            return client.goto(self.latitude, self.longitude, self.altitude_home_relative_m)
        if self.method == InterfaceMethod.LAND:
            return client.land()
        if self.method == InterfaceMethod.DISARM:
            return client.disarm()
        return client.return_home()  # type: ignore[attr-defined]


def to_drone_interface_request(
    command: DroneCommand,
    *,
    drone_id: str,
    protocol: ProtocolType,
    capabilities: DroneCapabilities,
) -> DroneInterfaceRequest:
    """Validate one command against its registered target and translate it."""
    if command.drone_id != drone_id:
        raise ValueError(f"command targets {command.drone_id}, but the client is {drone_id}")
    if command.protocol != protocol:
        raise UnsupportedDroneCommandError(
            f"command protocol {command.protocol.value} does not match "
            f"{drone_id} ({protocol.value})"
        )
    command_type = command.payload.command_type
    if command_type not in capabilities.commands:
        raise UnsupportedDroneCommandError(
            f"{drone_id} does not support {command_type.value}"
        )

    common = {
        "command_id": command.command_id,
        "drone_id": drone_id,
        "protocol": protocol,
        "method": InterfaceMethod(command_type.value),
    }
    payload = command.payload
    if isinstance(payload, TakeoffPayload):
        _require_safe_home_relative(payload.altitude_m, payload.altitude_reference)
        return DroneInterfaceRequest(**common, altitude_home_relative_m=payload.altitude_m)
    if isinstance(payload, GotoPayload):
        target = payload.target
        _require_safe_home_relative(target.altitude_m, target.altitude_reference)
        return DroneInterfaceRequest(
            **common,
            latitude=target.latitude,
            longitude=target.longitude,
            altitude_home_relative_m=target.altitude_m,
        )
    if command_type in (
        DroneCommandType.LAND,
        DroneCommandType.DISARM,
        DroneCommandType.RETURN_HOME,
    ):
        return DroneInterfaceRequest(**common)
    raise UnsupportedDroneCommandError(f"unknown command type {command_type.value}")


def _require_safe_home_relative(
    altitude_m: float, altitude_reference: AltitudeReference
) -> None:
    if altitude_reference != AltitudeReference.HOME_RELATIVE:
        raise UnsupportedDroneCommandError(
            "DroneInterface accepts home-relative altitude only"
        )
    if altitude_m < MIN_SAFE_ALTITUDE_M:
        raise UnsafeAltitudeError(
            f"altitude {altitude_m}m is below the minimum safe altitude "
            f"{MIN_SAFE_ALTITUDE_M}m"
        )
