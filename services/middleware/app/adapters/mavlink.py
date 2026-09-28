"""Map MAVLink adapter data to protocol-neutral middleware contracts.

This module deliberately does not open a MAVLink connection. The simulation service owns
the concrete ``MavlinkDrone`` client; middleware code consumes its telemetry dictionary and
can invoke that client through the small protocol defined here.
"""

from collections.abc import Mapping
from datetime import datetime
from enum import IntEnum
from math import isfinite
from typing import Any, Protocol, Self

from pydantic import Field, model_validator

from app.schemas import (
    AltitudeReference,
    CommandError,
    CommandResult,
    CommandStatus,
    DroneCapabilities,
    DroneCommand,
    DroneCommandType,
    DroneState,
    DroneStatus,
    ProtocolType,
    TakeoffPayload,
    TelemetryField,
)
from app.schemas.mission import ContractModel, Identifier

from .telemetry import TelemetryStateMapper

MAV_CMD_NAV_TAKEOFF = 22

MAVLINK_CAPABILITIES = DroneCapabilities(
    commands=[
        DroneCommandType.TAKEOFF,
        DroneCommandType.GOTO,
        DroneCommandType.LAND,
        DroneCommandType.DISARM,
    ],
    telemetry_fields=list(TelemetryField),
)


class MavlinkDroneBinding(ContractModel):
    """Stable association between a middleware drone ID and MAVLink IDs."""

    drone_id: Identifier
    system_id: int = Field(ge=1, le=255)
    component_id: int = Field(ge=0, le=255)


class MavlinkStateMapper:
    """Convert the simulation adapter telemetry dictionary into ``DroneState``.

    MAVLink-specific checks (SYSID binding, Heartbeat age) happen here; the field mapping
    itself is shared with other protocols through ``TelemetryStateMapper``.
    """

    def __init__(self, *, heartbeat_timeout_s: float = 3.0) -> None:
        if not isfinite(heartbeat_timeout_s) or heartbeat_timeout_s <= 0:
            raise ValueError("heartbeat_timeout_s must be a positive finite number")
        self.heartbeat_timeout_s = heartbeat_timeout_s
        self._telemetry_mapper = TelemetryStateMapper(
            telemetry_timeout_s=heartbeat_timeout_s
        )

    def map(
        self,
        telemetry: Mapping[str, Any],
        *,
        binding: MavlinkDroneBinding,
        source_system_id: int,
        status: DroneStatus,
        heartbeat_age_s: float | None,
    ) -> DroneState:
        """Normalize one MAVLink sample without opening or reading a connection."""
        if (
            isinstance(source_system_id, bool)
            or not isinstance(source_system_id, int)
            or not 1 <= source_system_id <= 255
        ):
            raise ValueError("source_system_id must be an integer from 1 to 255")
        if source_system_id != binding.system_id:
            raise ValueError(
                f"MAVLink SYSID {source_system_id} is not bound to "
                f"{binding.drone_id} (expected {binding.system_id})"
            )

        return self._telemetry_mapper.map(
            telemetry,
            drone_id=binding.drone_id,
            protocol=ProtocolType.MAVLINK,
            capabilities=MAVLINK_CAPABILITIES,
            status=status,
            telemetry_age_s=heartbeat_age_s,
        )


class MavlinkTakeoffClient(Protocol):
    """Subset of ``services.simulation.app.mavlink_drone.MavlinkDrone`` used here."""

    def takeoff(self, altitude_m: float) -> None: ...


class MavlinkTakeoffRequest(ContractModel):
    """Translated MAVLink takeoff request for the existing simulation client."""

    command_id: Identifier
    target_system: int = Field(ge=1, le=255)
    target_component: int = Field(ge=0, le=255)
    mav_command: int = MAV_CMD_NAV_TAKEOFF
    altitude_m: float = Field(gt=0, le=500)

    @model_validator(mode="after")
    def command_must_be_nav_takeoff(self) -> Self:
        if self.mav_command != MAV_CMD_NAV_TAKEOFF:
            raise ValueError(f"mav_command must be {MAV_CMD_NAV_TAKEOFF}")
        return self

    def invoke(self, client: MavlinkTakeoffClient) -> None:
        """Reuse the injected simulation adapter instead of creating a MAVLink client."""
        client.takeoff(self.altitude_m)


class UnsupportedMavlinkCommandError(ValueError):
    """Raised when the current MAVLink adapter cannot represent a command."""


def to_mavlink_takeoff_request(
    command: DroneCommand, binding: MavlinkDroneBinding
) -> MavlinkTakeoffRequest:
    """Translate one common takeoff command into the MAVLink adapter call contract."""
    if command.protocol != ProtocolType.MAVLINK:
        raise UnsupportedMavlinkCommandError("command protocol must be mavlink")
    if command.drone_id != binding.drone_id:
        raise ValueError(
            f"command targets {command.drone_id}, but binding belongs to {binding.drone_id}"
        )
    if not isinstance(command.payload, TakeoffPayload):
        raise UnsupportedMavlinkCommandError(
            "only the takeoff control request is implemented in this prototype"
        )
    if command.payload.altitude_reference != AltitudeReference.HOME_RELATIVE:
        raise UnsupportedMavlinkCommandError(
            "the current MavlinkDrone.takeoff() accepts home-relative altitude only"
        )
    return MavlinkTakeoffRequest(
        command_id=command.command_id,
        target_system=binding.system_id,
        target_component=binding.component_id,
        altitude_m=command.payload.altitude_m,
    )


class MavlinkAckResult(IntEnum):
    """MAV_RESULT values carried by ``COMMAND_ACK``."""

    ACCEPTED = 0
    TEMPORARILY_REJECTED = 1
    DENIED = 2
    UNSUPPORTED = 3
    FAILED = 4
    IN_PROGRESS = 5
    CANCELLED = 6


class MavlinkCommandLifecycle:
    """Convert MAVLink send, ACK, and completion observations to command results."""

    def __init__(self, command: DroneCommand) -> None:
        if command.protocol != ProtocolType.MAVLINK:
            raise ValueError("MavlinkCommandLifecycle requires a mavlink command")
        self.command = command
        self.status = CommandStatus.PENDING

    def mark_sent(self, *, at: datetime) -> CommandResult:
        return self._transition(CommandStatus.SENT, at=at)

    def handle_ack(self, result: int, *, at: datetime) -> CommandResult:
        """Map one COMMAND_ACK without treating acceptance as completion."""
        try:
            ack = MavlinkAckResult(result)
        except ValueError:
            return self._transition(
                CommandStatus.FAILED,
                at=at,
                error=CommandError(
                    code="MAVLINK_UNKNOWN_ACK",
                    message=f"Unknown MAVLink COMMAND_ACK result: {result}",
                ),
            )

        if ack == MavlinkAckResult.ACCEPTED:
            return self._transition(CommandStatus.ACCEPTED, at=at)
        if ack == MavlinkAckResult.IN_PROGRESS:
            return self._transition(CommandStatus.EXECUTING, at=at)
        if ack == MavlinkAckResult.UNSUPPORTED:
            return self._transition(
                CommandStatus.UNSUPPORTED,
                at=at,
                error=CommandError(
                    code="MAVLINK_COMMAND_UNSUPPORTED",
                    message="Vehicle reports that the MAVLink command is unsupported",
                ),
            )

        retryable = ack == MavlinkAckResult.TEMPORARILY_REJECTED
        return self._transition(
            CommandStatus.FAILED,
            at=at,
            error=CommandError(
                code=f"MAVLINK_ACK_{ack.name}",
                message=f"MAVLink command failed with ACK result {ack.name}",
                retryable=retryable,
            ),
        )

    def mark_executing(self, *, at: datetime) -> CommandResult:
        """Record telemetry evidence that the vehicle started the requested action."""
        return self._transition(CommandStatus.EXECUTING, at=at)

    def mark_completed(self, *, at: datetime) -> CommandResult:
        """Record telemetry evidence that the requested target was reached."""
        return self._transition(CommandStatus.SUCCEEDED, at=at)

    def mark_ack_timeout(self, *, at: datetime) -> CommandResult:
        return self._transition(
            CommandStatus.TIMED_OUT,
            at=at,
            error=CommandError(
                code="MAVLINK_ACK_TIMEOUT",
                message="No COMMAND_ACK was received before the deadline",
                retryable=True,
            ),
        )

    def mark_completion_timeout(self, *, at: datetime) -> CommandResult:
        return self._transition(
            CommandStatus.TIMED_OUT,
            at=at,
            error=CommandError(
                code="MAVLINK_COMPLETION_TIMEOUT",
                message="Telemetry did not confirm completion before the deadline",
                retryable=True,
            ),
        )

    def _transition(
        self,
        status: CommandStatus,
        *,
        at: datetime,
        error: CommandError | None = None,
    ) -> CommandResult:
        result = CommandResult(
            command_id=self.command.command_id,
            drone_id=self.command.drone_id,
            protocol=ProtocolType.MAVLINK,
            previous_status=self.status,
            status=status,
            updated_at=at,
            error=error,
        )
        self.status = status
        return result
