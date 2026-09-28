"""Protocol-neutral command and execution-result contracts."""

from datetime import datetime
from enum import Enum
from typing import Annotated, Literal, Self

from pydantic import Field, field_validator, model_validator

from .mission import (
    AltitudeReference,
    ContractModel,
    DroneCommandType,
    GeoPosition,
    Identifier,
    ProtocolType,
    _require_unique,
)


class TakeoffPayload(ContractModel):
    """Parameters for a takeoff request."""

    command_type: Literal[DroneCommandType.TAKEOFF]
    altitude_m: float = Field(gt=0, le=500)
    altitude_reference: AltitudeReference


class GotoPayload(ContractModel):
    """Parameters for a global-position request."""

    command_type: Literal[DroneCommandType.GOTO]
    target: GeoPosition


class LandPayload(ContractModel):
    """Parameters for a land request."""

    command_type: Literal[DroneCommandType.LAND]


class ReturnHomePayload(ContractModel):
    """Parameters for a return-to-home request."""

    command_type: Literal[DroneCommandType.RETURN_HOME]


class DisarmPayload(ContractModel):
    """Parameters for an emergency disarm request."""

    command_type: Literal[DroneCommandType.DISARM]


CommandPayload = Annotated[
    TakeoffPayload | GotoPayload | LandPayload | ReturnHomePayload | DisarmPayload,
    Field(discriminator="command_type"),
]


class DroneCommand(ContractModel):
    """One protocol-neutral command addressed to a drone."""

    command_id: Identifier
    mission_id: Identifier
    task_id: Identifier
    drone_id: Identifier
    protocol: ProtocolType
    payload: CommandPayload
    created_at: datetime

    @field_validator("created_at")
    @classmethod
    def created_at_must_include_timezone(cls, value: datetime) -> datetime:
        if value.utcoffset() is None:
            raise ValueError("created_at must include a timezone offset")
        return value


class MissionTask(ContractModel):
    """An ordered set of commands assigned to one drone for one mission."""

    task_id: Identifier
    mission_id: Identifier
    drone_id: Identifier
    commands: list[DroneCommand] = Field(min_length=1)
    created_at: datetime

    @field_validator("created_at")
    @classmethod
    def created_at_must_include_timezone(cls, value: datetime) -> datetime:
        if value.utcoffset() is None:
            raise ValueError("created_at must include a timezone offset")
        return value

    @model_validator(mode="after")
    def commands_must_belong_to_task(self) -> Self:
        _require_unique(
            [command.command_id for command in self.commands],
            "commands must have unique command_id values",
        )
        if len({command.protocol for command in self.commands}) != 1:
            raise ValueError("all commands must use the same protocol")
        for command in self.commands:
            if command.task_id != self.task_id:
                raise ValueError("all commands must reference the enclosing task_id")
            if command.mission_id != self.mission_id:
                raise ValueError("all commands must reference the enclosing mission_id")
            if command.drone_id != self.drone_id:
                raise ValueError("all commands must target the enclosing drone_id")
        return self


class CommandStatus(str, Enum):
    """Lifecycle state of a protocol command."""

    PENDING = "pending"
    SENT = "sent"
    ACCEPTED = "accepted"
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    TIMED_OUT = "timed_out"
    UNSUPPORTED = "unsupported"


class CommandError(ContractModel):
    """Machine-readable command failure information."""

    code: Identifier
    message: str = Field(min_length=1, max_length=1_000)
    retryable: bool = False


class CommandResult(ContractModel):
    """Current command result plus its immediately preceding state."""

    command_id: Identifier
    drone_id: Identifier
    protocol: ProtocolType
    previous_status: CommandStatus | None = None
    status: CommandStatus
    updated_at: datetime
    error: CommandError | None = None

    @field_validator("updated_at")
    @classmethod
    def updated_at_must_include_timezone(cls, value: datetime) -> datetime:
        if value.utcoffset() is None:
            raise ValueError("updated_at must include a timezone offset")
        return value

    @model_validator(mode="after")
    def status_and_error_must_be_consistent(self) -> Self:
        if self.previous_status is not None:
            allowed = _ALLOWED_STATUS_TRANSITIONS[self.previous_status]
            if self.status != self.previous_status and self.status not in allowed:
                raise ValueError(
                    f"invalid command status transition: "
                    f"{self.previous_status.value} -> {self.status.value}"
                )

        error_statuses = {
            CommandStatus.FAILED,
            CommandStatus.TIMED_OUT,
            CommandStatus.UNSUPPORTED,
        }
        if self.status in error_statuses and self.error is None:
            raise ValueError(f"error is required when status is {self.status.value}")
        if self.status not in error_statuses and self.error is not None:
            raise ValueError(f"error is not allowed when status is {self.status.value}")
        return self


_ALLOWED_STATUS_TRANSITIONS: dict[CommandStatus, set[CommandStatus]] = {
    CommandStatus.PENDING: {
        CommandStatus.SENT,
        CommandStatus.FAILED,
        CommandStatus.UNSUPPORTED,
    },
    CommandStatus.SENT: {
        CommandStatus.ACCEPTED,
        CommandStatus.EXECUTING,
        CommandStatus.SUCCEEDED,
        CommandStatus.FAILED,
        CommandStatus.TIMED_OUT,
        CommandStatus.UNSUPPORTED,
    },
    CommandStatus.ACCEPTED: {
        CommandStatus.EXECUTING,
        CommandStatus.SUCCEEDED,
        CommandStatus.FAILED,
        CommandStatus.TIMED_OUT,
    },
    CommandStatus.EXECUTING: {
        CommandStatus.SUCCEEDED,
        CommandStatus.FAILED,
        CommandStatus.TIMED_OUT,
    },
    CommandStatus.SUCCEEDED: set(),
    CommandStatus.FAILED: set(),
    CommandStatus.TIMED_OUT: set(),
    CommandStatus.UNSUPPORTED: set(),
}
