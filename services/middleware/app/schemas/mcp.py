"""Results returned by the middleware MCP tools.

``get_mission_context`` returns the existing ``MissionContext``. The models below cover
plan submission and mission progress, which have no earlier contract.
"""

from enum import Enum

from pydantic import Field

from .command import CommandResult, CommandStatus
from .mission import ContractModel, DroneState, Identifier


class ValidationIssue(ContractModel):
    """One reason a submitted plan was rejected."""

    code: Identifier
    message: str = Field(min_length=1, max_length=2_000)
    location: str | None = Field(
        default=None,
        description="Dotted path of the offending input field, when known.",
    )


class SubmissionStatus(str, Enum):
    ACCEPTED = "accepted"
    REJECTED = "rejected"


class TaskState(str, Enum):
    """Progress of one drone's task, derived from its command results."""

    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


class AcceptedTask(ContractModel):
    task_id: Identifier
    drone_id: Identifier
    area_id: Identifier
    command_count: int = Field(ge=1)


class PlanSubmissionResult(ContractModel):
    """Immediate answer to ``submit_mission_plan``; execution continues afterwards."""

    mission_id: str
    status: SubmissionStatus
    attempt: int = Field(
        ge=1, description="How many plans have been submitted for this mission so far."
    )
    errors: list[ValidationIssue] = Field(default_factory=list)
    tasks: list[AcceptedTask] = Field(default_factory=list)


class CommandReport(ContractModel):
    """Latest result of one command plus the ordered states it passed through."""

    command_id: Identifier
    command_type: str
    status: CommandStatus
    history: list[CommandResult]


class TaskReport(ContractModel):
    task_id: Identifier
    drone_id: Identifier
    area_id: Identifier
    state: TaskState
    commands: list[CommandReport]


class MissionState(str, Enum):
    """Overall mission progress across all accepted tasks."""

    IDLE = "idle"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


class MissionStatusReport(ContractModel):
    """Answer to ``get_mission_status``."""

    mission_id: Identifier
    state: MissionState
    drones: list[DroneState]
    tasks: list[TaskReport]
