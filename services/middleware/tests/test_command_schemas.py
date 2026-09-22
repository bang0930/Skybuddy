"""Tests for protocol-neutral command and result contracts."""

from datetime import datetime, timezone

import pytest
from pydantic import ValidationError

from app.schemas import (
    AltitudeReference,
    CommandError,
    CommandResult,
    CommandStatus,
    DroneCommand,
    GeoPosition,
    GotoPayload,
    LandPayload,
    MissionTask,
    ProtocolType,
    TakeoffPayload,
)

NOW = datetime(2026, 9, 19, 1, 0, tzinfo=timezone.utc)


def make_command(
    command_id: str = "command-001",
    *,
    mission_id: str = "mission-001",
    task_id: str = "task-001",
    drone_id: str = "drone-01",
) -> DroneCommand:
    return DroneCommand(
        command_id=command_id,
        mission_id=mission_id,
        task_id=task_id,
        drone_id=drone_id,
        protocol=ProtocolType.MAVLINK,
        payload=TakeoffPayload(
            command_type="takeoff",
            altitude_m=30,
            altitude_reference=AltitudeReference.HOME_RELATIVE,
        ),
        created_at=NOW,
    )


def test_command_payload_uses_discriminated_json_schema() -> None:
    schema = DroneCommand.model_json_schema()
    payload_schema = schema["properties"]["payload"]

    assert payload_schema["discriminator"]["propertyName"] == "command_type"
    assert set(payload_schema["discriminator"]["mapping"]) == {
        "takeoff",
        "goto",
        "land",
        "return_home",
        "disarm",
    }


def test_goto_command_requires_explicit_altitude_reference() -> None:
    command = DroneCommand(
        command_id="command-002",
        mission_id="mission-001",
        task_id="task-001",
        drone_id="drone-01",
        protocol=ProtocolType.MAVLINK,
        payload=GotoPayload(
            command_type="goto",
            target=GeoPosition(
                latitude=37.45,
                longitude=127.12,
                altitude_m=30,
                altitude_reference=AltitudeReference.HOME_RELATIVE,
            ),
        ),
        created_at=NOW,
    )

    assert command.payload.target.altitude_reference == AltitudeReference.HOME_RELATIVE


def test_command_rejects_naive_created_at() -> None:
    with pytest.raises(ValidationError, match="timezone offset"):
        DroneCommand(
            **make_command().model_dump(exclude={"created_at"}),
            created_at=datetime(2026, 9, 19, 1, 0),
        )


def test_mission_task_accepts_ordered_commands_for_same_target() -> None:
    commands = [
        make_command("command-001"),
        DroneCommand(
            command_id="command-002",
            mission_id="mission-001",
            task_id="task-001",
            drone_id="drone-01",
            protocol=ProtocolType.MAVLINK,
            payload=LandPayload(command_type="land"),
            created_at=NOW,
        ),
    ]

    task = MissionTask(
        task_id="task-001",
        mission_id="mission-001",
        drone_id="drone-01",
        commands=commands,
        created_at=NOW,
    )

    assert [command.command_id for command in task.commands] == [
        "command-001",
        "command-002",
    ]


@pytest.mark.parametrize(
    ("override", "expected_message"),
    [
        ({"task_id": "task-other"}, "enclosing task_id"),
        ({"mission_id": "mission-other"}, "enclosing mission_id"),
        ({"drone_id": "drone-other"}, "enclosing drone_id"),
    ],
)
def test_mission_task_rejects_commands_for_another_scope(
    override: dict[str, str], expected_message: str
) -> None:
    with pytest.raises(ValidationError, match=expected_message):
        MissionTask(
            task_id="task-001",
            mission_id="mission-001",
            drone_id="drone-01",
            commands=[make_command(**override)],
            created_at=NOW,
        )


def test_mission_task_rejects_duplicate_command_ids() -> None:
    with pytest.raises(ValidationError, match="unique command_id"):
        MissionTask(
            task_id="task-001",
            mission_id="mission-001",
            drone_id="drone-01",
            commands=[make_command(), make_command()],
            created_at=NOW,
        )


def test_mission_task_rejects_mixed_protocols_for_one_drone() -> None:
    dds_command = DroneCommand(
        **make_command("command-002").model_dump(exclude={"protocol"}),
        protocol=ProtocolType.AP_DDS,
    )

    with pytest.raises(ValidationError, match="same protocol"):
        MissionTask(
            task_id="task-001",
            mission_id="mission-001",
            drone_id="drone-01",
            commands=[make_command(), dds_command],
            created_at=NOW,
        )


def test_command_result_accepts_valid_state_transition() -> None:
    result = CommandResult(
        command_id="command-001",
        drone_id="drone-01",
        protocol=ProtocolType.MAVLINK,
        previous_status=CommandStatus.SENT,
        status=CommandStatus.ACCEPTED,
        updated_at=NOW,
    )

    assert result.status == CommandStatus.ACCEPTED


def test_command_result_allows_sent_to_succeeded_without_ack() -> None:
    result = CommandResult(
        command_id="command-001",
        drone_id="drone-01",
        protocol=ProtocolType.AP_DDS,
        previous_status=CommandStatus.SENT,
        status=CommandStatus.SUCCEEDED,
        updated_at=NOW,
    )

    assert result.status == CommandStatus.SUCCEEDED


def test_command_result_rejects_invalid_terminal_transition() -> None:
    with pytest.raises(ValidationError, match="succeeded -> executing"):
        CommandResult(
            command_id="command-001",
            drone_id="drone-01",
            protocol=ProtocolType.MAVLINK,
            previous_status=CommandStatus.SUCCEEDED,
            status=CommandStatus.EXECUTING,
            updated_at=NOW,
        )


def test_command_result_rejects_naive_updated_at() -> None:
    with pytest.raises(ValidationError, match="timezone offset"):
        CommandResult(
            command_id="command-001",
            drone_id="drone-01",
            protocol=ProtocolType.MAVLINK,
            status=CommandStatus.PENDING,
            updated_at=datetime(2026, 9, 19, 1, 0),
        )


@pytest.mark.parametrize(
    "status",
    [CommandStatus.FAILED, CommandStatus.TIMED_OUT, CommandStatus.UNSUPPORTED],
)
def test_error_status_requires_machine_readable_error(status: CommandStatus) -> None:
    with pytest.raises(ValidationError, match="error is required"):
        CommandResult(
            command_id="command-001",
            drone_id="drone-01",
            protocol=ProtocolType.MAVLINK,
            status=status,
            updated_at=NOW,
        )


def test_failed_command_serializes_error_contract() -> None:
    result = CommandResult(
        command_id="command-001",
        drone_id="drone-01",
        protocol=ProtocolType.MAVLINK,
        previous_status=CommandStatus.SENT,
        status=CommandStatus.FAILED,
        updated_at=NOW,
        error=CommandError(
            code="COMMAND_REJECTED",
            message="Vehicle rejected the command",
            retryable=False,
        ),
    )

    assert result.model_dump(mode="json")["error"] == {
        "code": "COMMAND_REJECTED",
        "message": "Vehicle rejected the command",
        "retryable": False,
    }


def test_non_error_status_rejects_error_details() -> None:
    with pytest.raises(ValidationError, match="error is not allowed"):
        CommandResult(
            command_id="command-001",
            drone_id="drone-01",
            protocol=ProtocolType.MAVLINK,
            status=CommandStatus.SENT,
            updated_at=NOW,
            error=CommandError(
                code="UNEXPECTED_ERROR",
                message="This error must not accompany a sent result",
            ),
        )


@pytest.mark.parametrize("model", [MissionTask, CommandResult])
def test_api_contracts_expose_strict_json_schema(model: type) -> None:
    assert model.model_json_schema()["additionalProperties"] is False


def test_command_contract_rejects_unknown_fields() -> None:
    with pytest.raises(ValidationError, match="unexpected"):
        DroneCommand.model_validate(
            {
                **make_command().model_dump(),
                "unexpected": True,
            }
        )
