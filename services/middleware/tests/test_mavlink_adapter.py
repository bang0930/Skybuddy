"""Tests for the MAVLink-to-middleware adapter boundary."""

from datetime import datetime, timezone

import pytest
from pydantic import ValidationError

from app.adapters import (
    MAVLINK_CAPABILITIES,
    MavlinkAckResult,
    MavlinkCommandLifecycle,
    MavlinkDroneBinding,
    MavlinkStateMapper,
    UnsupportedMavlinkCommandError,
    to_mavlink_takeoff_request,
)
from app.schemas import (
    AltitudeReference,
    CommandStatus,
    ConnectionStatus,
    DroneCommand,
    DroneCommandType,
    DroneStatus,
    GeoPosition,
    GotoPayload,
    ProtocolType,
    TakeoffPayload,
    TelemetryAvailability,
    TelemetryField,
)

NOW = datetime(2026, 9, 22, 1, 0, tzinfo=timezone.utc)
BINDING = MavlinkDroneBinding(drone_id="drone-01", system_id=1, component_id=1)


@pytest.fixture
def mavlink_sample() -> dict[str, float | int | None]:
    """Representative output from MavlinkDrone.get_telemetry()."""
    return {
        "lat": 37.45,
        "lon": 127.12,
        "relative_alt_m": 30.5,
        "vx": 1.25,
        "vy": -0.5,
        "vz": 0.1,
        "heading_deg": 91.2,
        "gps_fix_type": 3,
        "satellites_visible": 14,
        "gps_eph": 0.82,
        "battery_voltage_v": 15.8,
        "battery_remaining_pct": 76,
        "vibration_x": 0.02,
        "vibration_y": 0.03,
        "vibration_z": 0.04,
        "clipping": 0,
        "timestamp": NOW.timestamp(),
    }


def make_takeoff_command(
    *,
    protocol: ProtocolType = ProtocolType.MAVLINK,
    drone_id: str = "drone-01",
    altitude_reference: AltitudeReference = AltitudeReference.HOME_RELATIVE,
) -> DroneCommand:
    return DroneCommand(
        command_id="command-001",
        mission_id="mission-001",
        task_id="task-001",
        drone_id=drone_id,
        protocol=protocol,
        payload=TakeoffPayload(
            command_type="takeoff",
            altitude_m=30,
            altitude_reference=altitude_reference,
        ),
        created_at=NOW,
    )


def test_maps_actual_mavlink_sample_to_drone_state(
    mavlink_sample: dict[str, float | int | None],
) -> None:
    state = MavlinkStateMapper().map(
        mavlink_sample,
        binding=BINDING,
        source_system_id=1,
        status=DroneStatus.AVAILABLE,
        heartbeat_age_s=0.5,
    )

    assert state.drone_id == "drone-01"
    assert state.protocol == ProtocolType.MAVLINK
    assert state.capabilities == MAVLINK_CAPABILITIES
    assert state.connection_status == ConnectionStatus.CONNECTED
    assert state.observed_at == NOW
    assert state.position == GeoPosition(
        latitude=37.45,
        longitude=127.12,
        altitude_m=30.5,
        altitude_reference=AltitudeReference.HOME_RELATIVE,
    )
    assert state.velocity_ned_m_s.model_dump() == {
        "north_m_s": 1.25,
        "east_m_s": -0.5,
        "down_m_s": 0.1,
    }
    assert state.battery_percent == 76
    assert state.heading_deg == 91.2


def test_home_relative_altitude_is_not_mislabeled_as_msl(
    mavlink_sample: dict[str, float | int | None],
) -> None:
    state = MavlinkStateMapper().map(
        mavlink_sample,
        binding=BINDING,
        source_system_id=1,
        status=DroneStatus.ASSIGNED,
        heartbeat_age_s=0,
    )

    assert state.position.altitude_reference == AltitudeReference.HOME_RELATIVE


def test_legacy_empty_values_are_normalized_to_none(
    mavlink_sample: dict[str, float | int | None],
) -> None:
    sample = {
        **mavlink_sample,
        "satellites_visible": "",
        "vibration_x": "",
        "vibration_y": "",
        "vibration_z": "",
    }
    state = MavlinkStateMapper().map(
        sample,
        binding=BINDING,
        source_system_id=1,
        status=DroneStatus.AVAILABLE,
        heartbeat_age_s=0,
    )

    assert state.satellites_visible is None
    assert state.vibration is None
    assert (
        state.telemetry_availability(TelemetryField.SATELLITES_VISIBLE)
        == TelemetryAvailability.TEMPORARILY_UNAVAILABLE
    )


def test_partial_position_becomes_temporarily_unavailable(
    mavlink_sample: dict[str, float | int | None],
) -> None:
    state = MavlinkStateMapper().map(
        {**mavlink_sample, "relative_alt_m": None},
        binding=BINDING,
        source_system_id=1,
        status=DroneStatus.AVAILABLE,
        heartbeat_age_s=0,
    )

    assert state.position is None
    assert (
        state.telemetry_availability(TelemetryField.POSITION)
        == TelemetryAvailability.TEMPORARILY_UNAVAILABLE
    )


@pytest.mark.parametrize(
    ("heartbeat_age_s", "expected"),
    [
        (2.9, ConnectionStatus.CONNECTED),
        (3.1, ConnectionStatus.DISCONNECTED),
        (None, ConnectionStatus.DISCONNECTED),
    ],
)
def test_heartbeat_age_controls_connection_status(
    mavlink_sample: dict[str, float | int | None],
    heartbeat_age_s: float | None,
    expected: ConnectionStatus,
) -> None:
    state = MavlinkStateMapper(heartbeat_timeout_s=3).map(
        mavlink_sample,
        binding=BINDING,
        source_system_id=1,
        status=DroneStatus.AVAILABLE,
        heartbeat_age_s=heartbeat_age_s,
    )

    assert state.connection_status == expected


def test_rejects_mismatched_mavlink_system_id(
    mavlink_sample: dict[str, float | int | None],
) -> None:
    with pytest.raises(ValueError, match="SYSID 2 is not bound"):
        MavlinkStateMapper().map(
            mavlink_sample,
            binding=BINDING,
            source_system_id=2,
            status=DroneStatus.AVAILABLE,
            heartbeat_age_s=0,
        )


@pytest.mark.parametrize("source_system_id", [True, 0, 256, 1.5])
def test_rejects_invalid_source_system_id(
    mavlink_sample: dict[str, float | int | None], source_system_id: object
) -> None:
    with pytest.raises(ValueError, match="integer from 1 to 255"):
        MavlinkStateMapper().map(
            mavlink_sample,
            binding=BINDING,
            source_system_id=source_system_id,
            status=DroneStatus.AVAILABLE,
            heartbeat_age_s=0,
        )


@pytest.mark.parametrize("value", [True, float("nan"), float("inf"), "unknown"])
def test_rejects_invalid_numeric_telemetry(
    mavlink_sample: dict[str, float | int | None], value: object
) -> None:
    with pytest.raises(ValueError, match="battery_voltage_v"):
        MavlinkStateMapper().map(
            {**mavlink_sample, "battery_voltage_v": value},
            binding=BINDING,
            source_system_id=1,
            status=DroneStatus.AVAILABLE,
            heartbeat_age_s=0,
        )


def test_rejects_negative_unix_timestamp(
    mavlink_sample: dict[str, float | int | None],
) -> None:
    with pytest.raises(ValueError, match="non-negative Unix timestamp"):
        MavlinkStateMapper().map(
            {**mavlink_sample, "timestamp": -1},
            binding=BINDING,
            source_system_id=1,
            status=DroneStatus.AVAILABLE,
            heartbeat_age_s=0,
        )


def test_translates_takeoff_command_and_reuses_injected_client() -> None:
    request = to_mavlink_takeoff_request(make_takeoff_command(), BINDING)

    class FakeMavlinkDrone:
        altitude_m: float | None = None

        def takeoff(self, altitude_m: float) -> None:
            self.altitude_m = altitude_m

    client = FakeMavlinkDrone()
    request.invoke(client)

    assert request.mav_command == 22
    assert request.target_system == 1
    assert request.target_component == 1
    assert client.altitude_m == 30


def test_takeoff_translation_rejects_msl_until_conversion_is_available() -> None:
    with pytest.raises(UnsupportedMavlinkCommandError, match="home-relative"):
        to_mavlink_takeoff_request(
            make_takeoff_command(altitude_reference=AltitudeReference.MSL),
            BINDING,
        )


def test_takeoff_translation_rejects_wrong_protocol_or_drone() -> None:
    with pytest.raises(UnsupportedMavlinkCommandError, match="protocol"):
        to_mavlink_takeoff_request(
            make_takeoff_command(protocol=ProtocolType.AP_DDS), BINDING
        )
    with pytest.raises(ValueError, match="binding belongs"):
        to_mavlink_takeoff_request(
            make_takeoff_command(drone_id="drone-02"), BINDING
        )


def test_takeoff_translation_rejects_other_command_types() -> None:
    goto = DroneCommand(
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

    with pytest.raises(UnsupportedMavlinkCommandError, match="takeoff"):
        to_mavlink_takeoff_request(goto, BINDING)


def test_command_lifecycle_keeps_ack_separate_from_completion() -> None:
    lifecycle = MavlinkCommandLifecycle(make_takeoff_command())

    sent = lifecycle.mark_sent(at=NOW)
    accepted = lifecycle.handle_ack(MavlinkAckResult.ACCEPTED, at=NOW)

    assert sent.status == CommandStatus.SENT
    assert accepted.previous_status == CommandStatus.SENT
    assert accepted.status == CommandStatus.ACCEPTED
    assert lifecycle.status != CommandStatus.SUCCEEDED

    completed = lifecycle.mark_completed(at=NOW)
    assert completed.previous_status == CommandStatus.ACCEPTED
    assert completed.status == CommandStatus.SUCCEEDED


@pytest.mark.parametrize(
    ("ack", "status", "code", "retryable"),
    [
        (
            MavlinkAckResult.TEMPORARILY_REJECTED,
            CommandStatus.FAILED,
            "MAVLINK_ACK_TEMPORARILY_REJECTED",
            True,
        ),
        (
            MavlinkAckResult.DENIED,
            CommandStatus.FAILED,
            "MAVLINK_ACK_DENIED",
            False,
        ),
        (
            MavlinkAckResult.UNSUPPORTED,
            CommandStatus.UNSUPPORTED,
            "MAVLINK_COMMAND_UNSUPPORTED",
            False,
        ),
    ],
)
def test_maps_rejected_command_ack(
    ack: MavlinkAckResult,
    status: CommandStatus,
    code: str,
    retryable: bool,
) -> None:
    lifecycle = MavlinkCommandLifecycle(make_takeoff_command())
    lifecycle.mark_sent(at=NOW)

    result = lifecycle.handle_ack(ack, at=NOW)

    assert result.status == status
    assert result.error.code == code
    assert result.error.retryable is retryable


def test_maps_ack_and_completion_timeouts_separately() -> None:
    ack_lifecycle = MavlinkCommandLifecycle(make_takeoff_command())
    ack_lifecycle.mark_sent(at=NOW)
    ack_timeout = ack_lifecycle.mark_ack_timeout(at=NOW)

    completion_lifecycle = MavlinkCommandLifecycle(make_takeoff_command())
    completion_lifecycle.mark_sent(at=NOW)
    completion_lifecycle.handle_ack(MavlinkAckResult.ACCEPTED, at=NOW)
    completion_timeout = completion_lifecycle.mark_completion_timeout(at=NOW)

    assert ack_timeout.error.code == "MAVLINK_ACK_TIMEOUT"
    assert completion_timeout.error.code == "MAVLINK_COMPLETION_TIMEOUT"


def test_in_progress_ack_maps_to_executing() -> None:
    lifecycle = MavlinkCommandLifecycle(make_takeoff_command())
    lifecycle.mark_sent(at=NOW)

    result = lifecycle.handle_ack(MavlinkAckResult.IN_PROGRESS, at=NOW)

    assert result.status == CommandStatus.EXECUTING


def test_binding_and_request_contracts_reject_unknown_fields() -> None:
    with pytest.raises(ValidationError, match="unexpected"):
        MavlinkDroneBinding.model_validate(
            {
                "drone_id": "drone-01",
                "system_id": 1,
                "component_id": 1,
                "unexpected": True,
            }
        )
