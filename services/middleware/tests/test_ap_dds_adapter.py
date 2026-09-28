"""Tests for the AP_DDS-to-middleware adapter boundary."""

from datetime import datetime, timezone
from math import cos, radians, sin

import pytest
from pydantic import ValidationError

from app.adapters import (
    AP_DDS_CAPABILITIES,
    ApDdsBinding,
    ApDdsCommandLifecycle,
    ApDdsStateMapper,
    ApDdsTakeoffRequest,
    UnsupportedApDdsCommandError,
    to_ap_dds_takeoff_request,
)
from app.schemas import (
    AltitudeReference,
    CommandStatus,
    ConnectionStatus,
    DroneCommand,
    DroneStatus,
    GeoPosition,
    GotoPayload,
    ProtocolType,
    TakeoffPayload,
    TelemetryAvailability,
    TelemetryField,
)

NOW = datetime(2026, 9, 23, 1, 0, tzinfo=timezone.utc)
BINDING = ApDdsBinding(drone_id="drone-02", namespace="ap")


@pytest.fixture
def ap_dds_sample() -> dict[str, float | int | None]:
    """Representative flattened values from the subscribed ROS 2 messages."""
    return {
        "latitude": 37.45,
        "longitude": 127.12,
        "altitude_amsl_m": 130.5,
        "velocity_enu_x_m_s": -0.5,
        "velocity_enu_y_m_s": 1.25,
        "velocity_enu_z_m_s": -0.1,
        "orientation_x": 0,
        "orientation_y": 0,
        "orientation_z": 0,
        "orientation_w": 1,
        "navsat_status": 0,
        "position_covariance_type": 2,
        "position_covariance_xx": 0.6724,
        "battery_voltage_v": 15.8,
        "battery_percentage": 0.76,
        "timestamp": NOW.timestamp(),
    }


def make_takeoff_command(
    *,
    protocol: ProtocolType = ProtocolType.AP_DDS,
    drone_id: str = "drone-02",
    altitude_reference: AltitudeReference = AltitudeReference.HOME_RELATIVE,
) -> DroneCommand:
    return DroneCommand(
        command_id="command-019",
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


def test_maps_ap_dds_topics_to_drone_state(
    ap_dds_sample: dict[str, float | int | None],
) -> None:
    state = ApDdsStateMapper().map(
        ap_dds_sample,
        binding=BINDING,
        source_namespace="/ap",
        status=DroneStatus.AVAILABLE,
        topic_age_s=0.2,
        home_altitude_amsl_m=100,
    )

    assert state.drone_id == "drone-02"
    assert state.protocol == ProtocolType.AP_DDS
    assert state.capabilities == AP_DDS_CAPABILITIES
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
    assert state.heading_deg == pytest.approx(90)
    assert state.gps_fix_type == 3
    assert state.gps_eph_m == pytest.approx(0.82)
    assert state.battery_percent == pytest.approx(76)


def test_preserves_msl_reference_when_home_altitude_is_unknown(
    ap_dds_sample: dict[str, float | int | None],
) -> None:
    state = ApDdsStateMapper().map(
        ap_dds_sample,
        binding=BINDING,
        source_namespace="ap",
        status=DroneStatus.ASSIGNED,
        topic_age_s=0,
        home_altitude_amsl_m=None,
    )

    assert state.position.altitude_m == 130.5
    assert state.position.altitude_reference == AltitudeReference.MSL


@pytest.mark.parametrize(
    ("yaw_deg", "expected_heading"),
    [(0, 90), (90, 0), (180, 270), (-90, 180)],
)
def test_converts_enu_yaw_to_compass_heading(
    ap_dds_sample: dict[str, float | int | None],
    yaw_deg: float,
    expected_heading: float,
) -> None:
    half_angle = yaw_deg / 2
    sample = {
        **ap_dds_sample,
        "orientation_z": sin(radians(half_angle)),
        "orientation_w": cos(radians(half_angle)),
    }
    state = ApDdsStateMapper().map(
        sample,
        binding=BINDING,
        source_namespace="ap",
        status=DroneStatus.AVAILABLE,
        topic_age_s=0,
        home_altitude_amsl_m=100,
    )

    assert state.heading_deg == pytest.approx(expected_heading)


def test_missing_supported_group_is_temporarily_unavailable(
    ap_dds_sample: dict[str, float | int | None],
) -> None:
    sample = {**ap_dds_sample, "velocity_enu_z_m_s": None}
    state = ApDdsStateMapper().map(
        sample,
        binding=BINDING,
        source_namespace="ap",
        status=DroneStatus.AVAILABLE,
        topic_age_s=0,
        home_altitude_amsl_m=100,
    )

    assert state.velocity_ned_m_s is None
    assert (
        state.telemetry_availability(TelemetryField.VELOCITY_NED)
        == TelemetryAvailability.TEMPORARILY_UNAVAILABLE
    )


def test_unsupported_ap_dds_fields_are_explicit(
    ap_dds_sample: dict[str, float | int | None],
) -> None:
    state = ApDdsStateMapper().map(
        ap_dds_sample,
        binding=BINDING,
        source_namespace="ap",
        status=DroneStatus.AVAILABLE,
        topic_age_s=0,
        home_altitude_amsl_m=100,
    )

    for field in (
        TelemetryField.SATELLITES_VISIBLE,
        TelemetryField.VIBRATION,
        TelemetryField.CLIPPING,
    ):
        assert field not in state.capabilities.telemetry_fields
        assert state.telemetry_availability(field) == TelemetryAvailability.UNSUPPORTED
    assert state.satellites_visible is None
    assert state.vibration is None
    assert state.clipping_count is None


def test_ros_unknown_nan_values_become_none(
    ap_dds_sample: dict[str, float | int | None],
) -> None:
    state = ApDdsStateMapper().map(
        {
            **ap_dds_sample,
            "battery_voltage_v": float("nan"),
            "battery_percentage": float("nan"),
        },
        binding=BINDING,
        source_namespace="ap",
        status=DroneStatus.AVAILABLE,
        topic_age_s=0,
        home_altitude_amsl_m=100,
    )

    assert state.battery_voltage_v is None
    assert state.battery_percent is None


@pytest.mark.parametrize(
    ("topic_age_s", "expected"),
    [
        (2.9, ConnectionStatus.CONNECTED),
        (3.1, ConnectionStatus.DISCONNECTED),
        (None, ConnectionStatus.DISCONNECTED),
    ],
)
def test_topic_age_controls_connection_status(
    ap_dds_sample: dict[str, float | int | None],
    topic_age_s: float | None,
    expected: ConnectionStatus,
) -> None:
    state = ApDdsStateMapper(topic_timeout_s=3).map(
        ap_dds_sample,
        binding=BINDING,
        source_namespace="ap",
        status=DroneStatus.AVAILABLE,
        topic_age_s=topic_age_s,
        home_altitude_amsl_m=100,
    )

    assert state.connection_status == expected


def test_rejects_mismatched_namespace(
    ap_dds_sample: dict[str, float | int | None],
) -> None:
    with pytest.raises(ValueError, match="not bound"):
        ApDdsStateMapper().map(
            ap_dds_sample,
            binding=BINDING,
            source_namespace="ap2",
            status=DroneStatus.AVAILABLE,
            topic_age_s=0,
            home_altitude_amsl_m=100,
        )


@pytest.mark.parametrize("value", [True, float("inf"), "unknown"])
def test_rejects_invalid_numeric_topic_values(
    ap_dds_sample: dict[str, float | int | None], value: object
) -> None:
    with pytest.raises(ValueError, match="battery_voltage_v"):
        ApDdsStateMapper().map(
            {**ap_dds_sample, "battery_voltage_v": value},
            binding=BINDING,
            source_namespace="ap",
            status=DroneStatus.AVAILABLE,
            topic_age_s=0,
            home_altitude_amsl_m=100,
        )


def test_rejects_invalid_covariance_or_quaternion(
    ap_dds_sample: dict[str, float | int | None],
) -> None:
    with pytest.raises(ValueError, match="must not be negative"):
        ApDdsStateMapper().map(
            {**ap_dds_sample, "position_covariance_xx": -1},
            binding=BINDING,
            source_namespace="ap",
            status=DroneStatus.AVAILABLE,
            topic_age_s=0,
            home_altitude_amsl_m=100,
        )
    with pytest.raises(ValueError, match="zero length"):
        ApDdsStateMapper().map(
            {
                **ap_dds_sample,
                "orientation_x": 0,
                "orientation_y": 0,
                "orientation_z": 0,
                "orientation_w": 0,
            },
            binding=BINDING,
            source_namespace="ap",
            status=DroneStatus.AVAILABLE,
            topic_age_s=0,
            home_altitude_amsl_m=100,
        )


def test_translates_takeoff_and_reuses_injected_ros2_client() -> None:
    request = to_ap_dds_takeoff_request(make_takeoff_command(), BINDING)

    class FakeRos2Drone:
        altitude_m: float | None = None

        def takeoff(self, altitude_m: float) -> None:
            self.altitude_m = altitude_m

    client = FakeRos2Drone()
    request.invoke(client)

    assert request.service_name == "/ap/experimental/takeoff"
    assert request.altitude_home_relative_m == 30
    assert client.altitude_m == 30


def test_takeoff_request_rejects_service_outside_bound_namespace() -> None:
    with pytest.raises(ValidationError, match="service_name must be"):
        ApDdsTakeoffRequest(
            command_id="command-019",
            namespace="ap",
            service_name="/other/experimental/takeoff",
            altitude_home_relative_m=30,
        )


def test_takeoff_translation_rejects_wrong_scope_or_altitude_reference() -> None:
    with pytest.raises(UnsupportedApDdsCommandError, match="protocol"):
        to_ap_dds_takeoff_request(
            make_takeoff_command(protocol=ProtocolType.MAVLINK), BINDING
        )
    with pytest.raises(ValueError, match="binding belongs"):
        to_ap_dds_takeoff_request(
            make_takeoff_command(drone_id="drone-other"), BINDING
        )
    with pytest.raises(UnsupportedApDdsCommandError, match="home-relative"):
        to_ap_dds_takeoff_request(
            make_takeoff_command(altitude_reference=AltitudeReference.MSL), BINDING
        )


def test_takeoff_translation_rejects_other_command_types() -> None:
    goto = DroneCommand(
        command_id="command-020",
        mission_id="mission-001",
        task_id="task-001",
        drone_id="drone-02",
        protocol=ProtocolType.AP_DDS,
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

    with pytest.raises(UnsupportedApDdsCommandError, match="takeoff"):
        to_ap_dds_takeoff_request(goto, BINDING)


def test_service_acceptance_is_not_vehicle_completion() -> None:
    lifecycle = ApDdsCommandLifecycle(make_takeoff_command())

    sent = lifecycle.mark_sent(at=NOW)
    accepted = lifecycle.handle_service_response(True, at=NOW)

    assert sent.status == CommandStatus.SENT
    assert accepted.previous_status == CommandStatus.SENT
    assert accepted.status == CommandStatus.ACCEPTED
    assert lifecycle.status != CommandStatus.SUCCEEDED

    executing = lifecycle.mark_executing(at=NOW)
    completed = lifecycle.mark_completed(at=NOW)
    assert executing.status == CommandStatus.EXECUTING
    assert completed.status == CommandStatus.SUCCEEDED


def test_maps_rejected_service_response() -> None:
    lifecycle = ApDdsCommandLifecycle(make_takeoff_command())
    lifecycle.mark_sent(at=NOW)

    result = lifecycle.handle_service_response(False, at=NOW, message="takeoff denied")

    assert result.status == CommandStatus.FAILED
    assert result.error.code == "AP_DDS_SERVICE_REJECTED"
    assert result.error.message == "takeoff denied"
    assert result.error.retryable is False


def test_maps_service_and_completion_timeouts_separately() -> None:
    service_lifecycle = ApDdsCommandLifecycle(make_takeoff_command())
    service_lifecycle.mark_sent(at=NOW)
    service_timeout = service_lifecycle.mark_service_timeout(at=NOW)

    completion_lifecycle = ApDdsCommandLifecycle(make_takeoff_command())
    completion_lifecycle.mark_sent(at=NOW)
    completion_lifecycle.handle_service_response(True, at=NOW)
    completion_timeout = completion_lifecycle.mark_completion_timeout(at=NOW)

    assert service_timeout.error.code == "AP_DDS_SERVICE_TIMEOUT"
    assert completion_timeout.error.code == "AP_DDS_COMPLETION_TIMEOUT"


def test_binding_rejects_unknown_fields() -> None:
    with pytest.raises(ValidationError, match="unexpected"):
        ApDdsBinding.model_validate(
            {"drone_id": "drone-02", "namespace": "ap", "unexpected": True}
        )
