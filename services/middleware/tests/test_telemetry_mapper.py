"""Tests for the protocol-neutral Telemetry -> DroneState mapper."""

from datetime import datetime, timezone

import pytest

from app.adapters import AP_DDS_CAPABILITIES, MAVLINK_CAPABILITIES, TelemetryStateMapper
from app.schemas import (
    AltitudeReference,
    ConnectionStatus,
    DroneStatus,
    ProtocolType,
    TelemetryAvailability,
    TelemetryField,
)

NOW = datetime(2026, 9, 28, 3, 0, tzinfo=timezone.utc)


def telemetry(**overrides):
    sample = {
        "lat": -35.363261,
        "lon": 149.165197,
        "relative_alt_m": 30.2,
        "vx": 1.0,
        "vy": -0.5,
        "vz": 0.2,
        "heading_deg": 90.0,
        "gps_fix_type": 3,
        "satellites_visible": 12,
        "gps_eph": 0.8,
        "battery_voltage_v": 12.6,
        "battery_remaining_pct": 100.0,
        "vibration_x": 0.02,
        "vibration_y": 0.02,
        "vibration_z": 0.05,
        "clipping": 0,
        "timestamp": NOW.timestamp(),
    }
    sample.update(overrides)
    return sample


def ap_dds_telemetry():
    return telemetry(
        satellites_visible=None,
        vibration_x=None,
        vibration_y=None,
        vibration_z=None,
        clipping=None,
    )


def test_maps_mavlink_telemetry_with_home_relative_position() -> None:
    state = TelemetryStateMapper().map(
        telemetry(),
        drone_id="drone-01",
        protocol=ProtocolType.MAVLINK,
        capabilities=MAVLINK_CAPABILITIES,
        status=DroneStatus.AVAILABLE,
        telemetry_age_s=0.1,
    )

    assert state.position.altitude_m == 30.2
    assert state.position.altitude_reference == AltitudeReference.HOME_RELATIVE
    assert state.velocity_ned_m_s.down_m_s == 0.2
    assert state.vibration.z == 0.05
    assert state.connection_status == ConnectionStatus.CONNECTED
    assert state.observed_at == NOW


def test_ap_dds_telemetry_marks_missing_protocol_fields_unsupported() -> None:
    state = TelemetryStateMapper().map(
        ap_dds_telemetry(),
        drone_id="drone-02",
        protocol=ProtocolType.AP_DDS,
        capabilities=AP_DDS_CAPABILITIES,
        status=DroneStatus.AVAILABLE,
        telemetry_age_s=0.1,
    )

    for field in (
        TelemetryField.SATELLITES_VISIBLE,
        TelemetryField.VIBRATION,
        TelemetryField.CLIPPING,
    ):
        assert state.telemetry_availability(field) == TelemetryAvailability.UNSUPPORTED
    assert state.telemetry_availability(TelemetryField.POSITION) == (
        TelemetryAvailability.AVAILABLE
    )


def test_ap_dds_rejects_values_for_fields_it_does_not_declare() -> None:
    with pytest.raises(ValueError, match="not declared in capabilities"):
        TelemetryStateMapper().map(
            telemetry(),
            drone_id="drone-02",
            protocol=ProtocolType.AP_DDS,
            capabilities=AP_DDS_CAPABILITIES,
            status=DroneStatus.AVAILABLE,
            telemetry_age_s=0.1,
        )


@pytest.mark.parametrize(
    ("age", "expected"),
    [
        (0.0, ConnectionStatus.CONNECTED),
        (3.0, ConnectionStatus.CONNECTED),
        (3.01, ConnectionStatus.DISCONNECTED),
        (None, ConnectionStatus.DISCONNECTED),
    ],
)
def test_telemetry_age_controls_connection_status(age, expected) -> None:
    state = TelemetryStateMapper(telemetry_timeout_s=3).map(
        telemetry(),
        drone_id="drone-01",
        protocol=ProtocolType.MAVLINK,
        capabilities=MAVLINK_CAPABILITIES,
        status=DroneStatus.ASSIGNED,
        telemetry_age_s=age,
    )

    assert state.connection_status == expected
    assert state.status == DroneStatus.ASSIGNED


def test_empty_sample_produces_disconnected_state_without_fabricated_values() -> None:
    state = TelemetryStateMapper().map(
        {"timestamp": NOW.timestamp()},
        drone_id="drone-01",
        protocol=ProtocolType.MAVLINK,
        capabilities=MAVLINK_CAPABILITIES,
        status=DroneStatus.AVAILABLE,
        telemetry_age_s=None,
    )

    assert state.position is None
    assert state.battery_percent is None
    assert state.connection_status == ConnectionStatus.DISCONNECTED
    assert state.telemetry_availability(TelemetryField.POSITION) == (
        TelemetryAvailability.TEMPORARILY_UNAVAILABLE
    )


@pytest.mark.parametrize(("raw", "expected"), [(360.0, 0.0), (359.99, 359.99), (-90.0, 270.0)])
def test_heading_is_normalized_into_zero_to_360(raw, expected) -> None:
    # Adapters round after "% 360", so 359.996 arrives as 360.0 (same direction as 0.0).
    state = TelemetryStateMapper().map(
        ap_dds_telemetry() | {"heading_deg": raw},
        drone_id="drone-02",
        protocol=ProtocolType.AP_DDS,
        capabilities=AP_DDS_CAPABILITIES,
        status=DroneStatus.AVAILABLE,
        telemetry_age_s=0.1,
    )
    assert state.heading_deg == pytest.approx(expected)
