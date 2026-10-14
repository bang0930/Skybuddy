"""Tests for translating common commands into DroneInterface calls."""

from datetime import datetime, timezone

import pytest
from pydantic import ValidationError

from app.adapters import (
    AP_DDS_CAPABILITIES,
    MAVLINK_CAPABILITIES,
    DroneInterfaceRequest,
    InterfaceMethod,
    UnsafeAltitudeError,
    UnsupportedDroneCommandError,
    to_drone_interface_request,
)
from app.schemas import (
    AltitudeReference,
    DisarmPayload,
    DroneCommand,
    GeoPosition,
    GotoPayload,
    LandPayload,
    ProtocolType,
    ReturnHomePayload,
    TakeoffPayload,
)

NOW = datetime(2026, 9, 28, 3, 0, tzinfo=timezone.utc)


class RecordingClient:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def takeoff(self, altitude_m):
        self.calls.append(("takeoff", altitude_m))

    def goto(self, lat, lon, alt_m):
        self.calls.append(("goto", lat, lon, alt_m))

    def land(self, timeout=60):
        self.calls.append(("land",))
        return True

    def disarm(self, timeout=5):
        self.calls.append(("disarm",))
        return True

    def get_telemetry(self, timeout=2):
        return None


def command(payload, *, protocol=ProtocolType.MAVLINK, drone_id="drone-01"):
    return DroneCommand(
        command_id="command-001",
        mission_id="mission-001",
        task_id="task-001",
        drone_id=drone_id,
        protocol=protocol,
        payload=payload,
        created_at=NOW,
    )


def translate(payload, *, protocol=ProtocolType.MAVLINK, capabilities=MAVLINK_CAPABILITIES):
    return to_drone_interface_request(
        command(payload, protocol=protocol),
        drone_id="drone-01",
        protocol=protocol,
        capabilities=capabilities,
    )


def takeoff(altitude=30.0, reference=AltitudeReference.HOME_RELATIVE):
    return TakeoffPayload(
        command_type="takeoff", altitude_m=altitude, altitude_reference=reference
    )


def goto(altitude=30.0, reference=AltitudeReference.HOME_RELATIVE):
    return GotoPayload(
        command_type="goto",
        target=GeoPosition(
            latitude=-35.3632,
            longitude=149.1652,
            altitude_m=altitude,
            altitude_reference=reference,
        ),
    )


@pytest.mark.parametrize(
    ("payload", "expected_call"),
    [
        (takeoff(35.0), ("takeoff", 35.0)),
        (goto(40.0), ("goto", -35.3632, 149.1652, 40.0)),
        (LandPayload(command_type="land"), ("land",)),
        (DisarmPayload(command_type="disarm"), ("disarm",)),
    ],
)
def test_translates_and_invokes_drone_interface_methods(payload, expected_call) -> None:
    client = RecordingClient()

    request = translate(payload)
    request.invoke(client)

    assert request.method == InterfaceMethod(expected_call[0])
    assert client.calls == [expected_call]


def test_same_translation_serves_ap_dds_drones() -> None:
    request = translate(
        goto(), protocol=ProtocolType.AP_DDS, capabilities=AP_DDS_CAPABILITIES
    )

    assert request.protocol == ProtocolType.AP_DDS
    assert request.method == InterfaceMethod.GOTO


@pytest.mark.parametrize("payload", [takeoff(29.9), goto(10.0)])
def test_rejects_altitude_below_minimum_safe_altitude(payload) -> None:
    with pytest.raises(UnsafeAltitudeError, match="minimum safe altitude"):
        translate(payload)


@pytest.mark.parametrize(
    "payload",
    [takeoff(reference=AltitudeReference.MSL), goto(reference=AltitudeReference.MSL)],
)
def test_rejects_msl_altitude(payload) -> None:
    with pytest.raises(UnsupportedDroneCommandError, match="home-relative"):
        translate(payload)


def test_rejects_commands_missing_from_capabilities() -> None:
    with pytest.raises(UnsupportedDroneCommandError, match="return_home"):
        translate(ReturnHomePayload(command_type="return_home"))


def test_rejects_protocol_or_drone_mismatch() -> None:
    with pytest.raises(UnsupportedDroneCommandError, match="protocol"):
        to_drone_interface_request(
            command(takeoff(), protocol=ProtocolType.AP_DDS),
            drone_id="drone-01",
            protocol=ProtocolType.MAVLINK,
            capabilities=MAVLINK_CAPABILITIES,
        )
    with pytest.raises(ValueError, match="client is drone-02"):
        to_drone_interface_request(
            command(takeoff()),
            drone_id="drone-02",
            protocol=ProtocolType.MAVLINK,
            capabilities=MAVLINK_CAPABILITIES,
        )


def test_request_contract_rejects_inconsistent_arguments() -> None:
    with pytest.raises(ValidationError, match="altitude is required"):
        DroneInterfaceRequest(
            command_id="c-1", drone_id="drone-01", protocol="mavlink", method="takeoff"
        )
    with pytest.raises(ValidationError, match="position is not allowed"):
        DroneInterfaceRequest(
            command_id="c-1",
            drone_id="drone-01",
            protocol="mavlink",
            method="land",
            latitude=1.0,
            longitude=1.0,
        )
