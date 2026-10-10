"""Map the simulation ``Telemetry`` dictionary to ``DroneState`` for any protocol.

``MavlinkDrone`` and ``Ros2Drone`` both return the common ``Telemetry`` schema defined in
``services/simulation/app/drone_interface.py``: WGS84 position, home-relative altitude,
NED velocity, and ``None`` for values a protocol cannot provide. Protocol-specific frame
conversion therefore already happened in the simulation adapter, and the middleware only
needs one mapper whose capabilities differ per protocol.
"""

from collections.abc import Mapping
from datetime import datetime, timezone
from math import isfinite
from typing import Any

from app.schemas import (
    AltitudeReference,
    ConnectionStatus,
    DroneCapabilities,
    DroneState,
    DroneStatus,
    GeoPosition,
    NedVelocity,
    ProtocolType,
    Vibration,
)


class TelemetryStateMapper:
    """Convert one ``Telemetry`` sample into a normalized ``DroneState``."""

    def __init__(self, *, telemetry_timeout_s: float = 3.0) -> None:
        if (
            isinstance(telemetry_timeout_s, bool)
            or not isinstance(telemetry_timeout_s, (int, float))
            or not isfinite(telemetry_timeout_s)
            or telemetry_timeout_s <= 0
        ):
            raise ValueError("telemetry_timeout_s must be a positive finite number")
        self.telemetry_timeout_s = float(telemetry_timeout_s)

    def map(
        self,
        telemetry: Mapping[str, Any],
        *,
        drone_id: str,
        protocol: ProtocolType,
        capabilities: DroneCapabilities,
        status: DroneStatus,
        telemetry_age_s: float | None,
    ) -> DroneState:
        """Normalize one sample without opening or reading a connection."""
        timestamp = required_float(telemetry, "timestamp")
        if timestamp < 0:
            raise ValueError("timestamp must be a non-negative Unix timestamp")

        latitude = optional_float(telemetry, "lat")
        longitude = optional_float(telemetry, "lon")
        relative_altitude = optional_float(telemetry, "relative_alt_m")
        position = None
        if all(value is not None for value in (latitude, longitude, relative_altitude)):
            position = GeoPosition(
                latitude=latitude,
                longitude=longitude,
                altitude_m=relative_altitude,
                altitude_reference=AltitudeReference.HOME_RELATIVE,
            )

        north_velocity = optional_float(telemetry, "vx")
        east_velocity = optional_float(telemetry, "vy")
        down_velocity = optional_float(telemetry, "vz")
        velocity = None
        if all(
            value is not None
            for value in (north_velocity, east_velocity, down_velocity)
        ):
            velocity = NedVelocity(
                north_m_s=north_velocity,
                east_m_s=east_velocity,
                down_m_s=down_velocity,
            )

        vibration_x = optional_float(telemetry, "vibration_x")
        vibration_y = optional_float(telemetry, "vibration_y")
        vibration_z = optional_float(telemetry, "vibration_z")
        vibration = None
        if all(
            value is not None for value in (vibration_x, vibration_y, vibration_z)
        ):
            vibration = Vibration(x=vibration_x, y=vibration_y, z=vibration_z)

        return DroneState(
            drone_id=drone_id,
            protocol=protocol,
            capabilities=capabilities,
            position=position,
            velocity_ned_m_s=velocity,
            heading_deg=_normalize_heading(optional_float(telemetry, "heading_deg")),
            gps_fix_type=optional_int(telemetry, "gps_fix_type"),
            satellites_visible=optional_int(telemetry, "satellites_visible"),
            gps_eph_m=optional_float(telemetry, "gps_eph"),
            battery_voltage_v=optional_float(telemetry, "battery_voltage_v"),
            battery_percent=optional_float(telemetry, "battery_remaining_pct"),
            vibration=vibration,
            clipping_count=optional_int(telemetry, "clipping"),
            status=status,
            connection_status=self.connection_status(telemetry_age_s),
            observed_at=datetime.fromtimestamp(timestamp, tz=timezone.utc),
        )

    def connection_status(self, telemetry_age_s: float | None) -> ConnectionStatus:
        """Treat a sample older than the timeout, or no sample at all, as a lost link."""
        if telemetry_age_s is None:
            return ConnectionStatus.DISCONNECTED
        if isinstance(telemetry_age_s, bool) or not isinstance(
            telemetry_age_s, (int, float)
        ):
            raise ValueError("telemetry age must be numeric or None")
        if not isfinite(telemetry_age_s) or telemetry_age_s < 0:
            raise ValueError("telemetry age must be a non-negative finite number")
        if telemetry_age_s <= self.telemetry_timeout_s:
            return ConnectionStatus.CONNECTED
        return ConnectionStatus.DISCONNECTED


def _normalize_heading(heading: float | None) -> float | None:
    """Map any finite angle into [0, 360).

    Adapters compute ``round(angle % 360, 2)``, which yields exactly 360.0 for angles
    of 359.995 or more; that is the same direction as 0.0, not an invalid value.
    """
    if heading is None:
        return None
    normalized = heading % 360.0
    return 0.0 if normalized >= 360.0 else normalized


def required_float(values: Mapping[str, Any], key: str) -> float:
    value = optional_float(values, key)
    if value is None:
        raise ValueError(f"{key} is required")
    return value


def optional_float(values: Mapping[str, Any], key: str) -> float | None:
    value = values.get(key)
    if value is None or value == "":
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be numeric, None, or an empty string")
    normalized = float(value)
    if not isfinite(normalized):
        raise ValueError(f"{key} must be finite")
    return normalized


def optional_int(values: Mapping[str, Any], key: str) -> int | None:
    value = values.get(key)
    if value is None or value == "":
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be an integer, None, or an empty string")
    if not isfinite(value) or int(value) != value:
        raise ValueError(f"{key} must be an integer")
    return int(value)
