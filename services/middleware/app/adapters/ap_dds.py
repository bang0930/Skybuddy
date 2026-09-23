"""Map AP_DDS ROS 2 data to protocol-neutral middleware contracts.

The module intentionally has no dependency on ``rclpy`` or ROS message packages. The
simulation service owns the concrete ``Ros2Drone`` client, while middleware code consumes
flattened values from its topics and can invoke it through the small protocol below.
"""

from collections.abc import Mapping
from datetime import datetime, timezone
from math import atan2, degrees, isfinite, sqrt
from typing import Any, Protocol, Self

from pydantic import Field, model_validator

from app.schemas import (
    AltitudeReference,
    CommandError,
    CommandResult,
    CommandStatus,
    ConnectionStatus,
    DroneCapabilities,
    DroneCommand,
    DroneCommandType,
    DroneState,
    DroneStatus,
    GeoPosition,
    NedVelocity,
    ProtocolType,
    TakeoffPayload,
    TelemetryField,
)
from app.schemas.mission import ContractModel, Identifier

AP_DDS_TAKEOFF_SERVICE = "experimental/takeoff"

AP_DDS_CAPABILITIES = DroneCapabilities(
    commands=[
        DroneCommandType.TAKEOFF,
        DroneCommandType.GOTO,
        DroneCommandType.LAND,
        DroneCommandType.DISARM,
    ],
    telemetry_fields=[
        TelemetryField.POSITION,
        TelemetryField.VELOCITY_NED,
        TelemetryField.HEADING,
        TelemetryField.GPS_FIX_TYPE,
        TelemetryField.GPS_EPH,
        TelemetryField.BATTERY_VOLTAGE,
        TelemetryField.BATTERY_PERCENT,
    ],
)


class ApDdsBinding(ContractModel):
    """Stable association between a middleware drone ID and ROS 2 namespace."""

    drone_id: Identifier
    namespace: Identifier

    @property
    def takeoff_service(self) -> str:
        return f"/{self.namespace}/{AP_DDS_TAKEOFF_SERVICE}"


class ApDdsStateMapper:
    """Convert flattened AP_DDS topic values into ``DroneState``."""

    def __init__(self, *, topic_timeout_s: float = 3.0) -> None:
        if not _is_number(topic_timeout_s) or topic_timeout_s <= 0:
            raise ValueError("topic_timeout_s must be a positive finite number")
        self.topic_timeout_s = float(topic_timeout_s)

    def map(
        self,
        sample: Mapping[str, Any],
        *,
        binding: ApDdsBinding,
        source_namespace: str,
        status: DroneStatus,
        topic_age_s: float | None,
        home_altitude_amsl_m: float | None,
    ) -> DroneState:
        """Normalize one topic snapshot without importing or opening ROS 2."""
        normalized_namespace = source_namespace.strip("/")
        if normalized_namespace != binding.namespace:
            raise ValueError(
                f"AP_DDS namespace /{normalized_namespace} is not bound to "
                f"{binding.drone_id} (expected /{binding.namespace})"
            )

        timestamp = _required_float(sample, "timestamp")
        if timestamp < 0:
            raise ValueError("timestamp must be a non-negative Unix timestamp")

        latitude = _optional_float(sample, "latitude")
        longitude = _optional_float(sample, "longitude")
        altitude_amsl = _optional_float(sample, "altitude_amsl_m")
        home_altitude = _normalize_optional_number(
            home_altitude_amsl_m, "home_altitude_amsl_m"
        )
        position = None
        if all(value is not None for value in (latitude, longitude, altitude_amsl)):
            if home_altitude is None:
                altitude = altitude_amsl
                altitude_reference = AltitudeReference.MSL
            else:
                altitude = altitude_amsl - home_altitude
                altitude_reference = AltitudeReference.HOME_RELATIVE
            position = GeoPosition(
                latitude=latitude,
                longitude=longitude,
                altitude_m=altitude,
                altitude_reference=altitude_reference,
            )

        east = _optional_float(sample, "velocity_enu_x_m_s")
        north = _optional_float(sample, "velocity_enu_y_m_s")
        up = _optional_float(sample, "velocity_enu_z_m_s")
        velocity = None
        if all(value is not None for value in (east, north, up)):
            velocity = NedVelocity(
                north_m_s=north,
                east_m_s=east,
                down_m_s=-up,
            )

        heading = _heading_from_enu_quaternion(sample)
        navsat_status = _optional_int(sample, "navsat_status")
        gps_fix_type = None if navsat_status is None else (3 if navsat_status >= 0 else 1)

        covariance_type = _optional_int(sample, "position_covariance_type")
        covariance_xx = _optional_float(sample, "position_covariance_xx")
        gps_eph = None
        if covariance_type not in (None, 0) and covariance_xx is not None:
            if covariance_xx < 0:
                raise ValueError("position_covariance_xx must not be negative")
            gps_eph = sqrt(covariance_xx)

        battery_percentage = _optional_float(sample, "battery_percentage")
        if battery_percentage is not None:
            if not 0 <= battery_percentage <= 1:
                raise ValueError("battery_percentage must be between 0 and 1")
            battery_percentage *= 100

        return DroneState(
            drone_id=binding.drone_id,
            protocol=ProtocolType.AP_DDS,
            capabilities=AP_DDS_CAPABILITIES,
            position=position,
            velocity_ned_m_s=velocity,
            heading_deg=heading,
            gps_fix_type=gps_fix_type,
            satellites_visible=None,
            gps_eph_m=gps_eph,
            battery_voltage_v=_optional_float(sample, "battery_voltage_v"),
            battery_percent=battery_percentage,
            vibration=None,
            clipping_count=None,
            status=status,
            connection_status=self._connection_status(topic_age_s),
            observed_at=datetime.fromtimestamp(timestamp, tz=timezone.utc),
        )

    def _connection_status(self, topic_age_s: float | None) -> ConnectionStatus:
        age = _normalize_optional_number(topic_age_s, "topic_age_s")
        if age is None:
            return ConnectionStatus.DISCONNECTED
        if age < 0:
            raise ValueError("topic_age_s must be non-negative")
        if age <= self.topic_timeout_s:
            return ConnectionStatus.CONNECTED
        return ConnectionStatus.DISCONNECTED


class ApDdsTakeoffClient(Protocol):
    """Subset of ``services.simulation.app.ros2_drone.Ros2Drone`` used here."""

    def takeoff(self, altitude_m: float) -> None: ...


class ApDdsTakeoffRequest(ContractModel):
    """Translated request for the AP_DDS experimental takeoff service."""

    command_id: Identifier
    namespace: Identifier
    service_name: str = Field(min_length=1)
    altitude_home_relative_m: float = Field(gt=0, le=500)

    @model_validator(mode="after")
    def service_must_match_namespace(self) -> Self:
        expected = f"/{self.namespace}/{AP_DDS_TAKEOFF_SERVICE}"
        if self.service_name != expected:
            raise ValueError(f"service_name must be {expected}")
        return self

    def invoke(self, client: ApDdsTakeoffClient) -> None:
        """Reuse the injected simulation adapter instead of creating a ROS node."""
        client.takeoff(self.altitude_home_relative_m)


class UnsupportedApDdsCommandError(ValueError):
    """Raised when the AP_DDS prototype cannot represent a command."""


def to_ap_dds_takeoff_request(
    command: DroneCommand, binding: ApDdsBinding
) -> ApDdsTakeoffRequest:
    """Translate one common takeoff command to the AP_DDS service contract."""
    if command.protocol != ProtocolType.AP_DDS:
        raise UnsupportedApDdsCommandError("command protocol must be ap_dds")
    if command.drone_id != binding.drone_id:
        raise ValueError(
            f"command targets {command.drone_id}, but binding belongs to {binding.drone_id}"
        )
    if not isinstance(command.payload, TakeoffPayload):
        raise UnsupportedApDdsCommandError(
            "only the takeoff control request is implemented in this prototype"
        )
    if command.payload.altitude_reference != AltitudeReference.HOME_RELATIVE:
        raise UnsupportedApDdsCommandError(
            "the AP_DDS takeoff service accepts home-relative altitude only"
        )
    return ApDdsTakeoffRequest(
        command_id=command.command_id,
        namespace=binding.namespace,
        service_name=binding.takeoff_service,
        altitude_home_relative_m=command.payload.altitude_m,
    )


class ApDdsCommandLifecycle:
    """Keep AP_DDS service acceptance separate from vehicle completion."""

    def __init__(self, command: DroneCommand) -> None:
        if command.protocol != ProtocolType.AP_DDS:
            raise ValueError("ApDdsCommandLifecycle requires an ap_dds command")
        self.command = command
        self.status = CommandStatus.PENDING

    def mark_sent(self, *, at: datetime) -> CommandResult:
        return self._transition(CommandStatus.SENT, at=at)

    def handle_service_response(
        self, accepted: bool, *, at: datetime, message: str | None = None
    ) -> CommandResult:
        """Map a service response without treating acceptance as completion."""
        if not isinstance(accepted, bool):
            raise ValueError("accepted must be a boolean")
        if accepted:
            return self._transition(CommandStatus.ACCEPTED, at=at)
        return self._transition(
            CommandStatus.FAILED,
            at=at,
            error=CommandError(
                code="AP_DDS_SERVICE_REJECTED",
                message=message or "AP_DDS service rejected the command",
                retryable=False,
            ),
        )

    def mark_executing(self, *, at: datetime) -> CommandResult:
        return self._transition(CommandStatus.EXECUTING, at=at)

    def mark_completed(self, *, at: datetime) -> CommandResult:
        return self._transition(CommandStatus.SUCCEEDED, at=at)

    def mark_service_timeout(self, *, at: datetime) -> CommandResult:
        return self._transition(
            CommandStatus.TIMED_OUT,
            at=at,
            error=CommandError(
                code="AP_DDS_SERVICE_TIMEOUT",
                message="AP_DDS service did not respond before the deadline",
                retryable=True,
            ),
        )

    def mark_completion_timeout(self, *, at: datetime) -> CommandResult:
        return self._transition(
            CommandStatus.TIMED_OUT,
            at=at,
            error=CommandError(
                code="AP_DDS_COMPLETION_TIMEOUT",
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
            protocol=ProtocolType.AP_DDS,
            previous_status=self.status,
            status=status,
            updated_at=at,
            error=error,
        )
        self.status = status
        return result


def _heading_from_enu_quaternion(sample: Mapping[str, Any]) -> float | None:
    x = _optional_float(sample, "orientation_x")
    y = _optional_float(sample, "orientation_y")
    z = _optional_float(sample, "orientation_z")
    w = _optional_float(sample, "orientation_w")
    if not all(value is not None for value in (x, y, z, w)):
        return None
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm == 0:
        raise ValueError("orientation quaternion must not have zero length")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    yaw_enu = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return (90 - degrees(yaw_enu)) % 360


def _required_float(values: Mapping[str, Any], key: str) -> float:
    value = _optional_float(values, key)
    if value is None:
        raise ValueError(f"{key} is required")
    return value


def _optional_float(values: Mapping[str, Any], key: str) -> float | None:
    return _normalize_optional_number(values.get(key), key)


def _normalize_optional_number(value: Any, name: str) -> float | None:
    if value is None or value == "":
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric, None, or an empty string")
    normalized = float(value)
    if normalized != normalized:  # ROS sensor messages use NaN for unknown values.
        return None
    if not isfinite(normalized):
        raise ValueError(f"{name} must be finite")
    return normalized


def _optional_int(values: Mapping[str, Any], key: str) -> int | None:
    value = values.get(key)
    if value is None or value == "":
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be an integer, None, or an empty string")
    if not isfinite(value) or int(value) != value:
        raise ValueError(f"{key} must be an integer")
    return int(value)


def _is_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and isfinite(value)
    )
