"""Protocol adapter boundaries for the SkyBuddy middleware."""

from .ap_dds import (
    AP_DDS_CAPABILITIES,
    ApDdsBinding,
    ApDdsCommandLifecycle,
    ApDdsStateMapper,
    ApDdsTakeoffRequest,
    UnsupportedApDdsCommandError,
    to_ap_dds_takeoff_request,
)
from .drone_interface import (
    MIN_SAFE_ALTITUDE_M,
    DroneInterfaceClient,
    DroneInterfaceRequest,
    InterfaceMethod,
    UnsafeAltitudeError,
    UnsupportedDroneCommandError,
    to_drone_interface_request,
)
from .mavlink import (
    MAVLINK_CAPABILITIES,
    MavlinkAckResult,
    MavlinkCommandLifecycle,
    MavlinkDroneBinding,
    MavlinkStateMapper,
    MavlinkTakeoffRequest,
    UnsupportedMavlinkCommandError,
    to_mavlink_takeoff_request,
)
from .telemetry import TelemetryStateMapper

__all__ = [
    # AP-DDS
    "AP_DDS_CAPABILITIES",
    "ApDdsBinding",
    "ApDdsCommandLifecycle",
    "ApDdsStateMapper",
    "ApDdsTakeoffRequest",
    "UnsupportedApDdsCommandError",
    "to_ap_dds_takeoff_request",
    # MAVLink
    "MAVLINK_CAPABILITIES",
    "MavlinkAckResult",
    "MavlinkCommandLifecycle",
    "MavlinkDroneBinding",
    "MavlinkStateMapper",
    "MavlinkTakeoffRequest",
    "UnsupportedMavlinkCommandError",
    "to_mavlink_takeoff_request",
    # DroneInterface (protocol-neutral simulation adapter contract)
    "MIN_SAFE_ALTITUDE_M",
    "DroneInterfaceClient",
    "DroneInterfaceRequest",
    "InterfaceMethod",
    "UnsafeAltitudeError",
    "UnsupportedDroneCommandError",
    "to_drone_interface_request",
    # Common Telemetry mapping
    "TelemetryStateMapper",
]
