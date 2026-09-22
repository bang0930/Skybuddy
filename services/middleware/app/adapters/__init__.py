"""Protocol adapter boundaries for the SkyBuddy middleware."""

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

__all__ = [
    "MAVLINK_CAPABILITIES",
    "MavlinkAckResult",
    "MavlinkCommandLifecycle",
    "MavlinkDroneBinding",
    "MavlinkStateMapper",
    "MavlinkTakeoffRequest",
    "UnsupportedMavlinkCommandError",
    "to_mavlink_takeoff_request",
]
