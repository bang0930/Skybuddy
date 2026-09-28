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
]