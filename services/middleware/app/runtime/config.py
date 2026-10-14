"""Tunable runtime settings read from the registry configuration file."""

from pydantic import Field

from app.schemas.mission import ContractModel


class DispatcherConfig(ContractModel):
    """Arrival rules for ``goto`` legs.

    A leg fails with ``WAYPOINT_TIMEOUT`` when the drone does not come within
    ``reach_threshold_m`` of the target in time. The allowed time is the larger of
    ``waypoint_timeout_s`` and ``leg distance / min_ground_speed_m_s``, so long transit
    legs (home -> area, area -> home) are not cut off by the per-waypoint default.
    """

    reach_threshold_m: float = Field(
        default=3.0, gt=0, le=50, description="Arrival radius around a waypoint (m)."
    )
    waypoint_timeout_s: float = Field(
        default=60.0, gt=0, le=3_600, description="Minimum time allowed for any goto leg (s)."
    )
    min_ground_speed_m_s: float = Field(
        default=2.0,
        gt=0,
        le=30,
        description=(
            "Slowest expected horizontal speed (m/s). Measured SITL speeds were "
            "2.7~4.8 m/s, so 2.0 leaves margin."
        ),
    )

    def leg_timeout_s(self, distance_m: float | None) -> float:
        """Time allowed to fly ``distance_m``; the default applies when it is unknown."""
        if distance_m is None:
            return self.waypoint_timeout_s
        return max(self.waypoint_timeout_s, distance_m / self.min_ground_speed_m_s)
