"""Tunable runtime settings read from the registry configuration file."""

from typing import Literal

from pydantic import Field

from app.schemas.mission import ContractModel

from .waypoints import (
    DEFAULT_MAX_LEG_M,
    DEFAULT_SIDE_OVERLAP,
    CoverageGenerator,
    VertexTraversalGenerator,
    WaypointGenerator,
)


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


class WaypointConfig(ContractModel):
    """How each search area becomes waypoints.

    The coverage generator also needs every drone's ``swath_width_m`` (drone entry of
    the same file). ``RegistryConfig`` checks that and the ``max_leg_m`` limit at load.
    """

    generator: Literal["coverage", "vertex_traversal"] = Field(
        default="coverage",
        description="coverage: back-and-forth rows; vertex_traversal: earlier stand-in.",
    )
    side_overlap: float = Field(
        default=DEFAULT_SIDE_OVERLAP,
        ge=0,
        lt=1,
        description="Overlap between neighboring coverage rows (coverage only).",
    )
    max_leg_m: float = Field(
        default=DEFAULT_MAX_LEG_M,
        gt=0,
        le=10_000,
        description="Longest leg between neighboring area waypoints (m).",
    )

    def build_generator(self) -> WaypointGenerator:
        if self.generator == "vertex_traversal":
            return VertexTraversalGenerator(max_leg_m=self.max_leg_m)
        return CoverageGenerator(side_overlap=self.side_overlap, max_leg_m=self.max_leg_m)
