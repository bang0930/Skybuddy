"""Interchangeable search-area waypoint generators and the checks every path must pass.

- ``CoverageGenerator`` (default): back-and-forth rows from ``coverage.py``.
- ``VertexTraversalGenerator``: the earlier stand-in that visits the boundary vertices.

Both run in the same local plane as ``coverage.py`` (equirectangular around the vertex
centroid), so a boundary accepted by ``boundary_problem`` is also accepted by the
coverage generator.
"""

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import ClassVar, Protocol

from shapely.geometry import Point, Polygon
from shapely.validation import explain_validity

from app.schemas import AltitudeReference, GeoCoordinate, GeoPosition, SearchArea

from .coverage import _to_xy, generate_coverage_waypoints, split_long_legs

# Same as the defaults of ``generate_coverage_waypoints``.
DEFAULT_SIDE_OVERLAP = 0.2
DEFAULT_MAX_LEG_M = 100.0

# Waypoints are rounded to 1e-7 degrees (about 1 cm), so a point generated on the
# boundary, or a leg split to exactly max_leg_m, can overshoot by that much.
ROUNDING_TOLERANCE_M = 0.05


class WaypointCheckError(ValueError):
    """The generated area path cannot be flown; ``problems`` lists every reason."""

    def __init__(self, problems: list[str]) -> None:
        super().__init__("; ".join(problems))
        self.problems = problems


class WaypointGenerator(Protocol):
    """Turn a search area into the waypoints flown at its search altitude."""

    @property
    def name(self) -> str: ...

    @property
    def max_leg_m(self) -> float: ...

    def generate(
        self, area: SearchArea, *, swath_width_m: float | None
    ) -> list[GeoCoordinate]: ...


@dataclass(frozen=True)
class VertexTraversalGenerator:
    """Fly the polygon perimeter and close the loop at the first vertex.

    Edges longer than ``max_leg_m`` are split so the path passes the same leg check as
    coverage. ``swath_width_m`` is not used.
    """

    max_leg_m: float = DEFAULT_MAX_LEG_M
    name: ClassVar[str] = "vertex_traversal"

    def generate(
        self, area: SearchArea, *, swath_width_m: float | None = None
    ) -> list[GeoCoordinate]:
        return split_long_legs([*area.boundary, area.boundary[0]], self.max_leg_m)


@dataclass(frozen=True)
class CoverageGenerator:
    """Back-and-forth rows spaced by the drone's swath width."""

    side_overlap: float = DEFAULT_SIDE_OVERLAP
    max_leg_m: float = DEFAULT_MAX_LEG_M
    name: ClassVar[str] = "coverage"

    def generate(
        self, area: SearchArea, *, swath_width_m: float | None
    ) -> list[GeoCoordinate]:
        if swath_width_m is None:
            raise ValueError("coverage waypoints need the drone's swath_width_m")
        return generate_coverage_waypoints(
            area.boundary,
            swath_width_m,
            side_overlap=self.side_overlap,
            max_leg_m=self.max_leg_m,
        )


def boundary_problem(area: SearchArea) -> str | None:
    """Why the boundary is not a simple polygon (e.g. crossing edges), or None."""
    polygon, _, _ = _local_polygon(area)
    if polygon.is_valid:
        return None
    return explain_validity(polygon)


def path_problems(
    area: SearchArea, targets: Sequence[GeoPosition], *, max_leg_m: float
) -> list[str]:
    """Check that the area path stays in the area, at search altitude, in short legs."""
    if not targets:
        return [f"no waypoints were generated for {area.area_id}"]
    polygon, ref_lat, ref_lon = _local_polygon(area)
    problems = []
    for index, target in enumerate(targets):
        gap = polygon.distance(Point(_to_xy(target.latitude, target.longitude, ref_lat, ref_lon)))
        if gap > ROUNDING_TOLERANCE_M:
            problems.append(f"waypoint {index} is {gap:.2f}m outside {area.area_id}")
        if (
            target.altitude_reference != AltitudeReference.HOME_RELATIVE
            or target.altitude_m != area.search_altitude_m
        ):
            problems.append(
                f"waypoint {index} altitude {target.altitude_m}m "
                f"{target.altitude_reference.value} differs from the search altitude "
                f"{area.search_altitude_m}m home_relative"
            )
    for index, (previous, current) in enumerate(zip(targets, targets[1:]), 1):
        leg = math.hypot(
            *_to_xy(current.latitude, current.longitude, previous.latitude, previous.longitude)
        )
        if leg > max_leg_m + ROUNDING_TOLERANCE_M:
            problems.append(
                f"leg to waypoint {index} is {leg:.1f}m, longer than max_leg_m {max_leg_m}m"
            )
    return problems


def _local_polygon(area: SearchArea) -> tuple[Polygon, float, float]:
    """The boundary in local meters around the vertex centroid, as ``coverage.py`` does."""
    ref_lat = sum(point.latitude for point in area.boundary) / len(area.boundary)
    ref_lon = sum(point.longitude for point in area.boundary) / len(area.boundary)
    polygon = Polygon(
        [_to_xy(point.latitude, point.longitude, ref_lat, ref_lon) for point in area.boundary]
    )
    return polygon, ref_lat, ref_lon
