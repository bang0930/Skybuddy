"""Split a search area into sub-areas of equal size with north-south cuts."""

from pydantic import ValidationError
from shapely.geometry import Polygon, box

from app.schemas import SearchArea

from .coverage import _to_geo
from .waypoints import _local_polygon

MAX_PIECES = 10
# Pieces smaller than this (m²) are slivers left by the cut line, not real parts.
_MIN_PART_AREA_M2 = 1e-6
_SEARCH_ITERATIONS = 50


class AreaSplitError(ValueError):
    """Raised when an area cannot be split; ``code`` is reported to the LLM."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def split_area(area: SearchArea, n_pieces: int) -> list[SearchArea]:
    """Cut ``area`` into ``n_pieces`` equal-area strips, ordered west to east.

    Sub-areas are named ``<area_id>-1`` (westmost) to ``<area_id>-<n>`` and keep the
    parent's search altitude. Areas are measured in local meters, not degrees, because a
    degree of longitude is shorter than a degree of latitude away from the equator.
    """
    if not 2 <= n_pieces <= MAX_PIECES:
        raise AreaSplitError(
            "INVALID_N_PIECES", f"n_pieces must be between 2 and {MAX_PIECES}; got {n_pieces}"
        )
    polygon, ref_lat, ref_lon = _local_polygon(area)
    if not polygon.is_valid:
        raise AreaSplitError(
            "INVALID_AREA_BOUNDARY", f"{area.area_id} boundary is not a simple polygon"
        )

    minx, miny, maxx, maxy = polygon.bounds
    total = polygon.area
    cuts = [minx]
    cuts += [_find_cut_x(polygon, total * k / n_pieces) for k in range(1, n_pieces)]
    cuts.append(maxx)

    pieces = []
    for index in range(n_pieces):
        strip = box(cuts[index], miny, cuts[index + 1], maxy)
        piece = _single_polygon(polygon.intersection(strip))
        if piece is None:
            raise AreaSplitError(
                "SPLIT_NOT_CONTIGUOUS",
                f"piece {index + 1} of {area.area_id} falls apart into several parts; "
                "use fewer pieces or a convex area",
            )
        pieces.append(piece)

    try:
        return [
            SearchArea(
                area_id=f"{area.area_id}-{index}",
                boundary=[_to_geo(x, y, ref_lat, ref_lon) for x, y in piece.exterior.coords[:-1]],
                search_altitude_m=area.search_altitude_m,
                search_altitude_reference=area.search_altitude_reference,
            )
            for index, piece in enumerate(pieces, start=1)
        ]
    except ValidationError as exc:
        # e.g. a sub-area id over 64 characters or a boundary over 100 vertices
        raise AreaSplitError("SPLIT_FAILED", str(exc.errors()[0]["msg"])) from None


def _find_cut_x(polygon: Polygon, target_area: float) -> float:
    """The x where the part of ``polygon`` west of it has ``target_area`` (binary search)."""
    minx, miny, maxx, maxy = polygon.bounds
    lo, hi = minx, maxx
    for _ in range(_SEARCH_ITERATIONS):
        mid = (lo + hi) / 2
        if polygon.intersection(box(minx, miny, mid, maxy)).area < target_area:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def _single_polygon(geom) -> Polygon | None:
    """The one polygon in an intersection result, or None if it has zero or several parts."""
    if geom.geom_type == "Polygon":
        return geom
    parts = [
        g for g in getattr(geom, "geoms", [])
        if g.geom_type == "Polygon" and g.area > _MIN_PART_AREA_M2
    ]
    return parts[0] if len(parts) == 1 else None
