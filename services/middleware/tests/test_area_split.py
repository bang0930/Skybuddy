"""Tests for splitting a search area into equal-area sub-areas."""

import pytest
from conftest import scenario

from app.runtime.area_split import AreaSplitError, split_area
from app.runtime.waypoints import _local_polygon, boundary_problem
from app.schemas import SearchArea


def _area_m2(area: SearchArea) -> float:
    return _local_polygon(area)[0].area


def _box_area(area_id: str, points: list[tuple[float, float]]) -> SearchArea:
    return SearchArea(
        area_id=area_id,
        boundary=[{"latitude": lat, "longitude": lon} for lat, lon in points],
        search_altitude_m=30,
        search_altitude_reference="home_relative",
    )


@pytest.mark.parametrize("n_pieces", [2, 3, 4])
def test_pieces_have_equal_area_and_keep_altitude(n_pieces) -> None:
    north = scenario().search_areas[0]

    pieces = split_area(north, n_pieces)

    assert [p.area_id for p in pieces] == [f"area-north-{i}" for i in range(1, n_pieces + 1)]
    total = _area_m2(north)
    for piece in pieces:
        assert _area_m2(piece) == pytest.approx(total / n_pieces, rel=0.01)
        assert boundary_problem(piece) is None
        assert piece.search_altitude_m == north.search_altitude_m


def test_pieces_are_ordered_west_to_east() -> None:
    pieces = split_area(scenario().search_areas[0], 3)

    west_edges = [min(point.longitude for point in p.boundary) for p in pieces]
    assert west_edges == sorted(west_edges)


@pytest.mark.parametrize("n_pieces", [1, 11])
def test_rejects_piece_count_out_of_range(n_pieces) -> None:
    with pytest.raises(AreaSplitError) as error:
        split_area(scenario().search_areas[0], n_pieces)

    assert error.value.code == "INVALID_N_PIECES"


def test_rejects_piece_that_falls_apart() -> None:
    # A U shape opening east: the eastern strip crosses both arms.
    u_shape = _box_area(
        "area-u",
        [
            (0, 0), (0, 0.0003), (0.0001, 0.0003), (0.0001, 0.0001),
            (0.0002, 0.0001), (0.0002, 0.0003), (0.0003, 0.0003), (0.0003, 0),
        ],
    )

    with pytest.raises(AreaSplitError) as error:
        split_area(u_shape, 2)

    assert error.value.code == "SPLIT_NOT_CONTIGUOUS"
