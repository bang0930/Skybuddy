"""Tests for the coverage waypoint generator."""

import math
import pytest

from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

from app.runtime.coverage import _to_xy, generate_coverage_waypoints
from app.schemas import GeoCoordinate as G
from app.runtime.coverage import split_long_legs




# ~200 m x 200 m area near the simulator's default position.
BOUNDARY = [
    G(latitude=-35.3633, longitude=149.1652),
    G(latitude=-35.3633, longitude=149.1674),
    G(latitude=-35.3615, longitude=149.1674),
    G(latitude=-35.3615, longitude=149.1652),
]


def test_waypoint_count():
    points = generate_coverage_waypoints(BOUNDARY, 60)
    assert len(points) == 12


def test_same_input_gives_same_output():
    assert generate_coverage_waypoints(BOUNDARY, 60) == generate_coverage_waypoints(BOUNDARY, 60)


def test_no_leg_exceeds_max_leg():
    points = generate_coverage_waypoints(BOUNDARY, 60, max_leg_m=100)
    legs = [
        math.hypot(*_to_xy(b.latitude, b.longitude, a.latitude, a.longitude))
        for a, b in zip(points, points[1:])
    ]
    assert max(legs) <= 100

BOWTIE = [BOUNDARY[0], BOUNDARY[2], BOUNDARY[1], BOUNDARY[3]]   # 대각선 순서로 꼬인 경계


@pytest.mark.parametrize(
    "boundary, swath, overlap, max_leg",
    [
        (BOUNDARY, 0, 0.2, 100),        # 촬영 폭 0
        (BOUNDARY, 60, 1.5, 100),       # 겹침 범위 초과
        (BOUNDARY, 60, 0.2, 0),         # 구간 상한 0
        (BOUNDARY[:2], 60, 0.2, 100),   # 꼭짓점 2개
        (BOWTIE, 60, 0.2, 100),         # 꼬인 경계
    ],
)
def test_rejects_invalid_input(boundary, swath, overlap, max_leg):
    with pytest.raises(ValueError):
        generate_coverage_waypoints(boundary, swath, overlap, max_leg)


HOME = G(latitude=-35.3633, longitude=149.1652)


def test_split_keeps_endpoints_and_adds_midpoint():
    far = G(latitude=-35.3624, longitude=149.1652)   # about 100.2 m north
    result = split_long_legs([HOME, far], 100)
    assert result[0] == HOME and result[-1] == far
    assert len(result) == 3


def test_split_leaves_short_legs_alone():
    near = G(latitude=-35.3629, longitude=149.1652)  # about 44.5 m north
    assert split_long_legs([HOME, near], 100) == [HOME, near]

def test_rows_cover_the_whole_area():
    swath = 60
    points = generate_coverage_waypoints(BOUNDARY, swath)

    ref_lat = sum(p.latitude for p in BOUNDARY) / len(BOUNDARY)
    ref_lon = sum(p.longitude for p in BOUNDARY) / len(BOUNDARY)
    polygon = Polygon([_to_xy(p.latitude, p.longitude, ref_lat, ref_lon) for p in BOUNDARY])
    xy = [_to_xy(p.latitude, p.longitude, ref_lat, ref_lon) for p in points]

    strips = [
        LineString([a, b]).buffer(swath / 2, cap_style="flat")
        for a, b in zip(xy, xy[1:])
        if abs(a[1] - b[1]) < 0.05          # keep only the legs along a row
    ]
    uncovered = polygon.difference(unary_union(strips))
    assert uncovered.area < 0.001 * polygon.area