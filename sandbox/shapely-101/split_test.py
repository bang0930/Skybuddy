"""이슈 #16: Shapely로 다각형 탐색 구역을 n개 조각으로 나누는 실험.

두 가지 분할 방식을 비교한다.
  - split_polygon_into_strips: x축 방향 폭을 n등분해서 자른다.
  - split_polygon_equal_area:  각 조각의 면적이 같아지도록 자른다.

L자처럼 오목한 다각형을 세로선으로 자르면, 그 선이 다각형의 변과
겹쳐서 intersection 결과가 (Polygon + LineString)이 섞인
GeometryCollection으로 나올 수 있다. _only_polygons()는 여기서
면적이 0인 LineString을 버리고 Polygon/MultiPolygon만 남긴다.

사용법:
    cd sandbox/shapely-101
    venv/bin/python split_test.py
"""

from shapely.geometry import Polygon, box
from shapely.ops import unary_union


def _only_polygons(geom):
    """intersection 결과에서 면적이 있는 부분(Polygon/MultiPolygon)만 남긴다.

    컷 라인이 다각형의 변과 겹치면 결과가 GeometryCollection이 되는데,
    그 안에 섞인 LineString(면적 0)은 버리고 도형만 골라낸다.
    아무 도형도 없으면 None을 반환한다.
    """
    if geom.geom_type in ("Polygon", "MultiPolygon"):
        return geom

    polys = [g for g in geom.geoms if g.geom_type in ("Polygon", "MultiPolygon")]

    if len(polys) == 0:
        return None
    if len(polys) == 1:
        return polys[0]
    return unary_union(polys)


def split_polygon_into_strips(polygon, n_pieces):
    """x축 방향 폭을 n등분해서 자른다. 오목한 도형이면 조각 면적이 불균등할 수 있다."""
    minx, miny, maxx, maxy = polygon.bounds
    strip_width = (maxx - minx) / n_pieces

    pieces = []
    for i in range(n_pieces):
        strip = box(minx + i * strip_width, miny, minx + (i + 1) * strip_width, maxy)
        piece = _only_polygons(polygon.intersection(strip))
        if piece is not None:
            pieces.append(piece)
    return pieces


def _area_left_of(polygon, x):
    """다각형에서 x 왼쪽에 있는 부분의 면적."""
    minx, miny, _, maxy = polygon.bounds
    left = box(minx, miny, x, maxy)
    return polygon.intersection(left).area


def _find_cut_x(polygon, target_area, iterations=50):
    """왼쪽 누적 면적이 target_area가 되는 x를 이진 탐색으로 찾는다.

    _area_left_of(polygon, x)는 x가 커질수록 단조 증가하므로,
    각 반복마다 목표보다 작으면 오른쪽 절반을, 크거나 같으면
    왼쪽 절반을 남기는 방식으로 구간을 절반씩 좁혀 나간다.
    """
    minx, _, maxx, _ = polygon.bounds
    lo, hi = minx, maxx
    for _ in range(iterations):
        mid = (lo + hi) / 2
        if _area_left_of(polygon, mid) < target_area:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def split_polygon_equal_area(polygon, n_pieces):
    """각 조각의 면적이 (거의) 같아지도록 자른다."""
    minx, miny, maxx, maxy = polygon.bounds
    total_area = polygon.area

    cuts = [minx]
    for k in range(1, n_pieces):
        cuts.append(_find_cut_x(polygon, total_area * k / n_pieces))
    cuts.append(maxx)

    pieces = []
    for i in range(n_pieces):
        strip = box(cuts[i], miny, cuts[i + 1], maxy)
        piece = _only_polygons(polygon.intersection(strip))
        if piece is not None:
            pieces.append(piece)
    return pieces


def _print_pieces(label, pieces):
    print(f"  [{label}]")
    for i, p in enumerate(pieces):
        print(f"    조각 {i}: 면적 = {p.area:.2f}, 타입 = {p.geom_type}")


if __name__ == "__main__":
    square = Polygon([(0, 0), (0, 10), (10, 10), (10, 0)])
    print("정사각형 (전체 면적 100.00), 2조각")
    _print_pieces("폭 균등", split_polygon_into_strips(square, 2))
    _print_pieces("면적 균등", split_polygon_equal_area(square, 2))

    l_shape = Polygon([(0, 0), (0, 10), (5, 10), (5, 5), (10, 5), (10, 0)])
    print(f"\nL자 도형 (전체 면적 {l_shape.area:.2f})")

    print("\n2조각")
    _print_pieces("폭 균등", split_polygon_into_strips(l_shape, 2))
    _print_pieces("면적 균등", split_polygon_equal_area(l_shape, 2))

    print("\n3조각")
    _print_pieces("폭 균등", split_polygon_into_strips(l_shape, 3))
    _print_pieces("면적 균등", split_polygon_equal_area(l_shape, 3))
