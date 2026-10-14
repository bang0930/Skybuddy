"""Back-and-forth (boustrophedon) coverage waypoints for a polygonal search area."""
import math

from shapely.geometry import LineString, Polygon

from app.schemas import GeoCoordinate

_METERS_PER_DEGREE_LAT = 111_320.0

def _to_xy(lat, lon, ref_lat, ref_lon):
    x = (lon - ref_lon) * _METERS_PER_DEGREE_LAT * math.cos(math.radians(ref_lat))
    y = (lat - ref_lat) * _METERS_PER_DEGREE_LAT
    return x, y

def _to_geo(x, y, ref_lat, ref_lon):
    lat = ref_lat + y/_METERS_PER_DEGREE_LAT
    lon = ref_lon + x / (_METERS_PER_DEGREE_LAT* math.cos(math.radians(ref_lat)))
    return GeoCoordinate(latitude=round(lat, 7), longitude=round(lon, 7))


def _only_lines(geom):
    if geom.geom_type == "LineString":
        return [geom]
    if geom.geom_type in ("MultiLineString", "GeometryCollection"):
        return [g for g in geom.geoms if g.geom_type == "LineString"]
    return []

def _split_long_legs(waypoints, max_leg):
    if not waypoints:
        return []

    result = [waypoints[0]]
    for prev_point, cur_point in zip(waypoints, waypoints[1:]):
        dist = math.dist(prev_point, cur_point)
        n = math.ceil(dist/ max_leg)
        for k in range(1, n):
            t = k/n
            x = prev_point[0]+ (cur_point[0] - prev_point[0]) * t
            y = prev_point[1] + (cur_point[1] - prev_point[1]) * t
            result.append((x, y))
        result.append(cur_point)
    return result

def _sweep_rows(polygon, swath ,spacing):
    """Rows of the polygon in alternating direction, in local meters."""
    minx, miny, maxx, maxy = polygon.bounds
    waypoints = []
    extent = maxy - miny
    if extent <= swath:
        ys = [(miny + maxy) / 2]
    else:
        n = math.ceil((extent - swath) / spacing) +1
        step = (extent - swath) / (n - 1)
        ys = [miny + swath / 2 + i *step for i in range(n)]

    for row, y in enumerate(ys):
        line = LineString([(minx-1, y), (maxx + 1 , y)])
        segments = _only_lines(polygon.intersection(line))
        segments.sort(key=lambda s:min(s.coords))
        if row % 2 == 1:
            segments.reverse()
        for seg in segments:
            left, right = min(seg.coords), max(seg.coords)
            if row % 2 == 0:
                waypoints += [left, right]

            else:
                waypoints += [right, left]
    if not waypoints:
        return []      
    cleaned = [waypoints[0]]
    for p in waypoints[1:]:
        if p != cleaned[-1]:
            cleaned.append(p)

    return cleaned


def generate_coverage_waypoints(
        boundary: list[GeoCoordinate],
        swath_width_m:float,
        side_overlap: float = 0.2,
        max_leg_m: float = 100.0,
) -> list[GeoCoordinate]:
    if swath_width_m <= 0:
        raise ValueError("swath_width_m must be positive")
    if not 0 <= side_overlap < 1:
        raise ValueError("side_overlap must be in [0, 1)")
    if max_leg_m <= 0:
        raise ValueError("max_leg_m must be positive")
    if len(boundary) < 3:
        raise ValueError("boundary needs at least 3 vertices")

    
    ref_lat = sum(p.latitude for p in boundary) / len(boundary)
    ref_lon = sum(p.longitude for p in boundary) / len(boundary)
    xy = [_to_xy(p.latitude, p.longitude, ref_lat, ref_lon) for p in boundary]
    polygon = Polygon(xy)
    if not polygon.is_valid:
        raise ValueError("boundary vertices must trace the outline in order")

    spacing = swath_width_m*(1-side_overlap)
    rows = _sweep_rows(polygon, swath_width_m, spacing)

    legs = _split_long_legs(rows, max_leg_m)
    return [_to_geo(x, y, ref_lat, ref_lon) for x, y in legs]


def split_long_legs(points: list[GeoCoordinate], max_leg_m: float) -> list[GeoCoordinate]:
    """Insert evenly spaced points so no leg between neighbors exceeds max_leg_m."""
    if max_leg_m <= 0:
        raise ValueError("max_leg_m must be positive")
    if not points:
        return []

    result = [points[0]]
    for prev, cur in zip(points, points[1:]):
        dx, dy = _to_xy(cur.latitude, cur.longitude, prev.latitude, prev.longitude)
        n = math.ceil(math.hypot(dx, dy) / max_leg_m)
        for k in range(1, n):
            t = k / n
            lat = prev.latitude + (cur.latitude - prev.latitude) * t
            lon = prev.longitude + (cur.longitude - prev.longitude) * t
            result.append(GeoCoordinate(latitude=round(lat, 7), longitude=round(lon, 7)))
        result.append(cur)
    return result