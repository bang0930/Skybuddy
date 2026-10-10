from shapely.geometry import Polygon, LineString
import math

M_PER_DEG_LAT = 111320

def latlon_to_xy(lat, lon, ref_lat, ref_lon):
    x = (lon - ref_lon) * M_PER_DEG_LAT * math.cos(math.radians(ref_lat))
    y = (lat - ref_lat) * M_PER_DEG_LAT

    return x, y

def xy_to_latlon(x, y, ref_lat, ref_lon):
    lat = ref_lat + y/M_PER_DEG_LAT
    lon = ref_lon + x / (M_PER_DEG_LAT * math.cos(math.radians(ref_lat)))
    return lat, lon

def _only_lines(geom):
    if geom.geom_type == "LineString":
        return [geom]
    if geom.geom_type in ("MultiLineString", "GeometryCollection"):
        return [g for g in geom.geoms if g.geom_type == "LineString"]
    return []

def generate_sweep_waypoints(polygon, swath ,spacing):
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

def split_long_legs(waypoints, max_leg):
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


def generate_coverage_waypoints(boundary, swath_width_m, side_overlap=0.2, max_leg_m=100):

    ref_lat = sum(lat for lat, lon in boundary) / len(boundary)
    ref_lon = sum(lon for lat, lon in boundary) / len(boundary)

    xy = [latlon_to_xy(lat, lon, ref_lat, ref_lon) for lat, lon in boundary]
    polygon = Polygon(xy)

    spacing = swath_width_m * (1 - side_overlap)
    local = generate_sweep_waypoints(polygon, swath_width_m, spacing)
    local = split_long_legs(local, max_leg_m)

    return [xy_to_latlon(x, y, ref_lat, ref_lon) for x, y in local]


square = Polygon([(0, 0), (0, 100), (100, 100), (100, 0)])
L = Polygon([(0, 0), (0, 100), (50, 100), (50, 50), (100, 50), (100, 0)])
U = Polygon([(0, 0), (0, 100), (30, 100), (30, 40), (70, 40), (70, 100), (100, 100), (100, 0)])

# 1) 지그재그 (swath = spacing이면 예전 결과와 같아야 함)
print("사각형 20:", generate_sweep_waypoints(square, 20, 20))
print("L 20:", generate_sweep_waypoints(L, 20, 20))
print("L 25:", generate_sweep_waypoints(L, 25, 25))
print("U 20:", generate_sweep_waypoints(U, 20, 20))

# 2) 긴 구간 분할
wp = split_long_legs(generate_sweep_waypoints(square, 20, 20), 40)
print("분할 최대 구간:", max(math.dist(a, b) for a, b in zip(wp, wp[1:])))   # 40 이하

# 3) 위경도 변환 왕복
ref = (-35.3633, 149.1652)
x, y = latlon_to_xy(-35.3630, 149.1655, *ref)
print("변환:", x, y)                      # 약 27.2, 33.4
print("복원:", xy_to_latlon(x, y, *ref))  # (-35.363, 149.1655)

# 4) 캔버라 근처 200m 구역
boundary = [(-35.3633, 149.1652), (-35.3633, 149.1674),
            (-35.3615, 149.1674), (-35.3615, 149.1652)]
pts = generate_coverage_waypoints(boundary, 60)
print("웨이포인트 수:", len(pts))                # 12
print("첫 점:", pts[0])                          # 위도 약 -35.36303, 경도 149.1652

ref = pts[0]
xy = [latlon_to_xy(lat, lon, *ref) for lat, lon in pts]
print("최대 구간(m):", max(math.dist(a, b) for a, b in zip(xy, xy[1:])))