from shapely.geometry import Polygon, LineString
import math



def _only_lines(geom):
    if geom.geom_type == "LineString":
        return [geom]
    if geom.geom_type in ("MultiLineString", "GeometryCollection"):
        return [g for g in geom.geoms if g.geom_type == "LineString"]
    return []

def generate_sweep_waypoints(polygon, spacing):
    minx, miny, maxx, maxy = polygon.bounds
    waypoints = []
    y = miny + spacing / 2
    row =0
    while y < maxy:
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

        y += spacing
        row += 1
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

square = Polygon([(0, 0), (0, 100), (100, 100), (100, 0)])
print(generate_sweep_waypoints(square, 20))

L = Polygon([(0, 0), (0, 100), (50, 100), (50, 50), (100, 50), (100, 0)])
print("L 20:", generate_sweep_waypoints(L, 20))
print("L 25:", generate_sweep_waypoints(L, 25))

U = Polygon([(0, 0), (0, 100), (30, 100), (30, 40), (70, 40), (70, 100), (100, 100), (100, 0)])
print("U 20:", generate_sweep_waypoints(U, 20))

wp = split_long_legs(generate_sweep_waypoints(square, 20), 40)
print(wp)
print(max(math.dist(a, b) for a, b in zip(wp, wp[1:])))