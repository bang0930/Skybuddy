"""탐색 구역(섹션)을 정하고, 맵 측정 결과로 안전한지 검사한 뒤 미들웨어 임무 파일로 쓴다.

  python3 make_sections.py                       최신 map_probe 결과로 검사 -> scenarios/missions/
  python3 make_sections.py --probe <파일.json>    특정 측정 결과로 검사

구역은 AirSim 원점(기체 출발점) 기준 북·동 m 로 정의하고, 미들웨어와 같은 식
(위도 1도 = 111320m, 경도는 cos(위도)를 곱함)으로 위경도로 바꾼다. 탐색 고도는 홈(이륙 지점) 기준이다.

검사 (map_probe 결과 사용, 높이는 홈 지면 기준)
  1) 구역 안과 둘레 한 칸(격자 간격)의 가장 높은 물체가 탐색 고도보다 --margin 이상 낮을 것
  2) 두 기체 출발점에서 구역 네 꼭짓점까지 직선 경로 양옆 한 칸 안도 같은 조건
     미들웨어는 이륙 -> 구역 웨이포인트 -> 홈 복귀를 전부 탐색 고도로 직선 비행한다.
  격자(25m) 사이의 좁은 물체는 놓칠 수 있어 여유를 둔다.

결과
  scenarios/missions/mission-101.json   (미들웨어 MIDDLEWARE_SCENARIOS_DIR 로 지정해 쓴다)
  scenarios/missions/mission-001.json   (팀 저장소에서 복사. check_mvp_pipeline.py 가 이것을 쓴다)
"""
import argparse
import glob
import json
import math
import os
import shutil
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
# 개발 환경(평면 구조)은 이 폴더, 팀 저장소는 scripts/ 의 상위(services/simulation)가 기준이다.
BASE = _HERE if os.path.exists(os.path.join(_HERE, "kpi_mission.py")) else os.path.dirname(_HERE)
LOG_DIR = os.path.expanduser("~/skybuddy/logs")
OUT_DIR = os.path.join(BASE, "scenarios", "missions")
_MISSION_001 = [os.path.join(os.path.dirname(BASE), "middleware", "config", "scenarios", "mission-001.json"),
                os.path.expanduser("~/skybuddy-team/services/middleware/config/scenarios/mission-001.json")]
TEAM_MISSION_001 = next((p for p in _MISSION_001 if os.path.exists(p)), _MISSION_001[-1])
M_PER_DEG = 111320.0          # 미들웨어 coverage.py·planning 과 같은 값
MISSION_ID = "mission-101"

# 2026-11-09 산악맵 측정(map_probe): 홈 북동쪽은 지형이 홈보다 20~45m 낮아지는 열린 지대이고,
# 서쪽(30~60m)과 홈 남쪽(언덕, 출발점보다 20m 이상 높음)은 높다. 그래서 섹터는 북동쪽에 2x2 로 둔다.
# 이름은 sector-행-열: 행 0 = 홈에 가까운 남쪽 줄, 1 = 먼 북쪽 줄 / 열 0 = 서쪽, 1 = 동쪽.
# 한 변 125m: 촬영 폭 60m(30m 고도, 5주차 실측)로 지그재그 3줄이 나온다 (100m 면 2줄뿐이다).
# 고도를 섹터마다 10m 씩 다르게 해서, 어느 드론이 어느 섹터를 맡아도 두 드론의 비행 고도가 겹치지 않는다.
SECTIONS = [
    {"id": "sector-0-0", "south": 50, "north": 175, "west": 50, "east": 175, "alt": 30},
    {"id": "sector-0-1", "south": 50, "north": 175, "west": 175, "east": 300, "alt": 40},
    {"id": "sector-1-0", "south": 175, "north": 300, "west": 50, "east": 175, "alt": 50},
    {"id": "sector-1-1", "south": 175, "north": 300, "west": 175, "east": 300, "alt": 60},
]
LETTERS = "ABCDEFGH"


def latest_probe():
    files = sorted(glob.glob(os.path.join(LOG_DIR, "map_probe_*.json")), key=os.path.getmtime)
    return files[-1] if files else None


def height(value):
    """측정값 -> 비교용 숫자. 비어 있음(null)은 -inf, 너무 높음·실패는 inf(안전하지 않음으로 본다)."""
    if value is None:
        return -math.inf
    if isinstance(value, (int, float)):
        return float(value)
    return math.inf


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def to_geo(origin, north, east):
    lat0, lon0 = origin[0], origin[1]
    lat = lat0 + north / M_PER_DEG
    lon = lon0 + east / (M_PER_DEG * math.cos(math.radians(lat0)))
    return {"latitude": round(lat, 7), "longitude": round(lon, 7)}


def fmt(h):
    if h == -math.inf:
        return "없음"
    if h == math.inf:
        return "측정 한도 초과"
    return f"{h:.1f}m"


def main(argv=None):
    parser = argparse.ArgumentParser(description="탐색 구역 검사·임무 파일 생성")
    parser.add_argument("--probe", help="map_probe 결과 JSON (기본: logs 의 가장 최근 파일)")
    parser.add_argument("--margin", type=float, default=15.0,
                        help="탐색 고도와 가장 높은 물체 사이에 둘 최소 여유 m (기본 15)")
    parser.add_argument("--force", action="store_true", help="검사에 떨어진 구역이 있어도 파일을 쓴다")
    args = parser.parse_args(argv)

    path = args.probe or latest_probe()
    if not path or not os.path.exists(path):
        sys.exit("map_probe 결과가 없다. 먼저: python3 map_probe.py scan --label mountains")
    with open(path, encoding="utf-8") as f:
        probe = json.load(f)
    origin = probe["origin"]
    step = probe["step_m"]
    radius = probe["radius_m"]
    cells = {(c["n"], c["e"]): height(c["h_home"]) for c in probe["cells"]}
    homes = {name: tuple(v) for name, v in probe["vehicles"].items()}
    print(f"측정 결과: {path} ({probe.get('map')}, 홈 지면 = 원점 {probe['home_up_from_origin_m']:+}m)")
    print(f"검사 기준: 탐색 고도 - 가장 높은 물체 >= {args.margin:g}m\n")

    rows, ok_all, areas = [], True, []
    for sec in SECTIONS:
        s, n, w, e, alt = sec["south"], sec["north"], sec["west"], sec["east"], sec["alt"]
        if max(abs(s), abs(n), abs(w), abs(e)) > radius:
            print(f"[주의] {sec['id']} 가 측정 범위(반경 {radius}m) 밖으로 나간다 - 범위 밖은 검사하지 못한다")
        in_area = [h for (cn, ce), h in cells.items()
                   if s - step <= cn <= n + step and w - step <= ce <= e + step]
        area_max = max(in_area) if in_area else math.inf
        corners = [(s, w), (s, e), (n, e), (n, w)]
        # 경로 위 어느 점이든 가장 가까운 격자점까지는 최대 0.71칸이다. 0.75칸 안의 격자점을 보면
        # 경로 전체가 빠짐없이 덮인다.
        path_max = -math.inf
        for hn, he in homes.values():
            for cn_, ce_ in corners:
                for (cn, ce), h in cells.items():
                    if seg_dist(cn, ce, hn, he, cn_, ce_) <= 0.75 * step:
                        path_max = max(path_max, h)
        worst = max(area_max, path_max)
        clearance = alt - worst
        ok = clearance >= args.margin
        ok_all &= ok
        center = ((s + n) / 2, (w + e) / 2)
        dist = math.hypot(*center)
        far = max(math.hypot(a, b) for a, b in corners)
        rows.append((sec["id"], dist, far, alt, area_max, path_max, clearance, ok))
        sec["dist"], sec["far"] = dist, far
        areas.append({
            "area_id": sec["id"],
            "boundary": [to_geo(origin, a, b) for a, b in corners],
            "search_altitude_m": alt,
            "search_altitude_reference": "home_relative",
        })

    print(f"{'섹터':11s} {'홈->중심':>8s} {'가장 먼 곳':>9s} {'고도':>5s} {'섹터 최고':>9s} {'경로 최고':>9s} "
          f"{'여유':>7s}  판정")
    for sid, dist, far, alt, amax, pmax, clr, ok in rows:
        print(f"{sid:11s} {dist:7.0f}m {far:8.0f}m {alt:4.0f}m {fmt(amax):>9s} {fmt(pmax):>9s} "
              f"{(f'{clr:.1f}m' if math.isfinite(clr) else '-'):>7s}  {'OK' if ok else '위험'}")

    # 구역을 측정 지도 위에 글자로 표시한다 (A, B, ... = 구역, H = 원점)
    def char(nv, ev):
        if nv == 0 and ev == 0:
            return "H"
        for i, sec in enumerate(SECTIONS):
            if sec["south"] <= nv <= sec["north"] and sec["west"] <= ev <= sec["east"]:
                return LETTERS[i]
        h = cells.get((nv, ev), math.inf)
        if h < 2:
            return "."
        if h < 10:
            return "o"
        if h >= 100:
            return "#"
        return str(int(h // 10))
    norths = sorted({k[0] for k in cells}, reverse=True)
    easts = sorted({k[1] for k in cells})
    print("\n구역 배치 (지도 기호는 map_probe 와 같음)")
    for nv in norths:
        print(f"  N{nv:+5d}m " + "".join(char(nv, ev) for ev in easts))
    print("  " + ", ".join(f"{LETTERS[i]} = {sec['id']} (고도 {sec['alt']}m)" for i, sec in enumerate(SECTIONS)))

    if not ok_all and not args.force:
        sys.exit("\n위험한 구역이 있어 임무 파일을 쓰지 않았다. 구역이나 고도를 고쳐 다시 실행할 것 (--force 로 강제)")

    parts = ", ".join(f"{sec['id']} 약 {sec['dist']:.0f}m·고도 {sec['alt']}m" for sec in SECTIONS)
    instruction = (
        "홈(이륙 지점) 북동쪽 열린 지대를 2x2 섹터로 나눴다. 이름은 sector-행-열이며 행 0은 홈에 가까운 "
        "남쪽 줄, 1은 먼 북쪽 줄, 열 0은 서쪽, 1은 동쪽이다. 홈에서 섹터 중심까지 거리와 탐색 고도는 "
        f"{parts} 이다. 섹터마다 탐색 고도가 달라 두 드론의 비행 고도가 겹치지 않는다. "
        "배정된 섹터는 미들웨어가 지그재그로 수색한 뒤 홈으로 돌아와 착륙한다. "
        "드론 상태(배터리, 통신 연결)를 보고 드론마다 섹터를 하나씩 배정해 줘."
    )
    mission = {"mission_id": MISSION_ID, "instruction": instruction, "search_areas": areas}
    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, f"{MISSION_ID}.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(mission, f, ensure_ascii=False, indent=2)
        f.write("\n")
    print(f"\n저장: {out}")
    if os.path.exists(TEAM_MISSION_001):
        shutil.copy(TEAM_MISSION_001, os.path.join(OUT_DIR, "mission-001.json"))
        print(f"복사: mission-001.json (팀 저장소 원본, check_mvp_pipeline.py 용)")
    print("미들웨어에서 쓰려면 서버 실행 전에: export MIDDLEWARE_SCENARIOS_DIR=$HOME/skybuddy/scenarios/missions")
    print("AirSim 화면에 섹터와 지그재그 경로 그리기: python3 map_probe.py draw scenarios/missions/mission-101.json")


if __name__ == "__main__":
    main()
