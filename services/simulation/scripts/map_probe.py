"""AirSim 맵에서 탐색 구역(섹션)을 정하기 위해 홈 주변 장애물·지형 높이를 잰다. 기체에 명령은 보내지 않는다.

  python3 map_probe.py scan --label mountains                 홈 중심 반경 300m, 25m 간격
  python3 map_probe.py scan --label blocks --min-up 1         평지 맵은 지면 위만 재면 빠르다
  python3 map_probe.py draw scenarios/missions/mission-101.json   구역을 AirSim 화면에 그림 (영상·확인용)
  python3 map_probe.py draw --clear                           그린 것 지우기

원리
  AirSim 의 시선 검사(simTestLineOfSightBetweenPoints, AirSim 1.5 이상. 우리는 1.8.1)로 두 점 사이가
  막혔는지 본다. 각 지점의 높은 하늘에서 수직으로 내려다보며, 시선이 처음 막히는 높이(가장 높은
  물체나 지형의 윗면)를 이분 탐색으로 찾는다.
  격자 사이에 있는 좁은 물체는 놓칠 수 있다. 구역을 정한 뒤에는 실제 비행 경로를 따로 검사한다.

높이 기준
  AirSim 원점(OriginGeopoint)은 기체 출발 위치이고 지면보다 몇 m 위다. 비행 고도(미들웨어
  search_altitude_m)는 이륙 지점 지면 기준이다. 그래서 원점 바로 아래 지면을 먼저 재서 '홈 0m'로 잡고,
  지도와 요약은 홈 기준 높이로 보여 준다. (실측: 산악맵 AP_DDS 홈 해발 577.8m = 원점 583m 보다 5.2m 아래)

전제
  AirSim 과 SITL 2대가 떠 있어야 한다. lock-step 이라 SITL 이 없으면 AirSim 이 질의에 답하지 않는다.
  드론은 지상에 그대로 둔다. airsim 패키지 없이 msgpack 만 쓴다 (camera_check.py 와 같음).

결과
  화면: 글자 지도 (H = 홈, . = 홈 높이 이하, o = 10m 미만, 1~9 = 10m 단위, # = 100m 이상)
  ~/skybuddy/logs/map_probe_<label>_<시각>.json  (지점별 높이. 섹션 설계에 쓴다)
"""
import argparse
import json
import math
import os
import socket
import sys
import time
from datetime import datetime

try:
    import msgpack
except ImportError:
    sys.exit("msgpack 이 없다. 먼저 실행: python3 -m pip install --user msgpack")

DEFAULT_HOST, DEFAULT_PORT = "127.0.0.1", 41451  # AirSim 기본 RPC 포트 (settings.json ApiServerPort)
# AirSim EarthUtils 는 지구 반지름 6378137m 로 위경도를 NED 로 바꾼다. 1도 = 111319.49m.
# 미들웨어·어댑터가 쓰는 111320 과 300m 에서 1mm 차이라 같은 좌표로 봐도 된다.
M_PER_DEG = 6378137.0 * math.pi / 180.0
DEFAULT_ORIGIN = (-35.363261, 149.165230, 583.0)

_HERE = os.path.dirname(os.path.abspath(__file__))
# 개발 환경(평면 구조)은 이 폴더, 팀 저장소는 scripts/ 의 상위(services/simulation)가 기준이다.
BASE = _HERE if os.path.exists(os.path.join(_HERE, "kpi_mission.py")) else os.path.dirname(_HERE)
SETTINGS_CANDIDATES = [
    os.path.join(_HERE, "config_reference", "airsim_settings_reference.json"),   # 개발 환경
    os.path.join(os.path.dirname(_HERE), "config", "airsim_2vehicle.json"),       # 팀 저장소
]
OUT_DIR = os.path.expanduser("~/skybuddy/logs")

CONNECT_HELP = """AirSim RPC({host}:{port})에 연결하지 못했다: {err}
SITL 이 GPS 를 받고 있다면 AirSim 은 켜져 있는 것이다. 그렇다면 AirSim 의 API 서버가 안 열린 것이다.
  1) WSL 에서: cmd.exe /c "netstat -ano | findstr {port}"
     - 아무것도 안 나오면: AirSim 이 시작할 때 이 포트를 못 열었다 (창 왼쪽 위에 RpcLib/bind 오류).
       Windows 가 이 포트를 예약했는지 확인: netsh.exe int ipv4 show excludedportrange protocol=tcp
       예약 구간에 들어 있으면 settings.json 에 "ApiServerPort" 를 비어 있는 포트로 넣고
       AirSim 을 다시 켠 뒤 --port 로 같은 번호를 줄 것
     - LISTENING 줄이 있으면: WSL 에서 Windows 로 가는 길이 막힌 것이다. --host 로 Windows 주소를 줘 볼 것
       (WSL 에서 Windows 주소 확인: ip route show default | awk '{{print $3}}')
  2) 결과를 그대로 붙여 줄 것"""


class Rpc:
    """msgpack-rpc 최소 구현. 요청 [0, id, 메서드, 인자] -> 응답 [1, id, 오류, 결과]."""

    def __init__(self, host, port, timeout=30):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.unpacker = msgpack.Unpacker(raw=False)
        self.msgid = 0

    def call(self, method, *params):
        self.msgid += 1
        self.sock.sendall(msgpack.packb([0, self.msgid, method, list(params)], use_bin_type=True))
        while True:
            for msg in self.unpacker:
                if msg[0] == 1 and msg[1] == self.msgid:
                    if msg[2] is not None:
                        raise RuntimeError(f"{method}: {msg[2]}")
                    return msg[3]
            data = self.sock.recv(1 << 20)
            if not data:
                raise ConnectionError("AirSim 연결이 끊겼다")
            self.unpacker.feed(data)


def find_settings():
    """AirSim 이 실제로 읽는 Windows 쪽 settings.json 을 먼저 찾고, 없으면 저장소의 참고 사본."""
    import glob
    candidates = [os.environ.get("AIRSIM_SETTINGS")]
    candidates += sorted(glob.glob("/mnt/c/Users/*/OneDrive/Documents/AirSim/settings.json"))
    candidates += sorted(glob.glob("/mnt/c/Users/*/Documents/AirSim/settings.json"))
    candidates += SETTINGS_CANDIDATES
    for path in candidates:
        if path and os.path.exists(path):
            return path
    return None


def load_settings():
    """(원점 위도·경도·고도, {기체 이름: (X 북쪽 m, Y 동쪽 m)}, API 포트, 읽은 파일)."""
    path = find_settings()
    if path is None:
        return DEFAULT_ORIGIN, {"Copter1": (0.0, -3.0), "Copter2": (0.0, 3.0)}, DEFAULT_PORT, None
    with open(path, encoding="utf-8-sig") as f:      # Windows 메모장이 붙이는 BOM 도 읽는다
        data = json.load(f)
    o = data.get("OriginGeopoint", {})
    origin = (float(o.get("Latitude", DEFAULT_ORIGIN[0])),
              float(o.get("Longitude", DEFAULT_ORIGIN[1])),
              float(o.get("Altitude", DEFAULT_ORIGIN[2])))
    vehicles = {name: (float(v.get("X", 0)), float(v.get("Y", 0)))
                for name, v in data.get("Vehicles", {}).items()}
    port = int(data.get("ApiServerPort", DEFAULT_PORT))
    return origin, vehicles, port, path


class Probe:
    def __init__(self, rpc, origin):
        self.rpc = rpc
        self.lat0, self.lon0, self.alt0 = origin
        self.calls = 0

    def geo(self, north, east, up):
        """원점 기준 (북 m, 동 m, 위 m) -> AirSim GeoPoint. AirSim 의 역변환과 같은 식."""
        lat = self.lat0 + north / M_PER_DEG
        lon = self.lon0 + east / (M_PER_DEG * math.cos(math.radians(lat)))
        return {"latitude": lat, "longitude": lon, "altitude": self.alt0 + up}

    def clear(self, a, b):
        """a 에서 b 까지 시선이 뚫려 있으면 True."""
        self.calls += 1
        return bool(self.rpc.call("simTestLineOfSightBetweenPoints", a, b))

    def column(self, north, east, lo, hi, top, tol):
        """(north, east) 의 가장 높은 물체 윗면 높이(원점 기준). None = lo 위로 비어 있음, inf = hi 보다 높음."""
        sky = self.geo(north, east, top)
        if self.clear(sky, self.geo(north, east, lo)):
            return None
        if not self.clear(sky, self.geo(north, east, hi)):
            return math.inf
        blocked, open_ = lo, hi
        while open_ - blocked > tol:
            mid = (blocked + open_) / 2
            if self.clear(sky, self.geo(north, east, mid)):
                open_ = mid
            else:
                blocked = mid
        return round(open_, 1)


def find_ceiling(probe, low, high, tol=2.0):
    """원점 기둥에서, 그 높이에서 low 까지 내려다볼 때 시선이 막히기 시작하는 가장 낮은 높이.

    지형 위 하늘에 시선을 막는 물체(하늘 돔 등)가 있으면 그 높이를 돌려준다. 없으면 None.
    원점은 기체 출발점(골짜기 바닥)이라 low(지형보다 위) 에서 위쪽은 비어 있어야 정상이다.
    """
    def clear_from(h):
        return probe.clear(probe.geo(0, 0, h), probe.geo(0, 0, low))

    if clear_from(high):
        return None
    if not clear_from(low + tol):
        return low
    open_, blocked = low + tol, high
    while blocked - open_ > tol:
        mid = (open_ + blocked) / 2
        if clear_from(mid):
            open_ = mid
        else:
            blocked = mid
    return blocked


def to_home(h, home_up):
    """원점 기준 높이 -> 홈(출발점 지면) 기준. None 과 inf 와 '?' 는 그대로."""
    if h is None or h == "?" or h == math.inf:
        return h
    return round(h - home_up, 1)


def cell_char(h_home):
    if h_home == "?":
        return "?"
    if h_home is None:
        return "."
    if h_home == math.inf or h_home >= 100:
        return "#"
    if h_home < 2:
        return "."
    if h_home < 10:
        return "o"
    return str(int(h_home // 10))


def render(grid_home, norths, easts):
    lines = []
    width = len(easts)
    scale = "        " + "".join("|" if e == 0 else ("+" if e % 100 == 0 else " ") for e in easts)
    lines.append(scale)
    for n in sorted(norths, reverse=True):
        row = "".join("H" if (n == 0 and e == 0) else cell_char(grid_home.get((n, e), "?"))
                      for e in easts)
        lines.append(f"N{n:+5.0f}m " + row)
    lines.append(scale)
    lines.append(f"        서쪽 {easts[0]:+.0f}m {'-' * max(0, width - 22)} 동쪽 {easts[-1]:+.0f}m"
                 f"   (+ 는 100m 마다)")
    return "\n".join(lines)


def scan(args):
    origin, vehicles, settings_port, settings_path = load_settings()
    if args.port is None:
        args.port = settings_port
    print(f"원점 {origin[0]:.6f}, {origin[1]:.6f}, 해발 {origin[2]:.0f}m, API 포트 {args.port} "
          f"({settings_path or '기본값'})")
    try:
        rpc = Rpc(args.host, args.port)
    except OSError as e:
        sys.exit(CONNECT_HELP.format(host=args.host, port=args.port, err=e))
    probe = Probe(rpc, origin)

    # ---- 시선 검사가 이 맵에서 쓸 만한지 먼저 확인한다
    world_top = None
    try:
        extents = rpc.call("simGetWorldExtents")
        lows = [p.get("altitude") for p in extents]
        world_top = max(lows)
        print(f"맵 범위(simGetWorldExtents): 해발 {min(lows):.0f}~{max(lows):.0f}m")
    except Exception as e:  # noqa: BLE001 - 없어도 측정은 된다
        print(f"맵 범위 조회 실패 (측정은 계속): {e}")
    t0 = time.time()
    try:
        ceiling = find_ceiling(probe, args.max_up, args.top)
    except RuntimeError as e:
        sys.exit(f"시선 검사를 못 했다: {e}\n이 AirSim 빌드에 simTestLineOfSightBetweenPoints 가 없을 수 있다 "
                 f"(AirSim 1.5 이상 필요). 결과를 그대로 붙여 줄 것")
    except (socket.timeout, TimeoutError):
        sys.exit("AirSim 이 응답하지 않는다. lock-step 이라 SITL 2대가 떠 있어야 한다 (start_hetero_sim.sh 먼저)")
    per_call = (time.time() - t0) / max(probe.calls, 1)

    # 위에서 내려다보는 시작 높이(sky). 하늘에 시선을 막는 물체가 있으면 그보다 아래에서 시작한다.
    # 원점에서 멀어질수록 돔 같은 물체는 낮아질 수 있어 여유(--sky-margin)를 둔다.
    if ceiling is None:
        sky = args.top
        print(f"원점 위 하늘은 {args.top:g}m 까지 막힌 데가 없다")
    else:
        sky = ceiling - args.sky_margin
        if sky < args.max_up + 10:
            sky = ceiling - 3          # 여유를 다 못 두면 막는 물체 바로 아래에서 시작한다
        if sky <= args.max_up:
            sys.exit(f"원점 위 {ceiling:.0f}m 에서 시선이 막혀 {args.max_up:g}m 까지 잴 수 없다. "
                     f"--max-up 을 낮춰(예: 100) 다시 실행할 것")
        print(f"원점 위 {ceiling:.0f}m(해발 {origin[2] + ceiling:.0f}m)에서 시선을 막는 물체가 있다 "
              f"(지형보다 높아 하늘 돔 같은 것으로 보인다). 그 아래 {sky:.0f}m 에서 내려다본다")
        if world_top is not None and origin[2] + sky < world_top:
            print(f"[주의] 시작 높이(해발 {origin[2] + sky:.0f}m)가 맵에서 가장 높은 지형(해발 {world_top:.0f}m)보다 "
                  f"낮다. 반경 안에 그보다 높은 산이 있으면 그 지점은 '비어 있음'으로 잘못 나올 수 있다")
    args.top = sky
    if probe.clear(probe.geo(0, 0, sky), probe.geo(0, 0, -60)):
        sys.exit("원점 60m 아래까지 시선이 안 막힌다. 이 맵은 지면이 시선 검사에 잡히지 않아 잴 수 없다")
    print(f"시선 검사 정상 (1회 {per_call * 1000:.0f}ms)")

    # ---- 홈(원점 바로 아래 지면) 높이. 기체는 원점에서 3m 옆에 있어 이 기둥에 걸리지 않는다
    home_up = probe.column(0, 0, -60.0, 20.0, args.top, 0.2)
    if home_up is None or home_up == math.inf:
        print(f"[주의] 원점 아래 지면을 못 찾았다 ({home_up}). 원점을 홈 0m 로 본다")
        home_up = 0.0
    print(f"홈 지면: 원점 기준 {home_up:+.1f}m (해발 {origin[2] + home_up:.1f}m). 아래 높이는 모두 이 지면 기준")

    # 원점(0, 0)이 항상 격자에 들어가도록 대칭으로 만든다
    k = args.radius // args.step
    norths = [i * args.step for i in range(-k, k + 1)]
    easts = list(norths)
    total = len(norths) * len(easts)
    lo, hi = home_up + args.min_up, home_up + args.max_up      # 원점 기준으로 바꿔 잰다
    iters = math.ceil(math.log2((hi - lo) / args.tol)) + 2
    print(f"측정: {len(norths)}x{len(easts)} = {total}지점, {args.step}m 간격, 홈 기준 {args.min_up:g}~{args.max_up:g}m "
          f"(지점당 최대 {iters}회 질의, 예상 최대 {total * iters * per_call / 60:.1f}분)\n")

    grid_home, cells = {}, []
    started = time.time()
    try:
        for i, n in enumerate(sorted(norths, reverse=True), 1):
            for e in easts:
                try:
                    h = probe.column(n, e, lo, hi, args.top, args.tol)
                except RuntimeError as err:
                    h = "?"
                    print(f"  ({n:+}N {e:+}E) 실패: {err}")
                hh = to_home(h, home_up)
                grid_home[(n, e)] = hh
                cells.append({"n": n, "e": e,
                              "h_home": None if hh is None else ("above" if hh == math.inf else hh)})
            row = "".join(cell_char(grid_home[(n, e)]) for e in easts)
            print(f"  {i:2d}/{len(norths)}  N{n:+5d}m {row}", flush=True)
    except KeyboardInterrupt:
        print("\n중단됨 - 지금까지 잰 것만 저장한다")

    elapsed = time.time() - started
    print("\n" + render(grid_home, sorted({c["n"] for c in cells}), easts))
    print(f"\n범례 (홈 지면 기준): . 홈 높이 이하이거나 {args.min_up:g}m 위로 비어 있음  o 2~10m  "
          f"1~9 10m 단위 (3 = 30~39m)  # 100m 이상  ? 측정 실패")
    print(f"H = 원점. 기체 출발점: " + ", ".join(f"{name} 북{n:+.0f} 동{e:+.0f}m" for name, (n, e) in vehicles.items()))

    measured = [c for c in cells if isinstance(c["h_home"], (int, float))]
    above = [c for c in cells if c["h_home"] == "above"]
    if above:
        print(f"홈 기준 {args.max_up:g}m 보다 높은 지점: {len(above)}개")
    if measured:
        top_cell = max(measured, key=lambda c: c["h_home"])
        print(f"측정된 최고 높이: 홈 기준 {top_cell['h_home']}m (북 {top_cell['n']:+}m, 동 {top_cell['e']:+}m)")
    for alt in (30, 40, 50, 60):
        limit = alt - args.margin
        hit = [c for c in cells
               if c["h_home"] == "above" or (isinstance(c["h_home"], (int, float)) and c["h_home"] > limit)]
        print(f"  고도 {alt}m(홈 기준)로 날 때 여유 {args.margin:g}m 를 못 지키는 지점: {len(hit)}/{len(cells)}개")
    print(f"질의 {probe.calls}회, {elapsed:.0f}초")

    os.makedirs(OUT_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(OUT_DIR, f"map_probe_{args.label}_{stamp}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump({
            "map": args.label,
            "origin": list(origin),
            "home_up_from_origin_m": home_up,
            "vehicles": {k: list(v) for k, v in vehicles.items()},
            "step_m": args.step, "radius_m": args.radius,
            "min_up_m": args.min_up, "max_up_m": args.max_up, "tol_m": args.tol,
            "created_at": datetime.now().astimezone().isoformat(),
            "note": ("h_home: 홈(원점 아래 지면) 기준 가장 높은 물체·지형 윗면(m). "
                     "null = min_up 위로 비어 있음, above = max_up 보다 높음, ? = 측정 실패"),
            "cells": cells,
        }, f, ensure_ascii=False, indent=1)
    print(f"저장: {path}")


COLORS = [[1.0, 0.2, 0.2, 1.0], [0.2, 1.0, 0.2, 1.0], [0.2, 0.5, 1.0, 1.0], [1.0, 1.0, 0.2, 1.0],
          [1.0, 0.3, 1.0, 1.0], [0.2, 1.0, 1.0, 1.0]]


def vec(north, east, up):
    """원점 기준 (북, 동, 위) m -> AirSim Vector3r (NED, 아래가 +)."""
    return {"x_val": float(north), "y_val": float(east), "z_val": float(-up)}


def coverage_paths(mission, config_path):
    """미들웨어와 같은 함수로 섹터별 수색 경로(웨이포인트)를 만든다.

    미들웨어 드론 설정의 waypoints(생성기 종류)와 drones 의 swath_width_m 을 그대로 쓴다.
    드론마다 촬영 폭이 다르면 가장 좁은 값으로 그린다 (어느 드론이 맡아도 이 경로 이상으로 촘촘하다).
    돌려주는 값: ({area_id: [(위도, 경도), ...]}, 설명). 미들웨어 코드를 못 불러오면 예외.
    """
    middleware = None
    for c in (os.environ.get("SKYBUDDY_MIDDLEWARE_DIR"),
              os.path.join(os.path.dirname(BASE), "middleware"),
              os.path.expanduser("~/skybuddy-team/services/middleware")):
        if c and os.path.exists(os.path.join(c, "app", "runtime", "waypoints.py")):
            middleware = c
            break
    if middleware is None:
        raise RuntimeError("미들웨어 코드를 찾지 못했다 (~/skybuddy-team/services/middleware)")
    if middleware not in sys.path:
        sys.path.append(middleware)
    from app.runtime.config import WaypointConfig
    from app.schemas import SearchArea

    with open(config_path, encoding="utf-8") as f:
        config = json.load(f)
    waypoint_config = WaypointConfig.model_validate(config.get("waypoints", {}))
    generator = waypoint_config.build_generator()
    swaths = [d["swath_width_m"] for d in config.get("drones", []) if d.get("swath_width_m")]
    swath = min(swaths) if swaths else None
    paths = {}
    for area in mission.get("search_areas", []):
        points = generator.generate(SearchArea.model_validate(area), swath_width_m=swath)
        paths[area["area_id"]] = [(p.latitude, p.longitude) for p in points]
    return paths, f"생성기 {waypoint_config.generator}, 촬영 폭 {swath}m"


def draw(args):
    """임무 파일의 구역 경계를 탐색 고도에 그리고, 꼭짓점마다 지면에서 그 고도까지 기둥을 세운다.

    AirSim 화면에만 보이는 표시다. 드론 비행이나 충돌에는 영향이 없다.
    """
    origin, vehicles, settings_port, _ = load_settings()
    port = args.port or settings_port
    try:
        rpc = Rpc(args.host, port)
    except OSError as e:
        sys.exit(CONNECT_HELP.format(host=args.host, port=port, err=e))
    try:
        rpc.call("simFlushPersistentMarkers")
    except Exception as e:  # noqa: BLE001 - 지우기 실패는 그리기를 막지 않는다
        print(f"이전 표시 지우기 실패 (계속): {e}")
    if args.clear:
        print("AirSim 화면의 구역 표시를 지웠다")
        return
    if not args.mission:
        sys.exit("그릴 임무 파일을 줄 것. 예: python3 map_probe.py draw scenarios/missions/mission-101.json")
    with open(os.path.expanduser(args.mission), encoding="utf-8") as f:
        mission = json.load(f)

    probe = Probe(rpc, origin)
    ceiling = find_ceiling(probe, 30.0, 2000.0)
    sky = 2000.0 if ceiling is None else max(ceiling - 100.0, 40.0)
    home_up = probe.column(0, 0, -60.0, 20.0, sky, 0.2)
    if home_up is None or home_up == math.inf:
        home_up = 0.0
    lat0, lon0 = origin[0], origin[1]
    print(f"홈 지면 = 원점 {home_up:+.1f}m. 탐색 고도(홈 기준)에 맞춰 그린다")
    paths = {}
    if not args.no_paths:
        try:
            paths, info = coverage_paths(mission, args.drones_config)
            print(f"수색 경로: 미들웨어 함수로 계산 ({info}, 설정 {args.drones_config})")
        except Exception as e:  # noqa: BLE001 - 경로 없이 경계만 그려도 된다
            print(f"수색 경로는 그리지 못함 (경계만 그린다): {e}")

    def to_ned(lat, lon):
        return ((lat - lat0) * M_PER_DEG,
                (lon - lon0) * M_PER_DEG * math.cos(math.radians(lat)))

    for i, area in enumerate(mission.get("search_areas", [])):
        color = COLORS[i % len(COLORS)]
        top = home_up + float(area["search_altitude_m"])        # 원점 기준 높이
        pts = []
        for p in area["boundary"]:
            lat = p["latitude"]
            pts.append(((lat - lat0) * M_PER_DEG,
                        (p["longitude"] - lon0) * M_PER_DEG * math.cos(math.radians(lat))))
        ring = [vec(n, e, top) for n, e in pts] + [vec(pts[0][0], pts[0][1], top)]
        rpc.call("simPlotLineStrip", ring, color, args.thickness, -1.0, True)
        posts = []
        for n, e in pts:
            g = probe.column(n, e, home_up - 100.0, top, sky, 0.5)
            ground = home_up - 100.0 if g is None else (top if g == math.inf else g)
            posts += [vec(n, e, ground), vec(n, e, top)]
        rpc.call("simPlotLineList", posts, color, args.thickness * 0.6, -1.0, True)
        cn = sum(n for n, _ in pts) / len(pts)
        ce = sum(e for _, e in pts) / len(pts)
        try:
            rpc.call("simPlotStrings", [f"{area['area_id']} {area['search_altitude_m']:g}m"],
                     [vec(cn, ce, top + 5)], 10.0, color, 3600.0)
        except Exception:  # noqa: BLE001 - 글자 표시는 없어도 된다
            pass
        path = paths.get(area["area_id"])
        extra = ""
        if path:
            # 드론이 실제로 날 지그재그 경로. 경계와 같은 색의 옅은 선으로 같은 고도에 그린다.
            light = [min(1.0, c + 0.5) for c in color[:3]] + [1.0]
            rpc.call("simPlotLineStrip", [vec(*to_ned(lat, lon), top) for lat, lon in path],
                     light, args.thickness * 0.4, -1.0, True)
            extra = f", 지그재그 웨이포인트 {len(path)}개"
        print(f"  {area['area_id']}: 고도 {area['search_altitude_m']:g}m, 중심 북{cn:+.0f} 동{ce:+.0f}m{extra}")
    print("AirSim 화면에 그렸다. 지우기: python3 map_probe.py draw --clear")


def main(argv=None):
    parser = argparse.ArgumentParser(prog="map_probe.py", description="AirSim 맵 장애물·지형 높이 측정",
                                     formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    d = sub.add_parser("draw", help="임무 파일의 구역을 AirSim 화면에 그린다 (영상·확인용)")
    d.add_argument("mission", nargs="?", help="임무 파일 (예: scenarios/missions/mission-101.json)")
    d.add_argument("--clear", action="store_true", help="그린 것을 지우기만 한다")
    d.add_argument("--thickness", type=float, default=50.0, help="선 두께 (기본 50)")
    d.add_argument("--drones-config", default=os.path.join(BASE, "scenarios", "drones.sitl.json"),
                   help="수색 경로 계산에 쓸 미들웨어 드론 설정 (기본 scenarios/drones.sitl.json)")
    d.add_argument("--no-paths", action="store_true", help="섹터 경계만 그리고 수색 경로는 그리지 않는다")
    d.add_argument("--host", default=DEFAULT_HOST)
    d.add_argument("--port", type=int, default=None)
    d.set_defaults(func=draw)
    p = sub.add_parser("scan", help="홈 주변 장애물·지형 높이 지도")
    p.add_argument("--label", required=True, help="맵 이름 (파일 이름에 씀). 예: mountains, blocks")
    p.add_argument("--host", default=DEFAULT_HOST, help=f"AirSim 이 도는 Windows 주소 (기본 {DEFAULT_HOST})")
    p.add_argument("--port", type=int, default=None,
                   help=f"AirSim API 포트. 없으면 settings.json 의 ApiServerPort (없으면 {DEFAULT_PORT})")
    p.add_argument("--radius", type=int, default=300, help="원점에서 잴 거리 m (기본 300)")
    p.add_argument("--step", type=int, default=25, help="격자 간격 m (기본 25)")
    p.add_argument("--min-up", type=float, default=-100.0,
                   help="홈 기준 이 높이 위로 비어 있으면 '비어 있음' (기본 -100. 평지 맵은 1 이면 빠르다)")
    p.add_argument("--max-up", type=float, default=150.0, help="홈 기준 잴 최고 높이 m (기본 150)")
    p.add_argument("--tol", type=float, default=1.0, help="높이 정밀도 m (기본 1)")
    p.add_argument("--top", type=float, default=2000.0,
                   help="내려다보는 시작 높이의 최댓값 m, 원점 기준 (기본 2000). 하늘에 막는 물체가 있으면 자동으로 낮춘다")
    p.add_argument("--sky-margin", type=float, default=100.0,
                   help="하늘의 막는 물체보다 이만큼 아래에서 시작 (기본 100m)")
    p.add_argument("--margin", type=float, default=10.0, help="요약에 쓰는 고도 여유 m (기본 10)")
    p.set_defaults(func=scan)
    args = parser.parse_args(argv)
    if args.command == "scan" and (args.step <= 0 or args.radius <= 0 or args.min_up >= args.max_up):
        parser.error("--step, --radius 는 양수, --min-up 은 --max-up 보다 작아야 한다")
    args.func(args)


if __name__ == "__main__":
    main()
