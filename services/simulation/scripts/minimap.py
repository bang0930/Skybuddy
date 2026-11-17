"""섹터와 드론 위치를 작은 2D 지도(미니맵)로 보여 준다 (영상·시연용). 기체에 명령은 보내지 않는다.

  python3 minimap.py                                     mission-101 섹터 전부 (실제 드론)
  python3 minimap.py --sectors sector-0-0,sector-1-1     지정한 섹터만 (실제 드론)
  python3 minimap.py --demo                              가짜 움직임 (AirSim 없이 화면만 확인. 실제 드론과 무관)
  python3 minimap.py --selftest                          자체 점검 (AirSim·SITL 불필요)

실제 드론과 같은 값
  위치        AirSim 의 실제 기체 위치 (1초에 3번). 임무를 안 내리면 홈에 그대로 있다
  고도·배터리·링크  미들웨어가 받은 텔레메트리 값 그대로 (어댑터가 남기는 텔레메트리 기록 = fault.py stream).
              미들웨어 서버가 드론에 붙어 있을 때만 생긴다 (run_live.sh, run_plan.py, LLM 파이프라인)
  --stream 을 안 주면 가장 최근에 쓰인 텔레메트리 기록을 따라간다 (기본 기록, run_live.sh 의 실행별 기록 중)

켜면 이 컴퓨터에 브라우저 작은 창(주소창 없음)이 저절로 열린다 (--no-open 이면 안 연다).
  Windows·WSL = Edge, Mac = Chrome 또는 Edge (없으면 기본 브라우저), 리눅스 데스크톱 = Chrome·Chromium 또는 기본 브라우저.
  WSL 에서 안 열리면 Windows 프로그램 실행(interop)이 꺼진 것이다 (이유가 출력된다). 직접 열 때는 http://localhost:8765
  화면 없는 서버(AWS 리눅스 등)에서 켜면 창 대신 다른 컴퓨터가 열 주소(http://<서버 주소>:8765)를 알려 준다.
  보는 컴퓨터(AirSim 화면이 있는 컴퓨터 등)의 Edge·Chrome 에서 그 주소를 한 번 열면 된다.
  프로그램은 자기가 도는 컴퓨터에만 창을 띄울 수 있어서, 서버에서 다른 컴퓨터의 창을 대신 열 수는 없다.
이미 열어 둔 화면이 있으면 새 창을 열지 않고 그 화면이 이어서 보여 준다.
  Windows·WSL 에서는 그 창을 주 화면 오른쪽 아래(AirSim 작은 창 3 자리. 그 창은 숨긴다)에 항상 위로 저절로 붙인다
  (--corner left 면 왼쪽 아래, --no-pin 이면 안 붙인다).
  '항상 위에 띄우기'  지도를 다른 창(AirSim) 위에 떠 있는 작은 창으로 옮긴다. 오른쪽 아래로 끌어다 두면 된다
                     (Document Picture-in-Picture, Chrome·Edge 116 이상)
  OBS 로 녹화할 때    브라우저 소스 http://localhost:8765/?overlay=1 (배경 투명, 버튼 없음. 지도만은 &data=0)
  ?demo=1            지도는 그대로 두고 드론만 가짜로 움직인다
  브라우저에서 주소가 안 열리면 --bind 0.0.0.0 으로 다시 켠다 (WSL 네트워크 설정에 따라 127.0.0.1 이 Windows 에
  안 보일 수 있다). 서버에서는 기본이 0.0.0.0 이다 (AWS 보안 그룹에서 TCP 8765 를 보는 컴퓨터에 열어 준다)

AirSim 에는 미니맵(2D 지도) 기능이 없다. 화면에 겹쳐 띄울 수 있는 것은 기체 카메라 영상을 보여 주는 작은 창
(SubWindows) 3개뿐이라, 지도는 따로 그려서 브라우저 창으로 띄운다.

표시 (지도 + 아래 데이터 줄. 위쪽 제목 줄은 없다)
  점선      섹터 안 지그재그 수색 경로 (미들웨어와 같은 함수로 만든 길. --no-paths 면 안 그림). 격자·북쪽(N)·축척도 그린다.
            N 화살표는 섹터 이름과 맞춰 스폰에서 '북' 섹터 쪽을 가리킨다 (mission-101 은 오른쪽 위).
            지도 자체는 실제 진북이 위다
  섹터      임무 파일의 경계. 이름 옆에 탐색 고도. 2x2 섹터 이름: 스폰(홈)에서 가장 먼 섹터 = 북, 가장 가까운
            섹터 = 남, 홈에서 북을 볼 때 왼쪽 = 서, 오른쪽 = 동. mission-101 은 서 북 / 남 동 (sector-1-0 서,
            sector-1-1 북, sector-0-0 남, sector-0-1 동. 임무 파일 설명에도 적혀 있어 LLM 이 같은 이름을 쓴다).
            색은 이름으로 정한다: 북 빨강, 남 파랑, 동 주황, 서 하늘 (map_probe.py draw 가 AirSim 에 그린 선과 같은 색)
  H         홈 (AirSim 원점 = 스폰 지점)
  드론 핀    번호가 적힌 원. drone-01 초록, drone-02 연노랑 (track_drones.py 와 같은 색 = AirSim 작은 창 1·2 테두리 색).
            지나간 길이 가는 선으로 남고, 삼각형이 움직이는 방향을 가리킨다. 같은 자리(홈)면 옆으로 벌려 그린다.
            비행 중인데 텔레메트리 기록이 3초 넘게 끊기면 핀 둘레가 빨갛게 깜박인다 (통신 두절 등)
  아래 줄    드론별 고도·배터리·링크 품질 = 미들웨어가 받은 텔레메트리 값 (비행 중 끊기면 '끊김 N초').
            이륙 전·착륙 뒤(시동 꺼짐)에는 미들웨어가 값을 안 읽으므로 마지막 값을 흐리게 보인다.
            '지도만 보기' 버튼 (작은 창에서는 지도를 두 번 누른다. 주소로는 ?data=0) 으로 숨긴다
  주황 글씨  AirSim 연결 대기, 미니맵 서버 꺼짐 같은 문제가 있을 때만 왼쪽 위에 나온다

위치는 track_drones.py 와 같이 AirSim simGetGroundTruthEnvironment 의 위경도(참값)를 쓴다. 그래서 통신이 끊긴
동안에도 기체가 실제로 어디 있는지 보인다. AirSim 에 붙지 못하면 텔레메트리 기록의 위치로 대신한다.
"""
import argparse
import base64
import glob
import hashlib
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

_HERE = os.path.dirname(os.path.abspath(__file__))
APP_DIR = _HERE if os.path.exists(os.path.join(_HERE, "fault_injection.py")) \
          else os.path.join(os.path.dirname(_HERE), "app")
sys.path.insert(0, APP_DIR)
sys.path.insert(1, _HERE)

import map_probe as mp                              # noqa: E402  (Rpc, 설정 읽기, 섹터 색, 수색 경로)
import track_drones as td                           # noqa: E402  (기체 이름 -> 드론 ID, 드론 색)
from fault_injection import stream_log_path         # noqa: E402

DEFAULT_PORT = 8765
STALE_S = td.STALE_S       # 기록이 3초 넘게 없으면 끊김 (track_drones.py 이름표와 같은 기준)
GONE_S = 120.0             # 이보다 오래된 기록은 지난 시험 것으로 보고 값을 보이지 않는다
TRAIL_STEP_M = 2.0         # 이만큼 움직일 때마다 꼬리에 점을 더한다
TRAIL_MAX = 1500           # 드론마다 꼬리 점 최대 개수 (2m 간격이면 3km)
MAX_SPEED_M_S = 60.0       # 이보다 빠르게 옮겨지면(AirSim 재시작 등) 꼬리를 새로 시작한다
MARGIN = 0.12              # 지도 여백 (홈과 섹터를 감싼 범위에 대한 비율)
RECORD_TAIL_BYTES = 512 * 1024   # 텔레메트리 기록 끝에서 읽는 양 (두 드론 약 100초 분량)
COMPASS = mp.COMPASS


def _num(v):
    """JSON 에 넣을 수 있는 숫자로 (NaN·무한대·숫자 아님 -> None). 소수 첫째 자리까지."""
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return round(v, 1) if math.isfinite(v) else None


def rgb255(color):
    return [int(round(min(1.0, max(0.0, float(c))) * 255)) for c in color[:3]]


class Frame:
    """AirSim 원점(OriginGeopoint) 기준 평면 좌표 (동 x, 북 y) m. map_probe.py draw·track_drones.py 와 같은 식."""

    def __init__(self, origin):
        self.lat0, self.lon0, self.alt0 = (float(v) for v in origin)

    def xy(self, lat, lon):
        north = (lat - self.lat0) * mp.M_PER_DEG
        east = (lon - self.lon0) * mp.M_PER_DEG * math.cos(math.radians(lat))
        return east, north

    def geo(self, east, north):
        """xy 의 역변환 (자체 점검용)."""
        lat = self.lat0 + north / mp.M_PER_DEG
        return lat, self.lon0 + east / (mp.M_PER_DEG * math.cos(math.radians(lat)))


def find_mission(arg):
    """파일 경로 또는 mission-101 같은 이름 -> 임무 파일 경로."""
    path = os.path.expanduser(arg)
    if os.path.isfile(path):
        return path
    name = arg if arg.endswith(".json") else arg + ".json"
    for folder in (os.environ.get("MIDDLEWARE_SCENARIOS_DIR"), os.path.join(mp.BASE, "scenarios", "missions")):
        if folder:
            candidate = os.path.join(os.path.expanduser(folder), name)
            if os.path.isfile(candidate):
                return candidate
    raise FileNotFoundError(f"임무 파일을 찾지 못했다: {arg} (파일 경로나 mission-101 같은 이름을 줄 것)")


compass_names = mp.compass_names     # 2x2 섹터 동서남북 이름 (map_probe.py 와 같은 함수 = AirSim 선 색과 같은 기준)


def build_map(mission, frame, sectors=None, paths=None, demo=False):
    """브라우저에 줄 지도: 보일 섹터의 경계(동, 북 m)·이름·색, 홈, 보이는 범위, 드론 색.

    paths(수색 경로)는 데모 드론이 따라 움직일 길로만 쓴다 (화면에 그리지 않는다).
    """
    areas = mission.get("search_areas") or []
    ids = [a.get("area_id") for a in areas]
    wanted = [s for s in (sectors or []) if s]
    unknown = [s for s in wanted if s not in ids]
    if unknown:
        raise ValueError(f"임무에 없는 섹터: {', '.join(unknown)} (이 임무의 섹터: {', '.join(map(str, ids))})")
    centers = {}
    for area in areas:
        pts = [frame.xy(p["latitude"], p["longitude"]) for p in area.get("boundary") or []]
        if pts:
            centers[area.get("area_id")] = (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))
    names = compass_names(centers)          # 이름은 보일 섹터가 아니라 임무 전체(2x2)로 정한다
    # 화면의 N 화살표 방향 (동, 북 단위 벡터): 스폰(홈)에서 '북' 섹터 중심 쪽. 2x2 가 아니면 지도 위쪽(진북)
    north = [0.0, 1.0]
    north_id = next((k for k, v in names.items() if v == "북"), None)
    if north_id is not None and math.hypot(*centers[north_id]) > 1e-6:
        nx, ny = centers[north_id]
        norm = math.hypot(nx, ny)
        north = [round(nx / norm, 4), round(ny / norm, 4)]
    shown = []
    for i, area in enumerate(areas):
        area_id = area.get("area_id")
        if wanted and area_id not in wanted:
            continue
        ring = [[_num(v) for v in frame.xy(p["latitude"], p["longitude"])] for p in area.get("boundary") or []]
        path = [[_num(v) for v in frame.xy(lat, lon)] for lat, lon in (paths or {}).get(area_id) or []]
        shown.append({"id": area_id, "label": names.get(area_id) or str(area_id).replace("sector-", "", 1),
                      "alt_m": area.get("search_altitude_m"),
                      # map_probe.py draw 와 같은 색: 북 빨강·남 파랑·동 주황·서 하늘 (2x2 가 아니면 순서대로)
                      "color": rgb255(mp.sector_color(i, names.get(area_id))),
                      "ring": ring, "path": path})
    xs = [0.0] + [p[0] for s in shown for p in s["ring"]]
    ys = [0.0] + [p[1] for s in shown for p in s["ring"]]
    pad = max(max(xs) - min(xs), max(ys) - min(ys), 50.0) * MARGIN
    mission_id = str(mission.get("mission_id") or "")
    return {
        "mission_id": mission_id,
        # 서버를 다시 켜면 바뀐다. 열어 둔 화면은 이 값이 바뀌면 지도를 다시 받는다 (시험마다 섹터가 다르다)
        "map_id": f"{mission_id}|{','.join(str(s['id']) for s in shown)}|{time.time():.0f}",
        "sectors": shown,
        "north": north,
        "home": [0.0, 0.0],
        "bounds": [_num(min(xs) - pad), _num(min(ys) - pad), _num(max(xs) + pad), _num(max(ys) + pad)],
        "drone_colors": {d: rgb255(c) for d, c in td.DRONE_COLORS.items()},
        "demo": bool(demo),
    }


def latest_records(path, max_bytes=RECORD_TAIL_BYTES):
    """텔레메트리 기록 끝부분에서 드론별 마지막 기록 {drone: (기록 시각 epoch, 기록)}."""
    out = {}
    if not path:
        return out
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - max_bytes))
            raw = f.read().decode("utf-8", errors="replace")
    except OSError:
        return out
    for line in reversed(raw.splitlines()):
        try:
            rec = json.loads(line)
            ts = datetime.fromisoformat(rec["logged_at"]).timestamp()
        except (ValueError, KeyError, TypeError):
            continue                # 잘린 첫 줄, 깨진 줄
        name = rec.get("drone")
        if name and name not in out and rec.get("telemetry") is not None:
            out[name] = (ts, rec)
    return out


def newest_stream(logs_dir=None):
    """가장 최근에 쓰인 텔레메트리 기록: 기본 기록(SKYBUDDY_TELEMETRY_LOG 기본값)과 실행별 기록
    (run_live.sh 의 logs/live_*/telemetry.jsonl, 예전 시험 기록 logs/scenario_*/telemetry_*.jsonl) 중
    마지막으로 바뀐 것."""
    logs_dir = logs_dir or os.path.expanduser("~/skybuddy/logs")
    candidates = [stream_log_path()] + glob.glob(os.path.join(logs_dir, "*", "telemetry*.jsonl"))
    best, best_mtime = None, None
    for path in candidates:
        if not path:
            continue
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            continue
        if best_mtime is None or mtime > best_mtime:
            best, best_mtime = path, mtime
    return best or stream_log_path()


class Tracker:
    """드론 위치(AirSim 참값, 안 되면 텔레메트리 기록)와 고도·배터리·링크 값(텔레메트리 기록)을 주기적으로 읽는다."""

    def __init__(self, frame, vehicles, connect, stream=None, source="auto", override=None, clock=time.time,
                 follow=False):
        self.frame = frame
        self.vehicles = list(vehicles)
        self.connect = connect          # () -> AirSim RPC (call 메서드). 자체 점검은 가짜를 넣는다
        self.stream = stream
        self.follow = follow            # True 면 1초마다 가장 최근 텔레메트리 기록으로 바꿔 읽는다
        self.source = source
        self.override = override
        self.clock = clock
        self.lock = threading.Lock()
        self.rpc = None
        self.names = []
        self.mapping = {}
        self.note = None                # 화면 왼쪽 위에 보일 문제 (AirSim 연결 등)
        self.used = None                # 이번 위치의 출처: airsim / stream / None
        self.last = {}                  # drone -> (x, y, up, 읽은 시각)
        self.trails = {}                # drone -> [[x, y], ...]
        self.records = {}               # drone -> (기록 시각, 기록)
        self._records_at = None
        self._retry_at = 0.0

    def poll_once(self):
        now = self.clock()
        records = None
        if self._records_at is None or now - self._records_at >= 1.0:
            if self.follow:
                self.stream = newest_stream()
            if self.stream:
                records = latest_records(self.stream)
            self._records_at = now
        positions, used = None, None
        if self.source in ("auto", "airsim"):
            positions = self._airsim(now)
            used = "airsim" if positions is not None else None
        if positions is None and self.source in ("auto", "stream"):
            positions = self._stream_positions(now, records if records is not None else self.records)
            used = "stream" if positions else None
        with self.lock:
            if records is not None:
                self.records = records
            self.used = used
            for drone, (x, y, up) in (positions or {}).items():
                self._add(drone, x, y, up, now)

    def _airsim(self, now):
        if self.rpc is None:
            if now < self._retry_at:
                return None
            try:
                self.rpc = self.connect()
                try:
                    listed = list(self.rpc.call("listVehicles") or [])
                except RuntimeError:            # 목록 질의가 안 되면 설정 파일의 이름을 그대로 쓴다
                    listed = []
                names = [n for n in self.vehicles if not listed or n in listed] or listed
                if not names:
                    raise RuntimeError("기체가 없다 (settings.json Vehicles 와 listVehicles 가 비어 있음)")
            except Exception as e:  # noqa: BLE001 - AirSim 이 아직 안 켜졌을 수 있다. 2초 뒤 다시
                self._fail(now, f"AirSim 연결 대기 ({e})")
                return None
            self.names = names
            self.mapping = td.drone_mapping(names, self.override)
        try:
            out = {}
            for name in self.names:
                gp = self.rpc.call("simGetGroundTruthEnvironment", name)["geo_point"]
                x, y = self.frame.xy(float(gp["latitude"]), float(gp["longitude"]))
                out[self.mapping[name]] = (x, y, float(gp["altitude"]) - self.frame.alt0)
        except Exception as e:  # noqa: BLE001 - AirSim 을 다시 켜면 연결이 끊긴다. 다시 붙는다
            self._fail(now, f"AirSim 응답 없음 ({e})")
            return None
        self.note = None
        return out

    def _fail(self, now, note):
        sock = getattr(self.rpc, "sock", None)
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        self.rpc = None
        self._retry_at = now + 2.0
        self.note = note

    def _stream_positions(self, now, records):
        """AirSim 을 못 쓸 때: 텔레메트리 기록의 위치 (기체가 보고한 값. 기록이 끊기면 그 자리에 멈춘다)."""
        out = {}
        for drone, (ts, rec) in records.items():
            if now - ts > GONE_S:
                continue
            t = rec.get("telemetry") or {}
            pos = (rec.get("sim") or {}).get("position")
            lat, lon = (pos[0], pos[1]) if pos else (t.get("lat"), t.get("lon"))
            if lat is None or lon is None:
                continue
            x, y = self.frame.xy(float(lat), float(lon))
            out[drone] = (x, y, t.get("relative_alt_m"))
        return out

    def _add(self, drone, x, y, up, now):
        """위치를 갱신하고 꼬리를 잇는다. lock 을 잡은 채로 부른다."""
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        trail = self.trails.setdefault(drone, [])
        prev = self.last.get(drone)
        if prev is not None:
            dt = max(now - prev[3], 1e-3)
            if math.hypot(x - prev[0], y - prev[1]) > max(20.0, MAX_SPEED_M_S * dt):
                trail.clear()           # 순간 이동 = AirSim 재시작 등. 이전 꼬리와 잇지 않는다
        if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) >= TRAIL_STEP_M:
            trail.append([_num(x), _num(y)])
            if len(trail) > TRAIL_MAX:
                del trail[:len(trail) - TRAIL_MAX]
        self.last[drone] = (x, y, up, now)

    def reset_trails(self):
        with self.lock:
            self.trails = {d: [[_num(p[0]), _num(p[1])]] for d, p in self.last.items()}

    def snapshot(self):
        now = self.clock()
        with self.lock:
            fresh = {d for d, (ts, _) in self.records.items() if now - ts <= GONE_S}
            drones = []
            for drone in sorted(set(self.last) | fresh):
                item = {"id": drone, "x": None, "y": None, "up": None, "alt": None, "bat": None, "link": None,
                        "age": None, "armed": None, "link_state": None,
                        "trail": list(self.trails.get(drone, []))}
                pos = self.last.get(drone)
                if pos is not None:
                    item.update(x=_num(pos[0]), y=_num(pos[1]), up=_num(pos[2]))
                if drone in fresh:
                    ts, rec = self.records[drone]
                    t = rec.get("telemetry") or {}
                    sim = rec.get("sim") or {}
                    armed = sim.get("armed")
                    item.update(age=_num(max(0.0, now - ts)), bat=_num(t.get("battery_remaining_pct")),
                                link=_num(t.get("link_quality_pct")), alt=_num(t.get("relative_alt_m")),
                                # 시동이 꺼져 있으면(이륙 전·착륙 뒤) 미들웨어가 안 읽어 기록이 끊기는 것이라
                                # 화면이 '끊김' 으로 보이지 않게 넘긴다
                                armed=armed if isinstance(armed, bool) else None,
                                link_state=sim.get("link_state"))
                drones.append(item)
            return {"t": round(now, 2), "source": self.used, "note": self.note, "drones": drones}


# ---------------------------------------------------------------- 웹 서버

def in_wsl():
    if os.environ.get("WSL_DISTRO_NAME"):
        return True
    try:
        with open("/proc/sys/kernel/osrelease", encoding="utf-8") as f:
            return "microsoft" in f.read().lower()
    except OSError:
        return False


def interop_on():
    """WSL 에서 Windows 프로그램(cmd.exe 등)을 실행할 수 있나 (binfmt WSLInterop 등록)."""
    return any(os.path.exists(p) for p in ("/proc/sys/fs/binfmt_misc/WSLInterop",
                                           "/proc/sys/fs/binfmt_misc/WSLInterop-late"))


def windows_cmd():
    """WSL 에서 쓸 Windows cmd.exe. PATH 에 없으면(wsl.conf appendWindowsPath=false 등) 기본 위치를 쓴다."""
    found = shutil.which("cmd.exe")
    if found:
        return found
    for path in ("/mnt/c/Windows/System32/cmd.exe", "/mnt/c/WINDOWS/system32/cmd.exe"):
        if os.path.exists(path):
            return path
    return None


def has_display():
    """이 컴퓨터에 브라우저 창을 띄울 화면이 있나. 없으면 서버(AWS 등)라 다른 컴퓨터가 주소로 연다."""
    if sys.platform == "darwin" or sys.platform.startswith("win") or in_wsl():
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def browser_commands(url):
    """창을 여는 명령 후보 (앞에서부터 해 본다). Chrome·Edge 는 주소창 없는 작은 창(--app)으로 연다."""
    app = [f"--app={url}", "--window-size=380,420"]
    cands = []
    if sys.platform == "darwin":                                    # Mac
        cands += [["open", "-na", "Google Chrome", "--args", *app],
                  ["open", "-na", "Microsoft Edge", "--args", *app],
                  ["open", url]]
    elif sys.platform.startswith("win"):                            # Windows 에서 바로 실행
        cands += [["cmd", "/c", "start", "", "msedge", *app],
                  ["cmd", "/c", "start", "", "chrome", *app],
                  ["cmd", "/c", "start", "", url]]
    else:
        if in_wsl():                                                # WSL: Windows 쪽 Edge
            cmd = windows_cmd()
            if cmd:
                cands.append([cmd, "/c", "start", "msedge", *app])
        if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):   # 리눅스 데스크톱 (WSLg 포함)
            for name in ("google-chrome", "google-chrome-stable", "microsoft-edge", "chromium", "chromium-browser"):
                path = shutil.which(name)
                if path:
                    cands.append([path, *app])
            if shutil.which("xdg-open"):
                cands.append(["xdg-open", url])
    return cands


def _label(cmd):
    """오류 메시지에 쓸 짧은 이름 (start msedge, open Google Chrome, google-chrome ...)."""
    if "start" in cmd:
        rest = [c for c in cmd[cmd.index("start") + 1:] if c]
        return "start " + (rest[0] if rest else "")
    if cmd[0] == "open":
        return "open " + (cmd[2] if len(cmd) > 2 and cmd[1] == "-na" else cmd[-1])
    return os.path.basename(cmd[0])


def _launch(cmd):
    """명령 하나로 창을 열어 본다. (열었는지, 못 열었으면 이유)."""
    wsl_exe = in_wsl() and cmd[0].lower().endswith(".exe")
    try:
        proc = subprocess.Popen(cmd, cwd="/mnt/c" if wsl_exe and os.path.isdir("/mnt/c") else None,
                                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError as e:
        why = f"{_label(cmd)} 실행 실패: {e.strerror or e}"
        if wsl_exe and not interop_on():
            why += " (WSL 에서 Windows 프로그램을 실행하는 기능(interop)이 꺼져 있다)"
        return False, why
    try:
        code = proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        return True, ""             # 브라우저를 바로 띄운 경우 (안 끝나고 떠 있다)
    if code != 0:
        return False, f"{_label(cmd)} 종료 코드 {code}"
    return True, ""


def open_browser(url):
    """이 컴퓨터에 브라우저 창을 연다 (Windows·WSL·Mac·리눅스 데스크톱). (열었는지, 못 열었으면 이유)."""
    cands = browser_commands(url)
    if not cands:
        return False, "창을 여는 방법을 찾지 못했다 (Chrome·Edge·xdg-open 없음)"
    errors = []
    for cmd in cands:
        ok, why = _launch(cmd)
        if ok:
            return True, ""
        errors.append(why)
    return False, "; ".join(errors)


PIN_TITLE = "SkyBuddy Minimap"     # 화면(PAGE) <title> 과 같아야 한다. Edge 앱 창 제목이 된다

# Windows 창을 제목으로 찾아 주 화면 작업 영역 오른쪽(또는 왼쪽) 아래에 놓고 '항상 위'로 둔다 (SetWindowPos HWND_TOPMOST).
# 일반 Edge 창(제목 끝에 "- Microsoft Edge")은 건드리지 않는다. DPI 인식을 켜지 않아 좌표·크기는 화면 배율 기준이다.
PIN_SCRIPT = r"""
$ErrorActionPreference = 'Stop'
Add-Type -TypeDefinition @'
using System;
using System.Text;
using System.Runtime.InteropServices;
public static class SkyWin {
    public delegate bool EnumProc(IntPtr h, IntPtr l);
    [DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc cb, IntPtr l);
    [DllImport("user32.dll", CharSet = CharSet.Unicode)] public static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
    [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
    [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr h, IntPtr after, int x, int y, int cx, int cy, uint flags);
    public static IntPtr Find(string title) {
        IntPtr found = IntPtr.Zero;
        EnumWindows(delegate (IntPtr h, IntPtr l) {
            if (!IsWindowVisible(h)) { return true; }
            StringBuilder sb = new StringBuilder(512);
            GetWindowText(h, sb, 512);
            string t = sb.ToString();
            if (t == title || (t.StartsWith(title) && t.IndexOf("Edge") < 0 && t.IndexOf("Chrome") < 0)) {
                found = h;
                return false;
            }
            return true;
        }, IntPtr.Zero);
        return found;
    }
}
'@
Add-Type -AssemblyName System.Windows.Forms
$wa = [System.Windows.Forms.Screen]::PrimaryScreen.WorkingArea
$h = [IntPtr]::Zero
for ($i = 0; $i -lt __TRIES__; $i++) {
    $h = [SkyWin]::Find('__TITLE__')
    if ($h -ne [IntPtr]::Zero) { break }
    Start-Sleep -Milliseconds 250
}
if ($h -eq [IntPtr]::Zero) { Write-Output 'notfound'; exit 2 }
$x = __X__
$y = $wa.Bottom - __H__ - __M__
[void][SkyWin]::SetWindowPos($h, [IntPtr](-1), $x, $y, __W__, __H__, 0x0040)
Write-Output ("ok {0} {1}" -f $x, $y)
"""


PIN_CORNERS = {"right": "$wa.Right - __W__ - __M__",    # AirSim 작은 창 3 자리 (숨김, 기본)
               "left": "$wa.Left + __M__"}               # AirSim 작은 창 1 자리


def build_pin_script(width, height, margin=8, wait_s=15.0, title=PIN_TITLE, corner="right"):
    return (PIN_SCRIPT.replace("__X__", PIN_CORNERS[corner])
            .replace("__TITLE__", title).replace("__TRIES__", str(max(1, int(wait_s * 4))))
            .replace("__W__", str(int(width))).replace("__H__", str(int(height)))
            .replace("__M__", str(int(margin))))


def pin_window(width, height, margin=8, wait_s=15.0, title=PIN_TITLE, corner="right"):
    """Windows(WSL 포함): 미니맵 앱 창을 주 화면 오른쪽(corner) 아래에 놓고 항상 위로 둔다. (됐는지, 메시지)."""
    exe = powershell_exe()
    if exe is None:
        return False, "Windows 가 아니라 창 위치는 직접 옮긴다"
    script = build_pin_script(width, height, margin, wait_s, title, corner)
    encoded = base64.b64encode(script.encode("utf-16-le")).decode("ascii")
    try:
        r = subprocess.run([exe, "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass",
                            "-EncodedCommand", encoded],
                           cwd="/mnt/c" if in_wsl() and os.path.isdir("/mnt/c") else None,
                           stdin=subprocess.DEVNULL, capture_output=True, encoding="utf-8", errors="replace",
                           timeout=wait_s + 30)
    except (OSError, subprocess.TimeoutExpired) as e:
        why = f"powershell 실행 실패: {e}"
        if in_wsl() and not interop_on():
            why += " (WSL 에서 Windows 프로그램을 실행하는 기능(interop)이 꺼져 있다)"
        return False, why
    lines = (r.stdout or "").strip().splitlines()
    last = lines[-1].strip() if lines else ""
    if r.returncode == 0 and last.startswith("ok"):
        return True, last
    if last == "notfound":
        return False, f"'{title}' 창을 {wait_s:g}초 안에 찾지 못함 (Edge 앱 창이 아니면 위치를 바꾸지 않는다)"
    return False, last or (r.stderr or "").strip()[:200] or f"종료 코드 {r.returncode}"


def powershell_exe():
    """Windows·WSL 에서 쓸 PowerShell. 그 밖이면 None."""
    if sys.platform.startswith("win"):
        return "powershell"
    if in_wsl():
        return shutil.which("powershell.exe") or "/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe"
    return None


def local_ips():
    """다른 컴퓨터가 이 컴퓨터에 붙을 주소 (127.x 제외)."""
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("10.255.255.255", 1))        # 실제로 보내지 않는다. 바깥으로 나가는 주소만 정한다
            ips.add(s.getsockname()[0])
        finally:
            s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    return sorted(ip for ip in ips if not ip.startswith("127.") and ip != "0.0.0.0")


class App:
    def __init__(self, map_data, tracker=None):
        self.map = map_data
        self.tracker = tracker          # None 이면 데모 (화면이 드론을 가짜로 움직인다)
        self.last_request = None        # 브라우저 화면이 마지막으로 읽어 간 시각 (열어 둔 화면이 있는지 본다)

    def state(self):
        if self.tracker is None:
            state = {"t": round(time.time(), 2), "source": "demo", "note": None, "drones": []}
        else:
            state = self.tracker.snapshot()
        state["map_id"] = self.map["map_id"]
        state["page_id"] = PAGE_ID      # 화면 코드 버전. 열어 둔 화면이 옛 코드면 스스로 새로 불러온다
        return state

    def reset(self):
        if self.tracker is not None:
            self.tracker.reset_trails()


class Handler(BaseHTTPRequestHandler):
    server_version = "SkyBuddyMinimap/1.0"

    def do_GET(self):  # noqa: N802 - http.server 이름 규칙
        app = self.server.app
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            self._send(200, PAGE_HTML, "text/html; charset=utf-8")
        elif path == "/map":
            app.last_request = time.time()
            self._json(app.map)
        elif path == "/state":
            app.last_request = time.time()
            self._json(app.state())
        elif path == "/reset":
            app.reset()
            self._json({"ok": True})
        elif path == "/favicon.ico":
            self._send(204, b"", "image/x-icon")
        else:
            self._send(404, "없는 주소\n".encode("utf-8"), "text/plain; charset=utf-8")

    def _json(self, data):
        self._send(200, json.dumps(data, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if body:
            self.wfile.write(body)

    def log_message(self, fmt, *args):  # 요청마다 찍지 않는다
        pass


class MinimapServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, app):
        self.app = app
        super().__init__(address, Handler)

    def handle_error(self, request, client_address):
        if isinstance(sys.exc_info()[1], (ConnectionError, TimeoutError)):
            return              # 브라우저가 창을 닫으며 끊은 연결
        super().handle_error(request, client_address)


def poll_forever(tracker, period, stop):
    while not stop.is_set():
        started = time.time()
        try:
            tracker.poll_once()
        except Exception as e:  # noqa: BLE001 - 표시 도구라 멈추지 않는다
            tracker.note = f"위치 읽기 오류 ({e})"
        stop.wait(max(0.05, period - (time.time() - started)))


# ---------------------------------------------------------------- 자체 점검

class FakeAirSim:
    """자체 점검용 가짜 AirSim RPC."""

    def __init__(self, frame):
        self.frame = frame
        self.pos = {}
        self.fail = False

    def put(self, name, east, north, up):
        self.pos[name] = (east, north, up)

    def call(self, method, *params):
        if self.fail:
            raise ConnectionError("가짜 연결 끊김")
        if method == "listVehicles":
            return list(self.pos)
        if method == "simGetGroundTruthEnvironment":
            east, north, up = self.pos[params[0]]
            lat, lon = self.frame.geo(east, north)
            return {"geo_point": {"latitude": lat, "longitude": lon, "altitude": self.frame.alt0 + up}}
        raise RuntimeError(f"모르는 메서드 {method}")


def selftest():
    """AirSim·SITL 없이 지도 계산, 위치·꼬리, 텔레메트리 기록 읽기, 웹 주소를 확인한다."""
    import tempfile
    import urllib.error
    import urllib.request

    results = []

    def check(name, ok, detail=""):
        results.append(bool(ok))
        print(f"  {'통과' if ok else '실패'}  {name}" + (f"  ({detail})" if not ok and detail != "" else ""))

    frame = Frame(mp.DEFAULT_ORIGIN)

    def mission_from(corners):
        """{area_id: (동 시작, 동 끝, 북 시작, 북 끝)} -> 임무 (홈 기준 m)."""
        areas = []
        for area_id, (e0, e1, n0, n1) in corners.items():
            boundary = []
            for e, n in ((e0, n0), (e1, n0), (e1, n1), (e0, n1)):
                lat, lon = frame.geo(e, n)
                boundary.append({"latitude": lat, "longitude": lon})
            areas.append({"area_id": area_id, "boundary": boundary, "search_altitude_m": 30})
        return {"mission_id": "mission-test", "search_areas": areas}

    mission = mission_from({"sector-0-0": (50, 175, 50, 175), "sector-1-1": (175, 300, 175, 300)})

    print("지도")
    m = build_map(mission, frame)
    ring = m["sectors"][0]["ring"]
    want = [[50, 50], [175, 50], [175, 175], [50, 175]]
    check("섹터 경계를 홈 기준 동·북 m 로 바꾼다 (sector-0-0 = 동 50~175, 북 50~175)",
          all(abs(a - b) <= 0.2 for p, q in zip(ring, want) for a, b in zip(p, q)), ring)
    b = m["bounds"]
    check("보이는 범위에 홈(0, 0)과 섹터가 다 들어간다", b[0] < 0 < b[2] and b[1] < 0 < b[3] and b[2] > 300 and b[3] > 300, b)
    check("섹터 색이 map_probe.py draw 와 같다", m["sectors"][1]["color"] == rgb255(mp.COLORS[1]))
    four = mission_from({"sector-0-0": (50, 175, 50, 175), "sector-0-1": (175, 300, 50, 175),
                         "sector-1-0": (50, 175, 175, 300), "sector-1-1": (175, 300, 175, 300)})
    labels = {s["id"]: s["label"] for s in build_map(four, frame)["sectors"]}
    check("2x2 섹터 이름 = 서 북 / 남 동 (스폰에서 가장 먼 곳이 북)",
          labels == {"sector-1-0": "서", "sector-1-1": "북", "sector-0-0": "남", "sector-0-1": "동"}, labels)
    four_map = build_map(four, frame)
    by_name = {s["label"]: s["color"] for s in four_map["sectors"]}
    check("2x2 섹터 색 = 북 빨강·남 파랑·동 주황·서 하늘 (이름 기준)",
          by_name == {name: rgb255(c) for name, c in mp.COMPASS_COLORS.items()}, by_name)
    styles = mp.sector_styles(four, frame.lat0, frame.lon0)
    check("AirSim 섹터 선(map_probe.py draw)과 이름·색이 같다",
          all(styles[s["id"]][0] == s["label"] and rgb255(styles[s["id"]][1]) == s["color"]
              for s in four_map["sectors"]), styles)
    part = {s["id"]: s["label"] for s in build_map(four, frame, ["sector-0-0", "sector-0-1"])["sectors"]}
    check("일부 섹터만 보여도 이름은 2x2 전체 기준 (0-0 남, 0-1 동)", part == {"sector-0-0": "남", "sector-0-1": "동"},
          part)
    check("섹터가 4개가 아니면 원래 이름 (0-0)", m["sectors"][0]["label"] == "0-0", m["sectors"][0]["label"])
    nd = build_map(four, frame)["north"]
    check("N 화살표 = 스폰에서 '북' 섹터 쪽 (mission-101 은 오른쪽 위 45도, 시계로 1시 반)",
          abs(nd[0] - 0.7071) < 0.01 and abs(nd[1] - 0.7071) < 0.01 and m["north"] == [0.0, 1.0], (nd, m["north"]))
    only = build_map(mission, frame, ["sector-1-1"])
    check("--sectors 로 지정한 섹터만 보인다", [s["id"] for s in only["sectors"]] == ["sector-1-1"])
    try:
        build_map(mission, frame, ["sector-9-9"])
        check("없는 섹터 이름은 오류", False)
    except ValueError:
        check("없는 섹터 이름은 오류", True)
    with_path = build_map(mission, frame, paths={"sector-0-0": [frame.geo(70, 50), frame.geo(70, 175)]})
    check("데모용 수색 경로도 같은 좌표로 바꾼다", with_path["sectors"][0]["path"] == [[70.0, 50.0], [70.0, 175.0]],
          with_path["sectors"][0]["path"])
    check("JSON 에 못 넣는 값(NaN)은 비운다", _num(float("nan")) is None and _num(None) is None and _num("x") is None)

    print("위치·꼬리")
    clock = [1000.0]
    fake = FakeAirSim(frame)
    fake.put("Copter1", 100, 60, 30)
    fake.put("Copter2", 250, 280, 60)
    tracker = Tracker(frame, ["Copter1", "Copter2"], lambda: fake, clock=lambda: clock[0])
    tracker.poll_once()
    snap = tracker.snapshot()
    by = {d["id"]: d for d in snap["drones"]}
    d1 = by.get("drone-01") or {}
    check("AirSim Copter1 -> drone-01 (동 100, 북 60, 높이 30)",
          d1 and abs(d1["x"] - 100) < 0.2 and abs(d1["y"] - 60) < 0.2 and abs(d1["up"] - 30) < 0.2, d1)
    check("Copter2 -> drone-02, 위치 출처 = airsim", "drone-02" in by and snap["source"] == "airsim", snap)
    clock[0] += 0.2
    fake.put("Copter1", 101, 60, 30)
    tracker.poll_once()
    n1 = len(tracker.trails["drone-01"])
    clock[0] += 0.2
    fake.put("Copter1", 104, 60, 30)
    tracker.poll_once()
    n2 = len(tracker.trails["drone-01"])
    check("2m 넘게 움직였을 때만 꼬리에 점을 더한다", n1 == 1 and n2 == 2, (n1, n2))
    clock[0] += 0.2
    fake.put("Copter1", 0, 0, 0)
    tracker.poll_once()
    check("순간 이동(AirSim 재시작)이면 꼬리를 새로 시작한다", len(tracker.trails["drone-01"]) == 1,
          tracker.trails["drone-01"])
    clock[0] += 0.2
    fake.put("Copter1", 0, 10, 0)
    tracker.poll_once()
    tracker.reset_trails()
    check("꼬리 지우기", all(len(t) == 1 for t in tracker.trails.values()), tracker.trails)

    print("텔레메트리 기록")
    tmp = tempfile.mkdtemp(prefix="minimap_selftest_")
    try:
        stream = os.path.join(tmp, "telemetry.jsonl")
        now = time.time()

        def record(drone, age, bat, link, east, north, armed=True):
            lat, lon = frame.geo(east, north)
            return {"logged_at": datetime.fromtimestamp(now - age, tz=timezone.utc).isoformat(), "drone": drone,
                    "telemetry": {"lat": lat, "lon": lon, "relative_alt_m": 30.0,
                                  "battery_remaining_pct": bat, "link_quality_pct": link},
                    "sim": {"armed": armed, "position": [round(lat, 7), round(lon, 7)], "link_state": "ok"}}

        with open(stream, "w", encoding="utf-8") as f:
            f.write("{깨진 줄\n")
            for r in (record("drone-01", 0.5, 72.0, 100.0, 120, 70), record("drone-02", 10.0, 50.0, 0.0, 260, 290),
                      record("drone-03", 30.0, 100.0, 100.0, 0, 0, armed=False),
                      record("drone-09", 600.0, 10.0, 10.0, 0, 0)):
                f.write(json.dumps(r, ensure_ascii=False) + "\n")
        clock2 = [now]
        fake2 = FakeAirSim(frame)
        fake2.put("Copter1", 120, 70, 30)
        fake2.put("Copter2", 260, 290, 60)
        t2 = Tracker(frame, ["Copter1", "Copter2"], lambda: fake2, stream=stream, clock=lambda: clock2[0])
        t2.poll_once()
        by = {d["id"]: d for d in t2.snapshot()["drones"]}
        d1, d2 = by.get("drone-01") or {}, by.get("drone-02") or {}
        check("고도·배터리·링크 = 기록의 최신 값 (drone-01 30m, 72%, 100%)",
              d1.get("bat") == 72.0 and d1.get("link") == 100.0 and d1.get("alt") == 30.0, d1)
        check("기록이 10초 전이면 age 10 (화면에 '끊김 10초')", d2.get("age") is not None and abs(d2["age"] - 10.0) < 0.3, d2)
        check("2분 넘은 기록(지난 시험)은 보이지 않는다", "drone-09" not in by, sorted(by))
        d3 = by.get("drone-03") or {}
        check("시동 꺼진 드론(이륙 전·착륙 뒤)은 armed=false 로 넘겨 '끊김' 으로 안 보이게 한다",
              d1.get("armed") is True and d3.get("armed") is False, (d1.get("armed"), d3.get("armed")))
        check("화면에 수색 경로 점선·격자·북쪽·축척을 그린다",
              "setLineDash" in PAGE and "niceLength" in PAGE and '"N"' in PAGE)

        fake2.fail = True
        clock2[0] += 0.2
        t2.poll_once()
        snap = t2.snapshot()
        by = {d["id"]: d for d in snap["drones"]}
        d1 = by.get("drone-01") or {}
        check("AirSim 이 끊기면 기록의 위치로 대신한다",
              snap["source"] == "stream" and d1.get("x") is not None and abs(d1["x"] - 120) < 0.5, (snap["source"], d1))
        check("끊긴 이유를 화면 왼쪽 위에 보인다", "AirSim" in (snap["note"] or ""), snap["note"])

        old = os.path.join(tmp, "scenario_a", "telemetry_sections.jsonl")
        new = os.path.join(tmp, "scenario_b", "telemetry_battery.jsonl")
        for i, path in enumerate((old, new)):
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                f.write("\n")
            os.utime(path, (now + 1000 * (i + 1), now + 1000 * (i + 1)))   # 기본 기록보다 늘 나중
        check("--stream 이 없으면 가장 최근 시험 기록을 따라간다", newest_stream(tmp) == new, newest_stream(tmp))

        print("웹 주소")
        app = App(build_map(mission, frame), t2)
        server = MinimapServer(("127.0.0.1", 0), app)
        base = f"http://127.0.0.1:{server.server_address[1]}"
        threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.1}, daemon=True).start()
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))   # 프록시 설정을 타지 않게

        def get(path):
            with opener.open(base + path, timeout=5) as r:
                return r.status, r.headers.get("Content-Type", ""), r.read()

        try:
            status, ctype, body = get("/")
            check("/ = 미니맵 화면 (HTML, 화면 코드 버전 포함)",
                  status == 200 and "text/html" in ctype and b"<canvas" in body and PAGE_ID.encode() in body, status)
            status, ctype, body = get("/map")
            data = json.loads(body)
            check("/map = 섹터 JSON", status == 200 and len(data["sectors"]) == 2 and data["map_id"] == app.map["map_id"])
            status, ctype, body = get("/state")
            data = json.loads(body)
            check("/state = 드론 위치 JSON (+ 화면 코드 버전: 바뀌면 열어 둔 화면이 새로 불러온다)",
                  status == 200 and data["map_id"] == app.map["map_id"] and data.get("page_id") == PAGE_ID
                  and {d["id"] for d in data["drones"]} >= {"drone-01", "drone-02"}, data)
            status, ctype, body = get("/reset")
            check("/reset = 꼬리 지우기", status == 200 and json.loads(body).get("ok") is True)
            try:
                get("/nope")
                check("없는 주소는 404", False)
            except urllib.error.HTTPError as e:
                check("없는 주소는 404", e.code == 404, e.code)
        finally:
            server.shutdown()
            server.server_close()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    check("화면에 '항상 위에 띄우기'(Document PiP)와 OBS 투명 모드가 있다",
          "documentPictureInPicture" in PAGE and "overlay" in PAGE)
    script = build_pin_script(360, 430)
    check("Windows 창 고정: 찾는 제목 = 화면 제목, 자리표시가 다 채워짐 (오른쪽 아래 = 숨긴 작은 창 3 자리·항상 위)",
          f"<title>{PIN_TITLE}</title>" in PAGE and "__" not in script and "SetWindowPos" in script
          and "$x = $wa.Right - 360 - 8" in script and "- 430 - 8" in script)
    left = build_pin_script(360, 430, corner="left")
    check("--corner left 면 왼쪽 아래 (작은 창 1 자리)", "__" not in left and "$x = $wa.Left + 8" in left)
    drone_rgb = [rgb255(c) for c in td.DRONE_COLORS.values()]
    sector_rgb = [rgb255(c) for c in list(mp.COMPASS_COLORS.values()) + mp.COLORS]
    check("섹터 색이 드론 색(초록·연노랑)과 겹치지 않는다",
          all(sum(abs(a - b) for a, b in zip(s, d)) > 120 for s in sector_rgb for d in drone_rgb))

    passed = sum(results)
    print(f"자체 점검: {passed}/{len(results)} 통과")
    return 0 if passed == len(results) else 1


# ---------------------------------------------------------------- 실행

def main(argv=None):
    parser = argparse.ArgumentParser(prog="minimap.py", description="섹터와 드론 위치를 보여 주는 미니맵 (브라우저)",
                                     formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    parser.add_argument("--mission", default="mission-101", help="임무 파일 경로나 이름 (기본 mission-101)")
    parser.add_argument("--sectors", help="보일 섹터, 쉼표로 (예: sector-0-0,sector-1-1). 기본: 임무의 섹터 전부")
    parser.add_argument("--stream", help="고도·배터리·링크 값을 읽을 텔레메트리 기록 "
                                         "(기본: 가장 최근에 쓰인 기록. SKYBUDDY_TELEMETRY_LOG 가 있으면 그 파일)")
    parser.add_argument("--source", choices=("auto", "airsim", "stream"), default="auto",
                        help="드론 위치를 어디서 읽나. auto = AirSim, 안 되면 텔레메트리 기록 (기본)")
    parser.add_argument("--drones-config", help="수색 경로(점선) 계산에 쓸 미들웨어 드론 설정 "
                                                "(기본: MIDDLEWARE_DRONES_CONFIG, 없으면 scenarios/drones.sitl.json)")
    parser.add_argument("--no-paths", action="store_true", help="섹터 안 지그재그 수색 경로(점선)를 그리지 않는다")
    parser.add_argument("--hz", type=float, default=3.0,
                        help="1초에 위치를 읽는 횟수 (기본 3. AirSim API 호출이 많으면 화면이 끊겨 5에서 줄임)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"미니맵 주소의 포트 (기본 {DEFAULT_PORT})")
    parser.add_argument("--bind", default=None,
                        help="미니맵을 열 주소. 기본: 화면 있는 컴퓨터는 127.0.0.1 (이 컴퓨터에서만), "
                             "화면 없는 서버는 0.0.0.0 (다른 컴퓨터의 브라우저에서 연다)")
    parser.add_argument("--host", default=mp.DEFAULT_HOST, help="AirSim 주소 (기본: AIRSIM_HOST, 없으면 127.0.0.1)")
    parser.add_argument("--airsim-port", type=int, default=None,
                        help="AirSim API 포트 (기본: settings.json 의 ApiServerPort)")
    parser.add_argument("--map", help="기체 이름 -> 드론 ID (기본: 설정 순서대로 drone-01, drone-02). "
                                      "예: Copter1=drone-01,Copter2=drone-02")
    parser.add_argument("--no-open", action="store_true",
                        help="브라우저 창을 자동으로 열지 않는다 (기본: 2.5초 안에 붙은 화면이 없으면 Edge 작은 창을 연다)")
    parser.add_argument("--window", default="360x430",
                        help="Windows 에서 미니맵 창 크기 (화면 배율 기준 픽셀, 기본 360x430). 주 화면 아래 구석에 항상 위로 둔다")
    parser.add_argument("--corner", choices=sorted(PIN_CORNERS), default="right",
                        help="미니맵 창을 둘 아래 구석. right = AirSim 작은 창 3 자리 (기본, 숨긴 창), left = 작은 창 1 자리")
    parser.add_argument("--no-pin", action="store_true", help="미니맵 창을 화면 구석에 고정하지 않는다")
    parser.add_argument("--demo", action="store_true", help="AirSim 없이 드론을 가짜로 움직인다 (화면 확인용)")
    parser.add_argument("--selftest", action="store_true", help="자체 점검 (AirSim·SITL 불필요)")
    args = parser.parse_args(argv)
    if args.selftest:
        return selftest()
    if args.hz <= 0:
        parser.error("--hz 는 0보다 커야 한다")
    try:
        win_w, win_h = (int(v) for v in args.window.lower().split("x"))
        if win_w < 200 or win_h < 200:
            raise ValueError
    except ValueError:
        parser.error("--window 는 360x430 처럼 가로x세로 (각 200 이상)")

    origin, vehicles, settings_port, settings_path = mp.load_settings()
    frame = Frame(origin)
    try:
        mission_file = find_mission(args.mission)
        with open(mission_file, encoding="utf-8") as f:
            mission = json.load(f)
    except (OSError, ValueError) as e:
        sys.exit(f"[오류] {e}")
    sectors = [s.strip() for s in (args.sectors or "").split(",") if s.strip()]
    paths = {}
    if not args.no_paths:   # 섹터 안 지그재그 점선 (데모 드론도 이 길로 움직인다)
        config = os.path.expanduser(args.drones_config or os.environ.get("MIDDLEWARE_DRONES_CONFIG")
                                    or os.path.join(mp.BASE, "scenarios", "drones.sitl.json"))
        try:
            paths, info = mp.coverage_paths(mission, config)
            print(f"수색 경로(점선): 미들웨어 함수로 계산 ({info})")
        except Exception as e:  # noqa: BLE001 - 경로 없이 섹터만 그려도 된다
            print(f"수색 경로(점선)는 그리지 않는다: {e}")
            paths = {}
    try:
        map_data = build_map(mission, frame, sectors, paths, demo=args.demo)
    except ValueError as e:
        sys.exit(f"[오류] {e}")

    tracker = None
    airsim_port = args.airsim_port or settings_port
    if not args.demo:
        td.drone_mapping(list(vehicles), args.map)      # --map 형식이 틀리면 여기서 끝낸다
        host = args.host
        # --stream·SKYBUDDY_TELEMETRY_LOG 둘 다 없으면 가장 최근 기록을 따라간다 (미니맵을 먼저 켜 두고
        # run_live.sh 를 띄워도 그 실행의 기록을 읽는다)
        follow = not args.stream and not os.environ.get("SKYBUDDY_TELEMETRY_LOG")
        tracker = Tracker(frame, vehicles, lambda: mp.Rpc(host, airsim_port, timeout=5),
                          stream=args.stream or (newest_stream() if follow else stream_log_path()),
                          source=args.source, override=args.map, follow=follow)
    app = App(map_data, tracker)
    screen = has_display()
    bind = args.bind or ("127.0.0.1" if screen else "0.0.0.0")
    try:
        server = MinimapServer((bind, args.port), app)
    except OSError as e:
        sys.exit(f"[오류] 포트 {args.port} 를 열지 못했다: {e}\n"
                 f"  이미 켜 둔 미니맵이 있으면 그 화면을 그대로 쓰거나, --port 로 다른 번호를 줄 것")
    stop = threading.Event()
    if tracker is not None:
        threading.Thread(target=poll_forever, args=(tracker, 1.0 / args.hz, stop), daemon=True).start()

    shown = ", ".join(f"{s['id']}({s['label']})" for s in map_data["sectors"]) or "없음"
    print(f"임무 {map_data['mission_id']} ({mission_file})")
    print(f"섹터: {shown}")
    if tracker is None:
        print("데모: AirSim 없이 드론을 가짜로 움직인다 (실제 드론과 무관)")
    else:
        print(f"드론 위치: AirSim {args.host}:{airsim_port} ({settings_path or '기본 설정'})"
              + (", 안 되면 텔레메트리 기록" if args.source == "auto" else f" (출처 {args.source})"))
        if tracker.follow:
            print(f"고도·배터리·링크: 가장 최근 텔레메트리 기록을 따라간다 (지금 {tracker.stream})")
        else:
            print(f"고도·배터리·링크: {tracker.stream or '없음 (SKYBUDDY_TELEMETRY_LOG=0)'}")
    url = f"http://localhost:{args.port}"
    print(f"미니맵: {url}   <- Edge·Chrome 에서 열고 '항상 위에 띄우기'")
    remote = []
    if bind not in ("127.0.0.1", "localhost"):
        remote = [f"http://{ip}:{args.port}" for ip in local_ips()]
        for r in remote:
            print(f"  다른 컴퓨터에서: {r}   (방화벽·AWS 보안 그룹에서 TCP {args.port} 허용 필요)")
    print(f"OBS 녹화용 (배경 투명): {url}/?overlay=1")
    print("Ctrl+C 로 끝낸다", flush=True)
    if not args.no_open:
        def pin():
            # Windows·WSL: 미니맵 앱 창을 화면 오른쪽 아래(AirSim 작은 창 3 자리, 숨긴 창)에 항상 위로 둔다
            if args.no_pin or not (in_wsl() or sys.platform.startswith("win")):
                return
            ok, msg = pin_window(win_w, win_h, corner=args.corner)
            if ok:
                where = "왼쪽" if args.corner == "left" else "오른쪽"
                print(f"미니맵 창을 화면 {where} 아래에 항상 위로 고정했다 ({win_w}x{win_h})", flush=True)
            else:
                print(f"미니맵 창 위치는 그대로 둔다: {msg}", flush=True)

        def open_if_nobody():
            # 이미 열어 둔 화면이 있으면 0.2초마다 읽어 가므로 새 창을 열지 않는다
            # (시나리오마다 run_live.sh 를 다시 켜도 창이 늘지 않는다)
            if app.last_request is not None:
                print("열어 둔 미니맵 화면이 이어서 보여 준다 (새 창은 안 연다)", flush=True)
                pin()
                return
            if not screen:
                where = remote[0] if remote else f"http://<이 서버 주소>:{args.port}"
                print(f"브라우저 창을 띄울 화면이 없다 (서버). 보는 컴퓨터(AirSim 화면 등)의 Edge·Chrome 에서 {where} 를 연다\n"
                      f"  보안 그룹을 못 열면 보는 컴퓨터에서: ssh -L {args.port}:localhost:{args.port} <이 서버> 후 {url}",
                      flush=True)
                return
            ok, why = open_browser(f"{url}/?bare=1")      # bare: 버튼 줄 없이 지도와 데이터만
            if ok:
                print("브라우저 창을 열었다 (Edge 작은 창). 안 보이면 위 주소를 직접 연다", flush=True)
                pin()
            else:
                print(f"브라우저 창을 자동으로 열지 못했다: {why}\n  Edge·Chrome 에서 {url} 을 직접 연다", flush=True)
        timer = threading.Timer(2.5, open_if_nobody)
        timer.daemon = True
        timer.start()
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
    print("미니맵을 껐다")
    return 0


# ---------------------------------------------------------------- 브라우저 화면

PAGE = r"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SkyBuddy Minimap</title>
<style>
html, body { margin: 0; height: 100%; background: #0b0f16; color: #d8dee9; overflow: hidden;
  font-family: "Malgun Gothic", "Segoe UI", system-ui, sans-serif; }
body { display: flex; flex-direction: column; }
#bar { display: flex; align-items: center; gap: 8px; padding: 6px 8px; font-size: 12px; color: #9aa4b2; }
#bar button { font: inherit; color: #e6ebf2; background: #1d2633; border: 1px solid #3a4658;
  border-radius: 6px; padding: 4px 10px; cursor: pointer; white-space: nowrap; }
#bar button:hover { background: #263244; }
#bar button:disabled { opacity: 0.5; cursor: default; }
#hint { margin-left: auto; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; min-width: 0; }
#wrap { flex: 1; min-height: 0; position: relative; }
#map { position: absolute; inset: 0; width: 100%; height: 100%; display: block; }
body.overlay { background: transparent; }
body.overlay #bar, body.bare #bar { display: none; }
</style>
</head>
<body>
<div id="bar">
  <button id="pip" type="button">항상 위에 띄우기</button>
  <button id="clear" type="button">꼬리 지우기</button>
  <button id="data" type="button">지도만 보기</button>
  <span id="hint"></span>
</div>
<div id="wrap"><canvas id="map"></canvas></div>
<script>
"use strict";
// SkyBuddy 미니맵. 서버(minimap.py)의 /map(섹터) 와 /state(드론 위치·값) 를 0.2초마다 읽어 그린다.
const qs = new URLSearchParams(location.search);
const OVERLAY = qs.has("overlay");          // OBS 브라우저 소스용: 배경 투명, 버튼 숨김
const FORCE_DEMO = qs.has("demo");          // 서버 지도 그대로, 드론만 가짜로 움직인다
let DATA = qs.get("data") !== "0";          // 아래 데이터 줄(드론별 고도·배터리·링크). ?data=0 이면 지도만
const EMBED = window.MINIMAP_EMBED || null; // 미리보기용 (서버 없이)
const PAGE_ID = "__PAGE_ID__";              // 서버가 화면 코드 버전으로 바꿔 넣는다
const PAGE_PLACEHOLDER = "__PAGE_" + "ID__";
const PERIOD_MS = 200;
const STALE_S = 3, GONE_S = 120;
const FONT = '"Malgun Gothic","Segoe UI",system-ui,sans-serif';
const WHITE = [235, 240, 248];
const PIP_CSS = "html,body{margin:0;height:100%;background:#0b0f16;overflow:hidden}" +
  "#wrap{position:relative;height:100%}#map{position:absolute;inset:0;width:100%;height:100%;display:block}";

const wrap = document.getElementById("wrap");
const canvas = document.getElementById("map");
const pipBtn = document.getElementById("pip");
const clearBtn = document.getElementById("clear");
const dataBtn = document.getElementById("data");
const hint = document.getElementById("hint");
if (OVERLAY) document.body.classList.add("overlay");
if (qs.has("bare")) document.body.classList.add("bare");   // 자동으로 연 Edge 작은 창: 버튼 줄 없이 지도와 데이터만

let MAP = null, STATE = null, serverDown = false;
let hostWin = window, pipWin = null, gen = 0, demo = null;

const rgba = (c, a) => `rgba(${c[0]},${c[1]},${c[2]},${a})`;
const colorOf = id => (MAP && MAP.drone_colors && MAP.drone_colors[id]) || WHITE;
const numberOf = id => { const m = String(id).match(/(\d+)\D*$/); return m ? String(parseInt(m[1], 10)) : String(id).slice(0, 1).toUpperCase(); };
const isDemo = () => !!(MAP && (MAP.demo || FORCE_DEMO));

async function getJSON(name) {
  const ctl = new AbortController();
  const timer = setTimeout(() => ctl.abort(), 2000);
  try {
    const r = await fetch(name, { cache: "no-store", signal: ctl.signal });
    if (!r.ok) throw new Error("HTTP " + r.status);
    return await r.json();
  } finally {
    clearTimeout(timer);
  }
}

// ---------------------------------------------------------------- 데모 (AirSim 없이 화면 확인)
function demoInit() {
  const secs = MAP.sectors.length ? MAP.sectors : [{ ring: [[60, 60], [140, 60], [140, 140], [60, 140]], path: [] }];
  const ids = Object.keys(MAP.drone_colors || {});
  if (!ids.length) ids.push("drone-01", "drone-02");
  demo = { t0: performance.now(), trails: {}, downSince: {}, drones: ids.map((id, i) => {
    const s = secs[i % secs.length];
    const pts = s.path && s.path.length > 1 ? s.path : s.ring.concat([s.ring[0]]);
    const route = [[0, 0]].concat(pts, [[0, 0]]);
    const cum = [0];
    for (let k = 1; k < route.length; k++)
      cum.push(cum[k - 1] + Math.hypot(route[k][0] - route[k - 1][0], route[k][1] - route[k - 1][1]));
    return { id, route, cum, total: cum[cum.length - 1] || 1, speed: 14 + 4 * i, downM: i === 1 ? 330 : null, lastS: 0 };
  }) };
}

function demoState() {
  const t = (performance.now() - demo.t0) / 1000;
  const drones = demo.drones.map((d, idx) => {
    const s = (t * d.speed) % d.total;
    if (s < d.lastS) demo.trails[d.id] = [];
    d.lastS = s;
    let k = 1;
    while (k < d.cum.length - 1 && d.cum[k] < s) k++;
    const a = d.route[k - 1], b = d.route[k];
    const f = Math.min(1, Math.max(0, (s - d.cum[k - 1]) / ((d.cum[k] - d.cum[k - 1]) || 1)));
    const x = a[0] + (b[0] - a[0]) * f, y = a[1] + (b[1] - a[1]) * f;
    const tr = demo.trails[d.id] || (demo.trails[d.id] = []);
    const last = tr[tr.length - 1];
    if (!last || Math.hypot(x - last[0], y - last[1]) >= 2) tr.push([x, y]);
    const dist = Math.hypot(x, y);
    let link = 100, age = 0.2;
    if (d.downM) {
      link = Math.max(0, Math.round(100 * (d.downM - dist) / d.downM));
      if (dist > d.downM) {
        if (demo.downSince[d.id] == null) demo.downSince[d.id] = t;
        age = Math.max(age, t - demo.downSince[d.id]);
      } else {
        demo.downSince[d.id] = null;
      }
    }
    return { id: d.id, x, y, up: 30, alt: 30 + 10 * idx, bat: Math.max(0, Math.round(100 - 75 * s / d.total)), link, age, trail: tr };
  });
  return { source: "demo", note: null, drones };
}

// ---------------------------------------------------------------- 읽기
async function update() {
  if (!MAP) {
    if (EMBED) {
      MAP = EMBED.map;
    } else {
      try { MAP = await getJSON("map"); serverDown = false; } catch (e) { serverDown = true; return; }
    }
  }
  if (!EMBED) {
    try {
      const s = await getJSON("state");
      serverDown = false;
      if (s.page_id && PAGE_ID !== PAGE_PLACEHOLDER && s.page_id !== PAGE_ID) {
        location.reload();      // 서버의 화면 코드가 바뀌었다 (미니맵을 고친 뒤 다시 켬) -> 새 화면으로
        return;
      }
      if (s.map_id && s.map_id !== MAP.map_id) {   // 서버를 다시 켰다 (시험마다 섹터가 다르다. 데모 -> 실제도 여기서 바뀐다)
        MAP = await getJSON("map");
        demo = null;
      }
      if (!isDemo()) STATE = s;
    } catch (e) {
      serverDown = true;
    }
  }
  if (isDemo()) {
    if (!demo) demoInit();
    STATE = demoState();
  }
}

// 화면을 가린 창의 타이머는 브라우저가 늦춘다. 작은 창이 열려 있으면 그 창의 타이머로 돈다.
function kick() {
  const my = ++gen;
  const step = async () => {
    if (my !== gen) return;
    try { await update(); } catch (e) { console.error(e); }
    try { draw(); } catch (e) { console.error(e); }
    if (my === gen) hostWin.setTimeout(step, PERIOD_MS);
  };
  hostWin.setTimeout(step, 0);
}

// ---------------------------------------------------------------- 그리기
function roundRect(g, x, y, w, h, r) {
  g.beginPath();
  g.moveTo(x + r, y);
  g.arcTo(x + w, y, x + w, y + h, r);
  g.arcTo(x + w, y + h, x, y + h, r);
  g.arcTo(x, y + h, x, y, r);
  g.arcTo(x, y, x + w, y, r);
  g.closePath();
}

function clip(g, text, maxW) {
  if (g.measureText(text).width <= maxW) return text;
  let t = text;
  while (t.length > 1 && g.measureText(t + "…").width > maxW) t = t.slice(0, -1);
  return t + "…";
}

function note(g, text, W) {   // 문제(서버 꺼짐, AirSim 연결 대기 등)가 있을 때만 왼쪽 위에 작게
  g.font = `600 11px ${FONT}`;
  const t = clip(g, text, W - 40);
  roundRect(g, 10, 10, g.measureText(t).width + 16, 20, 10);
  g.fillStyle = "rgba(10,14,20,0.88)";
  g.fill();
  g.fillStyle = "#ffb347";
  g.textAlign = "left";
  g.fillText(t, 18, 20);
}

function heading(d) {         // 최근 이동 방향 (화면 각도). 3m 넘게 움직인 적이 없으면 null
  const tr = d.trail || [];
  for (let i = tr.length - 1, n = 0; i >= 0 && n < 12; i--, n++) {
    const dx = d.x - tr[i][0], dy = d.y - tr[i][1];
    if (Math.hypot(dx, dy) >= 3) return Math.atan2(-dy, dx);
  }
  return null;
}

const GRAY = "rgba(200,208,220,0.55)";

function niceLength(maxM) {
  let best = 10;
  for (const v of [10, 20, 50, 100, 200, 500, 1000, 2000, 5000]) if (v <= maxM) best = v;
  return best;
}

// 비행 중(또는 모름)인데 기록이 3초 넘게 없다 = 통신 두절 의심.
// 이륙 전·착륙 뒤(시동 꺼짐, 마지막 고도 1m 미만)에는 미들웨어가 텔레메트리를 안 읽어 기록이 없는 것이라 끊김이 아니다.
function isLost(d) {
  const stale = d.age != null && d.age > STALE_S && d.age <= GONE_S;
  const grounded = d.armed === false || (d.alt != null && d.alt < 1.0);
  return stale && !grounded;
}

function statusSegments(d) {
  if (d.age == null || d.age > GONE_S) return [["기록 없음", GRAY]];
  if (isLost(d)) return [[`끊김 ${Math.round(d.age)}초`, "#ff5a5a"]];
  const idle = d.age > STALE_S;     // 시동 꺼짐: 마지막으로 받은 값을 흐리게
  const out = [];
  if (d.alt != null) out.push([`고도 ${Math.round(d.alt)}m`, idle ? GRAY : "#e0e6ee"]);
  if (d.bat != null) out.push([`배터리 ${Math.round(d.bat)}%`, idle ? GRAY : d.bat < 20 ? "#ff5a5a" : d.bat < 40 ? "#ffc247" : "#cfe8cf"]);
  if (d.link != null) out.push([`링크 ${Math.round(d.link)}%`, idle ? GRAY : d.link < 30 ? "#ff5a5a" : d.link < 60 ? "#ffc247" : "#cfe0f5"]);
  return out.length ? out : [["값 없음", GRAY]];
}

function spread(pins) {       // 화면에서 14px 안으로 겹친 핀을 가로로 16px 간격으로 벌린다
  const groups = [];
  for (const p of pins) {
    const grp = groups.find(q => Math.hypot(q.cx - p.px, q.cy - p.py) < 14);
    if (grp) {
      grp.items.push(p);
      grp.cx = grp.items.reduce((a, q) => a + q.px, 0) / grp.items.length;
      grp.cy = grp.items.reduce((a, q) => a + q.py, 0) / grp.items.length;
    } else {
      groups.push({ cx: p.px, cy: p.py, items: [p] });
    }
  }
  for (const grp of groups) {
    if (grp.items.length < 2) continue;
    grp.items.forEach((p, k) => { p.px = grp.cx + (k - (grp.items.length - 1) / 2) * 16; p.py = grp.cy; });
  }
}

function polyline(g, pts, P) {
  g.beginPath();
  pts.forEach((p, i) => { const q = P(p[0], p[1]); if (i) g.lineTo(q[0], q[1]); else g.moveTo(q[0], q[1]); });
}

function draw() {
  const win = canvas.ownerDocument.defaultView || window;
  const dpr = win.devicePixelRatio || 1;
  const W = canvas.clientWidth, H = canvas.clientHeight;
  if (!W || !H) return;
  const bw = Math.round(W * dpr), bh = Math.round(H * dpr);
  if (canvas.width !== bw || canvas.height !== bh) { canvas.width = bw; canvas.height = bh; }
  const g = canvas.getContext("2d");
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, W, H);
  g.lineJoin = "round";
  g.lineCap = "round";
  g.textBaseline = "middle";
  roundRect(g, 0.5, 0.5, W - 1, H - 1, 10);
  g.fillStyle = OVERLAY ? "rgba(10,14,20,0.80)" : "rgba(14,19,27,0.96)";
  g.fill();
  g.strokeStyle = "rgba(255,255,255,0.16)";
  g.lineWidth = 1;
  g.stroke();

  // 평소에는 글씨 없이 지도만. 문제가 있을 때만 왼쪽 위에 한 줄
  let warning = null;
  if (!MAP) warning = serverDown ? "미니맵 서버에 연결하는 중…" : "지도 불러오는 중…";
  else if (serverDown) warning = "미니맵 서버 꺼짐 · 마지막 화면";
  else if (STATE && STATE.note) warning = STATE.note;
  if (!MAP) { note(g, warning, W); return; }

  const drones = (STATE && STATE.drones) || [];
  const rows = DATA ? Math.max(drones.length, 1) : 0;
  const dataH = rows ? 10 + 18 * rows : 0;
  const area = { x: 6, y: 6, w: W - 12, h: H - 12 - dataH };
  if (area.w < 40 || area.h < 40) return;
  const [x0, y0, x1, y1] = MAP.bounds;
  const s = Math.min(area.w / (x1 - x0), area.h / (y1 - y0));
  const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
  const ax = area.x + area.w / 2, ay = area.y + area.h / 2;
  const P = (x, y) => [ax + (x - cx) * s, ay - (y - cy) * s];

  g.save();
  roundRect(g, area.x, area.y, area.w, area.h, 6);
  g.fillStyle = "rgba(255,255,255,0.03)";
  g.fill();
  g.clip();

  // 격자
  const wx0 = cx - area.w / 2 / s, wx1 = cx + area.w / 2 / s, wy0 = cy - area.h / 2 / s, wy1 = cy + area.h / 2 / s;
  const step = niceLength((wx1 - wx0) / 6);
  g.strokeStyle = "rgba(255,255,255,0.05)";
  g.lineWidth = 1;
  g.beginPath();
  for (let x = Math.ceil(wx0 / step) * step; x <= wx1; x += step) { const px = P(x, 0)[0]; g.moveTo(px, area.y); g.lineTo(px, area.y + area.h); }
  for (let y = Math.ceil(wy0 / step) * step; y <= wy1; y += step) { const py = P(0, y)[1]; g.moveTo(area.x, py); g.lineTo(area.x + area.w, py); }
  g.stroke();

  // 섹터 (색은 AirSim 화면의 섹터 선과 같다) + 지그재그 수색 경로 점선 (미들웨어와 같은 함수로 만든 길)
  for (const sec of MAP.sectors) {
    if (!sec.ring || sec.ring.length < 2) continue;
    const c = sec.color || WHITE;
    polyline(g, sec.ring, P);
    g.closePath();
    g.fillStyle = rgba(c, 0.10);
    g.fill();
    g.strokeStyle = rgba(c, 0.85);
    g.lineWidth = 1.2;
    g.stroke();
    if (sec.path && sec.path.length > 1) {
      g.save();
      g.setLineDash([4, 4]);
      polyline(g, sec.path, P);
      g.strokeStyle = rgba(c, 0.4);
      g.lineWidth = 1;
      g.stroke();
      g.restore();
    }
  }
  // 섹터 이름 (2x2 는 서·북·남·동) + 탐색 고도
  g.textAlign = "center";
  g.font = `700 12px ${FONT}`;
  for (const sec of MAP.sectors) {
    if (!sec.ring || !sec.ring.length) continue;
    const mx = sec.ring.reduce((a, p) => a + p[0], 0) / sec.ring.length;
    const my = sec.ring.reduce((a, p) => a + p[1], 0) / sec.ring.length;
    const q = P(mx, my);
    const label = (sec.label || String(sec.id).replace(/^sector-/, "")) + (sec.alt_m != null ? ` · ${sec.alt_m}m` : "");
    g.lineWidth = 3;
    g.strokeStyle = "rgba(10,14,20,0.85)";
    g.strokeText(label, q[0], q[1]);
    g.fillStyle = rgba(sec.color || WHITE, 0.95);
    g.fillText(label, q[0], q[1]);
  }

  // 홈
  const h = P(MAP.home[0], MAP.home[1]);
  g.beginPath();
  g.arc(h[0], h[1], 7, 0, Math.PI * 2);
  g.fillStyle = "rgba(10,14,20,0.85)";
  g.fill();
  g.strokeStyle = "rgba(255,255,255,0.85)";
  g.lineWidth = 1.5;
  g.stroke();
  g.fillStyle = "#ffffff";
  g.font = `700 9px ${FONT}`;
  g.fillText("H", h[0], h[1] + 0.5);

  // 지나간 길 (가는 선)
  for (const d of drones) {
    const tr = (d.trail || []).slice();
    if (d.x != null) tr.push([d.x, d.y]);
    if (tr.length < 2) continue;
    polyline(g, tr, P);
    g.strokeStyle = rgba(colorOf(d.id), 0.7);
    g.lineWidth = 1.5;
    g.stroke();
  }
  g.restore();

  // 북쪽(N) 표시: 스폰(홈)에서 '북' 섹터(가장 먼 섹터) 쪽을 가리킨다. 2x2 가 아니면 지도 위쪽(진북)
  const nd = MAP.north || [0, 1];
  const ang = Math.atan2(nd[0], nd[1]);       // 화면 위쪽에서 시계 방향 각도 (동 = x, 북 = y)
  const nx = area.x + area.w - 20, ny = area.y + 18;
  g.save();
  g.translate(nx, ny);
  g.rotate(ang);
  g.beginPath();
  g.moveTo(0, -10); g.lineTo(5, 4); g.lineTo(0, 1); g.lineTo(-5, 4);
  g.closePath();
  g.fillStyle = "rgba(255,255,255,0.8)";
  g.fill();
  g.restore();
  g.font = `700 9px ${FONT}`;
  g.textAlign = "center";
  g.fillStyle = "rgba(255,255,255,0.8)";
  g.fillText("N", nx, ny + 16);              // 글자는 화살표 아래 (화살표만 기운다)

  // 축척
  const len = niceLength(area.w * 0.3 / s);
  const sx0 = area.x + 10, sy0 = area.y + area.h - 10;
  g.strokeStyle = "rgba(255,255,255,0.75)";
  g.lineWidth = 1.5;
  g.beginPath();
  g.moveTo(sx0, sy0 - 4); g.lineTo(sx0, sy0); g.lineTo(sx0 + len * s, sy0); g.lineTo(sx0 + len * s, sy0 - 4);
  g.stroke();
  g.fillStyle = "rgba(255,255,255,0.75)";
  g.font = `600 10px ${FONT}`;
  g.textAlign = "left";
  g.fillText(`${len} m`, sx0 + 4, sy0 - 9);

  // 드론 핀: 번호 원 + 이동 방향 삼각형. 비행 중인데 기록이 끊기면 빨간 테두리가 깜박인다.
  const blink = Math.floor(performance.now() / 500) % 2 === 0;
  const pins = [];
  for (const d of drones) {
    if (d.x == null || d.y == null) continue;
    let [px, py] = P(d.x, d.y);
    const inside = px >= area.x && px <= area.x + area.w && py >= area.y && py <= area.y + area.h;
    if (!inside) {          // 지도 밖이면 가장자리에 흐리게
      px = Math.min(Math.max(px, area.x + 9), area.x + area.w - 9);
      py = Math.min(Math.max(py, area.y + 9), area.y + area.h - 9);
    }
    pins.push({ d, px, py, inside });
  }
  spread(pins);             // 같은 자리(홈 등)에 겹치면 옆으로 벌린다
  for (const { d, px, py, inside } of pins) {
    const c = colorOf(d.id);
    g.globalAlpha = inside ? 1 : 0.55;
    const hd = heading(d);
    if (hd != null) {
      g.beginPath();
      g.moveTo(px + Math.cos(hd) * 16, py + Math.sin(hd) * 16);
      g.lineTo(px + Math.cos(hd + 0.45) * 10, py + Math.sin(hd + 0.45) * 10);
      g.lineTo(px + Math.cos(hd - 0.45) * 10, py + Math.sin(hd - 0.45) * 10);
      g.closePath();
      g.fillStyle = rgba(c, 1);
      g.fill();
    }
    if (isLost(d) && blink) {
      g.beginPath();
      g.arc(px, py, 12.5, 0, Math.PI * 2);
      g.strokeStyle = "#ff4d4d";
      g.lineWidth = 2.5;
      g.stroke();
    }
    g.beginPath();
    g.arc(px, py, 8, 0, Math.PI * 2);
    g.fillStyle = rgba(c, 1);
    g.fill();
    g.strokeStyle = "#ffffff";
    g.lineWidth = 2;
    g.stroke();
    g.fillStyle = "#14171c";
    g.font = `800 10px ${FONT}`;
    g.textAlign = "center";
    g.fillText(numberOf(d.id), px, py + 0.5);
    g.globalAlpha = 1;
  }

  // 아래: 드론별 고도·배터리·링크 (미들웨어가 받은 값)
  if (DATA) {
    const top = area.y + area.h + 5;
    g.strokeStyle = "rgba(255,255,255,0.08)";
    g.lineWidth = 1;
    g.beginPath();
    g.moveTo(12, top + 0.5);
    g.lineTo(W - 12, top + 0.5);
    g.stroke();
    let ly = top + 14;
    if (!drones.length) {
      g.fillStyle = "rgba(220,228,240,0.6)";
      g.font = `600 11px ${FONT}`;
      g.textAlign = "left";
      g.fillText("드론 위치 없음", 12, ly);
    }
    for (const d of drones) {
      g.beginPath();
      g.arc(16, ly, 5, 0, Math.PI * 2);
      g.fillStyle = rgba(colorOf(d.id), 1);
      g.fill();
      g.fillStyle = "#e8edf4";
      g.font = `700 11px ${FONT}`;
      g.textAlign = "left";
      g.fillText(d.id, 26, ly);
      const segs = statusSegments(d);
      let rx = W - 12;
      g.textAlign = "right";
      g.font = `600 11px ${FONT}`;
      for (let k = segs.length - 1; k >= 0; k--) {
        g.fillStyle = segs[k][1];
        g.fillText(segs[k][0], rx, ly);
        rx -= g.measureText(segs[k][0]).width + 8;
      }
      ly += 18;
    }
  }
  if (warning) note(g, warning, W);
}

// ---------------------------------------------------------------- 버튼
const pipOK = "documentPictureInPicture" in window;
if (!pipOK) {
  pipBtn.disabled = true;
  pipBtn.title = "Chrome·Edge 116 이상에서 된다";
}
hint.textContent = pipOK ? "AirSim 위에 띄우려면 '항상 위에 띄우기'" : "이 브라우저는 '항상 위' 창을 지원하지 않음 (Chrome·Edge 116 이상)";

pipBtn.addEventListener("click", async () => {
  if (pipWin) { pipWin.close(); return; }
  try {
    const w = await window.documentPictureInPicture.requestWindow(
      DATA ? { width: 320, height: 370 } : { width: 320, height: 320 });
    const st = w.document.createElement("style");
    st.textContent = PIP_CSS;
    w.document.head.append(st);
    w.document.title = "SkyBuddy 미니맵";
    w.document.body.append(wrap);
    w.addEventListener("resize", () => draw());
    pipWin = w;
    hostWin = w;
    pipBtn.textContent = "작은 창 닫기";
    hint.textContent = "작은 창을 오른쪽 아래(AirSim 작은 창 3 자리)로 끌어다 두면 된다";
    w.addEventListener("pagehide", () => {
      document.body.append(wrap);
      pipWin = null;
      hostWin = window;
      pipBtn.textContent = "항상 위에 띄우기";
      hint.textContent = "AirSim 위에 띄우려면 '항상 위에 띄우기'";
      kick();
    });
    kick();
  } catch (e) {
    hint.textContent = "작은 창을 열지 못했다: " + e.message;
  }
});

clearBtn.addEventListener("click", async () => {
  if (isDemo() && demo) { demo.trails = {}; return; }
  try { await getJSON("reset"); } catch (e) { /* 서버가 꺼져 있으면 무시 */ }
});

function setData(on) {
  DATA = on;
  dataBtn.textContent = DATA ? "지도만 보기" : "데이터 표시";
  draw();
}
dataBtn.addEventListener("click", () => setData(!DATA));
canvas.addEventListener("dblclick", () => setData(!DATA));   // 작은 창에서는 지도를 두 번 눌러 바꾼다
setData(DATA);

window.addEventListener("resize", () => draw());
kick();
</script>
</body>
</html>
"""
# 화면 코드 버전. 미니맵을 고쳐서 다시 켜면 바뀌고, 열어 둔 화면은 /state 로 이것을 보고 스스로 새로 불러온다.
PAGE_ID = hashlib.sha1(PAGE.encode("utf-8")).hexdigest()[:12]
PAGE_HTML = PAGE.replace("__PAGE_ID__", PAGE_ID).encode("utf-8")


if __name__ == "__main__":
    sys.exit(main())
