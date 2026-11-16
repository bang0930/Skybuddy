"""장애 상황(배터리 부족, 통신 악화·두절, 웨이포인트 미도달)을 어댑터 안에서 만든다.

[왜 어댑터 안에서 만드는가]
미들웨어는 DroneInterface 메서드만 부르고 그 결과로 드론 상태를 판단한다.
장애를 어댑터 안에서 만들면 미들웨어 코드를 바꾸지 않고도, 실제 장애가 났을 때와
같은 경로로 값이 전달된다. 값은 텔레메트리 샘플마다(약 4Hz) 이어서 바뀐다.

  장애      어댑터에서 바뀌는 값                                  미들웨어에 보이는 것
  -------   ---------------------------------------------------   -----------------------------------
  battery   battery_remaining_pct, battery_voltage_v               DroneState.battery_percent / _voltage_v
            drain: 시동이 걸린 동안만 줄어든다 (비행 시간 기준)
            0% 가 되면 더 날 수 없으므로 그 자리 강제 착륙(LAND)
  link      link_quality_pct (0~100, 텔레메트리 새 필드)          매퍼에 한 줄 추가하면 DroneState 로
            range: 홈에서 멀어질수록 품질이 내려가고,
                   degraded_m 을 넘으면 샘플이 드물게,             observed_at 이 늦어짐 (connected 유지)
                   down_m 을 넘으면 샘플·명령이 끊긴다             telemetry_timeout_s 후 disconnected
            degraded: 샘플을 interval_s 에 1개만 줌               observed_at 이 늦어짐 (connected 유지)
            down: 샘플 없음 + 명령이 기체에 안 닿음                telemetry_timeout_s 후 disconnected
  stall     goto() 가 기체에 전달되지 않음                        Dispatcher 의 WAYPOINT_TIMEOUT

SITL 은 배터리가 비행 내내 100% 이고(실측) 통신도 끊기지 않으므로 값은 여기서 정한다.
비행 컨트롤러(ArduPilot)는 이 장애를 모른다. 그래서 실제 기체라면 스스로 했을 동작을 어댑터가 대신 보낸다.
  - 통신 두절이 failsafe_after_s(기본 5초 = ArduPilot FS_GCS_TIMEOUT 기본값) 이어지면 RTL 등.
    미들웨어(#42)는 두절 중 명령을 보내지 않고 이 failsafe 를 기다린다.
  - 배터리 0%: 그 자리 착륙(LAND). 그 전에는 아무것도 하지 않는다. ArduPilot 배터리 failsafe 의
    기본 동작(BATT_FS_LOW_ACT, BATT_FS_CRT_ACT = 0)이 "경고만"이기 때문이다.
    즉 0% 전에 귀환시키는 판단은 미들웨어·LLM 몫이다.

[제어 방법]
fault.py 가 제어 파일(JSON)을 쓰고 어댑터가 0.2초마다 읽는다. 프로세스가 달라도 되므로
미들웨어 서버를 띄운 채 다른 터미널에서 장애를 걸고 풀 수 있다.

    python3 fault.py battery drone-01 0 --drain 120     비행 120초 동안 지금 값 -> 0%
    python3 fault.py link drone-02 range --degraded-m 250 --down-m 330
    python3 fault.py clear

[기록]
  장애 이벤트   장애 시작·종료, 통신 상태 변화, 배터리 10% 단위, 강제 착륙. 미들웨어 이벤트 로그와 같은
               logged_at / event 형식이라 시각으로 합쳐 볼 수 있다.
  텔레메트리    미들웨어에 넘긴 샘플을 그대로 한 줄씩 남긴다 (telemetry). 시뮬레이터만 아는 참값
               (시동 여부, 비행 시간, 홈 거리, 걸린 장애)은 따로 붙인다 (sim). fault.py stream 으로 본다.
현재 주입 상태는 어댑터의 fault_status() 로도 읽을 수 있다 (블로킹 없음, 스레드 안전).

[환경 변수]
  SKYBUDDY_FAULT_FILE       제어 파일   (기본 /tmp/skybuddy_faults.json)
  SKYBUDDY_FAULT_LOG        이벤트 기록 (기본 ~/skybuddy/logs/fault-events.jsonl)
  SKYBUDDY_TELEMETRY_LOG    텔레메트리 기록 (기본 ~/skybuddy/logs/telemetry-stream.jsonl, 0 이면 끔)
  SKYBUDDY_FAULT_INJECTION  0 이면 주입 기능 전체를 끈다 (실기체에서는 반드시 0)
  SKYBUDDY_FAULT_POLL_S     제어 파일 확인 주기 (기본 0.2초)

이 모듈은 표준 라이브러리만 쓴다. fault.py 와 시험 코드도 같은 함수를 쓴다.
"""
import contextlib
import json
import math
import os
import sys
import threading
import time
from datetime import datetime, timezone

CONTROL_ENV = "SKYBUDDY_FAULT_FILE"
LOG_ENV = "SKYBUDDY_FAULT_LOG"
STREAM_ENV = "SKYBUDDY_TELEMETRY_LOG"
ENABLE_ENV = "SKYBUDDY_FAULT_INJECTION"
POLL_ENV = "SKYBUDDY_FAULT_POLL_S"

DEFAULT_CONTROL_FILE = "/tmp/skybuddy_faults.json"
DEFAULT_EVENT_LOG = "~/skybuddy/logs/fault-events.jsonl"
DEFAULT_STREAM_LOG = "~/skybuddy/logs/telemetry-stream.jsonl"
STREAM_ROTATE_BYTES = 50 * 1024 * 1024    # 이보다 커지면 .1 로 옮기고 새로 쓴다
DEFAULT_POLL_S = 0.2

KINDS = ("battery", "link", "stall")
LINK_MODES = ("degraded", "down", "range")
START_TYPES = ("now", "at", "after_takeoff")

# degraded 에서 샘플 1개를 내주는 간격. 미들웨어 기본 telemetry_timeout_s(3초)보다 짧아야
# "connected 이지만 늦게 들어오는" 상태가 된다. 3초 이상이면 disconnected 와 구분되지 않는다.
DEFAULT_DEGRADED_INTERVAL_S = 2.0
# range 모드에서 경계 근처를 오갈 때 상태가 깜빡이지 않도록 되돌아올 때만 이만큼 더 들어와야 한다.
DEFAULT_RANGE_HYSTERESIS_M = 10.0
# 시간 기준 degraded 에서 보고하는 링크 품질(%). range 는 거리로 계산한다.
DEFAULT_DEGRADED_QUALITY = 20.0

# 통신 두절 중 기체 자체 failsafe 흉내. ArduPilot FS_GCS_ENABLE 에 해당한다.
#   rtl  = 홈으로 돌아와 착륙 (FS_GCS_ENABLE=1)
#   land = 그 자리 착륙       (FS_GCS_ENABLE=5)
#   none = 아무것도 안 함, 마지막 목표에서 제자리 비행 (FS_GCS_ENABLE=0, ArduCopter 기본값)
FAILSAFE_MODES = ("rtl", "land", "none")
DEFAULT_FAILSAFE = "rtl"
# ArduPilot FS_GCS_TIMEOUT 기본값. 미들웨어 DispatcherConfig.link_loss_timeout_s 기본값과 같다.
DEFAULT_FAILSAFE_AFTER_S = 5.0

# 배터리 잔량(%) -> LiPo 셀 하나의 전압(V). 널리 쓰이는 휴지 전압 표(근사)이고 사이는 선형 보간한다.
# 비행 중 실제 전압은 부하 때문에 조금 더 낮다. 목적은 잔량과 전압이 같은 방향으로 함께 움직이는 것이다.
LIPO_CELL_CURVE = (
    (0, 3.27), (5, 3.61), (10, 3.69), (15, 3.71), (20, 3.73), (25, 3.75), (30, 3.77),
    (35, 3.79), (40, 3.80), (45, 3.82), (50, 3.84), (55, 3.85), (60, 3.87), (65, 3.91),
    (70, 3.95), (75, 3.98), (80, 4.02), (85, 4.08), (90, 4.11), (95, 4.15), (100, 4.20),
)
FULL_CELL_V = 4.20
DEFAULT_CELLS = 3            # 실제 전압을 아직 못 봤으면 3셀(만충 12.6V)로 본다
BATTERY_LEVEL_STEP = 10      # 잔량이 이 단위(%)를 내려갈 때마다 기록한다

# MAVLink 상수 (pymavlink 없이 쓰려고 숫자로 둔다)
MAV_AUTOPILOT_INVALID = 8    # 지상국·보조 장치가 보내는 하트비트
MAV_MODE_FLAG_SAFETY_ARMED = 128

METERS_PER_DEG_LAT = 111320.0   # DroneInterface.distance_m 과 같은 평면 근사

_STREAM_LOCK = threading.Lock()  # 한 프로세스의 두 어댑터가 같은 텔레메트리 기록 파일에 쓴다


# ------------------------------------------------------------------ 공용 함수

def control_path():
    return os.path.expanduser(os.environ.get(CONTROL_ENV) or DEFAULT_CONTROL_FILE)


def event_log_path():
    return os.path.expanduser(os.environ.get(LOG_ENV) or DEFAULT_EVENT_LOG)


def stream_log_path():
    """미들웨어에 넘긴 텔레메트리를 남길 파일. SKYBUDDY_TELEMETRY_LOG=0 이면 None (기록 안 함)."""
    raw = (os.environ.get(STREAM_ENV) or "").strip()
    if raw.lower() in ("0", "off", "false", "no"):
        return None
    return os.path.expanduser(raw or DEFAULT_STREAM_LOG)


def injection_enabled():
    return os.environ.get(ENABLE_ENV, "1").strip().lower() not in ("0", "false", "off", "no")


def drone_id_from_config(identities):
    """미들웨어 드론 설정(MIDDLEWARE_DRONES_CONFIG)에서 연결 문자열이 같은 드론의 drone_id. 없으면 None.

    미들웨어(ClientFactory)는 MavlinkDrone 에 drone_id 를 넘기지 않는다. 설정 파일로 미리 알아 두면
    장애를 걸기 전의 기록에도 연결 문자열 대신 drone-01 같은 이름이 남는다.
    """
    path = os.environ.get("MIDDLEWARE_DRONES_CONFIG")
    if not path:
        return None
    try:
        with open(os.path.expanduser(path), encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    drones = data.get("drones", []) if isinstance(data, dict) else []
    names = {str(i) for i in identities if i}
    for d in drones:
        if isinstance(d, dict) and d.get("drone_id") and d.get("connection_string") in names:
            return str(d["drone_id"])
    return None


def utc_iso(ts):
    return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat()


def distance_m(lat1, lon1, lat2, lon2):
    dlat = (lat2 - lat1) * METERS_PER_DEG_LAT
    dlon = (lon2 - lon1) * METERS_PER_DEG_LAT * math.cos(math.radians(lat1))
    return math.hypot(dlat, dlon)


def cell_voltage(pct):
    """잔량(%) -> LiPo 셀 하나의 전압(V). LIPO_CELL_CURVE 사이는 선형 보간."""
    pct = min(max(float(pct), 0.0), 100.0)
    for (p0, v0), (p1, v1) in zip(LIPO_CELL_CURVE, LIPO_CELL_CURVE[1:]):
        if pct <= p1:
            return v0 + (v1 - v0) * (pct - p0) / (p1 - p0)
    return LIPO_CELL_CURVE[-1][1]


def battery_level(pct):
    """10% 단위 구간. 29.9 -> 30, 30.0 -> 30, 30.1 -> 40. '30% 이하로 내려갔다'를 판단하는 데 쓴다."""
    return int(math.ceil(round(float(pct), 1) / BATTERY_LEVEL_STEP - 1e-9)) * BATTERY_LEVEL_STEP


def load_control(path=None):
    """제어 파일을 읽는다. 없으면 빈 구성. 형식이 깨졌으면 ValueError."""
    path = path or control_path()
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    except FileNotFoundError:
        return {"version": 1, "entries": []}
    if not isinstance(data, dict) or not isinstance(data.get("entries", []), list):
        raise ValueError(f"{path}: entries 목록이 없다")
    data.setdefault("version", 1)
    data.setdefault("entries", [])
    return data


def save_control(data, path=None):
    """제어 파일을 원자적으로 바꾼다. 어댑터가 쓰는 도중의 파일을 읽는 일이 없다."""
    path = path or control_path()
    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    data = dict(data)
    data["version"] = 1
    data["updated_at"] = utc_iso(time.time())
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")
    os.replace(tmp, path)


def validate_entry(entry):
    """제어 파일 항목 하나를 검사한다. 문제가 있으면 ValueError (한국어 설명)."""
    kind = entry.get("kind")
    if kind not in KINDS:
        raise ValueError(f"kind 는 {KINDS} 중 하나여야 한다: {kind!r}")
    if not entry.get("id"):
        raise ValueError("id 가 없다")
    if not entry.get("match"):
        raise ValueError("match(대상 드론 식별자 목록)가 없다")
    start = entry.get("start") or {"type": "now"}
    if start.get("type") not in START_TYPES:
        raise ValueError(f"start.type 은 {START_TYPES} 중 하나여야 한다")
    if start.get("type") == "after_takeoff" and float(start.get("delay_s", 0)) < 0:
        raise ValueError("이륙 후 지연 시간은 0 이상이어야 한다")
    duration = entry.get("duration_s")
    if duration is not None and float(duration) <= 0:
        raise ValueError("지속 시간은 0보다 커야 한다")

    p = entry.get("params") or {}
    if kind == "battery":
        value = float(p.get("pct", 100.0))
        if not 0.0 <= value <= 100.0:
            raise ValueError(f"배터리 pct 는 0~100 이어야 한다: {value}")
        if p.get("from_pct") is not None:
            value = float(p["from_pct"])
            if not 0.0 <= value <= 100.0:
                raise ValueError(f"배터리 from_pct 는 0~100 이어야 한다: {value}")
        ramp = float(p.get("ramp_s", 0) or 0)
        drain = float(p.get("drain_s", 0) or 0)
        if ramp < 0 or drain < 0:
            raise ValueError("ramp_s, drain_s 는 0 이상이어야 한다")
        if ramp > 0 and drain > 0:
            raise ValueError("--ramp(시계 기준)와 --drain(비행 시간 기준)은 하나만 쓴다")
    elif kind == "link":
        mode = p.get("mode")
        if mode not in LINK_MODES:
            raise ValueError(f"통신 mode 는 {LINK_MODES} 중 하나여야 한다: {mode!r}")
        if mode in ("degraded", "range") and float(p.get("interval_s", DEFAULT_DEGRADED_INTERVAL_S)) <= 0:
            raise ValueError("interval_s 는 0보다 커야 한다")
        if mode == "degraded":
            quality = float(p.get("quality_pct", DEFAULT_DEGRADED_QUALITY))
            if not 0.0 <= quality <= 100.0:
                raise ValueError(f"링크 품질은 0~100 이어야 한다: {quality}")
        if mode == "range":
            degraded_m, down_m = float(p.get("degraded_m", 0)), float(p.get("down_m", 0))
            if not 0 < degraded_m < down_m:
                raise ValueError("range 는 0 < degraded_m < down_m 이어야 한다")
            center = p.get("center")
            if center is not None and len(center) != 2:
                raise ValueError("center 는 [위도, 경도] 여야 한다")
        if mode in ("down", "range"):
            if p.get("failsafe", DEFAULT_FAILSAFE) not in FAILSAFE_MODES:
                raise ValueError(f"failsafe 는 {FAILSAFE_MODES} 중 하나여야 한다")
            if float(p.get("failsafe_after_s", DEFAULT_FAILSAFE_AFTER_S)) <= 0:
                raise ValueError("failsafe_after_s 는 0보다 커야 한다")
    elif kind == "stall":
        count = p.get("count")
        if count is not None and int(count) < 1:
            raise ValueError("count 는 1 이상이거나 null(무제한)이어야 한다")


def describe_params(kind, params):
    """사람이 읽을 한 줄 설명. 로그와 fault.py status 에서 쓴다."""
    p = params or {}
    if kind == "battery":
        target = float(p.get("pct", 0))
        start = p.get("from_pct")
        start_text = "지금 값" if start is None else f"{float(start):g}%"
        drain = float(p.get("drain_s", 0) or 0)
        ramp = float(p.get("ramp_s", 0) or 0)
        if drain > 0:
            text = f"{start_text}에서 비행 {drain:g}초 동안 {target:g}%까지 감소 (시동 걸린 동안만)"
        elif ramp > 0:
            text = f"{start_text}에서 {ramp:g}초 동안 {target:g}%까지 감소"
        else:
            text = f"{target:g}%"
        if target <= 0:
            text += ", 0%가 되면 그 자리 강제 착륙"
        return text
    if kind == "link":
        mode = p.get("mode")
        failsafe = p.get("failsafe", DEFAULT_FAILSAFE)
        after = float(p.get("failsafe_after_s", DEFAULT_FAILSAFE_AFTER_S))
        fs_text = ("기체 failsafe 없음" if failsafe == "none"
                   else f"{after:g}초 뒤 기체 failsafe {failsafe.upper()}")
        if mode == "degraded":
            return (f"degraded (샘플 {float(p.get('interval_s', DEFAULT_DEGRADED_INTERVAL_S)):g}초에 1개, "
                    f"링크 품질 {float(p.get('quality_pct', DEFAULT_DEGRADED_QUALITY)):g}%)")
        if mode == "down":
            return f"down (샘플 없음, 명령 전달 안 됨, {fs_text})"
        if mode == "range":
            down = float(p["down_m"])
            return (f"range (링크 품질 홈 100% -> {down:g}m 0%, {float(p['degraded_m']):g}m 넘으면 degraded, "
                    f"{down:g}m 넘으면 down, {fs_text})")
        return str(mode)
    if kind == "stall":
        count = p.get("count")
        return f"goto 무시 {'무제한' if count is None else f'{int(count)}회'}"
    return json.dumps(p, ensure_ascii=False)


# ------------------------------------------------------------------ 어댑터 쪽

class FaultInjector:
    """어댑터 하나에 붙어 제어 파일을 읽고 장애를 적용한다.

    identities : 이 어댑터를 가리키는 이름들 (연결 문자열 등).
                 제어 파일 항목의 match 중 하나라도 겹치면 적용 대상이다. "*" 는 전부.
    drone_id   : 알면 넘긴다. 미들웨어(ClientFactory)는 MavlinkDrone 에 drone_id 를 넘기지
                 않으므로, 그때는 미들웨어 드론 설정(MIDDLEWARE_DRONES_CONFIG)에서 연결 문자열로 찾고,
                 그것도 없으면 연결 문자열로 맞춘 항목의 target(예: drone-01)을 이름으로 배운다.
    stream_log : 텔레메트리 기록 파일. None 이면 환경 변수(SKYBUDDY_TELEMETRY_LOG), "" 이면 기록 안 함.
    clock/sleep: 시험에서 바꿔 끼우려고 받는다.

    어댑터가 알려 줘야 하는 것 (통신 두절 중에도 계속 알려야 기체 쪽 참값이 유지된다)
      observe_position(lat, lon)  실제 위치 -> range 모드 거리, 홈 거리
      observe_armed(armed)        실제 시동 상태 -> 배터리가 비행 중에만 닳게
    """

    def __init__(self, identities, *, drone_id=None, control_file=None, event_log=None,
                 stream_log=None, enabled=None, poll_s=None, clock=time.time, sleep=time.sleep):
        if not drone_id:
            drone_id = drone_id_from_config(identities)
        seen = []
        for ident in [drone_id, *identities]:
            if ident and str(ident) not in seen:
                seen.append(str(ident))
        self.identities = seen
        self.name = seen[0] if seen else "unknown"
        self._learn_name = not drone_id
        self.enabled = injection_enabled() if enabled is None else bool(enabled)
        self.control_file = control_file or control_path()
        self.event_log = event_log or event_log_path()
        self.stream_log = stream_log_path() if stream_log is None else (stream_log or None)
        if poll_s is None:
            try:
                poll_s = float(os.environ.get(POLL_ENV, DEFAULT_POLL_S))
            except ValueError:
                poll_s = DEFAULT_POLL_S
        self.poll_s = poll_s
        self.clock = clock
        self.sleep = sleep

        self._lock = threading.RLock()
        self._file_sig = None
        self._entries = []
        self._last_poll = -math.inf
        self._load_warned = False
        self._log_failed = False
        self._stream_failed = False
        self._stream_writes = 0

        self._takeoffs = []           # 이륙 완료 시각들 (after_takeoff 기준)
        self._pos = None              # 기체의 실제 위치 (통신 두절 중에도 갱신)
        self._center_auto = None      # range 모드 기준점: 처음 관측한 위치(= 지상국 근처 홈)
        self._last_distance = None
        self._home = None             # 마지막으로 시동이 걸린 위치 (ArduPilot 홈과 같은 기준)

        self._armed = None            # 기체의 실제 시동 상태. None = 아직 모름
        self._flight_s = 0.0          # 시동이 걸린 채 지난 시간 누적 (배터리 drain 기준)
        self._flight_mark = None      # 마지막으로 누적한 시각

        self._raw_battery = None      # 어댑터가 실제로 읽은 잔량 (주입 전)
        self._cells = None            # 배터리 셀 수 (처음 본 실제 전압으로 추정)
        self._battery_start = {}      # 배터리 항목 id -> (시작 잔량, 시작 시점의 비행 누적초)
        self._battery_level_logged = {}  # 배터리 항목 id -> 마지막으로 기록한 10% 구간
        self._battery_empty_done = {}    # 배터리 항목 id -> 0% 강제 착륙을 보냈는지
        self._battery_value_now = None   # 지금 주입 중인 잔량 (없으면 None)

        self._active = {}             # kind -> 지금 적용 중인 항목 id
        self._active_since = {}       # kind -> 적용 시작 시각
        self._link_state = "ok"
        self._link_since = clock()
        self._tx_dropped = 0          # 이번 두절 동안 전달되지 않은 송신 수
        self._next_sample_at = 0.0    # degraded 에서 다음 샘플을 내줄 수 있는 시각
        self._stall_dropped = {}      # stall 항목 id -> 무시한 goto 수

        self._down_since = None       # 이번 두절이 시작된 시각
        self._failsafe_done = False   # 이번 두절에서 failsafe 를 이미 처리했는지
        self._failsafe_sent = None    # 마지막으로 보낸 failsafe 모드 (상태 보고용)
        self._failsafe_handler = None
        self._bypass_tx = False       # failsafe 송신 중에는 두절 차단을 우회한다

    def set_failsafe_handler(self, handler):
        """기체 안에서 일어날 모드 전환을 흉내 낼 함수. handler("rtl" 또는 "land").

        통신 두절 failsafe(RTL 등)와 배터리 0% 강제 착륙(LAND)에 쓴다.
        """
        self._failsafe_handler = handler

    @contextlib.contextmanager
    def bypass(self):
        """기체 안에서 일어나는 일(failsafe)을 흉내 낼 때만 쓴다. 이 안의 송신은 두절에 막히지 않는다."""
        with self._lock:
            previous = self._bypass_tx
            self._bypass_tx = True
        try:
            yield
        finally:
            with self._lock:
                self._bypass_tx = previous

    # ---------------------------------------------------------- 제어 파일

    def refresh(self, force=True):
        """제어 파일을 다시 읽고 상태를 갱신한다. 시험에서 주기를 기다리지 않으려고 쓴다."""
        if not self.enabled:
            return
        with self._lock:
            self._refresh(force=force)
            self._evaluate(refresh=False)

    def _matches(self, entry):
        match = entry.get("match") or [entry.get("target")]
        return "*" in match or any(str(m) in self.identities for m in match)

    def _refresh(self, force=False):
        now = self.clock()
        if not force and now - self._last_poll < self.poll_s:
            return
        self._last_poll = now
        try:
            st = os.stat(self.control_file)
            sig = (st.st_ino, st.st_mtime_ns, st.st_size)
        except FileNotFoundError:
            sig = None
        except OSError:
            return
        if sig == self._file_sig:
            return
        if sig is None:
            entries = []
        else:
            try:
                with open(self.control_file, encoding="utf-8") as f:
                    data = json.load(f)
                entries = data.get("entries", []) if isinstance(data, dict) else []
            except (OSError, ValueError) as e:
                # 손으로 고치다 저장한 중간 상태일 수 있다. 이전 상태를 유지하고 다음 주기에 다시 읽는다.
                if not self._load_warned:
                    self._stderr(f"제어 파일을 읽지 못함 ({self.control_file}): {e} - 이전 상태 유지")
                    self._load_warned = True
                return
        self._file_sig = sig
        self._load_warned = False
        self._entries = [e for e in entries
                         if isinstance(e, dict) and e.get("kind") in KINDS and self._matches(e)]
        # 연결 문자열로 맞춘 항목의 target(drone-01 등)을 별칭으로 배운다. 기록에 드론 이름이 남고,
        # 이후 drone_id 만 적은 항목도 이 어댑터에 적용된다.
        for e in self._entries:
            target = e.get("target")
            if target and target not in ("all", "*") and target not in self.identities:
                self.identities.append(str(target))
                if self._learn_name:
                    self.name = str(target)
                    self._learn_name = False

    def _entry(self, entry_id):
        for e in self._entries:
            if e.get("id") == entry_id:
                return e
        return None

    def _latest(self, kind):
        """같은 종류가 여러 개 맞으면 가장 나중에 만든 것만 쓴다 (새 명령이 이전 것을 대체)."""
        candidates = [e for e in self._entries if e.get("kind") == kind]
        if not candidates:
            return None
        return max(candidates, key=lambda e: float(e.get("created_at", 0)))

    def _window(self, entry):
        """(시작, 끝) 시각. 아직 정할 수 없으면(이륙 대기) (None, None). 끝이 None 이면 계속."""
        created = float(entry.get("created_at", 0))
        start = entry.get("start") or {"type": "now"}
        typ = start.get("type", "now")
        if typ == "now":
            begin = created
        elif typ == "at":
            begin = float(start.get("at", created))
        elif typ == "after_takeoff":
            # 항목을 만든 뒤의 첫 이륙을 기준으로 한다. 그전 임무의 이륙은 세지 않는다.
            later = [t for t in self._takeoffs if t >= created]
            if not later:
                return None, None
            begin = later[0] + float(start.get("delay_s", 0))
        else:
            return None, None
        duration = entry.get("duration_s")
        end = begin + float(duration) if duration is not None else None
        return begin, end

    def _stall_exhausted(self, entry):
        count = (entry.get("params") or {}).get("count")
        return count is not None and self._stall_dropped.get(entry.get("id"), 0) >= int(count)

    def _active_entry(self, kind, now):
        entry = self._latest(kind)
        if entry is None:
            return None, None
        begin, end = self._window(entry)
        if begin is None or now < begin or (end is not None and now >= end):
            return None, None
        if kind == "stall" and self._stall_exhausted(entry):
            return None, None
        return entry, begin

    def _end_reason(self, kind, old_id):
        entry = self._entry(old_id)
        if entry is None:
            return "cleared"
        latest = self._latest(kind)
        if latest is not None and latest.get("id") != old_id:
            return "replaced"
        if kind == "stall" and self._stall_exhausted(entry):
            return "count_reached"
        return "expired"

    def _evaluate(self, refresh=True, actions=True):
        """제어 파일과 시각으로 지금 걸린 장애를 정한다.

        actions=False 면 failsafe·강제 착륙 송신 같은 동작은 하지 않는다. fault_status() 는 미들웨어의
        다른 스레드에서 불릴 수 있어서, pymavlink 연결에 동시에 쓰지 않도록 이 값으로 부른다.
        """
        if refresh:
            self._refresh()
        now = self.clock()
        self._advance_flight(now)
        for kind in KINDS:
            entry, begin = self._active_entry(kind, now)
            new_id = entry.get("id") if entry else None
            old_id = self._active.get(kind)
            if new_id == old_id:
                continue
            if old_id is not None:
                self._log("fault_ended", kind=kind, entry_id=old_id,
                          reason=self._end_reason(kind, old_id))
            if entry is not None:
                extra = {}
                if kind == "battery":
                    extra["start_pct"] = self._battery_begin(new_id, entry)
                self._log("fault_started", kind=kind, entry_id=new_id,
                          params=entry.get("params") or {}, start=entry.get("start"),
                          duration_s=entry.get("duration_s"), active_since=utc_iso(begin),
                          summary=describe_params(kind, entry.get("params")), **extra)
            self._active[kind] = new_id
            self._active_since[kind] = begin
        self._update_link_state(now)
        self._update_battery(now, actions)
        if actions:
            self._check_failsafe(now)

    # ---------------------------------------------------------- 비행 시간 (배터리 기준)

    def _advance_flight(self, now):
        """시동이 걸려 있던 동안의 시간을 누적한다. 시동 상태가 바뀌기 전에 반드시 먼저 부른다."""
        if self._flight_mark is not None and self._armed:
            self._flight_s += max(0.0, now - self._flight_mark)
        self._flight_mark = now

    def _where(self):
        """기록에 붙일 기체 참값: 시동 여부, 비행 누적 시간, 위치, 홈에서의 거리."""
        out = {"armed": self._armed, "flight_s": round(self._flight_s, 1)}
        if self._pos is not None:
            out["position"] = [round(self._pos[0], 7), round(self._pos[1], 7)]
            home = self._home or self._center_auto
            if home is not None:
                out["home_distance_m"] = round(distance_m(home[0], home[1], self._pos[0], self._pos[1]), 1)
        return out

    # ---------------------------------------------------------- 배터리

    def _battery_begin(self, entry_id, entry):
        """배터리 항목이 적용되기 시작할 때 시작 잔량과 기준 비행 시간을 정한다. 시작 잔량을 돌려준다."""
        p = entry.get("params") or {}
        start = p.get("from_pct")
        if start is None:
            raw = self._raw_battery
            start = raw if raw is not None and raw > 0 else 100.0
        start = float(start)
        self._battery_start[entry_id] = (start, self._flight_s)
        self._battery_level_logged[entry_id] = battery_level(start)
        return start

    def _battery_value(self, now):
        """지금 주입할 잔량(%). 배터리 장애가 없으면 None."""
        entry_id = self._active.get("battery")
        entry = self._entry(entry_id) if entry_id else None
        if entry is None:
            return None
        p = entry.get("params") or {}
        target = float(p.get("pct", 100.0))
        start, flight0 = self._battery_start.get(entry_id, (100.0, self._flight_s))
        drain = float(p.get("drain_s", 0) or 0)
        ramp = float(p.get("ramp_s", 0) or 0)
        if drain > 0:
            frac = (self._flight_s - flight0) / drain          # 시동 걸린 시간만 센다
        elif ramp > 0:
            since = self._active_since.get("battery") or now
            frac = (now - since) / ramp                        # 시계 기준 (지상에서도 줄어든다)
        else:
            return round(target, 1)
        frac = min(max(frac, 0.0), 1.0)
        return round(start + (target - start) * frac, 1)

    def _voltage_for(self, pct):
        return round((self._cells or DEFAULT_CELLS) * cell_voltage(pct), 2)

    def _battery_empty(self):
        return self._battery_value_now is not None and self._battery_value_now <= 0

    def _update_battery(self, now, actions):
        """잔량을 다시 계산하고, 10% 구간을 내려갔으면 기록하고, 비행 중 0% 면 강제 착륙시킨다."""
        entry_id = self._active.get("battery")
        if entry_id is None:
            self._battery_value_now = None
            return
        value = self._battery_value(now)
        self._battery_value_now = value
        if value is None:
            return
        level = battery_level(value)
        if level < self._battery_level_logged.get(entry_id, 100):
            self._battery_level_logged[entry_id] = level
            self._log("fault_battery_level", entry_id=entry_id, level=level, pct=value,
                      voltage_v=self._voltage_for(value), **self._where())
        if (value <= 0 and actions and self._armed is True
                and entry_id not in self._battery_empty_done):
            self._battery_forced_landing(entry_id)

    def _battery_forced_landing(self, entry_id):
        """배터리 0%: 기체가 더 날 수 없다. 실제라면 추락·불시착이고, 시뮬레이션에서는 그 자리 착륙으로 흉내 낸다."""
        where = self._where()
        self._battery_empty_done[entry_id] = False
        handler = self._failsafe_handler
        if handler is None:
            self._log("fault_battery_empty", entry_id=entry_id, sent=False, reason="no_handler", **where)
            return
        try:
            handler("land")
        except Exception as e:  # noqa: BLE001 - 장애 흉내가 어댑터 동작을 깨면 안 된다
            self._log("fault_battery_empty", entry_id=entry_id, sent=False,
                      reason=f"{type(e).__name__}: {e}", **where)
            return
        self._battery_empty_done[entry_id] = True
        self._log("fault_battery_empty", entry_id=entry_id, sent=True, mode="land", **where)

    # ---------------------------------------------------------- 통신 상태

    def _range_state(self, params, previous):
        center = params.get("center") or self._center_auto
        if center is None or self._pos is None:
            return previous            # 위치를 아직 모르면 상태를 바꾸지 않는다
        d = distance_m(center[0], center[1], self._pos[0], self._pos[1])
        self._last_distance = d
        degraded_m, down_m = float(params["degraded_m"]), float(params["down_m"])
        h = float(params.get("hysteresis_m", DEFAULT_RANGE_HYSTERESIS_M))
        if previous == "down":
            if d <= degraded_m - h:
                return "ok"
            return "degraded" if d <= down_m - h else "down"
        if previous == "degraded":
            if d > down_m:
                return "down"
            return "ok" if d <= degraded_m - h else "degraded"
        if d > down_m:
            return "down"
        return "degraded" if d > degraded_m else "ok"

    def _update_link_state(self, now):
        entry = self._entry(self._active.get("link")) if self._active.get("link") else None
        params = (entry or {}).get("params") or {}
        mode = params.get("mode") if entry else None
        previous = self._link_state
        if mode in ("degraded", "down"):
            new = mode
        elif mode == "range":
            new = self._range_state(params, previous)
        else:
            new = "ok"
        if new == previous:
            return
        fields = {"state_from": previous, "state_to": new, "mode": mode}
        if mode == "range" and self._last_distance is not None:
            fields["distance_m"] = round(self._last_distance, 1)
        if previous == "down":
            fields["tx_dropped"] = self._tx_dropped
        self._link_state = new
        fields["link_quality_pct"] = self._link_quality()
        self._log("fault_link_state", **fields)
        self._link_since = now
        if new == "down":
            self._tx_dropped = 0
            self._down_since = now
            self._failsafe_done = False
            self._failsafe_sent = None
        else:
            self._down_since = None
        if new == "degraded":
            self._next_sample_at = 0.0   # 악화 직후 첫 샘플은 바로 준다

    def _link_params(self):
        entry = self._entry(self._active.get("link")) if self._active.get("link") else None
        return (entry or {}).get("params") or {}

    def _link_interval(self):
        return float(self._link_params().get("interval_s", DEFAULT_DEGRADED_INTERVAL_S))

    def _link_quality(self):
        """링크 품질(%). 실기체의 RSSI·패킷 수신율에 해당하는 모의값.

        장애 없음 100 / degraded quality_pct(기본 20) / down 0 /
        range 는 홈(기준점)에서 0m 일 때 100, down_m 에서 0 이 되도록 거리에 비례해 줄어든다.
        """
        if not self._active.get("link"):
            return 100.0
        params = self._link_params()
        mode = params.get("mode")
        if mode == "down":
            return 0.0
        if mode == "degraded":
            return float(params.get("quality_pct", DEFAULT_DEGRADED_QUALITY))
        if mode == "range":
            center = params.get("center") or self._center_auto
            if center is None or self._pos is None:
                return 100.0
            d = distance_m(center[0], center[1], self._pos[0], self._pos[1])
            down_m = float(params["down_m"])
            return round(min(100.0, max(0.0, 100.0 * (down_m - d) / down_m)), 1)
        return 100.0

    def _check_failsafe(self, now):
        """두절이 failsafe_after_s 이어지면 기체 failsafe 를 한 번 흉내 낸다 (두절마다 한 번)."""
        if self._link_state != "down" or self._failsafe_done or self._down_since is None:
            return
        params = self._link_params()
        mode = params.get("failsafe", DEFAULT_FAILSAFE)
        after = float(params.get("failsafe_after_s", DEFAULT_FAILSAFE_AFTER_S))
        if mode == "none":
            self._failsafe_done = True
            return
        if now - self._down_since < after:
            return
        self._failsafe_done = True
        handler = self._failsafe_handler
        if handler is None:
            self._log("fault_failsafe", mode=mode, after_s=after, sent=False, reason="no_handler")
            return
        try:
            handler(mode)
        except Exception as e:  # noqa: BLE001 - 장애 흉내가 어댑터 동작을 깨면 안 된다
            self._log("fault_failsafe", mode=mode, after_s=after, sent=False,
                      reason=f"{type(e).__name__}: {e}")
            return
        self._failsafe_sent = mode
        self._log("fault_failsafe", mode=mode, after_s=after, sent=True)

    # ---------------------------------------------------------- 어댑터가 부르는 것

    def link_state(self):
        """'ok' / 'degraded' / 'down'."""
        if not self.enabled:
            return "ok"
        with self._lock:
            self._evaluate()
            return self._link_state

    def link_down(self):
        return self.link_state() == "down"

    def observe_position(self, lat, lon):
        """기체의 실제 위치를 알려 준다. 통신 두절 중에도 불러야 range 모드가 복구를 안다."""
        if not self.enabled or lat is None or lon is None:
            return
        if abs(lat) < 1e-6 and abs(lon) < 1e-6:
            return                     # EKF 원점이 잡히기 전의 (0, 0) 은 버린다
        with self._lock:
            self._pos = (float(lat), float(lon))
            if self._center_auto is None:
                self._center_auto = self._pos

    def observe_armed(self, armed):
        """기체의 실제 시동 상태를 알려 준다. 배터리 drain 은 시동이 걸린 동안만 줄어든다.

        통신 두절 중에도 불러야 한다 (배터리는 링크와 상관없이 닳는다).
        시동이 걸리는 순간의 위치를 홈으로 기억한다 (ArduPilot 도 시동 때 홈을 다시 잡는다).
        """
        if not self.enabled or armed is None:
            return
        armed = bool(armed)
        with self._lock:
            self._advance_flight(self.clock())     # 바뀌기 전 상태로 지난 시간을 먼저 센다
            if armed and self._armed is not True and self._pos is not None:
                self._home = self._pos
            self._armed = armed

    def note_connected(self):
        if not self.enabled:
            return
        with self._lock:
            self._refresh(force=True)
            self._evaluate(refresh=False)
            entries = [{"entry_id": e.get("id"), "kind": e.get("kind"),
                        "summary": describe_params(e.get("kind"), e.get("params"))}
                       for e in self._entries]
        self._log("fault_adapter_attached", pid=os.getpid(), control_file=self.control_file,
                  stream_log=self.stream_log, entries=entries, quiet=True)
        self._stderr(f"{self.name} 장애 주입 대기 중 (제어 파일 {self.control_file})")
        if entries:
            listed = ", ".join(f"{e['kind']} {e['summary']}" for e in entries)
            self._stderr(f"주의: 연결 시점에 이미 걸려 있는 장애가 있다 - {listed}. "
                         f"필요 없으면 python3 fault.py clear")

    def note_takeoff(self):
        """takeoff() 가 목표 고도에 도달해 끝났을 때 부른다 (after_takeoff 기준 시각)."""
        if not self.enabled:
            return
        with self._lock:
            self._takeoffs.append(self.clock())
            del self._takeoffs[:-20]

    def takeoff_block(self):
        """이륙할 수 없는 이유 (없으면 None). takeoff() 첫 줄에서 부르고, 이유가 있으면 예외로 끝낸다.

        배터리 0% 면 날 수 없으므로 시동 단계에서 거부된 것으로 본다.
        """
        if not self.enabled:
            return None
        with self._lock:
            self._evaluate(actions=False)
            value = self._battery_value_now
        if value is not None and value <= 0:
            self._log("fault_takeoff_blocked", pct=value)
            return f"ARM 실패 (장애 주입: 배터리 {value:g}%)"
        return None

    def telemetry_gate(self, timeout):
        """get_telemetry() 첫 줄에서 부른다. (읽어도 되는가, 실제 읽기에 쓸 timeout) 을 돌려준다.

        False 면 이번 호출은 데이터 없이 None 으로 끝내야 한다 (이미 timeout 만큼 기다렸다).
        degraded 에서만 기다린다. 기다리지 않고 바로 None 을 주면 호출 측 while 루프가
        CPU 를 다 먹으므로, 실제로 데이터가 안 올 때처럼 timeout 동안 기다린다.
        """
        if not self.enabled:
            return True, timeout
        with self._lock:
            self._evaluate()
            if self._link_state != "degraded":
                return True, timeout
            wait = self._next_sample_at - self.clock()
        if wait <= 0:
            return True, timeout
        if timeout is None:                  # 무한 대기 호출: 다음 샘플 시각까지만 기다린다
            self.sleep(wait)
            return True, None
        if wait >= timeout:
            self.sleep(timeout)
            return False, timeout
        self.sleep(wait)
        return True, max(timeout - wait, 0.05)

    def deliver(self, sample):
        """get_telemetry() 가 샘플을 돌려주기 직전에 부른다.

        배터리 잔량·전압과 링크 품질을 바꾸고, degraded 간격을 재고, 미들웨어에 넘기는 샘플을 그대로 기록한다.
        """
        if not self.enabled or sample is None:
            return sample
        with self._lock:
            raw = sample.get("battery_remaining_pct")
            if raw is not None:
                self._raw_battery = float(raw)
            volt = sample.get("battery_voltage_v")
            if self._cells is None and volt:
                self._cells = max(1, int(round(float(volt) / FULL_CELL_V)))
            self._evaluate()
            now = self.clock()
            if self._link_state == "degraded":
                self._next_sample_at = now + self._link_interval()
            battery = self._battery_value_now
            quality = self._link_quality()
            context = self._stream_context() if self.stream_log else None
        if battery is not None:
            sample["battery_remaining_pct"] = battery
            sample["battery_voltage_v"] = self._voltage_for(battery)
        sample["link_quality_pct"] = quality
        if context is not None:
            # 주입 전 원래 값 (SITL 은 배터리가 늘 100%). 주입이 값을 바꿨는지 확인용.
            context.update(raw_battery_pct=raw, raw_battery_voltage_v=volt)
            self._write_stream(sample, context)
        return sample

    def drop_goto(self, lat=None, lon=None, alt_m=None):
        """goto 를 기체에 보내면 안 되면 True. 호출 측은 보내지 않고 돌아가야 한다.

        stall 이 걸려 있거나, 배터리가 0% 라 기체가 날 수 없을 때다.
        """
        if not self.enabled:
            return False
        with self._lock:
            self._evaluate()
            if self._battery_empty():
                self._log("fault_goto_dropped", reason="battery_empty",
                          entry_id=self._active.get("battery"), target=[lat, lon, alt_m])
                return True
            entry_id = self._active.get("stall")
            if entry_id is None:
                return False
            dropped = self._stall_dropped.get(entry_id, 0) + 1
            self._stall_dropped[entry_id] = dropped
            count = ((self._entry(entry_id) or {}).get("params") or {}).get("count")
            self._log("fault_goto_dropped", reason="stall", entry_id=entry_id, dropped=dropped,
                      count=count, target=[lat, lon, alt_m])
            self._evaluate(refresh=False)    # 횟수를 다 썼으면 여기서 종료가 기록된다
            return True

    def count_tx_drop(self):
        """통신 두절로 송신이 버려졌을 때 부른다. 두절이 끝날 때 합계를 기록한다."""
        if not self.enabled:
            return
        with self._lock:
            self._tx_dropped += 1

    def attach_mavlink(self, master):
        """pymavlink 연결 객체의 수신·송신 함수를 감싼다. connect() 에서 한 번 부른다.

        pymavlink 는 recv_match()/wait_heartbeat() 가 모두 recv_msg() 를 거치고,
        mav.*_send() 는 모두 연결 객체의 write() 로 나간다. 이 두 곳만 감싸면
        MavlinkDrone 의 모든 수신·송신(모드 전환 재시도, ACK 대기, 하트비트 대기 포함)이
        실제로 링크가 끊겼을 때와 똑같이 동작한다.
        두절 중에도 메시지는 읽어서 해석한다. 위치(range 모드)와 시동 상태(배터리 drain)는
        기체 쪽 참값으로 쓰고, 어댑터에는 숨긴다. 그래서 복구 직후 쌓인 옛 메시지가 한꺼번에 나오는 일도 없다.
        """
        if not self.enabled:
            return
        original_recv = getattr(master, "recv_msg", None)
        original_write = getattr(master, "write", None)
        if not callable(original_recv) or not callable(original_write):
            self._stderr("pymavlink 연결에 recv_msg/write 가 없어 통신 장애를 적용하지 못함")
            return
        delivered = {}          # 종류별로 마지막에 어댑터까지 전달된 메시지

        def recv_msg():
            msg = original_recv()
            if msg is None:
                return None
            get_type = getattr(msg, "get_type", None)
            mtype = get_type() if get_type is not None else None
            if mtype == "GLOBAL_POSITION_INT":
                self.observe_position(msg.lat / 1e7, msg.lon / 1e7)
            elif mtype == "HEARTBEAT" and getattr(msg, "autopilot", None) != MAV_AUTOPILOT_INVALID:
                # 비행 컨트롤러의 하트비트만 본다 (지상국·보조 장치는 autopilot=8)
                self.observe_armed(bool(getattr(msg, "base_mode", 0) & MAV_MODE_FLAG_SAFETY_ARMED))
            if self.link_down():
                # pymavlink 는 숨기기 전에 이미 master.messages(종류별 최신값 캐시)를 갱신했다.
                # 그대로 두면 어댑터가 캐시로 끊긴 링크 너머의 상태(예: 착륙 후 시동 꺼짐)를 알게 되므로
                # 마지막으로 전달된 값으로 되돌린다.
                messages = getattr(master, "messages", None)
                if isinstance(messages, dict) and mtype is not None:
                    if mtype in delivered:
                        messages[mtype] = delivered[mtype]
                    else:
                        messages.pop(mtype, None)
                return None
            if mtype is not None:
                delivered[mtype] = msg
            return msg

        def write(buf):
            if self._bypass_tx:                  # 기체 failsafe 흉내 (bypass 참고)
                return original_write(buf)
            if self.link_down():
                self.count_tx_drop()
                return None
            return original_write(buf)

        master.recv_msg = recv_msg
        master.write = write

    def status(self):
        """현재 주입 상태 (JSON 으로 바로 쓸 수 있는 dict). 미들웨어 기록용."""
        if not self.enabled:
            return {"enabled": False, "drone": self.name, "identities": list(self.identities)}
        with self._lock:
            self._evaluate(actions=False)
            now = self.clock()
            out = {
                "enabled": True,
                "drone": self.name,
                "identities": list(self.identities),
                "armed": self._armed,
                "flight_s": round(self._flight_s, 1),
                "battery": None,
                "link": {"state": self._link_state, "since": utc_iso(self._link_since),
                         "quality_pct": self._link_quality()},
                "stall": None,
                "pending": [],
            }
            battery_id = self._active.get("battery")
            if battery_id:
                p = (self._entry(battery_id) or {}).get("params") or {}
                start, flight0 = self._battery_start.get(battery_id, (None, None))
                value = self._battery_value_now
                out["battery"] = {
                    "entry_id": battery_id,
                    "battery_remaining_pct": value,
                    "battery_voltage_v": self._voltage_for(value) if value is not None else None,
                    "target_pct": float(p.get("pct", 100.0)),
                    "from_pct": start,
                    "ramp_s": float(p.get("ramp_s", 0) or 0),
                    "drain_s": float(p.get("drain_s", 0) or 0),
                    "flight_s_since_start": (round(self._flight_s - flight0, 1)
                                             if flight0 is not None else None),
                    "empty": value is not None and value <= 0,
                    "forced_landing_sent": self._battery_empty_done.get(battery_id),
                    "active_since": utc_iso(self._active_since["battery"]),
                }
            link_id = self._active.get("link")
            if link_id:
                p = (self._entry(link_id) or {}).get("params") or {}
                out["link"].update(entry_id=link_id, mode=p.get("mode"))
                if p.get("mode") in ("degraded", "range"):
                    out["link"]["interval_s"] = self._link_interval()
                if p.get("mode") in ("down", "range"):
                    out["link"].update(
                        failsafe=p.get("failsafe", DEFAULT_FAILSAFE),
                        failsafe_after_s=float(p.get("failsafe_after_s", DEFAULT_FAILSAFE_AFTER_S)),
                        failsafe_sent=(self._failsafe_sent
                                       if self._link_state == "down" and self._failsafe_done else None))
                if p.get("mode") == "range":
                    out["link"].update(degraded_m=float(p["degraded_m"]), down_m=float(p["down_m"]),
                                       distance_m=(round(self._last_distance, 1)
                                                   if self._last_distance is not None else None))
            stall_id = self._active.get("stall")
            if stall_id:
                p = (self._entry(stall_id) or {}).get("params") or {}
                out["stall"] = {"entry_id": stall_id, "dropped": self._stall_dropped.get(stall_id, 0),
                                "count": p.get("count"),
                                "active_since": utc_iso(self._active_since["stall"])}
            active_ids = set(self._active.values())
            for e in self._entries:
                if e.get("id") in active_ids:
                    continue
                begin, end = self._window(e)
                if begin is None or now < begin:
                    out["pending"].append({"entry_id": e.get("id"), "kind": e.get("kind"),
                                           "start": e.get("start")})
            return out

    # ---------------------------------------------------------- 기록

    def _stream_context(self):
        """텔레메트리 기록에 붙이는 시뮬레이터 쪽 참값. 미들웨어에는 가지 않는다 (분석·검증용)."""
        context = self._where()
        context.update(link_state=self._link_state,
                       faults={k: v for k, v in self._active.items() if v})
        return context

    def _write_stream(self, sample, context):
        record = {"logged_at": utc_iso(self.clock()), "drone": self.name,
                  "telemetry": sample, "sim": context}
        line = json.dumps(record, ensure_ascii=False)
        path = self.stream_log
        with _STREAM_LOCK:
            try:
                os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
                self._stream_writes += 1
                if self._stream_writes % 500 == 1 and os.path.exists(path) \
                        and os.path.getsize(path) > STREAM_ROTATE_BYTES:
                    os.replace(path, path + ".1")
                with open(path, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
            except OSError as e:
                if not self._stream_failed:
                    self._stderr(f"텔레메트리 기록 실패 ({path}): {e}")
                    self._stream_failed = True

    def _stderr(self, text):
        # 미들웨어는 stdout 을 MCP 통신에 쓸 수 있으므로 항상 stderr 로 쓴다.
        print(f"[장애 주입] {text}", file=sys.stderr, flush=True)

    def _log(self, event, quiet=False, **fields):
        record = {"logged_at": utc_iso(self.clock()), "event": event,
                  "drone": self.name, "identities": list(self.identities), **fields}
        try:
            os.makedirs(os.path.dirname(self.event_log) or ".", exist_ok=True)
            with open(self.event_log, "a", encoding="utf-8") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")
        except OSError as e:
            if not self._log_failed:
                self._stderr(f"이벤트 기록 실패 ({self.event_log}): {e}")
                self._log_failed = True
        if quiet:
            return
        if event == "fault_started":
            self._stderr(f"{self.name} {fields['kind']} 시작: {fields.get('summary')}")
        elif event == "fault_ended":
            self._stderr(f"{self.name} {fields['kind']} 종료 ({fields.get('reason')})")
        elif event == "fault_link_state":
            extra = f" (홈에서 {fields['distance_m']}m)" if "distance_m" in fields else ""
            self._stderr(f"{self.name} 통신 {fields['state_from']} -> {fields['state_to']}{extra}, "
                         f"링크 품질 {fields.get('link_quality_pct')}%")
        elif event == "fault_goto_dropped":
            if fields.get("reason") == "battery_empty":
                self._stderr(f"{self.name} goto 무시 (배터리 0%, 기체가 날 수 없음)")
            else:
                count = fields.get("count")
                self._stderr(f"{self.name} goto 무시 ({fields['dropped']}/"
                             f"{'무제한' if count is None else count})")
        elif event == "fault_failsafe":
            if fields.get("sent"):
                self._stderr(f"{self.name} 두절 {fields['after_s']:g}초 - 기체 failsafe {fields['mode'].upper()} "
                             f"(실제 기체가 스스로 할 동작을 대신 보냄)")
            else:
                self._stderr(f"{self.name} 기체 failsafe {fields['mode'].upper()} 를 보내지 못함: "
                             f"{fields.get('reason')}")
        elif event == "fault_battery_level":
            where = (f", 홈에서 {fields['home_distance_m']:g}m"
                     if fields.get("home_distance_m") is not None else "")
            self._stderr(f"{self.name} 배터리 {fields['pct']:g}% {fields['voltage_v']:.2f}V "
                         f"(비행 {fields['flight_s']:g}초{where})")
        elif event == "fault_battery_empty":
            if fields.get("sent"):
                self._stderr(f"{self.name} 배터리 0% - 더 날 수 없어 그 자리 강제 착륙(LAND) "
                             f"(실제 기체라면 추락·불시착)")
            else:
                self._stderr(f"{self.name} 배터리 0% - 강제 착륙을 보내지 못함: {fields.get('reason')}")
        elif event == "fault_takeoff_blocked":
            self._stderr(f"{self.name} 배터리 {fields.get('pct')}% - 이륙(시동) 거부")
