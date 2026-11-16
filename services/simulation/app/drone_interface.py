"""드론 프로토콜 어댑터가 지켜야 하는 계약(contract).

이 파일이 SkyBuddy 이기종 구조의 핵심이다.
미션 코드(kpi_mission.py)와 미들웨어(박병언)는 이 계약만 알면 되고,
그 아래에 MAVLink 로 말하는 기체가 붙든 DDS 로 말하는 기체가 붙든 신경 쓰지 않는다.

  kpi_mission.py / 미들웨어
        |
        v
  DroneInterface  <-- 여기까지만 안다
        |
    +---+---+
    |       |
 MavlinkDrone   Ros2Drone
 (ArduPilot 4.3)  (ArduPilot 4.7.1 + AP_DDS)
   MAVLink         ROS 2 / DDS

구현체를 바꿔도 kpi_mission.py 의 미션 로직은 한 줄도 바뀌지 않는다.
이게 "미들웨어는 프로토콜을 몰라도 된다"는 주장의 실물 증거다.
"""
import math
import sys
import time
from abc import ABC, abstractmethod
from typing import Optional, TypedDict


class Telemetry(TypedDict):
    """get_telemetry() 반환 타입. 미들웨어가 그대로 모델로 옮길 수 있는 형태다.

    [값이 없을 때는 None 이다. 빈 문자열이 아니다.]
    같은 필드가 상황에 따라 숫자와 문자열 두 타입을 갖지 않도록,
    어댑터는 항상 숫자 아니면 None 을 반환한다.
    빈 문자열로 바꾸는 것은 CSV 로 쓸 때뿐이며 telemetry_to_csv_row() 가 담당한다.

    [필드는 항상 전부 존재한다.]
    프로토콜이 제공하지 못하는 값도 키 자체는 있고 None 이 들어간다.
    미들웨어가 키 존재 여부를 매번 확인하지 않아도 된다.
    """
    lat: Optional[float]                    # WGS-84 위도(도)
    lon: Optional[float]                    # WGS-84 경도(도)
    relative_alt_m: Optional[float]         # 홈(이륙 지점) 기준 상대 고도(m). AMSL 아님
    vx: Optional[float]                     # NED 북쪽 속도(m/s)
    vy: Optional[float]                     # NED 동쪽 속도(m/s)
    vz: Optional[float]                     # NED 아래쪽 속도(m/s). 아래가 양수
    heading_deg: Optional[float]            # 진북 기준 시계방향 0~360도
    gps_fix_type: Optional[int]             # 3 = 3D Fix 이상이 정상
    satellites_visible: Optional[int]       # 위성 수. AP_DDS 는 제공하지 않음
    gps_eph: Optional[float]                # 수평 위치 정밀도(m)
    battery_voltage_v: Optional[float]      # 전압(V)
    battery_remaining_pct: Optional[float]  # 잔량(%)
    vibration_x: Optional[float]            # 진동. AP_DDS 는 제공하지 않음
    vibration_y: Optional[float]
    vibration_z: Optional[float]
    clipping: Optional[int]                 # 가속도계 포화 횟수. AP_DDS 는 제공하지 않음
    link_quality_pct: Optional[float]       # 링크 품질 0~100(%). 시뮬레이션은 장애 주입이 채운다.
                                            # 실기체 어댑터는 아직 None (텔레메트리 라디오 RSSI 등으로 채울 자리)
    timestamp: float                        # Unix epoch. 항상 값이 있다


# 공통 텔레메트리 스키마. Telemetry TypedDict 의 키와 순서가 일치해야 한다.
# 두 프로토콜이 같은 CSV 컬럼으로 떨어져야 나란히 비교할 수 있다.
TELEMETRY_FIELDS = list(Telemetry.__annotations__.keys())

# 프로토콜별로 제공하지 못하는 항목. None 이 들어간다.
# 이건 구현 부족이 아니라 프로토콜 간 실제 기능 격차이므로 숨기지 않고 기록한다.
# 미들웨어는 이 필드들을 필수값으로 가정하면 안 된다.
UNSUPPORTED_BY_DDS = ["vibration_x", "vibration_y", "vibration_z", "clipping",
                      "satellites_visible"]


def telemetry_to_csv_row(t):
    """Telemetry 를 CSV 한 줄로 직렬화한다. None 은 빈 칸으로 쓴다.

    어댑터 반환값에서는 None 을 그대로 두고, 파일로 나갈 때만 빈 문자열이 된다.
    이렇게 나눠야 미들웨어가 받는 값의 타입이 흔들리지 않는다.
    """
    return {k: ("" if v is None else v) for k, v in t.items()}


class DroneInterface(ABC):
    """모든 드론 어댑터의 공통 계약.

    좌표 규약 (구현체가 반드시 맞춰야 한다):
      lat, lon        : WGS-84 도(degree)
      relative_alt_m  : 홈(이륙 지점) 기준 상대 고도, 미터. AMSL 아님.
      vx, vy, vz      : NED 기준 m/s (vz 는 아래쪽이 양수)
      heading_deg     : 진북 기준 시계방향 0~360도

    연결 유지(상주) 운용 규약:
      미들웨어 Registry 는 connect() 를 한 번만 하고 객체를 계속 들고 있으면서
      명령이 올 때마다 메서드를 부른다. 그래서 구현체는
        1) 한참 쉬었다가 불려도 '그 시점의' 최신 상태를 돌려줘야 하고
        2) 같은 연결로 이륙-착륙을 여러 번 반복할 수 있어야 한다.
      구현체는 스레드 안전하지 않다. 여러 스레드에서 쓸 때는 호출 측이 한 번에
      하나씩만 부르도록 직렬화해야 한다 (미들웨어의 DroneHandle.exclusive 가 담당).
    """

    @abstractmethod
    def connect(self, timeout=30):
        """기체에 연결하고 self 를 반환한다. 실패하면 예외."""

    @abstractmethod
    def takeoff(self, altitude_m):
        """시동 -> 이륙 -> **목표 고도 도달까지** 기다린 뒤 반환한다. 실패하면 예외.

        [완료 조건은 두 구현이 같아야 한다]
        명령이 수락된 시점이 아니라 고도가 실제로 올라온 시점에 반환해야 한다.
        그렇지 않으면 호출 측이 아직 지상 근처에 있는 기체에 goto() 를 보내게 된다.
        현재 두 구현 모두 목표 고도의 95% 도달을 완료 기준으로 쓴다.

        성공 시 반환값은 없다. 실패는 예외로만 알린다.
        """

    @abstractmethod
    def goto(self, lat, lon, alt_m):
        """목표 지점 지정. 도달을 기다리지 않는다(fire-and-forget).
        도달 판정은 호출 측이 get_telemetry() 로 직접 한다."""

    @abstractmethod
    def land(self, timeout=None):
        """착륙 + 시동 꺼짐까지 확인. 성공하면 True, 기한 안에 확인 못 하면 False.

        timeout 이 None 이면 현재 고도로 정한다 (landing_timeout_s 참고).
        미들웨어는 land() 를 인자 없이 부르므로, 기본값이 고도와 무관한 고정값이면
        높이 떠 있는 기체가 다 내려오기 전에 '착륙 실패'로 판정된다.
        timeout 이 지나도 아직 내려가는 중이면 더 기다린다 (LandingWait). 고도는 홈 기준이라
        착륙 지점 지형이 홈보다 낮으면 그만큼 더 내려가야 하기 때문이다.
        """

    @abstractmethod
    def disarm(self, timeout=5):
        """강제 시동 끄기(안전장치). 성공하면 True."""

    @abstractmethod
    def get_telemetry(self, timeout=2) -> Optional[Telemetry]:
        """Telemetry 를 반환. 데이터가 아직 없으면 None.

        값이 없는 개별 필드도 None 이다(빈 문자열 아님).
        새 데이터가 올 때까지 짧게 블로킹한다 - 호출 측이 while 루프로 폴링하므로
        논블로킹이면 CPU 를 다 먹는다.
        """

    @abstractmethod
    def close(self):
        """연결 정리."""

    def fault_status(self):
        """장애 주입 상태(dict). 선택 메서드라 주입 기능이 없는 구현은 None 을 돌려준다.

        MavlinkDrone / Ros2Drone 은 fault_injection.FaultInjector.status() 를 돌려준다.
        블로킹하지 않고 스레드 안전하므로 미들웨어가 DroneHandle.exclusive 없이 불러도 된다.
        미들웨어는 이 값을 기록용으로 쓰고, 장애 판단은 실제로 받은 데이터
        (battery_remaining_pct, 샘플 간격, connection_status, WAYPOINT_TIMEOUT)로 하는 것을 권한다.
        """
        return None

    @staticmethod
    def landing_timeout_s(relative_alt_m):
        """착륙 확인을 기다릴 시간(초). 고도 1m당 4초, 최소 60초.

        고정 60초로 두면 40m 에서 출발한 기체도 다 내려오기 전에 끊긴다 (실측 확인됨).
        """
        alt = relative_alt_m if relative_alt_m is not None else 0.0
        return max(60.0, alt * 4.0)

    @staticmethod
    def distance_m(lat1, lon1, lat2, lon2):
        """두 좌표 사이 수평 거리(m). 짧은 거리용 평면 근사."""
        dlat = (lat2 - lat1) * 111320
        dlon = (lon2 - lon1) * 111320 * math.cos(math.radians(lat1))
        return math.sqrt(dlat ** 2 + dlon ** 2)


# land() 가 기본 대기 시간(landing_timeout_s) 뒤에도 기다리는 조건.
# 대기 시간은 홈 기준 고도로 정하므로, 착륙 지점 지형이 홈보다 낮으면 그 안에 다 내려오지 못한다.
# 2026-11-09 SITL 실측: 배터리 0% 강제 착륙 지점(호수 쪽, 지형이 홈보다 20m 넘게 낮음)에서 60초 뒤에도
# 0.3m/s 로 내려가는 중이었고, 미들웨어는 land() 의 False 를 착륙 실패로 보고 공중에서 강제 disarm 했다.
LANDING_DESCENT_MIN_M_S = 0.15   # 이보다 빨리 내려가고 있으면 '착륙 중'으로 본다
LANDING_DESCENT_GRACE_S = 5.0    # 하강이 잠깐 멈춰도 이 시간은 더 기다린다
LANDING_EXTEND_MAX_S = 300.0     # 기본 대기 시간을 넘겨 더 기다리는 최대 시간


class LandingWait:
    """land() 의 대기 판단. 기본 시간이 지나도 아직 내려가고 있으면 계속 기다린다.

        wait = LandingWait(timeout)
        while wait.waiting():
            wait.observe(self.get_telemetry(timeout=1))    # 샘플의 vz 로 하강 여부를 본다
            if 시동이 꺼졌으면:
                return True
        return False

    내려가지도 않고 시동도 안 꺼지면 기본 시간에 끝난다 (그다음 조치는 호출 측 정책).
    통신이 끊겨 샘플이 없으면 하강을 확인할 수 없으므로 더 기다리지 않는다.
    """

    def __init__(self, timeout, clock=time.time):
        self.clock = clock
        self.deadline = clock() + float(timeout)
        self.hard_deadline = self.deadline + LANDING_EXTEND_MAX_S
        self.last_descent_at = None
        self.extended = False

    def observe(self, sample):
        vz = sample.get("vz") if sample else None
        if vz is not None and vz > LANDING_DESCENT_MIN_M_S:
            self.last_descent_at = self.clock()

    def waiting(self):
        now = self.clock()
        if now < self.deadline:
            return True
        if now >= self.hard_deadline or self.last_descent_at is None:
            return False
        if now - self.last_descent_at > LANDING_DESCENT_GRACE_S:
            return False
        if not self.extended:
            self.extended = True
            # 미들웨어가 stdout 을 MCP 통신에 쓸 수 있으므로 stderr 로 쓴다.
            print("[착륙] 기본 대기 시간이 지났지만 아직 내려가는 중이라 계속 기다린다 "
                  "(착륙 지점이 홈보다 낮을 수 있다)", file=sys.stderr, flush=True)
        return True
