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
    def land(self, timeout=60):
        """착륙 + 시동 꺼짐까지 확인. 성공하면 True."""

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

    @staticmethod
    def distance_m(lat1, lon1, lat2, lon2):
        """두 좌표 사이 수평 거리(m). 짧은 거리용 평면 근사."""
        dlat = (lat2 - lat1) * 111320
        dlon = (lon2 - lon1) * 111320 * math.cos(math.radians(lat1))
        return math.sqrt(dlat ** 2 + dlon ** 2)
