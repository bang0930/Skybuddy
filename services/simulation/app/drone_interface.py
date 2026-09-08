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


# 공통 텔레메트리 스키마.
# 두 프로토콜이 같은 CSV 컬럼으로 떨어져야 나란히 비교할 수 있다.
TELEMETRY_FIELDS = [
    "lat", "lon", "relative_alt_m",
    "vx", "vy", "vz",
    "heading_deg",
    "gps_fix_type", "satellites_visible", "gps_eph",
    "battery_voltage_v", "battery_remaining_pct",
    "vibration_x", "vibration_y", "vibration_z", "clipping",
    "timestamp",
]

# 프로토콜별로 제공하지 못하는 항목. 빈 문자열("")로 채운다.
# 이건 구현 부족이 아니라 프로토콜 간 실제 기능 격차이므로 숨기지 않고 기록한다.
UNSUPPORTED_BY_DDS = ["vibration_x", "vibration_y", "vibration_z", "clipping",
                      "satellites_visible"]


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
        """시동 -> 이륙 -> 목표 고도 도달까지. 실패하면 예외."""

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
    def get_telemetry(self, timeout=2):
        """TELEMETRY_FIELDS 스키마의 dict 를 반환. 데이터가 없으면 None.
        새 데이터가 올 때까지 짧게 블로킹한다 - 호출 측이 while 루프로 폴링하므로
        논블로킹이면 CPU 를 다 먹는다."""

    @abstractmethod
    def close(self):
        """연결 정리."""

    @staticmethod
    def distance_m(lat1, lon1, lat2, lon2):
        """두 좌표 사이 수평 거리(m). 짧은 거리용 평면 근사."""
        dlat = (lat2 - lat1) * 111320
        dlon = (lon2 - lon1) * 111320 * math.cos(math.radians(lat1))
        return math.sqrt(dlat ** 2 + dlon ** 2)
