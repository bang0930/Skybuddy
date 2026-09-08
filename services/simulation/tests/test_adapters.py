"""어댑터 두 개가 같은 계약을 지키는지 정적으로 검증한다.

시뮬레이터 없이 돌아간다. 실제 비행 검증은 그다음:
  python3 ros2_drone.py          <- DDS 드론 단독 (이륙-이동-복귀-착륙)
  python3 kpi_mission_multi.py   <- 2대 동시 미션

실행 전:
  source /opt/ros/humble/setup.bash
  source ~/ardu_ws/install/setup.bash
  cd ~/skybuddy && python3 verify_adapters.py
"""
import inspect
import os
import sys

# 어댑터 코드가 이 파일과 같은 폴더에 있을 수도(평면 구조), 옆의 app/ 에 있을 수도 있다.
# 둘 다 지원해서 어디서 실행하든 동작하게 한다.
_HERE = os.path.dirname(os.path.abspath(__file__))
APP_DIR = _HERE if os.path.exists(os.path.join(_HERE, "kpi_mission.py")) \
          else os.path.join(os.path.dirname(_HERE), "app")
sys.path.insert(0, APP_DIR)

from drone_interface import TELEMETRY_FIELDS, UNSUPPORTED_BY_DDS, DroneInterface  # noqa: E402

REQUIRED = ["connect", "takeoff", "goto", "land", "disarm", "get_telemetry", "close",
            "distance_m"]

ok = True


def check(label, condition, detail=""):
    global ok
    mark = "OK  " if condition else "실패"
    print(f"  [{mark}] {label}" + (f"  -> {detail}" if detail and not condition else ""))
    if not condition:
        ok = False


print("=== 1) 어댑터 로드 ===")
from mavlink_drone import MavlinkDrone        # noqa: E402
print("  MavlinkDrone 로드됨 (pymavlink)")
try:
    from ros2_drone import Ros2Drone          # noqa: E402
    print("  Ros2Drone 로드됨 (rclpy)")
    adapters = [MavlinkDrone, Ros2Drone]
except ImportError as e:
    print(f"  [실패] Ros2Drone 로드 불가: {e}")
    print("         ROS 환경을 소싱했는지 확인할 것:")
    print("           source /opt/ros/humble/setup.bash")
    print("           source ~/ardu_ws/install/setup.bash")
    sys.exit(1)

print("\n=== 2) 계약 준수 ===")
for cls in adapters:
    print(f" {cls.__name__}")
    check("DroneInterface 상속", issubclass(cls, DroneInterface))
    for name in REQUIRED:
        check(f"{name}() 존재", hasattr(cls, name))
    check("추상 메서드 미구현 없음", not getattr(cls, "__abstractmethods__", None),
          str(getattr(cls, "__abstractmethods__", None)))

print("\n=== 3) 시그니처 호환 (kpi_mission.py 가 호출하는 형태) ===")
CALLS = {
    "connect": ["timeout"],
    "takeoff": ["altitude_m"],
    "goto": ["lat", "lon", "alt_m"],
    "land": ["timeout"],
    "disarm": ["timeout"],
    "get_telemetry": ["timeout"],
}
for cls in adapters:
    print(f" {cls.__name__}")
    for name, params in CALLS.items():
        sig = inspect.signature(getattr(cls, name))
        actual = [p for p in sig.parameters if p != "self"]
        check(f"{name}{tuple(actual)}", actual == params, f"기대 {params}")

print("\n=== 4) 텔레메트리 스키마 = CSV 컬럼 ===")
# kpi_mission.py 가 kpi_log_*.csv 에 쓰는 컬럼과 계약 스키마가 어긋나면
# DictWriter 가 ValueError 를 낸다. 미션 다 돌고 저장 직전에 터지면 최악이다.
CSV_EXTRA = {"elapsed_s", "drone_id", "waypoint"}
src = open(os.path.join(APP_DIR, "kpi_mission.py"), encoding="utf-8").read()
start = src.index('fieldnames = ["elapsed_s"')
block = src[start:src.index("]", start) + 1]
csv_fields = set(eval(block.split("=", 1)[1].strip()))  # noqa: S307
check("CSV 컬럼이 스키마를 모두 포함",
      set(TELEMETRY_FIELDS) <= csv_fields,
      f"누락 {set(TELEMETRY_FIELDS) - csv_fields}")
check("CSV 에 스키마 밖 컬럼 없음",
      csv_fields - CSV_EXTRA <= set(TELEMETRY_FIELDS),
      f"여분 {csv_fields - CSV_EXTRA - set(TELEMETRY_FIELDS)}")

print("\n=== 5) 프로토콜 기능 격차 (정상 - 기록용) ===")
print("  DDS 가 제공하지 못하는 항목 (빈 값으로 채워짐):")
for f in UNSUPPORTED_BY_DDS:
    print(f"    - {f}")
print("  이건 구현 부족이 아니라 프로토콜 간 실제 차이다. 이행서 근거로 쓸 것.")

print("\n=== 6) 미션 로직이 프로토콜을 모르는가 ===")
# run_mission() 본문에 프로토콜 이름이 등장하면 추상화가 샌 것이다.
body = src[src.index("def run_mission("):]
leaks = [w for w in ("MavlinkDrone", "Ros2Drone", "mavutil", "rclpy", "pymavlink")
         if w in body]
check("run_mission() 본문에 프로토콜 의존 없음", not leaks, f"발견 {leaks}")

print("\n" + ("전부 통과." if ok else "실패 항목이 있다. 위를 확인할 것."))
sys.exit(0 if ok else 1)
