"""어댑터 두 개가 같은 계약을 지키는지 정적으로 검증한다.

시뮬레이터 없이 돌아간다. 더 나아가 **pymavlink 도 ROS 2 도 없는 환경**에서
그냥 실행해도 된다. 9개 검사 중 6개는 파이썬 표준 라이브러리만 쓰므로
그대로 수행되고, 어댑터 임포트가 필요한 2)3) 만 건너뛴다.
macOS 를 쓰는 팀원이 코드를 받자마자 확인할 수 있게 하려고 이렇게 만들었다.

  python3 tests/test_adapters.py     <- 팀 저장소 기준 경로
  python3 verify_adapters.py         <- 개발 환경(평면 구조) 기준 경로

전부 갖춘 환경에서 실제 비행까지 검증하는 순서는 다음과 같다.
  source /opt/ros/humble/setup.bash
  source ~/ardu_ws/install/setup.bash
  python3 ros2_drone.py          <- DDS 드론 단독 (이륙-이동-복귀-착륙)
  python3 kpi_mission_multi.py   <- 2대 동시 미션
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

from drone_interface import (  # noqa: E402
    TELEMETRY_FIELDS, UNSUPPORTED_BY_DDS, DroneInterface, Telemetry,
    telemetry_to_csv_row,
)

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
# 어댑터를 임포트하려면 pymavlink(MAVLink) 와 rclpy+ardupilot_msgs(DDS) 가 필요하다.
# 둘 중 없는 것이 있으면 그 어댑터만 빼고 계속 진행한다.
# 여기서 중단시키면 환경이 없는 팀원은 나머지 6개 검사도 못 보게 된다.
adapters = []
skipped = []

try:
    from mavlink_drone import MavlinkDrone    # noqa: E402
    adapters.append(MavlinkDrone)
    print("  MavlinkDrone 로드됨 (pymavlink)")
except ImportError as e:
    skipped.append(("MavlinkDrone", str(e), "pip install pymavlink"))
    print(f"  [건너뜀] MavlinkDrone: {e}")

try:
    from ros2_drone import Ros2Drone          # noqa: E402
    adapters.append(Ros2Drone)
    print("  Ros2Drone 로드됨 (rclpy)")
except ImportError as e:
    skipped.append(("Ros2Drone", str(e),
                    "source /opt/ros/humble/setup.bash && "
                    "source ~/ardu_ws/install/setup.bash"))
    print(f"  [건너뜀] Ros2Drone: {e}")

if skipped:
    print("\n  위 어댑터는 실행 환경이 없어 2) 3) 검사를 건너뜁니다.")
    for name, err, how in skipped:
        print(f"    {name}: {how}")
    print("  4)~9) 는 표준 라이브러리만 쓰므로 그대로 수행합니다.")

# kpi_mission 은 프로토콜 라이브러리를 모듈 최상단에서 임포트하지 않는다
# (make_drone() 안에서만 한다). 그래서 어떤 환경에서도 임포트된다.
# 이 성질 자체가 "미션 로직은 프로토콜을 모른다"는 주장의 일부다.
import kpi_mission  # noqa: E402

print("\n=== 2) 계약 준수 ===")
if not adapters:
    print("  (어댑터 없음 - 건너뜀)")
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
if not adapters:
    print("  (어댑터 없음 - 건너뜀)")
for cls in adapters:
    print(f" {cls.__name__}")
    for name, params in CALLS.items():
        sig = inspect.signature(getattr(cls, name))
        actual = [p for p in sig.parameters if p != "self"]
        check(f"{name}{tuple(actual)}", actual == params, f"기대 {params}")

print("\n=== 4) 텔레메트리 타입 계약 ===")
# 미들웨어가 Pydantic 모델로 그대로 옮길 수 있으려면 필드와 타입이 고정되어야 한다.
check("Telemetry TypedDict 정의됨", hasattr(Telemetry, "__annotations__"))
check("TELEMETRY_FIELDS 가 Telemetry 키와 일치",
      TELEMETRY_FIELDS == list(Telemetry.__annotations__.keys()),
      f"{TELEMETRY_FIELDS} vs {list(Telemetry.__annotations__.keys())}")

# 누락값은 어댑터 반환에서 None, CSV 로 나갈 때만 빈 칸이어야 한다.
sample = {k: None for k in TELEMETRY_FIELDS}
sample["lat"] = 1.5
row = telemetry_to_csv_row(sample)
check("None 은 CSV 에서 빈 칸으로 직렬화", row["vibration_z"] == "")
check("값이 있는 필드는 그대로 유지", row["lat"] == 1.5)

# 어댑터가 누락값을 빈 문자열로 내보내면 같은 필드가 두 타입을 갖게 된다.
# 실제 소스를 읽어 확인한다 (어댑터를 인스턴스화하지 않고 검사할 수 있는 방법).
for fname in ("mavlink_drone.py", "ros2_drone.py"):
    text = open(os.path.join(APP_DIR, fname), encoding="utf-8").read()
    body = text[text.index("def get_telemetry"):]
    offenders = [pat for pat in ('else ""', ': ""') if pat in body]
    check(f"{fname} 가 누락값에 빈 문자열을 쓰지 않음", not offenders,
          f"발견 {offenders} - None 을 써야 한다")

print("\n=== 5) CSV 컬럼 = 스키마 ===")
# kpi_mission 이 쓰는 컬럼과 계약 스키마가 어긋나면 DictWriter 가 ValueError 를 낸다.
# 미션 다 돌고 저장 직전에 터지면 최악이다.
CSV_EXTRA = {"elapsed_s", "drone_id", "waypoint"}
csv_fields = set(kpi_mission.LOG_FIELDS)
check("CSV 컬럼이 스키마를 모두 포함",
      set(TELEMETRY_FIELDS) <= csv_fields,
      f"누락 {set(TELEMETRY_FIELDS) - csv_fields}")
check("CSV 에 스키마 밖 컬럼 없음",
      csv_fields - CSV_EXTRA <= set(TELEMETRY_FIELDS),
      f"여분 {csv_fields - CSV_EXTRA - set(TELEMETRY_FIELDS)}")

print("\n=== 6) 실행 경계의 안전 검증 ===")
# 최소 안전 고도는 미들웨어에서도 보지만, 실제로 명령을 보내는 쪽이 마지막 방어선이다.
try:
    kpi_mission.validate_altitude(kpi_mission.MIN_SAFE_ALT_M - 1)
    check(f"{kpi_mission.MIN_SAFE_ALT_M}m 미만 고도 거부", False, "예외가 나지 않았다")
except ValueError:
    check(f"{kpi_mission.MIN_SAFE_ALT_M}m 미만 고도 거부", True)
try:
    kpi_mission.validate_altitude(kpi_mission.MIN_SAFE_ALT_M)
    check(f"{kpi_mission.MIN_SAFE_ALT_M}m 이상 고도 허용", True)
except ValueError as e:
    check(f"{kpi_mission.MIN_SAFE_ALT_M}m 이상 고도 허용", False, str(e))

print("\n=== 7) 웨이포인트 입력 경로 ===")
# 미들웨어가 계산한 경로를 주입할 수 있어야 하고,
# 내부 생성 경로와 섞이면 어느 쪽이 비행되는지 알 수 없으므로 막혀야 한다.
sig = inspect.signature(kpi_mission.run_mission)
check("run_mission 에 waypoints 파라미터 존재", "waypoints" in sig.parameters)
check("execute_waypoint 가 단독 호출 가능", callable(getattr(kpi_mission, "execute_waypoint", None)))

external = kpi_mission.resolve_waypoints(waypoints=[(1.0, 2.0), (3.0, 4.0)])
check("외부 waypoints 를 그대로 사용", external == [(1.0, 2.0), (3.0, 4.0)], str(external))

internal = kpi_mission.resolve_waypoints(pattern='perimeter', home_lat=0.0, home_lon=0.0)
check("내부 생성 경로 동작", len(internal) == 5, f"{len(internal)}개")

for kwargs, why in [
    ({"waypoints": [(1.0, 2.0)], "pattern": "grid"}, "pattern 동시 지정"),
    ({"waypoints": [(1.0, 2.0)], "reverse": True}, "reverse 동시 지정"),
    ({"waypoints": [(1.0, 2.0)], "zone_offset_north_m": 5}, "zone_offset 동시 지정"),
    ({"waypoints": []}, "빈 waypoints"),
]:
    try:
        kpi_mission.resolve_waypoints(**kwargs)
        check(f"{why} 거부", False, "예외가 나지 않았다")
    except ValueError:
        check(f"{why} 거부", True)

print("\n=== 8) 프로토콜 기능 격차 (정상 - 기록용) ===")
print("  AP_DDS 가 제공하지 못하는 항목 (None 으로 채워짐):")
for f in UNSUPPORTED_BY_DDS:
    print(f"    - {f}")
print("  이건 구현 부족이 아니라 프로토콜 간 실제 차이다.")
print("  미들웨어는 이 필드들을 필수값으로 가정하면 안 된다.")

print("\n=== 9) 미션 로직이 프로토콜을 모르는가 ===")
# run_mission() 본문에 프로토콜 이름이 등장하면 추상화가 샌 것이다.
src = open(os.path.join(APP_DIR, "kpi_mission.py"), encoding="utf-8").read()
body = src[src.index("def run_mission("):]
leaks = [w for w in ("MavlinkDrone", "Ros2Drone", "mavutil", "rclpy", "pymavlink")
         if w in body]
check("run_mission() 본문에 프로토콜 의존 없음", not leaks, f"발견 {leaks}")

print()
if skipped:
    print(f"건너뛴 어댑터: {', '.join(n for n, _, _ in skipped)} "
          f"(실행 환경 미설치 - 코드 문제 아님)")
print("전부 통과." if ok else "실패 항목이 있다. 위를 확인할 것.")
sys.exit(0 if ok else 1)
