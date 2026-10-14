"""run_mission() 의 실패 처리 정책을 시뮬레이터 없이 검증한다.

정해진 대로 성공하거나 실패하는 가짜 드론(ScriptedDrone)을 run_mission() 에 넣고,
어떤 순서로 착륙·시동 해제·연결 정리가 일어나는지 기록해 확인한다.
pymavlink·ROS 2·AirSim 이 없어도 돌아간다 (macOS 포함).

  python3 tests/test_mission_policy.py     <- 팀 저장소 기준 경로
  python3 verify_mission_policy.py         <- 개발 환경(평면 구조) 기준 경로

검사하는 정책 (PR #14 머지 후 후속 리뷰 2건):
  1) 착륙(시동 꺼짐)을 확인하지 못하면 비상 정지를 수행하고 미션을 실패로 처리한다.
     비상 정지 = 착륙 재시도 → 그래도 안 되면 강제 disarm.
  2) 이륙 명령 이후의 실패(상승 도중 시간 초과 등)는 공중일 수 있으므로
     disarm 이 아니라 착륙부터 시도한다. 공중에서 바로 disarm 하면 기체가 떨어진다.
  그리고 어떤 경우든 연결은 정리된다(close).
"""
import os
import sys
import tempfile
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
APP_DIR = _HERE if os.path.exists(os.path.join(_HERE, "kpi_mission.py")) \
          else os.path.join(os.path.dirname(_HERE), "app")
sys.path.insert(0, APP_DIR)

import kpi_mission                                           # noqa: E402
from drone_interface import TELEMETRY_FIELDS, DroneInterface  # noqa: E402

HOME = (-35.363261, 149.165230)

ok = True


def check(label, condition, detail=""):
    global ok
    mark = "OK  " if condition else "실패"
    print(f"  [{mark}] {label}" + (f"  -> {detail}" if detail and not condition else ""))
    if not condition:
        ok = False


class ScriptedDrone(DroneInterface):
    """시나리오대로 움직이는 가짜 드론. 호출된 메서드를 calls 에 순서대로 남긴다.

    takeoff_error : 지정하면 상승 도중(고도 절반)에 이 메시지로 예외를 던진다.
    land_results  : land() 가 차례로 돌려줄 값. 다 쓰면 False.
    goto 는 즉시 도착한 것으로 처리해 웨이포인트 대기 없이 끝난다.
    """

    def __init__(self, takeoff_error=None, land_results=(True,)):
        self.calls = []
        self._pos = HOME
        self._alt = 0.0
        self._takeoff_error = takeoff_error
        self._land_results = list(land_results)

    def connect(self, timeout=30):
        self.calls.append("connect")
        return self

    def takeoff(self, altitude_m):
        self.calls.append("takeoff")
        self._alt = altitude_m / 2           # 시동이 걸리고 올라가는 중
        if self._takeoff_error:
            raise RuntimeError(self._takeoff_error)
        self._alt = altitude_m

    def goto(self, lat, lon, alt_m):
        self.calls.append("goto")
        self._pos = (lat, lon)

    def land(self, timeout=None):
        self.calls.append("land")
        landed = self._land_results.pop(0) if self._land_results else False
        if landed:
            self._alt = 0.0
        return landed

    def disarm(self, timeout=5):
        self.calls.append("disarm")
        return True

    def get_telemetry(self, timeout=2):
        t = {k: None for k in TELEMETRY_FIELDS}
        t.update(lat=self._pos[0], lon=self._pos[1], relative_alt_m=self._alt,
                 gps_fix_type=3, timestamp=time.time())
        return t

    def close(self):
        self.calls.append("close")


def run(drone):
    """run_mission() 을 가짜 드론으로 실행하고 (결과, 예외) 를 돌려준다."""
    kpi_mission.make_drone = lambda *args, **kwargs: drone
    try:
        result = kpi_mission.run_mission(connection_string="fake", drone_id="policy_test",
                                         target_alt=30, pattern="perimeter")
        return result, None
    except Exception as e:      # noqa: BLE001 - 실패 처리 자체를 검사한다
        return None, e


def main():
    # 시험 중 생기는 CSV 가 작업 폴더를 어지럽히지 않게 임시 폴더에서 돌린다.
    original_cwd = os.getcwd()
    workdir = tempfile.mkdtemp(prefix="mission_policy_")
    os.chdir(workdir)
    kpi_mission.POST_TAKEOFF_SETTLE_S = 0
    try:
        print("=== 1) 정상 임무 ===")
        d = ScriptedDrone()
        result, err = run(d)
        check("예외 없이 끝남", err is None, repr(err))
        check("웨이포인트 5/5 도달",
              result is not None and result["waypoints_reached"] == 5, str(result))
        check("착륙 1회, 강제 disarm 없음",
              d.calls.count("land") == 1 and "disarm" not in d.calls, str(d.calls))
        check("마지막에 연결 정리", d.calls[-1] == "close", str(d.calls))

        print("\n=== 2) 착륙 확인 실패 → 착륙 재시도로 수습 ===")
        d = ScriptedDrone(land_results=(False, True))
        result, err = run(d)
        check("미션이 실패로 처리됨 (예외 전달)", err is not None)
        check("비상 정지에서 착륙을 다시 시도함", d.calls.count("land") == 2, str(d.calls))
        check("재시도로 착륙했으므로 강제 disarm 은 안 함", "disarm" not in d.calls, str(d.calls))
        check("연결 정리", d.calls[-1] == "close", str(d.calls))

        print("\n=== 3) 착륙 확인 실패 → 재시도도 실패 → 강제 disarm ===")
        d = ScriptedDrone(land_results=(False, False))
        result, err = run(d)
        check("미션이 실패로 처리됨", err is not None)
        check("착륙 재시도 뒤에 disarm",
              "disarm" in d.calls and d.calls.index("disarm") > d.calls.index("land"),
              str(d.calls))
        check("연결 정리", d.calls[-1] == "close", str(d.calls))

        print("\n=== 4) 이륙 도중 실패 → 착륙부터 시도 ===")
        d = ScriptedDrone(takeoff_error="이륙 후 고도 도달 실패 (목표 30m, 최종 15m)")
        result, err = run(d)
        check("미션이 실패로 처리됨", err is not None)
        after = d.calls[d.calls.index("takeoff") + 1:]
        check("곧바로 disarm 하지 않고 착륙부터 시도",
              bool(after) and after[0] == "land", str(d.calls))
        check("착륙을 확인했으므로 강제 disarm 없음", "disarm" not in d.calls, str(d.calls))
        check("연결 정리", d.calls[-1] == "close", str(d.calls))

        print("\n=== 5) 착륙 대기 시간 규칙 ===")
        check("고도 정보 없음 → 60초", DroneInterface.landing_timeout_s(None) == 60)
        check("30m → 120초", DroneInterface.landing_timeout_s(30) == 120)
        check("50m → 200초 (고정 60초면 다 내려오기 전에 끊김)",
              DroneInterface.landing_timeout_s(50) == 200)
    finally:
        os.chdir(original_cwd)

    print("\n" + ("전부 통과." if ok else "실패 항목이 있다. 위를 확인할 것."))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
