"""연결을 한 번만 하고 같은 연결로 임무를 두 번 수행해 본다 (실제 SITL 필요).

미들웨어 Registry 는 드론마다 connect() 를 한 번만 하고 객체를 계속 들고 있으면서
명령이 올 때마다 메서드를 부른다. run_mission() 은 매번 새로 연결하므로 이 방식을
검증하지 못한다. 이 스크립트가 그 사용 방식을 그대로 재현한다.

  1회차: 이륙 → 북쪽 15m 이동 → 출발점 복귀 → 착륙
  (연결을 유지한 채 IDLE_S 초 동안 아무것도 읽지 않고 쉰다)
  2회차: 같은 순서를 같은 연결로 반복
  마지막에만 close()

쉬는 동안 MAVLink 수신 버퍼에 메시지가 쌓이므로, 2회차에서 오래된 응답이나
위치를 잘못 읽으면 이륙·도착·착륙 판정이 틀어진다. 두 회차가 모두 통과해야 한다.

실행 (환경 기동 후):
  source /opt/ros/humble/setup.bash
  source ~/ardu_ws/install/setup.bash
  cd ~/skybuddy && python3 session_check.py
"""
import os
import sys
import threading
import time

# 개발 환경(평면 구조)에서도, 팀 저장소의 scripts/ 에서도 app/ 코드를 찾을 수 있게 한다.
_HERE = os.path.dirname(os.path.abspath(__file__))
APP_DIR = _HERE if os.path.exists(os.path.join(_HERE, "kpi_mission.py")) \
          else os.path.join(os.path.dirname(_HERE), "app")
sys.path.insert(0, APP_DIR)

from kpi_mission import (POST_TAKEOFF_SETTLE_S, _emergency_stop,  # noqa: E402
                         execute_waypoint, make_drone, offset_latlon, validate_altitude)

IDLE_S = 30          # 회차 사이에 연결만 유지한 채 쉬는 시간
LEG_NORTH_M = 15     # 이동 거리. 두 기체 모두 북쪽으로 평행하게 움직인다

# 드론 ID 는 미들웨어 설정(#24 drones.real.example.json)과 똑같이 쓴다.
# 하이픈이 들어간 ID 로 AP_DDS 노드가 만들어지는지도 함께 확인된다.
DRONES = [
    {"drone_id": "drone-01", "protocol": "mavlink",
     "connection_string": "udpin:127.0.0.1:14561", "alt": 30},
    {"drone_id": "drone-02", "protocol": "ros2",
     "connection_string": "ros2:ap", "alt": 50},
]


def one_round(drone, alt, log):
    """이륙 → 이동 → 복귀 → 착륙. 실패하면 예외."""
    drone.takeoff(alt)
    time.sleep(POST_TAKEOFF_SETTLE_S)   # run_mission 과 같은 안정화 대기
    home = drone.get_telemetry()
    if home is None:
        raise RuntimeError("이륙 후 텔레메트리 없음")
    north_lat, north_lon = offset_latlon(home["lat"], home["lon"], LEG_NORTH_M, 0)

    out = execute_waypoint(drone, north_lat, north_lon, alt, log=log)
    if not out["reached"]:
        raise RuntimeError("이동 지점 미도달")
    back = execute_waypoint(drone, home["lat"], home["lon"], alt, log=log)
    if not back["reached"]:
        raise RuntimeError("출발점 복귀 미도달")
    if not drone.land():
        raise RuntimeError("착륙 확인 실패")


def run(cfg, results):
    drone_id = cfg["drone_id"]

    def log(msg):
        print(f"[{drone_id}] {msg}")

    alt = validate_altitude(cfg["alt"])
    drone = make_drone(cfg["connection_string"], drone_id, cfg["protocol"]).connect()
    passed = []
    try:
        for n in (1, 2):
            if n == 2:
                log(f"연결 유지한 채 {IDLE_S}초 대기 (이 동안 아무것도 읽지 않음)")
                time.sleep(IDLE_S)
            log(f"{n}회차 시작")
            try:
                one_round(drone, alt, log)
                log(f"{n}회차 통과")
                passed.append(True)
            except Exception as e:      # noqa: BLE001 - 회차별 결과를 남긴다
                log(f"{n}회차 실패: {e}")
                passed.append(False)
                # run_mission 과 같은 정책으로 기체를 내려놓는다 (착륙 → 안 되면 disarm).
                _emergency_stop(drone, log, maybe_airborne=True)
                break
    finally:
        drone.close()
    results[drone_id] = passed


def main():
    results = {}
    threads = [threading.Thread(target=run, args=(cfg, results)) for cfg in DRONES]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    print("\n=== 연결 유지 운용 점검 결과 ===")
    all_ok = True
    for cfg in DRONES:
        passed = results.get(cfg["drone_id"], [])
        ok = passed == [True, True]
        all_ok &= ok
        detail = ", ".join(f"{i}회차 {'통과' if p else '실패'}" for i, p in enumerate(passed, 1))
        print(f"  {cfg['drone_id']}: {'통과' if ok else '실패'} ({detail or '연결 실패'})")
    print("전부 통과." if all_ok else "실패 항목이 있다. 위 로그를 확인할 것.")


if __name__ == "__main__":
    main()
