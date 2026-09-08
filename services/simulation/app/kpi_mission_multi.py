"""드론 2대를 같은 AirSim 씬에서 동시에 띄우고 각자 임무를 수행시킨다.

프로젝트의 목표 구성인 "이기종 프로토콜 2대"를 실행하는 진입점이다.
아래 HETEROGENEOUS 구성에서 drone1은 MAVLink로, drone2는 AP_DDS(ROS 2)로
제어되지만, 두 대 모두 같은 run_mission() 코드가 처리한다.

    # 환경 기동 후
    source /opt/ros/humble/setup.bash
    source ~/ardu_ws/install/setup.bash
    python3 kpi_mission_multi.py

산출물은 드론별로 나뉜다.
    kpi_summary_drone1.csv / kpi_summary_drone2.csv   웨이포인트별 지표
    kpi_log_drone1.csv     / kpi_log_drone2.csv       원시 텔레메트리

두 드론이 같은 씬에 동시에 존재해야 하므로 settings.json에 Copter1/Copter2가
둘 다 정의돼 있어야 한다 (config_reference/airsim_settings_reference.json).

[실행 전 준비]
환경 기동은 start_hetero_sim.sh가 전부 처리한다. 그 스크립트가 하는 일과
각 단계가 필요한 이유는 스크립트 주석에 정리돼 있다.
    bash start_hetero_sim.sh

[미션은 스레드로 병렬 수행한다]
과거 ArduCopter 4.3 2대 구성에서는 동시 이륙 시 내부 오류(PANIC: flow_of_ctrl)로
크래시가 나서 순차 실행이 필요했다. 현재 이기종 구성(4.3 + 4.7.1)에서는 재현되지
않아 병렬로 돌린다. 문제가 재발하면 아래 PARALLEL을 False로 두면 된다.

[SITL 인스턴스에 관한 실측 규칙 3가지]
설정을 바꿀 때 이 세 가지를 어기면 원인을 찾기 어려운 증상이 나온다.

1) -I0을 쓰지 말 것
   인스턴스 0으로 띄운 기체는 상승은 하지만 호버에서 추력이 부족해 흔들린다.
   기체(pawn)를 바꿔도, 이륙 순서를 바꿔도 -I0 쪽만 증상이 나왔다. -I1, -I2를 쓴다.

2) 각 SITL은 반드시 별도 디렉토리에서 실행할 것
   같은 디렉토리에서 두 개를 돌리면 eeprom/파일디스크립터 충돌로
   "Bad file descriptor"가 난다. copter1_instance / copter2_instance를 쓴다.

3) --out을 임의로 지정하지 말 것
   sim_vehicle.py가 인스턴스별로 포트를 자동 배정한다 (-I1 → 14560/14561,
   -I2 → 14570/14571). 여기에 --out을 덧붙이면 드론1의 자동 포트와 드론2의 수동
   포트가 14560에서 충돌해 두 기체의 MAVLink가 한 포트에 섞인다.
"""
import threading

from kpi_mission import run_mission

# ── 비행 구성 ───────────────────────────────────────────────────────────────
#
# 두 대가 같은 X/Y 경로를 고도만 다르게 겹쳐 난다. 카메라 한 화면에 두 대가
# 위아래로 나란히 도는 게 보여서 시연에 좋다.
#
# 비행 순서
#   스폰 자리에서 순항 고도까지 수직 상승 → 그 고도를 유지한 채 20x20m 정사각형
#   한 바퀴 → 스폰 위치로 복귀 → 착륙.
#   낮은 고도에서 수평 이동을 아예 하지 않으므로 주변 지형지물에 걸리지 않는다.
#
# 충돌을 막는 세 가지 장치
#   고도 분리   30m / 50m로 20m 벌린다. 두 경로가 겹쳐도 서로 만나지 않는다.
#               (10~20m는 Blocks 환경의 장애물 높이와 겹쳐 실제로 충돌했다)
#   방향 분리   drone2는 reverse=True로 반대 방향으로 돈다.
#   경로 중심   각자 자기 스폰이 사각형의 정중앙이라 상대 스폰 위를 지나지 않고,
#               한 바퀴 돌면 그대로 출발점으로 돌아온다. 그래서 오프셋이 필요 없다.
#
# 각 항목의 의미
#   connection_string  MAVLink는 pymavlink 형식, DDS는 "ros2:<네임스페이스>"
#   protocol           'mavlink' 또는 'ros2'. 이 값이 어댑터를 결정한다.
#   target_alt         순항 고도(m). 홈 기준 상대 고도다.
#   pattern            'perimeter'(사각형 순회, 시연용) 또는 'grid'(격자 탐색)
#   reverse            웨이포인트 순서를 뒤집어 반대 방향으로 돌게 한다.

# ── 구성 1: MAVLink 2대 ─────────────────────────────────────────────────────
# 단일 프로토콜 구성. 이기종 구성에서 문제가 생겼을 때 이쪽으로 되돌려
# "이기종 때문인지 아닌지"를 분리하는 용도로 남겨둔다.
MAVLINK_2DRONE = [
    {"connection_string": "udpin:127.0.0.1:14561", "drone_id": "drone1",
     "protocol": "mavlink", "target_alt": 30, "pattern": "perimeter", "reverse": False},
    {"connection_string": "udpin:127.0.0.1:14571", "drone_id": "drone2",
     "protocol": "mavlink", "target_alt": 50, "pattern": "perimeter", "reverse": True},
]

# ── 구성 2: 이기종 (MAVLink 1대 + DDS 1대) ───────────────────────────────────
# 프로젝트의 목표 구성. 두 기체가 서로 다른 프로토콜로 같은 씬에서 각자 임무를 수행하고
# 각자 KPI 를 수집한다.
#   drone1: ArduPilot 4.3   + MAVLink   (~/ardupilot,   -I1, 포트 14561)
#   drone2: ArduPilot 4.7.1 + AP_DDS    (~/ardupilot45, -I2, ROS 2 /ap)
# 실행 전 ROS 환경 소싱 필요:
#   source /opt/ros/humble/setup.bash && source ~/ardu_ws/install/setup.bash
HETEROGENEOUS = [
    {"connection_string": "udpin:127.0.0.1:14561", "drone_id": "drone1",
     "protocol": "mavlink", "target_alt": 30, "pattern": "perimeter", "reverse": False},
    {"connection_string": "ros2:ap", "drone_id": "drone2",
     "protocol": "ros2", "target_alt": 50, "pattern": "perimeter", "reverse": True},
]

# 어느 구성을 돌릴지 여기서 고른다.
# 문제가 생기면 MAVLINK_2DRONE 으로 되돌려서 원인이 이기종 구성 때문인지 분리한다.
DRONES = HETEROGENEOUS

# True  = 두 대가 동시에 미션 수행 (미들웨어가 다루게 될 실제 형태)
# False = 한 대씩 순차 수행 (문제 생겼을 때 원인 분리용 안전 모드)
PARALLEL = True


def main():
    results = {}
    errors = {}

    def run_one(d):
        drone_id = d["drone_id"]
        try:
            results[drone_id] = run_mission(
                connection_string=d["connection_string"],
                drone_id=drone_id,
                zone_offset_north_m=d.get("zone_offset_north_m", 0),
                zone_offset_east_m=d.get("zone_offset_east_m", 0),
                target_alt=d.get("target_alt"),
                pattern=d.get("pattern", "grid"),
                reverse=d.get("reverse", False),
                protocol=d.get("protocol", "mavlink"))
        except Exception as e:
            errors[drone_id] = e
            print(f"[{drone_id}] 미션 실패: {e}")

    print("\n=== 구성 ===")
    for d in DRONES:
        print(f"  {d['drone_id']}: {d.get('protocol', 'mavlink'):8s} "
              f"{d['connection_string']:28s} 고도 {d.get('target_alt')}m")

    if PARALLEL:
        print("\n=== 두 대 동시 미션 시작 ===")
        threads = [threading.Thread(target=run_one, args=(d,)) for d in DRONES]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
    else:
        for d in DRONES:
            print(f"\n=== {d['drone_id']} 미션 시작 (다른 드론은 대기 중, 같은 씬에 계속 떠 있음) ===")
            run_one(d)

    print("\n=== 전체 결과 ===")
    for d in DRONES:
        drone_id = d["drone_id"]
        if drone_id in results:
            r = results[drone_id]
            print(f"{drone_id}: {r['waypoints_reached']}/{r['waypoints_total']} 웨이포인트, "
                  f"{r['total_duration_s']:.1f}초, {r['total_path_distance_m']:.1f}m")
        else:
            print(f"{drone_id}: 실패 - {errors.get(drone_id)}")


if __name__ == "__main__":
    main()
