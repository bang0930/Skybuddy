"""드론 한 대의 탐색 임무를 실행하고 KPI를 CSV로 남긴다.

이 파일이 SkyBuddy 시뮬레이션 파트의 중심이다. 그리고 **프로토콜을 모른다.**
MAVLink 드론이든 DDS 드론이든 아래 run_mission()은 완전히 같은 코드가 돈다.
프로토콜을 아는 곳은 make_drone() 팩토리 한 곳뿐이며,
tests/test_adapters.py가 이 성질이 깨지지 않았는지 자동으로 검사한다.

    from kpi_mission import run_mission

    run_mission(connection_string='udpin:127.0.0.1:14561',
                drone_id='drone1', protocol='mavlink')     # MAVLink 드론
    run_mission(connection_string='ros2:ap',
                drone_id='drone2', protocol='ros2')        # DDS 드론

산출물:
    kpi_log_<drone_id>.csv       원시 텔레메트리 (약 4Hz)
    kpi_summary_<drone_id>.csv   웨이포인트별 지표

임무 흐름:
    연결 → 이륙 → 웨이포인트 순회 → 출발점 귀환 → 착륙 → CSV 저장

[알려진 구조적 한계]
run_mission()이 위 전 과정을 한 함수에 담고 있어서, 밖에서 "웨이포인트 하나만
실행" 같은 요청을 받을 수 없다. 최종 형태는 미들웨어가 명령을 내리고 시뮬이
수행하는 방향이므로, 명령 단위 분해가 다음 작업이다. (INTERFACE.md 2-5 참고)
"""
import csv
import math
import time


def make_drone(connection_string, drone_id, protocol='mavlink'):
    """프로토콜에 맞는 어댑터를 생성한다.

    이 함수가 이 파일에서 프로토콜을 아는 유일한 곳이다.
    아래 run_mission() 의 미션·KPI 로직은 drone_interface.DroneInterface 계약만
    사용하므로, MAVLink 든 DDS 든 한 줄도 바뀌지 않는다.
    이게 "미들웨어는 프로토콜을 몰라도 된다"는 주장의 실물 증거다."""
    if protocol == 'mavlink':
        from mavlink_drone import MavlinkDrone
        return MavlinkDrone(connection_string)
    if protocol == 'ros2':
        # rclpy 는 ROS 환경을 소싱해야 import 되므로 필요할 때만 불러온다.
        # (MAVLink 단독 실행 시 ROS 없이도 동작해야 한다)
        from ros2_drone import Ros2Drone
        return Ros2Drone(connection_string, drone_id=drone_id)
    raise ValueError(f"알 수 없는 protocol: {protocol} ('mavlink' 또는 'ros2')")


# ── 탐색 구역 및 판정 기준 ──────────────────────────────────────────────────
# 값들은 대부분 실측으로 정해졌다. 임의로 낮추면 미션이 실패한다.
AREA_WIDTH_M = 20         # 탐색 구역 동서 폭 (정사각형이 되도록 남북과 동일)
AREA_HEIGHT_M = 20        # 탐색 구역 남북 폭
LANE_SPACING_M = 10       # 격자 탐색 라인 간격
TARGET_ALT = 30           # 순항 고도. 10~20m는 지형지물과 충돌해서 30m로 올렸다.
REACH_THRESHOLD_M = 3     # 도달 판정 반경. 2m는 기체가 조금만 흔들려도 못 들어간다.
WAYPOINT_TIMEOUT_S = 60   # 웨이포인트 하나당 제한 시간. 30초는 부족했다.
RTH_TIMEOUT_S = 60        # 임무 종료 후 출발점 복귀 대기 시간
GPS_HEALTHY_FIX_TYPE = 3  # 3 = 3D Fix. 이 미만이면 "GPS 저하"로 기록한다.


def offset_latlon(lat, lon, north_m, east_m):
    dlat = north_m / 111320
    dlon = east_m / (111320 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def generate_grid_waypoints(width_m, height_m, spacing_m):
    """왕복형(boustrophedon) 격자 탐색 패턴.
    구역을 spacing_m 간격의 띠(lane)로 나눠 지그재그로 훑는다 - 표준 수색/커버리지 패턴."""
    waypoints = []
    lane = 0
    north = 0
    while north <= height_m:
        if lane % 2 == 0:
            waypoints.append((north, 0))
            waypoints.append((north, width_m))
        else:
            waypoints.append((north, width_m))
            waypoints.append((north, 0))
        lane += 1
        north = lane * spacing_m
    return waypoints


def generate_perimeter_waypoints(width_m, height_m):
    """출발점을 중심에 두고 사각형 둘레를 한 바퀴 도는 경로 (시연용).

    출발점이 사각형의 정중앙이라 한 바퀴 돌고 나면 그대로 제자리로 복귀한다.
    두 대가 각자 자기 스폰을 중심으로 같은 크기의 사각형을 서로 반대 방향으로 돌면
    카메라 한 화면에서 각자 임무를 수행하는 게 한눈에 보인다."""
    h = height_m / 2.0
    w = width_m / 2.0
    return [
        (-h, -w),
        (-h,  w),
        ( h,  w),
        ( h, -w),
        (-h, -w),
    ]


def run_mission(connection_string='udpin:127.0.0.1:14550', drone_id='drone1',
                zone_offset_north_m=0, zone_offset_east_m=0, target_alt=None,
                pattern='grid', reverse=False, protocol='mavlink'):
    """드론 한 대의 격자 탐색 미션을 실행하고 KPI를 kpi_log_<id>.csv / kpi_summary_<id>.csv로 기록한다.
    여러 대를 동시에 돌리려면 이 함수를 스레드로 병렬 호출하면 된다 (kpi_mission_multi.py 참고).

    zone_offset_*  : 담당 구역을 홈에서 얼마나 떨어뜨릴지. 여러 대를 동시에 띄울 때
                     구역을 나누는 용도다. 두 드론의 스폰 위치는 6m밖에 차이가 안 나서,
                     오프셋 없이 돌리면 같은 공간을 같은 고도로 날아 충돌한다.
    target_alt     : 이 드론의 순항 고도. 동시 비행 시 드론마다 다르게 주면
                     같은 경로를 날아도 수직으로 분리된다.
    pattern        : 'grid'      - 격자형 왕복 탐색 (실제 수색 시나리오, KPI 측정용)
                     'perimeter' - 사각형 둘레 한 바퀴 (시연용)
    reverse        : True 면 웨이포인트 순서를 뒤집는다. 두 대가 같은 경로를
                     서로 반대 방향으로 돌게 할 때 쓴다.
    protocol       : 'mavlink' - ArduPilot 4.3 + MAVLink   (connection_string 예: udpin:127.0.0.1:14561)
                     'ros2'    - ArduPilot 4.7.1 + AP_DDS  (connection_string 예: ros2:ap)
                     이 값이 바뀌어도 아래 미션 로직은 전혀 달라지지 않는다."""

    alt = TARGET_ALT if target_alt is None else target_alt

    def log(msg):
        print(f"[{drone_id}] {msg}")

    drone = make_drone(connection_string, drone_id, protocol).connect()
    if pattern == 'perimeter':
        waypoints = generate_perimeter_waypoints(AREA_WIDTH_M, AREA_HEIGHT_M)
    else:
        waypoints = generate_grid_waypoints(AREA_WIDTH_M, AREA_HEIGHT_M, LANE_SPACING_M)
    if reverse:
        waypoints = list(reversed(waypoints))
    waypoints = [(n + zone_offset_north_m, e + zone_offset_east_m) for n, e in waypoints]
    log_rows = []
    wp_summary = []
    mission_start = time.time()

    try:
        drone.takeoff(alt)
    except Exception:
        # 이착륙 도중 실패해도 armed 상태로 남기지 않는다 - 순차 미션에서 다음 드론이
        # 뜨는 동안 이 드론 모터가 계속 살아있으면 동시 스로틀 상황이 재현돼 크래시로 이어짐 (실측 확인됨).
        log("이착륙 실패 - 강제 disarm 시도")
        if drone.disarm():
            log("disarm 확인됨")
        else:
            log("[경고] disarm 확인 실패 - armed 상태로 남아있을 수 있음")
        drone.close()
        raise

    time.sleep(5)

    home = drone.get_telemetry()
    home_lat, home_lon = home['lat'], home['lon']
    log(f"홈 위치: {home_lat:.6f}, {home_lon:.6f}")
    log(f"탐색 구역: {AREA_WIDTH_M}m x {AREA_HEIGHT_M}m, 라인 간격 {LANE_SPACING_M}m, "
        f"웨이포인트 {len(waypoints)}개")
    log(f"패턴: {pattern}{' (역방향)' if reverse else ''}, "
        f"구역 오프셋 N{zone_offset_north_m}m E{zone_offset_east_m}m, 순항 고도 {alt}m")

    try:
        for i, (north, east) in enumerate(waypoints, 1):
            target_lat, target_lon = offset_latlon(home_lat, home_lon, north, east)
            log(f"웨이포인트 {i}/{len(waypoints)} 이동 중 (N{north}m E{east}m)...")
            drone.goto(target_lat, target_lon, alt)

            wp_start = time.time()
            leg_points = []
            reached = False
            gps_degraded = False
            min_dist = float('inf')
            last_dist = None
            last_alt = None

            while time.time() - wp_start < WAYPOINT_TIMEOUT_S:
                t = drone.get_telemetry()
                if t is None:
                    continue
                log_rows.append({**t, "drone_id": drone_id, "waypoint": i,
                                  "elapsed_s": time.time() - mission_start})
                leg_points.append(t)
                if t.get("gps_fix_type", "") != "" and t["gps_fix_type"] < GPS_HEALTHY_FIX_TYPE:
                    if not gps_degraded:
                        log(f"  [경고] GPS 저하 감지 (fix_type={t['gps_fix_type']})")
                    gps_degraded = True
                dist = drone.distance_m(t['lat'], t['lon'], target_lat, target_lon)
                min_dist = min(min_dist, dist)
                last_dist = dist
                last_alt = t['relative_alt_m']
                if dist < REACH_THRESHOLD_M:
                    reached = True
                    break

            time_to_reach = time.time() - wp_start
            if reached:
                log(f"  도달 ({time_to_reach:.1f}초)")
            else:
                # 타임아웃이면 왜 못 갔는지 판단할 수 있게 거리를 남긴다.
                #   최소 거리가 임계값 근처면  -> 도달 판정 기준이 빡빡한 것
                #   최소 거리가 여전히 크면    -> 실제로 목표까지 못 간 것
                #   고도가 순항 고도와 다르면  -> 고도 유지 실패
                log(f"  타임아웃 ({WAYPOINT_TIMEOUT_S}초) - "
                    f"최종거리 {last_dist:.1f}m, 최소거리 {min_dist:.1f}m, "
                    f"고도 {last_alt:.1f}m (목표 {alt}m, 도달기준 {REACH_THRESHOLD_M}m)"
                    if last_dist is not None else "  타임아웃 (텔레메트리 없음)")

            prev_point = leg_points[0] if leg_points else home
            path_distance = 0.0
            for p in leg_points[1:]:
                path_distance += drone.distance_m(prev_point['lat'], prev_point['lon'], p['lat'], p['lon'])
                prev_point = p
            leg_start = leg_points[0] if leg_points else home
            leg_end = leg_points[-1] if leg_points else home
            straight_line = drone.distance_m(leg_start['lat'], leg_start['lon'], target_lat, target_lon)
            # 효율은 "목표까지 거리"가 아니라 "실제 도달한 지점까지 직선거리" 기준 (항상 <=1.0)
            achieved_straight_line = drone.distance_m(leg_start['lat'], leg_start['lon'], leg_end['lat'], leg_end['lon'])
            efficiency = (achieved_straight_line / path_distance) if path_distance > 0 else ""

            # 구간별 최대 진동 - 눈으로 보이는 흔들림을 웨이포인트 단위로 남긴다.
            vib_z_vals = [p["vibration_z"] for p in leg_points if p.get("vibration_z") != ""]
            clip_vals = [p["clipping"] for p in leg_points if p.get("clipping") != ""]

            wp_summary.append({
                "drone_id": drone_id,
                "waypoint": i,
                "north_m": north,
                "east_m": east,
                "target_lat": round(target_lat, 6),
                "target_lon": round(target_lon, 6),
                "time_to_reach_s": round(time_to_reach, 2),
                "straight_line_distance_m": round(straight_line, 2),
                "path_distance_m": round(path_distance, 2),
                "path_efficiency": round(efficiency, 3) if efficiency != "" else "",
                "reached": reached,
                "gps_degraded": gps_degraded,
                "vibration_z_max": round(max(vib_z_vals), 3) if vib_z_vals else "",
                "clipping_max": max(clip_vals) if clip_vals else "",
            })
    except Exception:
        # 웨이포인트 순회 도중 실패해도 armed/비행 중 상태로 남기지 않는다 - 이유는 takeoff
        # 실패 처리와 동일 (다음 순서 드론과 동시 스로틀 상황 방지).
        log("미션 도중 실패 - 강제 disarm 시도")
        if drone.disarm():
            log("disarm 확인됨")
        else:
            log("[경고] disarm 확인 실패 - armed 상태로 남아있을 수 있음")
        drone.close()
        raise

    # 출발점으로 귀환한 뒤 착륙한다.
    # 마지막 웨이포인트에 그대로 내려앉으면, 여러 대가 같은 경로를 날 때 같은 지점에
    # 착륙하게 되어 서로 부딪힌다. 각자 자기 홈으로 돌아가면 스폰 위치만큼 떨어져 착륙한다.
    log("귀환 중 (출발점으로 복귀)...")
    drone.goto(home_lat, home_lon, alt)
    rth_start = time.time()
    while time.time() - rth_start < RTH_TIMEOUT_S:
        t = drone.get_telemetry()
        if t is None:
            continue
        log_rows.append({**t, "drone_id": drone_id, "waypoint": "home",
                         "elapsed_s": time.time() - mission_start})
        if drone.distance_m(t['lat'], t['lon'], home_lat, home_lon) < REACH_THRESHOLD_M:
            log(f"  귀환 완료 ({time.time() - rth_start:.1f}초)")
            break
    else:
        log("  [경고] 귀환 타임아웃 - 현재 위치에서 착륙합니다")

    # 착륙 타임아웃은 고도에 맞춰 잡는다. 고정 60초로 두면 40m에서 출발한 기체가
    # 다 내려오기 전에 타임아웃이 나서 "여전히 armed일 수 있음" 경고가 뜬다 (실측 확인됨).
    land_timeout = max(60, int(alt * 4))
    landed = drone.land(timeout=land_timeout)
    log("착륙 및 disarm 확인됨" if landed else "[경고] 착륙 확인 타임아웃 (여전히 armed일 수 있음)")

    total_duration = time.time() - mission_start
    reached_count = sum(1 for w in wp_summary if w["reached"])
    total_path_distance = sum(w["path_distance_m"] for w in wp_summary)

    log(f"=== 미션 완료: {total_duration:.1f}초, 웨이포인트 {reached_count}/{len(waypoints)}, "
        f"이동거리 {total_path_distance:.1f}m ===")

    with open(f'kpi_log_{drone_id}.csv', 'w', newline='') as f:
        fieldnames = ["elapsed_s", "drone_id", "waypoint", "lat", "lon", "relative_alt_m",
                      "vx", "vy", "vz", "heading_deg", "gps_fix_type",
                      "satellites_visible", "gps_eph", "battery_voltage_v",
                      "battery_remaining_pct",
                      "vibration_x", "vibration_y", "vibration_z", "clipping",
                      "timestamp"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(log_rows)

    with open(f'kpi_summary_{drone_id}.csv', 'w', newline='') as f:
        fieldnames = ["drone_id", "waypoint", "north_m", "east_m", "target_lat", "target_lon",
                      "time_to_reach_s", "straight_line_distance_m", "path_distance_m",
                      "path_efficiency", "reached", "gps_degraded",
                      "vibration_z_max", "clipping_max"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(wp_summary)

    log(f"로그 저장: kpi_log_{drone_id}.csv, kpi_summary_{drone_id}.csv")
    drone.close()

    return {
        "drone_id": drone_id,
        "total_duration_s": total_duration,
        "waypoints_reached": reached_count,
        "waypoints_total": len(waypoints),
        "total_path_distance_m": total_path_distance,
    }


if __name__ == "__main__":
    run_mission()
