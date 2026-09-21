"""드론 한 대의 탐색 임무를 실행하고 KPI를 CSV로 남긴다.

이 파일이 SkyBuddy 시뮬레이션 파트의 중심이다. 그리고 **프로토콜을 모른다.**
MAVLink 드론이든 DDS 드론이든 아래 run_mission()은 완전히 같은 코드가 돈다.
프로토콜을 아는 곳은 make_drone() 팩토리 한 곳뿐이며,
tests/test_adapters.py가 이 성질이 깨지지 않았는지 자동으로 검사한다.

    from kpi_mission import run_mission

    # 내부에서 경로를 만드는 방식 (단독 실험용)
    run_mission(connection_string='udpin:127.0.0.1:14561',
                drone_id='drone1', protocol='mavlink', pattern='perimeter')

    # 미들웨어가 계산한 경로를 주입하는 방식 (통합 운용 형태)
    run_mission(connection_string='ros2:ap', drone_id='drone2', protocol='ros2',
                waypoints=[(-35.3632, 149.1651), (-35.3630, 149.1653)])

산출물:
    kpi_log_<drone_id>.csv       원시 텔레메트리 (약 4Hz)
    kpi_summary_<drone_id>.csv   웨이포인트별 지표

임무 흐름:
    연결 → 이륙 → 웨이포인트 순회 → 출발점 귀환 → 착륙 → CSV 저장

[명령 단위 분해 상태]
웨이포인트 하나를 실행하는 부분은 execute_waypoint() 로 분리해두었다.
미들웨어가 재할당이나 중단을 구현할 때 이 함수를 직접 호출하면 된다.
다만 연결·이륙·착륙까지 포함한 완전한 명령 단위 분해(예: arm/takeoff/goto/land 를
각각 외부에서 호출)는 미들웨어 연결 방식이 확정된 뒤에 진행한다. (INTERFACE.md 2-5)
"""
import csv
import math
import time

from drone_interface import telemetry_to_csv_row


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
TARGET_ALT = 30           # 순항 고도 기본값
MIN_SAFE_ALT_M = 30       # 최소 안전 고도. 아래 참고
REACH_THRESHOLD_M = 3     # 도달 판정 반경. 2m는 기체가 조금만 흔들려도 못 들어간다.
WAYPOINT_TIMEOUT_S = 60   # 웨이포인트 하나당 제한 시간. 30초는 부족했다.
RTH_TIMEOUT_S = 60        # 임무 종료 후 출발점 복귀 대기 시간
GPS_HEALTHY_FIX_TYPE = 3  # 3 = 3D Fix. 이 미만이면 "GPS 저하"로 기록한다.

# [최소 안전 고도를 실행 경계에서도 검증하는 이유]
# 10~20m 로 날렸을 때 지형지물과 충돌하는 것을 실측으로 확인했다.
# 미들웨어에서도 검증하겠지만, 실제로 기체에 명령을 보내는 쪽이 마지막 방어선이다.
# 상위 계층의 버그나 LLM 의 잘못된 출력이 그대로 비행 명령이 되는 것을 막는다.


def offset_latlon(lat, lon, north_m, east_m):
    dlat = north_m / 111320
    dlon = east_m / (111320 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def latlon_to_offset(home_lat, home_lon, lat, lon):
    """홈 기준 북/동 오프셋(m)으로 역산한다. 요약 CSV 의 north_m/east_m 컬럼용."""
    north_m = (lat - home_lat) * 111320
    east_m = (lon - home_lon) * 111320 * math.cos(math.radians(home_lat))
    return north_m, east_m


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


def validate_altitude(alt_m):
    """실행 직전 최소 안전 고도 검증. 위반하면 ValueError.

    미들웨어가 앞단에서 검증하더라도, 실제 명령을 보내는 이 계층에서 한 번 더 막는다.
    """
    if alt_m is None:
        raise ValueError("target_alt 가 None 이다")
    if alt_m < MIN_SAFE_ALT_M:
        raise ValueError(
            f"순항 고도 {alt_m}m 는 최소 안전 고도 {MIN_SAFE_ALT_M}m 미만이다. "
            f"저고도는 지형지물 충돌이 실측으로 확인됐다.")
    return alt_m


def resolve_waypoints(waypoints=None, pattern=None, reverse=False,
                      zone_offset_north_m=0, zone_offset_east_m=0,
                      home_lat=None, home_lon=None):
    """실행할 웨이포인트를 (위도, 경도) 리스트로 확정한다.

    두 가지 입력 경로가 있고, 어느 쪽을 쓸지 명확히 갈린다.

      waypoints 가 주어지면  외부(미들웨어)가 계산한 절대 좌표를 그대로 쓴다.
                            이때 pattern / reverse / zone_offset 은 쓸 수 없다.
      waypoints 가 없으면    pattern 으로 내부 생성한 뒤 홈 기준으로 변환한다.

    둘을 섞으면 어느 경로가 실제로 비행되는지 호출 측이 알 수 없으므로 예외로 막는다.
    """
    if waypoints is not None:
        conflicting = []
        if pattern is not None:
            conflicting.append("pattern")
        if reverse:
            conflicting.append("reverse")
        if zone_offset_north_m or zone_offset_east_m:
            conflicting.append("zone_offset_*")
        if conflicting:
            raise ValueError(
                f"waypoints 를 직접 주면 {', '.join(conflicting)} 은 쓸 수 없다. "
                f"외부 경로와 내부 생성 경로 중 하나만 선택해야 한다.")
        resolved = [(float(lat), float(lon)) for lat, lon in waypoints]
        if not resolved:
            raise ValueError("waypoints 가 비어 있다")
        return resolved

    if home_lat is None or home_lon is None:
        raise ValueError("내부 경로 생성에는 홈 좌표가 필요하다")

    if pattern is None:
        pattern = 'grid'
    if pattern == 'perimeter':
        rel = generate_perimeter_waypoints(AREA_WIDTH_M, AREA_HEIGHT_M)
    elif pattern == 'grid':
        rel = generate_grid_waypoints(AREA_WIDTH_M, AREA_HEIGHT_M, LANE_SPACING_M)
    else:
        raise ValueError(f"알 수 없는 pattern: {pattern} ('grid' 또는 'perimeter')")

    if reverse:
        rel = list(reversed(rel))
    rel = [(n + zone_offset_north_m, e + zone_offset_east_m) for n, e in rel]
    return [offset_latlon(home_lat, home_lon, n, e) for n, e in rel]


def execute_waypoint(drone, target_lat, target_lon, alt,
                     timeout_s=WAYPOINT_TIMEOUT_S, on_sample=None, log=print):
    """웨이포인트 하나로 이동하고 도달하거나 시간이 다할 때까지 폴링한다.

    미들웨어가 임무 재할당이나 중단을 구현할 때 이 함수를 단독으로 호출하면 된다.
    run_mission() 은 이 함수를 반복 호출하는 얇은 껍데기다.

    on_sample : 텔레메트리 1건마다 호출되는 콜백. 로그 수집에 쓴다.
    반환       : 이 구간의 지표 dict (도달 여부, 소요 시간, 경로 효율 등)
    """
    drone.goto(target_lat, target_lon, alt)

    wp_start = time.time()
    leg_points = []
    reached = False
    gps_degraded = False
    min_dist = float('inf')
    last_dist = None
    last_alt = None

    while time.time() - wp_start < timeout_s:
        t = drone.get_telemetry()
        if t is None:
            continue
        if on_sample is not None:
            on_sample(t)
        leg_points.append(t)

        fix = t["gps_fix_type"]
        if fix is not None and fix < GPS_HEALTHY_FIX_TYPE:
            if not gps_degraded:
                log(f"  [경고] GPS 저하 감지 (fix_type={fix})")
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
    elif last_dist is not None:
        # 타임아웃이면 왜 못 갔는지 판단할 수 있게 거리를 남긴다.
        #   최소 거리가 임계값 근처면  -> 도달 판정 기준이 빡빡한 것
        #   최소 거리가 여전히 크면    -> 실제로 목표까지 못 간 것
        #   고도가 순항 고도와 다르면  -> 고도 유지 실패
        log(f"  타임아웃 ({timeout_s}초) - 최종거리 {last_dist:.1f}m, "
            f"최소거리 {min_dist:.1f}m, 고도 {last_alt}m "
            f"(목표 {alt}m, 도달기준 {REACH_THRESHOLD_M}m)")
    else:
        log(f"  타임아웃 ({timeout_s}초) - 텔레메트리 없음")

    path_distance = 0.0
    if leg_points:
        prev_point = leg_points[0]
        for p in leg_points[1:]:
            path_distance += drone.distance_m(
                prev_point['lat'], prev_point['lon'], p['lat'], p['lon'])
            prev_point = p

    leg_start = leg_points[0] if leg_points else None
    leg_end = leg_points[-1] if leg_points else None
    if leg_start is not None:
        straight_line = drone.distance_m(
            leg_start['lat'], leg_start['lon'], target_lat, target_lon)
        # 효율은 "목표까지 거리"가 아니라 "실제 도달한 지점까지 직선거리" 기준 (항상 <=1.0)
        achieved = drone.distance_m(
            leg_start['lat'], leg_start['lon'], leg_end['lat'], leg_end['lon'])
        efficiency = (achieved / path_distance) if path_distance > 0 else None
    else:
        straight_line = None
        efficiency = None

    # 구간별 최대 진동 - 눈으로 보이는 흔들림을 웨이포인트 단위로 남긴다.
    # AP_DDS 는 이 토픽이 없어 None 으로 오므로 빈 리스트가 된다.
    vib_z_vals = [p["vibration_z"] for p in leg_points if p["vibration_z"] is not None]
    clip_vals = [p["clipping"] for p in leg_points if p["clipping"] is not None]

    return {
        "target_lat": round(target_lat, 6),
        "target_lon": round(target_lon, 6),
        "time_to_reach_s": round(time_to_reach, 2),
        "straight_line_distance_m": round(straight_line, 2) if straight_line is not None else None,
        "path_distance_m": round(path_distance, 2),
        "path_efficiency": round(efficiency, 3) if efficiency is not None else None,
        "reached": reached,
        "gps_degraded": gps_degraded,
        "vibration_z_max": round(max(vib_z_vals), 3) if vib_z_vals else None,
        "clipping_max": max(clip_vals) if clip_vals else None,
    }


def _emergency_stop(drone, log, airborne):
    """비행 중 실패했을 때의 정지 정책. 한 곳에서만 수행한다.

    공중이면 착륙을 먼저 시도하고, 실패하면 강제 disarm 한다.
    모터가 시동 상태로 남으면 여러 대 운용 시 다음 기체가 뜨는 동안 이 기체의
    스로틀도 살아 있는 상태가 되어 AirSim/ArduPilot 크래시를 유발한다 (실측 확인됨).
    """
    if airborne:
        log("비상 착륙 시도")
        try:
            if drone.land(timeout=60):
                log("착륙 확인됨")
                return
        except Exception as e:
            log(f"[경고] 착륙 시도 중 오류: {e}")
        log("[경고] 착륙 확인 실패 - 강제 disarm 시도")
    try:
        if drone.disarm():
            log("disarm 확인됨")
        else:
            log("[경고] disarm 확인 실패 - armed 상태로 남아있을 수 있음")
    except Exception as e:
        log(f"[경고] disarm 시도 중 오류: {e}")


def _write_csv(path, fieldnames, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


LOG_FIELDS = ["elapsed_s", "drone_id", "waypoint", "lat", "lon", "relative_alt_m",
              "vx", "vy", "vz", "heading_deg", "gps_fix_type",
              "satellites_visible", "gps_eph", "battery_voltage_v",
              "battery_remaining_pct",
              "vibration_x", "vibration_y", "vibration_z", "clipping",
              "timestamp"]

SUMMARY_FIELDS = ["drone_id", "waypoint", "north_m", "east_m", "target_lat", "target_lon",
                  "time_to_reach_s", "straight_line_distance_m", "path_distance_m",
                  "path_efficiency", "reached", "gps_degraded",
                  "vibration_z_max", "clipping_max"]


def run_mission(connection_string='udpin:127.0.0.1:14550', drone_id='drone1',
                zone_offset_north_m=0, zone_offset_east_m=0, target_alt=None,
                pattern=None, reverse=False, protocol='mavlink', waypoints=None):
    """드론 한 대의 탐색 임무를 실행하고 KPI를 CSV로 기록한다.
    여러 대를 동시에 돌리려면 이 함수를 스레드로 병렬 호출하면 된다 (kpi_mission_multi.py 참고).

    waypoints      : 외부(미들웨어)가 계산한 (위도, 경도) 리스트. 주면 이 경로를 그대로 난다.
                     이 값을 주면 pattern / reverse / zone_offset_* 은 쓸 수 없다.
    pattern        : 내부 경로 생성 방식. waypoints 를 주지 않았을 때만 유효하다.
                     'grid'      - 격자형 왕복 탐색 (실제 수색 시나리오, KPI 측정용)
                     'perimeter' - 사각형 둘레 한 바퀴 (시연용)
                     None 이면 'grid'
    zone_offset_*  : 내부 생성 경로를 홈에서 얼마나 떨어뜨릴지. 여러 대를 동시에 띄울 때
                     구역을 나누는 용도다.
    reverse        : 내부 생성 경로의 순서를 뒤집는다. 두 대가 같은 경로를
                     서로 반대 방향으로 돌게 할 때 쓴다.
    target_alt     : 순항 고도(m). MIN_SAFE_ALT_M 미만이면 ValueError.
    protocol       : 'mavlink' - ArduPilot + MAVLink  (connection_string 예: udpin:127.0.0.1:14561)
                     'ros2'    - ArduPilot + AP_DDS   (connection_string 예: ros2:ap)
                     이 값이 바뀌어도 아래 미션 로직은 전혀 달라지지 않는다.
    """

    def log(msg):
        print(f"[{drone_id}] {msg}")

    # 연결 전에 먼저 막는다. 잘못된 인자로 기체를 띄우고 나서 알아차리면 이미 공중이다.
    alt = validate_altitude(TARGET_ALT if target_alt is None else target_alt)
    # 경로 인자도 여기서 검증한다. 홈 좌표는 아직 모르므로 검증용 더미를 쓴다
    # (외부 주입 경로는 홈과 무관하고, 내부 생성 경로는 패턴 이름만 확인하면 된다).
    resolve_waypoints(waypoints=waypoints, pattern=pattern, reverse=reverse,
                      zone_offset_north_m=zone_offset_north_m,
                      zone_offset_east_m=zone_offset_east_m,
                      home_lat=0.0, home_lon=0.0)

    drone = make_drone(connection_string, drone_id, protocol).connect()

    log_rows = []
    wp_summary = []
    mission_start = time.time()
    airborne = False
    home_lat = home_lon = None

    # 연결부터 CSV 저장까지 전 구간을 감싼다.
    # 귀환·착륙·저장 중 어디서 실패하든 close() 가 반드시 실행되어야 하고,
    # 비행 중이었다면 기체를 지상으로 내려놓아야 한다.
    try:
        # takeoff() 는 목표 고도 도달까지 기다린 뒤 돌아온다(계약).
        drone.takeoff(alt)
        airborne = True

        # 고도는 도달했지만 자세와 위치 추정이 흔들리는 순간이 있다.
        # 여기서 바로 홈 좌표를 읽으면 몇 m 어긋난 값이 잡혀 이후 웨이포인트가
        # 전부 밀린다. 짧게 안정화 시간을 준다.
        time.sleep(5)

        home = drone.get_telemetry()
        if home is None:
            raise RuntimeError("이륙 후 텔레메트리를 받지 못했다")
        home_lat, home_lon = home['lat'], home['lon']

        targets = resolve_waypoints(
            waypoints=waypoints, pattern=pattern, reverse=reverse,
            zone_offset_north_m=zone_offset_north_m,
            zone_offset_east_m=zone_offset_east_m,
            home_lat=home_lat, home_lon=home_lon)

        source = "외부 주입" if waypoints is not None else f"내부 생성({pattern or 'grid'})"
        log(f"홈 위치: {home_lat:.6f}, {home_lon:.6f}")
        log(f"경로: {source}, 웨이포인트 {len(targets)}개, 순항 고도 {alt}m")

        for i, (target_lat, target_lon) in enumerate(targets, 1):
            north, east = latlon_to_offset(home_lat, home_lon, target_lat, target_lon)
            log(f"웨이포인트 {i}/{len(targets)} 이동 중 (N{north:.1f}m E{east:.1f}m)...")

            def collect(t, _i=i):
                log_rows.append({**t, "drone_id": drone_id, "waypoint": _i,
                                 "elapsed_s": time.time() - mission_start})

            result = execute_waypoint(drone, target_lat, target_lon, alt,
                                      on_sample=collect, log=log)
            wp_summary.append({
                "drone_id": drone_id,
                "waypoint": i,
                "north_m": round(north, 1),
                "east_m": round(east, 1),
                **result,
            })

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
        landed = drone.land(timeout=max(60, int(alt * 4)))
        airborne = not landed
        log("착륙 및 disarm 확인됨" if landed else "[경고] 착륙 확인 타임아웃 (여전히 armed일 수 있음)")

    except Exception:
        log("미션 실패 - 비상 정지 절차 수행")
        _emergency_stop(drone, log, airborne)
        airborne = False
        raise

    finally:
        # 실패해도 여기까지 수집된 로그는 남긴다. 원인 분석에 필요하다.
        # None 은 CSV 에서만 빈 칸이 된다 (어댑터 반환값에서는 None 그대로).
        # 여기서 예외가 나면 원래 실패 원인을 덮어써 버리므로 각각 감싼다.
        try:
            _write_csv(f'kpi_log_{drone_id}.csv', LOG_FIELDS,
                       [telemetry_to_csv_row(r) for r in log_rows])
            _write_csv(f'kpi_summary_{drone_id}.csv', SUMMARY_FIELDS,
                       [telemetry_to_csv_row(r) for r in wp_summary])
            log(f"로그 저장: kpi_log_{drone_id}.csv, kpi_summary_{drone_id}.csv")
        except Exception as e:
            log(f"[경고] CSV 저장 실패: {e}")
        try:
            drone.close()
        except Exception as e:
            log(f"[경고] 연결 정리 실패: {e}")

    total_duration = time.time() - mission_start
    reached_count = sum(1 for w in wp_summary if w["reached"])
    total_path_distance = sum(w["path_distance_m"] for w in wp_summary)

    log(f"=== 미션 완료: {total_duration:.1f}초, 웨이포인트 {reached_count}/{len(wp_summary)}, "
        f"이동거리 {total_path_distance:.1f}m ===")

    return {
        "drone_id": drone_id,
        "total_duration_s": total_duration,
        "waypoints_reached": reached_count,
        "waypoints_total": len(wp_summary),
        "total_path_distance_m": total_path_distance,
    }


if __name__ == "__main__":
    run_mission()
