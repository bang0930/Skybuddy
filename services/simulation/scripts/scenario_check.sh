#!/bin/bash
# SITL 2대 + 미들웨어로 장애 주입과 탐색 섹터를 확인한다. 실행해 두고 기다리면 된다.
#
# 먼저: Windows 에서 AirSim(산악맵) 실행 -> WSL 에서 bash ~/skybuddy/start_hetero_sim.sh 가 끝날 것.
# 미들웨어 서버는 이 스크립트가 직접 띄우고 내린다 (따로 켜 둔 서버가 있으면 먼저 끌 것).
#
#   bash ~/skybuddy/scenario_check.sh sections   섹터 비행만 (mission-101, 장애 없음, 약 4분)
#   bash ~/skybuddy/scenario_check.sh battery    배터리 소진 (mission-101, 약 4분)
#   bash ~/skybuddy/scenario_check.sh link       거리에 따른 통신 악화·두절 (mission-101, 약 4분)
#   bash ~/skybuddy/scenario_check.sh faults     통신 두절 20초 + 웨이포인트 미도달 (mission-001, 약 5분)
#   bash ~/skybuddy/scenario_check.sh            faults -> sections
#   (팀 저장소에서는 bash services/simulation/scripts/scenario_check.sh <모드>)
#
# 장애 시나리오(battery, link, faults)는 드론이 홈이 아닌 곳에 내려앉을 수 있다.
# 다음 시나리오 전에 AirSim 과 SITL 을 다시 켜서 출발점으로 돌려놓는다 (scenarios/README.md 주의).
#
# 어느 경우든 먼저 AirSim 화면에 섹터 경계와 지그재그 경로를 그리고, 비행하는 동안 드론 위치를
# 색으로 표시한다 (track_drones.py: drone-01 자홍, drone-02 주황, 이름표에 배터리·링크 품질).
#
# battery  drone-01 -> sector-0-0(30m): 시동이 걸린 동안만 배터리가 줄어 120초 비행에 0%.
#          2026-11-09 섹터 비행 실측 시간에 맞춘 값이라 섹터 가운데쯤 30%, 마지막 줄에서 0% 가 된다.
#          아직 미들웨어·LLM 이 배터리로 귀환시키지 않으므로 0% 에서 그 자리 강제 착륙 -> goto 무시 ->
#          WAYPOINT_TIMEOUT 으로 끝나는 것이 지금은 정상이다. drone-02 -> sector-0-1(40m) 은 장애 없이 비교용.
# link     drone-02 -> sector-1-1(60m, 가장 먼 섹터): 링크 품질이 거리에 따라 내려가고 250m 넘으면 degraded,
#          330m 넘으면 두절 -> 5초 뒤 기체 failsafe RTL -> 돌아오며 복구 -> 미들웨어(#42)가 귀환·착륙.
#          drone-01 -> sector-0-0(30m) 은 장애 없이 비교용.
# faults   drone-01: 이륙 15초 뒤 20초 통신 두절 -> 미들웨어 LINK_LOST, 기체 failsafe RTL, 복구 후 착륙
#          drone-02: 이륙 5초 뒤 다음 goto 1번 무시 -> 60초 뒤 WAYPOINT_TIMEOUT, 그 자리 착륙
#          일부러 낸 장애라 두 작업 모두 failed 가 정상이다.
# sections mission-101 의 sector-0-0(30m)·sector-0-1(40m) 를 장애 없이 지그재그로 수색. 모두 성공해야 정상이다.
#
# 기록은 ~/skybuddy/logs/scenario_<시각>/ 에 남는다
#   server_<시험>.log            미들웨어 서버 출력 ([장애 주입] 줄 포함)
#   mission-events_<시험>.jsonl  미들웨어 이벤트
#   fault-events_<시험>.jsonl    장애 주입 이벤트 (배터리 10% 단위, 통신 상태 변화, 강제 착륙 등)
#   telemetry_<시험>.jsonl       미들웨어에 넘긴 텔레메트리 전부 (미들웨어 개발에 그대로 쓰는 원본)
#   telemetry_<시험>.csv         같은 내용 CSV (그래프용)

# ROS setup.bash 는 정의 안 된 변수를 참조하므로 set -u 보다 먼저 불러온다.
source /opt/ros/humble/setup.bash
source "$HOME/ardu_ws/install/setup.bash"
set -u
set -m          # 백그라운드 프로세스가 SIGINT 를 받도록 (그래야 서버가 정상 종료 기록을 남긴다)

MODE="${1:-all}"
# 개발 환경(평면 구조, ~/skybuddy)과 팀 저장소(services/simulation/scripts) 어느 쪽에서도 돈다.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$HERE/kpi_mission.py" ]; then
    BASE="$HERE"                                       # 개발 환경
    TEAM="${SKYBUDDY_TEAM:-$HOME/skybuddy-team}"       # 미들웨어가 들어 있는 팀 저장소
else
    BASE="$(dirname "$HERE")"                          # services/simulation
    TEAM="$(cd "$BASE/../.." && pwd)"                   # 팀 저장소 최상위
fi
SCRIPTS="$HERE"
OUT="$HOME/skybuddy/logs/scenario_$(date +%Y%m%d_%H%M%S)"
URL="http://127.0.0.1:9000/mcp"

case "$MODE" in
    all|faults|sections|battery|link) ;;
    *) echo "모드는 sections / battery / link / faults / all 중 하나: $MODE"; exit 2 ;;
esac
mkdir -p "$OUT"

export MCP_TRANSPORT=http
export MIDDLEWARE_DRONES_CONFIG="$BASE/scenarios/drones.sitl.json"
export MIDDLEWARE_SCENARIOS_DIR="$BASE/scenarios/missions"
SERVER=""
TRACKER=""

start_tracker() {    # $1 = 시험 이름. AirSim 화면에 드론 위치 표시 (실패해도 시험은 계속)
    python3 "$SCRIPTS/track_drones.py" --stream "$OUT/telemetry_$1.jsonl" > "$OUT/tracker_$1.log" 2>&1 &
    TRACKER=$!
    sleep 2
    if kill -0 "$TRACKER" 2>/dev/null; then
        echo "    AirSim 드론 표시 켜짐 (drone-01 자홍, drone-02 주황)"
    else
        echo "    (AirSim 드론 표시를 켜지 못함 - 시험은 계속. $OUT/tracker_$1.log 참고)"
        TRACKER=""
    fi
}

stop_tracker() {
    [ -n "$TRACKER" ] || return 0
    kill -INT "$TRACKER" 2>/dev/null
    for _ in $(seq 10); do
        kill -0 "$TRACKER" 2>/dev/null || break
        sleep 0.5
    done
    kill -9 "$TRACKER" 2>/dev/null
    TRACKER=""
}

start_server() {     # $1 = 시험 이름
    export MIDDLEWARE_EVENT_LOG="$OUT/mission-events_$1.jsonl"
    export SKYBUDDY_FAULT_LOG="$OUT/fault-events_$1.jsonl"
    export SKYBUDDY_TELEMETRY_LOG="$OUT/telemetry_$1.jsonl"
    (cd "$TEAM" && PYTHONPATH="services/middleware:${PYTHONPATH:-}" exec python3 -m app.mcp_server) \
        > "$OUT/server_$1.log" 2>&1 &
    SERVER=$!
    for _ in $(seq 90); do
        if grep -q "listening on" "$OUT/server_$1.log"; then
            echo "    미들웨어 서버 준비됨 (pid $SERVER)"
            grep -E "연결됨|장애 주입|connect failed" "$OUT/server_$1.log" | sed 's/^/    | /'
            start_tracker "$1"
            return 0
        fi
        kill -0 "$SERVER" 2>/dev/null || break
        sleep 1
    done
    echo "!! 미들웨어 서버가 뜨지 않았다. 서버 출력 마지막 부분:"
    tail -n 20 "$OUT/server_$1.log" | sed 's/^/    | /'
    return 1
}

stop_server() {
    stop_tracker
    [ -n "$SERVER" ] || return 0
    kill -INT "$SERVER" 2>/dev/null
    for _ in $(seq 30); do
        kill -0 "$SERVER" 2>/dev/null || break
        sleep 1
    done
    kill -9 "$SERVER" 2>/dev/null
    SERVER=""
}
trap stop_server EXIT

summary() {          # $1 = 시험 이름
    echo "--- 장애 주입 기록 ---"
    python3 "$SCRIPTS/fault.py" log -n 40
    echo "--- 미들웨어 이벤트 (장애·작업 결과) ---"
    grep -E '"event": "(link_lost|link_restored|link_not_restored|link_recovery_finished|emergency_stop|task_finished)"' \
        "$OUT/mission-events_$1.jsonl" | cut -c1-260 | sed 's/^/    /'
    echo "--- 서버 터미널에 나온 [장애 주입] 줄 ---"
    grep "장애 주입" "$OUT/server_$1.log" | tail -n 30 | sed 's/^/    /'
    echo "--- 미들웨어에 넘긴 텔레메트리 (드론마다 약 2초에 1줄, 마지막 40줄) ---"
    python3 "$SCRIPTS/fault.py" stream --file "$OUT/telemetry_$1.jsonl" --every 8 -n 40
    if python3 "$SCRIPTS/fault.py" stream --file "$OUT/telemetry_$1.jsonl" --csv "$OUT/telemetry_$1.csv" > /dev/null; then
        echo "    CSV: $OUT/telemetry_$1.csv"
    fi
}

echo "=== 0) AirSim 화면에 섹터 경계와 지그재그 수색 경로 그리기 ==="
python3 "$SCRIPTS/map_probe.py" draw "$BASE/scenarios/missions/mission-101.json" \
    --drones-config "$MIDDLEWARE_DRONES_CONFIG" || echo "    (그리기 실패 - 시험은 계속)"

if [ "$MODE" = all ] || [ "$MODE" = faults ]; then
    echo
    echo "=== 장애 시험: mission-001 / drone-01 통신 두절 20초, drone-02 웨이포인트 미도달 ==="
    if start_server faults; then
        python3 "$SCRIPTS/fault.py" clear
        python3 "$SCRIPTS/fault.py" link drone-01 down --after-takeoff 15 --for 20
        python3 "$SCRIPTS/fault.py" stall drone-02 --after-takeoff 5
        (cd "$TEAM" && python3 services/middleware/scripts/check_mvp_pipeline.py --url "$URL" --timeout 400)
        python3 "$SCRIPTS/fault.py" clear
        stop_server
        summary faults
    fi
fi

if [ "$MODE" = all ] || [ "$MODE" = sections ]; then
    echo
    echo "=== 섹터 비행: mission-101 / drone-01 -> sector-0-0(30m), drone-02 -> sector-0-1(40m), 장애 없음 ==="
    if start_server sections; then
        python3 "$SCRIPTS/fault.py" clear
        python3 "$SCRIPTS/run_plan.py" mission-101 drone-01=sector-0-0 drone-02=sector-0-1 --timeout 600
        stop_server
        summary sections
    fi
fi

if [ "$MODE" = battery ]; then
    echo
    echo "=== 배터리 소진: mission-101 / drone-01 -> sector-0-0 (비행 120초에 0%), drone-02 -> sector-0-1 (장애 없음) ==="
    if start_server battery; then
        python3 "$SCRIPTS/fault.py" clear
        python3 "$SCRIPTS/fault.py" battery drone-01 0 --drain 120
        python3 "$SCRIPTS/run_plan.py" mission-101 drone-01=sector-0-0 drone-02=sector-0-1 --timeout 600
        python3 "$SCRIPTS/fault.py" clear
        stop_server
        summary battery
    fi
fi

if [ "$MODE" = link ]; then
    echo
    echo "=== 통신 거리: mission-101 / drone-02 -> sector-1-1 (250m 악화, 330m 두절), drone-01 -> sector-0-0 (장애 없음) ==="
    if start_server link; then
        python3 "$SCRIPTS/fault.py" clear
        python3 "$SCRIPTS/fault.py" link drone-02 range --degraded-m 250 --down-m 330
        python3 "$SCRIPTS/run_plan.py" mission-101 drone-01=sector-0-0 drone-02=sector-1-1 --timeout 600
        python3 "$SCRIPTS/fault.py" clear
        stop_server
        summary link
    fi
fi

echo
echo "기록 폴더: $OUT"
if [ "$MODE" != sections ]; then
    echo "장애 시나리오를 돌렸다면 다음 시나리오 전에 AirSim 과 SITL 을 다시 켤 것 (드론이 출발점이 아닌 곳에 있을 수 있다)"
fi
