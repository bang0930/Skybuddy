#!/bin/bash
# 이기종 2대 구성을 기동한다.
#   drone1 : ArduCopter 4.3   + MAVLink   (~/ardupilot,   -I1, Copter1 9013/9012, 포트 14561)
#   drone2 : ArduCopter 4.7.1 + AP_DDS    (~/ardupilot45, -I2, Copter2 9023/9022, ROS 2 /ap)
#
# 같은 AirSim 씬 안에서 서로 다른 프로토콜을 쓰는 두 기체가 각자 임무를 수행하고
# 각자 KPI 를 수집한다. 이게 프로젝트의 목표 구성이다.
#
# 사전 조건:
#   1) settings.json 이 2기체 구성이어야 한다 (Copter1 + Copter2).
#      PowerShell:
#        Copy-Item "\\wsl.localhost\ubuntu-22.04\home\l9302\skybuddy\config_reference\airsim_settings_reference.json" `
#                  "$env:USERPROFILE\OneDrive\Documents\AirSim\settings.json" -Force
#   2) 바꿨으면 Blocks 를 완전히 껐다 켤 것 (settings.json 은 시작할 때만 읽는다)
#   3) Blocks 실행 중일 것
#      (프로세스가 2개로 보이는 건 정상 - 런처 + 본체)
#
# 사용법:  bash ~/skybuddy/start_hetero_sim.sh
#
# AirSim 이 다른 기계(예: AWS Windows 서버)에 있으면 그 주소를 SIM_ADDRESS 또는 AIRSIM_HOST 로 준다
# (기본 127.0.0.1 = 같은 기계). AIRSIM_HOST 는 팀 문서 docs/development/aws-deployment.md 와 같은 이름이고,
# map_probe.py·track_drones.py·minimap.py 도 이 값으로 AirSim API 에 붙는다.
#   export AIRSIM_HOST=10.0.1.23 && bash start_hetero_sim.sh
# 그때 AirSim 쪽 settings.json 은 UdpIp = 이 기계(SITL) 주소, LocalHostIp = 0.0.0.0 (또는 AirSim 기계 주소),
# 방화벽은 UDP 9012·9022 (SITL -> AirSim), 9013·9023 (AirSim -> SITL) 을 열어야 한다 (ArduPilot AirSim 문서).

set -e

# 개발 환경(평면 구조, ~/skybuddy)과 팀 저장소(services/simulation/scripts) 어느 쪽에서도 돈다.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$HERE/kpi_mission.py" ]; then
    CONF="$HERE/config_reference"                        # 개발 환경
else
    CONF="$(dirname "$HERE")/config"                     # 팀 저장소 services/simulation/config
fi
PARM="$CONF/dds_drone.parm"          # drone2 부팅 파라미터 (시동 검사 생략 + 비행 속도 WP_SPD·WP_ACC)
PARM1="$CONF/mavlink_drone.parm"     # drone1 비행 속도 (WPNAV_*·LAND_SPEED) - 실제 속도가 drone2 와 같게 맞춘 값
TUNE="$HERE/tune_damping.py"
APPLY="$HERE/apply_params.py"
SIM_ADDRESS="${SIM_ADDRESS:-${AIRSIM_HOST:-127.0.0.1}}"   # 팀 AWS 문서처럼 AIRSIM_HOST 만 줘도 된다

AP43="${AP43:-$HOME/ardupilot}"        # MAVLink 드론 (ArduCopter 4.3)
AP47="${AP47:-$HOME/ardupilot45}"      # DDS 드론 (디렉토리 이름은 45지만 내용물은 4.7.1)
INST1="$AP43/copter1_instance"
INST2="$AP47/copter2_instance"
LOGDIR="$HOME/skybuddy/logs"
AGENT_LOG="$LOGDIR/hetero_agent.log"
LOG1="$LOGDIR/hetero_drone1_mavlink.log"
LOG2="$LOGDIR/hetero_drone2_dds.log"
SETTINGS="/mnt/c/Users/l9302/OneDrive/Documents/AirSim/settings.json"

mkdir -p "$LOGDIR"

echo "=== 0) 기존 프로세스 정리 ==="
for sig in TERM KILL; do
    pkill -$sig -f arducopter     2>/dev/null || true
    pkill -$sig -f MicroXRCEAgent 2>/dev/null || true
    pkill -$sig -f mavproxy       2>/dev/null || true
    sleep 1
done
if [ "$(pgrep -cf 'arducopter|MicroXRCEAgent|mavproxy')" != "0" ]; then
    echo "!! 프로세스가 남아있다: pgrep -af 'arducopter|mavproxy'"
    exit 1
fi
echo "    정리 완료"

echo
echo "=== 1) settings.json 확인 ==="
# 2기체 구성이 아니면 lock-step 때문에 Blocks 가 그대로 멈춘다. 미리 잡는다.
if [ -f "$SETTINGS" ]; then
    N=$(grep -cE '"Copter[12]"' "$SETTINGS" || true)
    if [ "$N" -lt 2 ]; then
        echo "!! settings.json 에 Copter1/Copter2 가 둘 다 없다 (찾은 개수: $N)."
        echo "   AirSim 은 lock-step 이라 정의된 기체의 SITL 이 하나라도 안 붙으면 멈추고,"
        echo "   반대로 기체가 하나뿐인데 SITL 을 둘 띄우면 두 번째가 붙을 곳이 없다."
        echo
        echo "   PowerShell 에서 2기체 구성을 배포하고 Blocks 를 재시작할 것:"
        echo '     Copy-Item "\\wsl.localhost\ubuntu-22.04\home\l9302\skybuddy\config_reference\airsim_settings_reference.json" `'
        echo '               "$env:USERPROFILE\OneDrive\Documents\AirSim\settings.json" -Force'
        exit 1
    fi
    echo "    OK - Copter1/Copter2 2기체 구성"
    # AirSim 작은 창 배치: 1(왼쪽) = drone-01, 2(가운데) = drone-02 촬영 카메라, 3 = 숨김(미니맵 자리).
    # 다르면 고친다. settings.json 은 AirSim 이 켜질 때만 읽으므로 AirSim 을 다시 켜라고 멈춘다.
    # (건너뛰려면 SKYBUDDY_SKIP_LAYOUT=1)
    if [ "${SKYBUDDY_SKIP_LAYOUT:-0}" != 1 ] && [ -f "$HERE/track_drones.py" ]; then
        if python3 "$HERE/track_drones.py" layout --check > /tmp/skybuddy_layout_check.txt 2>&1; then
            echo "    OK - 작은 창 1(왼쪽) = drone-01, 2(가운데) = drone-02 촬영 카메라, 3 = 숨김(미니맵 자리)"
        else
            sed 's/^/    /' /tmp/skybuddy_layout_check.txt
            if python3 "$HERE/track_drones.py" layout; then
                echo "!! AirSim 작은 창 배치를 위처럼 고쳤다. AirSim 을 완전히 끄고 다시 켠 뒤 이 스크립트를 다시 실행할 것"
            else
                echo "!! 작은 창 배치를 고치지 못했다 (위 오류). 직접: python3 $HERE/track_drones.py layout"
            fi
            exit 1
        fi
    fi
else
    echo "    [건너뜀] $SETTINGS 를 읽을 수 없다. 직접 확인할 것."
fi

echo
echo "=== 2) DDS 드론용 인스턴스 디렉토리 ==="
# 각 SITL 은 반드시 별도 디렉토리에서 실행해야 한다.
# 같은 곳에서 두 개를 돌리면 eeprom/fd 충돌로 "Bad file descriptor" 가 난다 (실측 확인됨).
mkdir -p "$INST2"
# eeprom 을 지운다. ArduPilot 은 eeprom 에 저장된 값이 있으면 --add-param-file 의
# 기본값보다 그걸 우선하므로, 한 번이라도 기본값으로 부팅했으면 ARMING_SKIPCHK 0 이
# 굳어버려서 파라미터 파일이 무시된다 (2026-09-07 실측: 두 번째 실행부터 재현).
# 이 인스턴스는 시뮬레이션 전용이고 보존할 튜닝값이 없으므로 매번 초기화한다.
rm -f "$INST2/eeprom.bin"
echo "    $INST2  (eeprom 초기화됨)"

echo
echo "=== 3) micro-XRCE-DDS Agent 기동 (포트 2019) ==="
nohup MicroXRCEAgent udp4 -p 2019 > "$AGENT_LOG" 2>&1 < /dev/null &
sleep 2
pgrep -f MicroXRCEAgent > /dev/null || { echo "!! Agent 기동 실패: $AGENT_LOG"; exit 1; }
echo "    OK"

wait_gps() {  # $1=로그 $2=이름
    local deadline=$((SECONDS + 180))
    while [ $SECONDS -lt $deadline ]; do
        grep -q "EKF3 IMU0 is using GPS" "$1" 2>/dev/null && { echo "    OK - $2 GPS 준비됨"; return 0; }
        grep -q "MAVProxy exited" "$1" 2>/dev/null && { echo "!! $2: MAVProxy 종료됨. AirSim 실행 여부 확인. 로그: $1"; return 1; }
        sleep 3
    done
    echo "!! $2: 3분 안에 GPS 준비 신호 없음. 로그: $1"
    return 1
}

echo
echo "=== 4) SITL 2기를 연달아 기동 ==="
# [중요] 반드시 둘 다 띄운 다음에 기다려야 한다.
#   AirSim 은 lock-step 이라 settings.json 에 정의된 기체의 SITL 이 하나라도
#   안 붙으면 시뮬레이션 시계를 진행시키지 않는다. 따라서 drone1 의 GPS 를
#   먼저 기다린 뒤 drone2 를 띄우는 순서로 짜면 영원히 멈춘다 -
#   drone1 은 Blocks 를 기다리고 Blocks 는 drone2 를 기다리는 데드락이다.
#   (2026-09-07 실측: 4단계에서 무한 대기)
# 먼저 뜬 SITL 이 잠시 센서 없이 도는 건 정상이다.
#
# -m "--daemon" 필수: MAVProxy 는 대화형이라 stdin 이 /dev/null 이면 즉시 종료한다.

# AirSim 주소 (SIM_ADDRESS) 를 넘기는 방법이 두 버전에서 다르다.
#   4.3.0 의 sim_vehicle.py 에는 --sim-address 옵션이 없다 -> -A 로 SITL 실행 파일에 직접 넘긴다
#         (실행 파일은 4.3.0 에서도 --sim-address 를 받는다. ArduPilot Copter-4.3.0 소스로 확인)
#   4.7.1 의 sim_vehicle.py 에는 --sim-address 가 있다 (기본 127.0.0.1)
echo "    AirSim 주소: $SIM_ADDRESS"
echo "    drone1 (MAVLink, ArduCopter 4.3, -I1)"
cd "$INST1"
nohup "$AP43/Tools/autotest/sim_vehicle.py" -v ArduCopter -f airsim-copter -I1 \
      -A "--sim-address=$SIM_ADDRESS" \
      -m "--daemon" > "$LOG1" 2>&1 < /dev/null &
sleep 3

echo "    drone2 (DDS, ArduCopter 4.7.1, -I2)"
# --add-param-file 로 ARMING_SKIPCHK -1 을 넣는다.
# copter2_instance 는 새로 만든 디렉토리라 eeprom 이 비어 있고, 기본값으로 뜨면
# AirSim GPS 가 prearm 을 통과하지 못해 "Vehicle is Not Armable" 로 시동이 거부된다.
# (2026-09-07 이기종 첫 시도에서 실제로 여기서 막혔다)
cd "$INST2"
nohup "$AP47/Tools/autotest/sim_vehicle.py" -v ArduCopter -f airsim-copter -I2 -N \
      --sim-address="$SIM_ADDRESS" \
      --add-param-file="$PARM" \
      -m "--daemon" > "$LOG2" 2>&1 < /dev/null &

echo
echo "=== 5) 두 기체 GPS 준비 대기 (최대 3분) ==="
wait_gps "$LOG1" "drone1(MAVLink)" || exit 1
wait_gps "$LOG2" "drone2(DDS)" || exit 1

echo
echo "=== 6) DDS 연결 확인 ==="
# [확인 필요] SITL 은 인스턴스 번호만큼 포트를 옮기는 습관이 있다.
# -I2 에서도 AP_DDS 가 2019 로 Agent 에 붙는지가 관건이다.
if grep -q "DDS: Initialization passed" "$LOG2"; then
    echo "    OK - DDS 초기화 통과 (Agent 포트 2019 로 접속됨)"
else
    echo "!! drone2 에서 DDS 초기화 메시지가 없다."
    echo "   -I2 에서 DDS_UDP_PORT 가 2019 가 아닐 수 있다. 확인 방법:"
    echo "     grep -i dds $LOG2 | head -20"
    echo "   포트가 다르면 Agent 를 그 포트로 다시 띄우거나,"
    echo "   MAVProxy 에서 param set DDS_UDP_PORT 2019 후 재부팅할 것."
    exit 1
fi

echo
echo "=== 7) MAVLink 드론 감쇠 튜닝 + 비행 속도 ==="
# 4.3 기체는 AirSim 센서 지연 때문에 자세 루프가 발진한다. damped 프리셋으로 잡는다.
# (2026-09-03 실측: vibration_z 1.6 -> 0.05)
# 비행 속도는 drone2(dds_drone.parm, 부팅 때 적용)와 실제로 같게 넣는다 (수평 10 m/s. 4.3 은 실제 속도가
# 설정의 약 0.555 배라 설정값이 다르다 - mavlink_drone.parm 주석). 비행 뒤 확인: python3 speed_check.py
sleep 5
python3 "$TUNE" 14561 damped || echo "    [경고] 튜닝 실패 - 수동 확인 필요"
python3 "$APPLY" 14561 "$PARM1" || echo "    [경고] 비행 속도 적용 실패 - 수동 확인 필요"

echo
echo "=== 8) ROS 2 서비스 확인 ==="
source /opt/ros/humble/setup.bash
source "$HOME/ardu_ws/install/setup.bash"
sleep 2
ros2 service list | grep "ap/" | sed 's/^/    /'

echo
echo "──────────────────────────────────────────────────────────────"
echo "준비 끝. 다음 (같은 터미널에서 이어서 해도 된다):"
echo "  bash $HERE/run_live.sh             미들웨어 + LLM + AirSim 표시 + 미니맵 (LLM 키를 묻는다)"
echo "  bash $HERE/run_live.sh --no-llm    LLM 없이 (run_plan.py 로 섹터 배정)"
echo "  그다음 다른 터미널에서 run_live.sh check -> run_live.sh say \"...\" (scenarios/README.md)"
echo
echo "예전 단독 KPI 시험: kpi_mission_multi.py (DRONES = HETEROGENEOUS 로 바꾼 뒤)"
echo "로그: $LOG1"
echo "      $LOG2"
echo "      $AGENT_LOG"
echo
echo "CPU 여유가 관건이다. 로그에 'Main loop slow' 가 뜨면"
echo "SCHED_LOOP_RATE 를 낮추고 tune_damping.py 로 재조정할 것."
echo "──────────────────────────────────────────────────────────────"
