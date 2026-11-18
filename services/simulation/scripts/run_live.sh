#!/bin/bash
# 실제 운용 (시험 아님): LLM 에 자연어로 명령하면 미들웨어가 드론 2대를 날린다.
# AirSim 에 섹터 경계(선만)와 드론이 지나간 길을 그리고, 섹터 4개 전부 보이는 미니맵을 띄운다.
#
# 먼저: Windows 에서 AirSim(산악맵) 실행 -> WSL 에서 bash ~/skybuddy/start_hetero_sim.sh 가 끝날 것.
#
#   bash ~/skybuddy/run_live.sh             미들웨어 + LLM 오케스트레이터 + AirSim 표시 + 미니맵
#   bash ~/skybuddy/run_live.sh --no-llm    LLM 없이 (run_plan.py 로 섹터를 직접 배정)
#
# 다 뜨면 다른 터미널에서:
#   bash ~/skybuddy/run_live.sh check       비행 없이 점검: 오케스트레이터·미들웨어 응답 + LLM 이 도구로 임무를 읽는지
#   bash ~/skybuddy/run_live.sh say "북쪽과 남쪽 섹터를 드론 두 대로 하나씩 맡아 수색해 줘"
#       자연어 명령 한 번 -> LLM 답 -> 계획 수락·이륙·이동·착륙을 끝까지 따라가며 보여 주고 '판정: 통과/실패'
#       (mission_check.py. 따라가기만 끄려면 say --no-wait. Ctrl+C 로 따라가기를 멈춰도 비행은 계속된다)
#   (말에 임무 ID 가 없으면 'mission-101 임무: ' 를 붙여 보낸다. 섹터는 동서남북으로 말해도 된다 -
#    mission-101 설명에 sector-0-0 남, 0-1 동, 1-0 서, 1-1 북이 적혀 있다)
#
# 정상 운용 + 장애 시나리오 3개 (11/9 팀 회의). 모두 같은 임무·같은 섹터(남·북)·같은 미니맵·같은 첫 명령으로 돈다:
#   "남쪽과 북쪽 섹터를 드론 두 대로 하나씩 맡아 수색해 줘. 어느 드론이 어느 섹터를 맡을지는 드론 상태를 보고 정해 줘."
#   (mission-101 설명의 규칙: 배터리 적은 드론에 가까운 섹터, 비슷하면 drone-01 남·drone-02 북)
#   bash ~/skybuddy/run_live.sh demo normal     0. 정상 운용(장애 없음): drone-01 남·drone-02 북 끝까지 수색 -> 둘 다 홈 착륙
#                                                 (실패한 드론이 없어 다시 켜지 않고 장애 시나리오 하나를 이어서 할 수 있다)
#   bash ~/skybuddy/run_live.sh demo link       1. 통신 두절: 북으로 간 drone-02 링크가 250m 부터 나빠지고 330m 에서 끊김
#                                                 -> 기체 귀환(RTL) -> 미들웨어 LINK_LOST -> 복구 -> 귀환·착륙. drone-01 은 성공
#   bash ~/skybuddy/run_live.sh demo battery    2. 배터리 부족: drone-01 40% (drone-02 100%) -> LLM 이 drone-01 에 남(가까운 곳)
#                                                 -> 20% 에서 귀환 -> 미들웨어 timeout 처럼 처리(WAYPOINT_TIMEOUT -> 귀환·착륙)
#                                                 -> "못 끝낸 섹터 다시" -> drone-02 가 남 수색
#   bash ~/skybuddy/run_live.sh demo timeout    3. 웨이포인트 미도달: drone-02 가 북에서 웨이포인트로 못 감 -> 60초 뒤
#                                                 WAYPOINT_TIMEOUT -> 귀환·착륙 -> "못 끝낸 섹터 다시" -> drone-01 이 북 수색
#   (LLM 없이 같은 배정으로: 뒤에 --no-llm. 예: bash ~/skybuddy/run_live.sh demo link --no-llm)
#   장애 드론은 unavailable 로 남는다. 다음 시나리오 전에 run_live.sh 를 끄고 AirSim -> start_hetero_sim.sh ->
#   run_live.sh 를 다시 켠다 (demo 가 먼저 확인하고 알려 준다). 검증 현황: python3 ~/skybuddy/mission_check.py --report
#
# LLM 키(GOOGLE_API_KEY)는 환경 변수에 없으면 실행할 때 묻는다. 입력한 글자는 화면에 안 보이고 파일에도 안 남는다.
# 시작할 때 장애 주입을 지운다 (시험에서 남은 장애가 실제 비행에 끼어들지 않게).
# 비행이 끝나고 착륙한 뒤 Ctrl+C 로 전부 끈다. 기록은 ~/skybuddy/logs/live_<시각>/ 에 남는다.

API="http://127.0.0.1:8001"
MCP_URL="http://127.0.0.1:9000/mcp"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

newest_log() {       # 가장 최근 run_live 기록 폴더
    ls -td "$HOME"/skybuddy/logs/live_* 2>/dev/null | head -n 1
}

ask_llm() {          # $1 = 보낼 말. LLM 답(result)을 출력한다. 오케스트레이터 오류면 이유와 기록 끝부분을 보여 주고 1
    local out code http body log
    out="$(curl -s --max-time 600 -w '\n%{http_code}' -X POST -G "$API/command" --data-urlencode "message=$1")"
    code=$?
    if [ $code -ne 0 ]; then
        echo "!! 오케스트레이터에 보내지 못했다 (curl 종료 코드 $code). run_live.sh 를 LLM 과 함께(--no-llm 없이) 띄웠는지 볼 것"
        return 1
    fi
    http="${out##*$'\n'}"
    body="${out%$'\n'*}"
    if [ "$http" != "200" ]; then
        echo "!! 오케스트레이터 오류 (HTTP $http): ${body:0:300}"
        log="$(newest_log)/orchestrator.log"
        if [ -f "$log" ]; then
            echo "   오케스트레이터 기록 끝부분 ($log):"
            tail -n 15 "$log" | sed 's/^/   | /'
        fi
        return 1
    fi
    echo "$body" | python3 -c 'import json, sys; d = json.load(sys.stdin); print(d.get("result", d) if isinstance(d, dict) else d)' \
        2>/dev/null || echo "$body"
}

with_mission() {     # 임무 ID 가 없으면 mission-101 을 붙인다
    case "$1" in
        *mission-*) echo "$1" ;;
        *) echo "mission-101 임무: $1" ;;
    esac
}

# ------------------------------------------------------------------ 자연어 명령 보내기 + 끝까지 판정
if [ "${1:-}" = "say" ]; then
    shift
    WAIT=1
    if [ "${1:-}" = "--no-wait" ]; then
        WAIT=0
        shift
    fi
    if [ -z "$*" ]; then
        echo "보낼 말을 줄 것. 예: bash $0 say \"북쪽과 남쪽 섹터를 드론 두 대로 하나씩 맡아 수색해 줘\""
        exit 2
    fi
    MSG="$(with_mission "$*")"
    echo "LLM 에 보냄: $MSG"
    echo "(LLM 이 도구를 여러 번 부르면 답이 오기까지 수십 초 걸린다)"
    SINCE="$(date +%s.%N)"
    echo "--- LLM 답 ---"
    ask_llm "$MSG" || exit 1
    if [ "$WAIT" = 1 ]; then
        echo
        exec python3 "$HERE/mission_check.py" --follow --since "$SINCE"
    fi
    echo "(진행 판정: python3 $HERE/mission_check.py)"
    exit 0
fi

# ------------------------------------------------------------------ 정상 운용·장애 시나리오를 자연어 명령으로 (판정까지)
if [ "${1:-}" = "demo" ]; then
    NAME="${2:-}"
    NOLLM=0
    [ "${3:-}" = "--no-llm" ] && NOLLM=1
    case "$NAME" in
        normal|link|battery|timeout) ;;
        *)
            echo "시나리오를 줄 것: bash $0 demo <normal | link | battery | timeout> [--no-llm]"
            echo "  normal   0. 정상 운용: 장애 없이 drone-01 남·drone-02 북 끝까지 수색 → 둘 다 홈 착륙"
            echo "  link     1. 통신 두절: 북으로 간 드론 링크 250m 악화 → 330m 두절 → 기체 귀환 → 복구 → 귀환·착륙"
            echo "  battery  2. 배터리 부족: drone-01 40% → 가까운 남 배정 → 20% 에서 귀환(timeout 처럼) → 남을 drone-02 에 재배정"
            echo "  timeout  3. 웨이포인트 미도달: drone-02 가 북에서 못 감 → 60초 뒤 귀환·착륙 → 북을 drone-01 에 재배정"
            exit 2 ;;
    esac
    if [ "$NOLLM" = 0 ] && ! curl -s --max-time 5 "$API/health" | grep -q '"ok"'; then
        echo "!! LLM 오케스트레이터가 응답하지 않는다. 다른 터미널에서 bash $0 (LLM 포함) 를 띄우거나,"
        echo "   LLM 없이 같은 배정으로: bash $0 demo $NAME --no-llm  (그때는 bash $0 --no-llm 로 띄워도 된다)"
        exit 1
    fi
    LOG="$(newest_log)"
    if [ -z "$LOG" ]; then
        echo "!! run_live.sh 기록이 없다. 먼저 다른 터미널에서 bash $0 를 띄울 것"
        exit 1
    fi
    EVENTS="$LOG/mission-events.jsonl"
    # 앞 시나리오에서 실패한 드론은 unavailable 로 남는다 -> 다시 켜야 한다
    python3 "$HERE/mission_check.py" "$EVENTS" --preflight || exit 1
    if [ -f "$HERE/kpi_mission.py" ]; then
        DRONES_CONFIG="$HERE/scenarios/drones.sitl.json"
    else
        DRONES_CONFIG="$(dirname "$HERE")/scenarios/drones.sitl.json"
    fi
    fault() { python3 "$HERE/fault.py" "$@" --config "$DRONES_CONFIG" || { echo "!! 장애를 걸지 못했다"; exit 1; }; }
    clear_faults() { python3 "$HERE/fault.py" clear > /dev/null; }
    trap 'clear_faults' EXIT        # 끝나거나 Ctrl+C 로 멈춰도 장애 주입을 지운다 (비행은 계속된다)
    send_order() {                  # $1 = 자연어 명령, $2 = LLM 없이 낼 같은 배정 (run_plan.py 인자)
        if [ "$NOLLM" = 1 ]; then
            echo "LLM 없이 배정: $2"
            # shellcheck disable=SC2086 - 배정은 낱말마다 인자로 넘긴다
            python3 "$HERE/run_plan.py" mission-101 $2 --no-wait
        else
            echo "LLM 에 보냄: $1"
            echo "(LLM 이 도구를 여러 번 부르면 답이 오기까지 수십 초 걸린다)"
            echo "--- LLM 답 ---"
            ask_llm "$1"
        fi
    }
    # 네 경우 모두 같은 첫 명령. LLM 은 mission-101 설명의 규칙으로 정한다:
    # 배터리 적은 드론에 가까운 섹터, 비슷하면 drone-01 남·drone-02 북 (LLM 없을 때도 같은 배정)
    ORDER1="mission-101 임무: 남쪽과 북쪽 섹터를 드론 두 대로 하나씩 맡아 수색해 줘. 어느 드론이 어느 섹터를 맡을지는 드론 상태를 보고 정해 줘."
    PLAN1="drone-01=sector-0-0 drone-02=sector-1-1"
    ORDER2="mission-101 임무: 장애로 못 끝낸 섹터를 지금 쓸 수 있는 드론에 다시 배정해서 수색해 줘."

    SINCE="$(date +%s.%N)"
    clear_faults
    case "$NAME" in
        normal)
            echo "=== 0. 정상 운용 - 장애 없음 (기록 $LOG) ===" ;;
        link)
            echo "=== 1. 통신 두절 (기록 $LOG) ==="
            # 지상국에서 멀수록 통신이 나빠지는 환경: 두 드론 모두 같은 규칙. 북(먼 섹터)으로 간 드론만 330m 를 넘는다
            fault link drone-01 range --degraded-m 250 --down-m 330
            fault link drone-02 range --degraded-m 250 --down-m 330 ;;
        battery)
            echo "=== 2. 배터리 부족 (기록 $LOG) ==="
            # drone-01 만 부족: 40% 에서 시작해 비행 중 줄고 20% 에서 기체 배터리 failsafe 귀환
            fault battery drone-01 0 --from 40 --drain 140 --rtl-at 20 ;;
        timeout)
            echo "=== 3. 웨이포인트 미도달 (기록 $LOG) ==="
            # drone-02 가 이륙하고 20초 뒤 다음 웨이포인트 하나를 못 받는다 (북 섹터 입구에서 멈춤)
            fault stall drone-02 --after-takeoff 20 ;;
    esac
    send_order "$ORDER1" "$PLAN1" || exit 1
    echo
    if [ "$NAME" = normal ] || [ "$NAME" = link ]; then
        python3 "$HERE/mission_check.py" "$EVENTS" --follow --since "$SINCE" --scenario "$NAME"
        RC=$?
        echo "검증 현황: python3 $HERE/mission_check.py --report"
        exit $RC
    fi
    # 배터리 부족·미도달: 1단계(장애 → 귀환·착륙, 다른 드론 성공)가 기대대로면 재배정 명령을 보낸다
    if ! python3 "$HERE/mission_check.py" "$EVENTS" --follow --since "$SINCE" --scenario "$NAME-1"; then
        echo "1단계가 기대와 달라 재배정 명령을 보내지 않는다 (위 [실패] 줄 참고)"
        exit 1
    fi
    clear_faults
    if [ "$NAME" = battery ]; then PLAN2="drone-02=sector-0-0"; else PLAN2="drone-01=sector-1-1"; fi
    echo
    echo "=== 2단계: 못 끝낸 섹터 재배정 ==="
    SINCE2="$(date +%s.%N)"
    send_order "$ORDER2" "$PLAN2" || exit 1
    echo
    python3 "$HERE/mission_check.py" "$EVENTS" --follow --since "$SINCE2" --judge-since "$SINCE" --scenario "$NAME"
    RC=$?
    echo "검증 현황: python3 $HERE/mission_check.py --report"
    exit $RC
fi

# ------------------------------------------------------------------ 비행 없이 점검
if [ "${1:-}" = "check" ]; then
    echo "=== 띄운 것 점검 (비행 명령 없음) ==="
    FAIL=0
    if curl -s --max-time 5 "$API/health" | grep -q '"ok"'; then
        echo "  [OK]   오케스트레이터 ($API/health)"
    else
        echo "  [실패] 오케스트레이터가 응답하지 않는다 ($API). run_live.sh 를 --no-llm 없이 띄울 것"
        FAIL=1
    fi
    MCP_CODE="$(curl -s -o /dev/null -w '%{http_code}' --max-time 5 "$MCP_URL")"
    if [ "$MCP_CODE" != "000" ]; then
        echo "  [OK]   미들웨어 MCP 서버 ($MCP_URL 응답 HTTP $MCP_CODE. 406 이면 정상)"
    else
        echo "  [실패] 미들웨어 MCP 서버가 응답하지 않는다 ($MCP_URL)"
        FAIL=1
    fi
    [ "$FAIL" = 0 ] || exit 1
    SERVER_LOG="$(newest_log)/server.log"
    count_mcp() {    # 미들웨어 서버가 받은 MCP 요청 수 (LLM 이 도구 목록·호출을 할 때마다 는다)
        local n
        n="$(grep -c 'POST /mcp' "$SERVER_LOG" 2>/dev/null)"
        echo "${n:-0}"
    }
    BEFORE="$(count_mcp)"
    Q="$(with_mission "드론 상태와 탐색 구역(동서남북 이름)만 짧게 알려 줘. 계획은 제출하지 마.")"
    echo "  LLM 에 질문 (읽기만): $Q"
    SINCE="$(date +%s.%N)"
    ANSWER="$(ask_llm "$Q")" || { echo "$ANSWER"; exit 1; }
    echo "$ANSWER" | sed 's/^/    | /'
    AFTER="$(count_mcp)"
    PLANS="$(python3 - "$SINCE" "$(newest_log)/mission-events.jsonl" <<'PY'
import json, sys
from datetime import datetime
since, path, n = float(sys.argv[1]), sys.argv[2], 0
try:
    for line in open(path, encoding="utf-8"):
        try:
            e = json.loads(line)
            t = datetime.fromisoformat(str(e.get("logged_at", "")).replace("Z", "+00:00")).timestamp()
        except ValueError:
            continue
        n += e.get("event") == "plan_submitted" and t >= since
except OSError:
    pass
print(n)
PY
)"
    if echo "$ANSWER" | grep -q "drone-01" && echo "$ANSWER" | grep -q "drone-02"; then
        echo "  [OK]   LLM 이 미들웨어 도구로 두 드론 상태를 읽어 답했다 (MCP 요청 $((AFTER - BEFORE))개)"
    else
        echo "  [주의] 답에 drone-01·drone-02 가 없다. 위 답을 볼 것 (MCP 요청 $((AFTER - BEFORE))개)"
        FAIL=1
    fi
    if [ "$PLANS" != "0" ]; then
        echo "  [주의] LLM 이 계획을 제출했다 ($PLANS 번). 드론이 이륙했을 수 있다: python3 $HERE/mission_check.py"
        FAIL=1
    fi
    if [ "$FAIL" = 0 ]; then
        echo "점검 통과. 이제 자연어 명령: bash $0 say \"북쪽과 남쪽 섹터를 드론 두 대로 하나씩 맡아 수색해 줘\""
    fi
    exit "$FAIL"
fi

# ------------------------------------------------------------------ 띄우기
# ROS setup.bash 는 정의 안 된 변수를 참조하므로 set -u 보다 먼저 불러온다 (DDS 드론 어댑터에 필요).
source /opt/ros/humble/setup.bash
source "$HOME/ardu_ws/install/setup.bash"
set -u
set -m          # 백그라운드 프로세스가 SIGINT 를 받도록 (그래야 정상 종료 기록을 남긴다)

LLM=1
case "${1:-}" in
    "") ;;
    --no-llm) LLM=0 ;;
    *) echo "모르는 옵션: $1 (--no-llm, check, say \"말\", demo <normal|link|battery|timeout>)"; exit 2 ;;
esac

# 개발 환경(평면 구조, ~/skybuddy)과 팀 저장소(services/simulation/scripts) 어느 쪽에서도 돈다.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$HERE/kpi_mission.py" ]; then
    BASE="$HERE"                                       # 개발 환경
    TEAM="${SKYBUDDY_TEAM:-$HOME/skybuddy-team}"       # 미들웨어·오케스트레이터가 들어 있는 팀 저장소
else
    BASE="$(dirname "$HERE")"                          # services/simulation
    TEAM="$(cd "$BASE/../.." && pwd)"                   # 팀 저장소 최상위
fi
SCRIPTS="$HERE"
ORCH="$TEAM/services/orchestrator"
M101="$BASE/scenarios/missions/mission-101.json"
OUT="$HOME/skybuddy/logs/live_$(date +%Y%m%d_%H%M%S)"

if [ "$LLM" = 1 ]; then
    if [ ! -x "$ORCH/venv/bin/python" ]; then
        echo "!! LLM 오케스트레이터 가상환경이 없다: $ORCH/venv"
        echo "   처음 한 번만 설치 (팀 문서 docs/development/local-full-pipeline.md):"
        echo "     cd $TEAM"
        echo "     sudo apt install -y python3.10-venv"
        echo "     python3 -m venv services/orchestrator/venv"
        echo "     services/orchestrator/venv/bin/pip install -r services/orchestrator/requirements.txt"
        echo "   LLM 없이 띄우려면: bash $0 --no-llm"
        exit 1
    fi
    # 오케스트레이터가 HTTP 로 미들웨어에 붙으려면 mcp 2.x 가 필요하다 (1.x 면 접속할 때 'too many values to unpack')
    VENV_PROBLEM="$("$ORCH/venv/bin/python" - <<'PY' 2>&1
import importlib.metadata as m
problems = []
try:
    version = m.version("mcp")
    if int(version.split(".")[0]) < 2:
        problems.append(f"mcp {version} (2.x 필요)")
except m.PackageNotFoundError:
    problems.append("mcp 없음")
try:
    import google.genai  # noqa: F401
except Exception as e:  # noqa: BLE001
    problems.append(f"google-genai 불러오기 실패 ({e})")
print("; ".join(problems))
PY
)"
    if [ -n "$VENV_PROBLEM" ]; then
        echo "!! 오케스트레이터 가상환경 문제: $VENV_PROBLEM"
        echo "   고치기: $ORCH/venv/bin/pip install -r $ORCH/requirements.txt \"mcp>=2\""
        exit 1
    fi
    # 키를 받자마자 확인한다: 오케스트레이터가 쓰는 모델 정보를 한 번 읽어 본다 (생성 호출이 아니라 한도를 안 쓴다).
    # 틀리면 나중에 /command 가 500 (API key not valid) 으로 끝나므로 여기서 다시 묻는다. 키는 환경 변수로만 넘긴다.
    MODEL="$(sed -n 's/^_MODEL *= *"\(models\/[^"]*\)".*/\1/p' "$ORCH/app/providers/gemini.py" 2>/dev/null | head -n 1)"
    MODEL="${MODEL:-models/gemini-3.6-flash}"
    check_key() {
        GOOGLE_API_KEY="$1" GEMINI_MODEL="$MODEL" python3 - <<'PY'
import json, os, urllib.error, urllib.request
key, model = os.environ.get("GOOGLE_API_KEY", "").strip(), os.environ["GEMINI_MODEL"]
if not key or any(c.isspace() for c in key):
    print("빈칸이 있거나 비어 있다 (키 한 줄만 붙여 넣을 것)")
    raise SystemExit
request = urllib.request.Request(f"https://generativelanguage.googleapis.com/v1beta/{model}",
                                 headers={"x-goog-api-key": key})
try:
    urllib.request.urlopen(request, timeout=15).read()
    print("ok")
except urllib.error.HTTPError as e:
    try:
        message = json.loads(e.read().decode("utf-8")).get("error", {}).get("message", "")
    except Exception:  # noqa: BLE001
        message = ""
    print(f"HTTP {e.code} {message}".strip())
except Exception as e:  # noqa: BLE001
    print(f"network {e}")
PY
    }
    for TRY in 1 2 3; do
        if [ -z "${GOOGLE_API_KEY:-}" ]; then
            read -rsp "GOOGLE_API_KEY 입력 (키 한 줄만 붙여 넣고 Enter. 화면에 안 보인다): " GOOGLE_API_KEY
            echo
        fi
        KEY_STATE="$(check_key "${GOOGLE_API_KEY:-}")"
        case "$KEY_STATE" in
            ok)
                echo "    LLM 키 확인됨 (${MODEL#models/} 사용 가능)"
                break ;;
            "HTTP 404"*)
                echo "!! 키는 맞지만 모델 ${MODEL#models/} 을 쓸 수 없다: $KEY_STATE"
                echo "   팀 오케스트레이터 services/orchestrator/app/providers/gemini.py 의 _MODEL 을 확인할 것"
                exit 1 ;;
            "HTTP 429"*)
                echo "    [주의] 키는 맞지만 호출 한도를 넘었다 ($KEY_STATE). 잠시 뒤 다시 하거나 한도를 확인할 것"
                break ;;
            network*)
                echo "    [주의] 인터넷 문제로 키를 확인하지 못했다 (${KEY_STATE#network }). 그대로 띄운다"
                break ;;
            *)
                echo "!! 키가 맞지 않다 ($TRY/3): $KEY_STATE"
                GOOGLE_API_KEY="" ;;
        esac
    done
    if [ -z "${GOOGLE_API_KEY:-}" ]; then
        echo "!! 맞는 LLM 키를 받지 못했다. Google AI Studio 에서 키를 다시 복사하거나, LLM 없이: bash $0 --no-llm"
        exit 1
    fi
fi

mkdir -p "$OUT"
# 이번 비행의 속도 파라미터를 기록에 같이 남긴다 (speed_check.py 가 비교할 때 이 값을 쓴다)
for CONF_DIR in "$BASE/config_reference" "$BASE/config"; do
    if [ -f "$CONF_DIR/mavlink_drone.parm" ]; then
        cp "$CONF_DIR/mavlink_drone.parm" "$CONF_DIR/dds_drone.parm" "$OUT/" 2>/dev/null
        break
    fi
done
export MCP_TRANSPORT=http
export MIDDLEWARE_DRONES_CONFIG="$BASE/scenarios/drones.sitl.json"
export MIDDLEWARE_SCENARIOS_DIR="$BASE/scenarios/missions"
export MIDDLEWARE_EVENT_LOG="$OUT/mission-events.jsonl"
export SKYBUDDY_FAULT_LOG="$OUT/fault-events.jsonl"
export SKYBUDDY_TELEMETRY_LOG="$OUT/telemetry.jsonl"
cp "$MIDDLEWARE_DRONES_CONFIG" "$OUT/drones.sitl.json" 2>/dev/null    # mission_check.py --report 가 설정을 비교한다

SERVER=""
HELPERS=()

stop_pid() {         # $1 = pid, $2 = 기다릴 초
    [ -n "$1" ] || return 0
    kill -INT "$1" 2>/dev/null
    for _ in $(seq $(($2 * 2))); do
        kill -0 "$1" 2>/dev/null || return 0
        sleep 0.5
    done
    kill -9 "$1" 2>/dev/null
}

stop_all() {
    trap - EXIT INT TERM
    echo
    echo "=== 끄는 중 ==="
    for pid in "${HELPERS[@]}"; do
        stop_pid "$pid" 5
    done
    stop_pid "$SERVER" 30
    echo "다 껐다. 기록: $OUT"
}
trap stop_all EXIT
trap 'exit 130' INT TERM

echo "=== 1) 장애 주입 지우기 (시험에서 남은 장애가 실제 비행에 끼어들지 않게) ==="
python3 "$SCRIPTS/fault.py" clear

echo
echo "=== 2) AirSim 화면에 섹터 경계 그리기 (선만) ==="
python3 "$SCRIPTS/map_probe.py" draw "$M101" --drones-config "$MIDDLEWARE_DRONES_CONFIG" \
    || echo "    (그리기 실패 - 계속)"

echo
echo "=== 3) 미들웨어 서버 ($MCP_URL) ==="
(cd "$TEAM" && PYTHONPATH="services/middleware:${PYTHONPATH:-}" exec python3 -m app.mcp_server) \
    > "$OUT/server.log" 2>&1 &
SERVER=$!
for _ in $(seq 90); do
    grep -q "listening on" "$OUT/server.log" && break
    kill -0 "$SERVER" 2>/dev/null || break
    sleep 1
done
if ! grep -q "listening on" "$OUT/server.log"; then
    echo "!! 미들웨어 서버가 뜨지 않았다 (이미 켜 둔 서버가 9000 을 쓰고 있으면 먼저 끌 것). 출력 마지막 부분:"
    tail -n 20 "$OUT/server.log" | sed 's/^/    | /'
    exit 1
fi
echo "    준비됨"
grep -E "연결됨|장애 주입|connect failed" "$OUT/server.log" | sed 's/^/    | /'

echo
echo "=== 4) AirSim 에 드론 지나간 길 + 미니맵 (섹터 4개 전부) ==="
python3 "$SCRIPTS/track_drones.py" --stream "$SKYBUDDY_TELEMETRY_LOG" > "$OUT/tracker.log" 2>&1 &
HELPERS+=($!)
python3 "$SCRIPTS/minimap.py" --mission "$M101" --stream "$SKYBUDDY_TELEMETRY_LOG" > "$OUT/minimap.log" 2>&1 &
MINIMAP=$!
for _ in $(seq 40); do
    grep -qE "^(브라우저 창|열어 둔 미니맵)" "$OUT/minimap.log" 2>/dev/null && break
    kill -0 "$MINIMAP" 2>/dev/null || break
    sleep 0.25
done
if kill -0 "$MINIMAP" 2>/dev/null; then
    HELPERS+=($MINIMAP)
    echo "    미니맵: http://localhost:8765"
    sed -n '/^\(  다른 컴퓨터에서\|브라우저 창\|열어 둔 미니맵\)/,$p' "$OUT/minimap.log" | sed 's/^/    | /'
else
    echo "    (미니맵을 새로 켜지 못함. 따로 켜 둔 미니맵이 있으면 그것이 이 기록을 따라간다. $OUT/minimap.log)"
fi

if [ "$LLM" = 1 ]; then
    echo
    echo "=== 5) LLM 오케스트레이터 ($API) ==="
    # 키는 명령줄(ps 에 보임)이 아니라 환경 변수로만 넘긴다
    (cd "$ORCH" && export GOOGLE_API_KEY MCP_SERVER_URL="$MCP_URL" && unset MCP_TRANSPORT \
        && exec venv/bin/python -m uvicorn app.main:app --host 127.0.0.1 --port 8001) > "$OUT/orchestrator.log" 2>&1 &
    ORCH_PID=$!
    unset GOOGLE_API_KEY
    for _ in $(seq 60); do
        grep -q "Application startup complete" "$OUT/orchestrator.log" && break
        kill -0 "$ORCH_PID" 2>/dev/null || break
        sleep 1
    done
    if grep -q "Application startup complete" "$OUT/orchestrator.log"; then
        HELPERS=($ORCH_PID "${HELPERS[@]}")
        echo "    준비됨"
    else
        echo "!! 오케스트레이터가 뜨지 않았다 (8001 을 이미 쓰고 있으면 먼저 끌 것). 출력 마지막 부분:"
        tail -n 20 "$OUT/orchestrator.log" | sed 's/^/    | /'
        kill -9 "$ORCH_PID" 2>/dev/null
        echo "    미들웨어·미니맵은 그대로 둔다 (LLM 없이 run_plan.py 로 배정할 수 있다)"
    fi
fi

echo
echo "=== 준비 끝 ==="
if [ "$LLM" = 1 ]; then
    echo "다른 터미널에서 (장애 시나리오가 끝나면 이 창 Ctrl+C → AirSim·start_hetero_sim.sh·run_live.sh 다시):"
    echo "  bash $SCRIPTS/run_live.sh demo normal     0. 정상 운용 (장애 없음. 끝나고 다시 켜지 않고 장애 하나를 이어서 해도 된다)"
    echo "  bash $SCRIPTS/run_live.sh demo link       1. 통신 두절"
    echo "  bash $SCRIPTS/run_live.sh demo battery    2. 배터리 부족"
    echo "  bash $SCRIPTS/run_live.sh demo timeout    3. 웨이포인트 미도달"
    echo "  (그냥 명령: bash $SCRIPTS/run_live.sh say \"...\",  점검: bash $SCRIPTS/run_live.sh check)"
else
    echo "다른 터미널에서 LLM 없이 같은 배정으로: bash $SCRIPTS/run_live.sh demo <normal|link|battery|timeout> --no-llm"
fi
echo "검증 현황: python3 $SCRIPTS/mission_check.py --report"
echo "섹터 이름: sector-0-0 남, sector-0-1 동, sector-1-0 서, sector-1-1 북 (미니맵 글씨)"
echo "착륙한 뒤 Ctrl+C 로 전부 끈다. 기록: $OUT"
echo "--- 미들웨어 이벤트 ---"
tail -n 0 -F "$MIDDLEWARE_EVENT_LOG" 2>/dev/null | python3 -u -c '
import json, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
for line in sys.stdin:
    try:
        e = json.loads(line)
        keys = [k for k in ("drone_id", "area_id", "task_id", "status", "state", "reason") if e.get(k) is not None]
        t = str(e.get("logged_at") or e.get("timestamp") or "")[11:19]
        print(t, e.get("event"), " ".join(f"{k}={e[k]}" for k in keys))
    except Exception:
        print(line.rstrip()[:200])
'
