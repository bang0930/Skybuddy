# 로컬 전체 파이프라인 실행

LLM 오케스트레이터 → 미들웨어 MCP 서버 → 실제 SITL 2대 전체를 한 랩탑에서 실행하는 절차입니다.
기준 환경은 시뮬레이션 담당 랩탑입니다.

| 구성 요소 | 실행 위치 | Python |
|---|---|---|
| AirSim (Blocks) | Windows | - |
| ArduPilot SITL 2대, Micro XRCE-DDS Agent, ROS 2 Humble | WSL2 Ubuntu 22.04 | - |
| 미들웨어 MCP 서버 (HTTP 상주) | WSL2 | **시스템 Python 3.10.12** (`rclpy`가 3.10에서만 동작) |
| 오케스트레이터 (FastAPI + LLM) | Windows 또는 WSL2 | **별도 가상환경** (`mcp` 2.x) |

```text
[Windows] 오케스트레이터 ──HTTP /mcp──▶ [WSL] 미들웨어 MCP 서버 ─┬─ MavlinkDrone ─ UDP 14561 ─ SITL 1 (ArduCopter 4.3)
   POST /command                                                └─ Ros2Drone ─── DDS /ap ──── SITL 2 (ArduCopter 4.7.1)
                                                                                              │
[Windows] AirSim ◀────────────────────────────────────────────────────────────────────────────┘
```

미들웨어를 HTTP 상주 서버로 띄우는 이유:

- stdio로 연결하면 오케스트레이터가 `/command` 요청마다 서버를 띄웠다가 닫습니다. 응답이 끝나면
  Dispatcher 스레드가 비행 도중에 함께 끝나고(K5), 요청마다 임무 상태가 초기화되며, 요청이 겹치면
  `udpin:127.0.0.1:14561` 포트가 충돌합니다.
- MCP stdio 클라이언트는 `HOME`, `PATH` 등 일부 환경 변수만 서버에 넘기므로 ROS 2 환경과
  `MIDDLEWARE_DRONES_CONFIG`가 빠집니다.
- HTTP 서버는 직접 실행한 터미널의 환경을 그대로 쓰고, 클라이언트 연결이 끊겨도 비행을 끝까지
  진행합니다. 다음 세션에서 같은 임무 상태를 조회할 수 있습니다.

미들웨어와 오케스트레이터는 서로 다른 Python 환경에서 실행합니다. 미들웨어는 `mcp` 1.x
(`fastmcp` 3.x)이고, 오케스트레이터의 HTTP 접속 코드(#38)는 `mcp` 2.x에서만 동작합니다.

## 준비 (한 번만)

### 미들웨어 의존성 (WSL, 시스템 Python)

저장소 루트에서 실행합니다. 가상환경을 쓰면 ROS 2 `rclpy`를 불러올 수 없으므로 시스템 `python3`에
설치합니다.

```bash
python3 --version   # Python 3.10.12
python3 -m pip install --user -r services/middleware/requirements.txt -r services/simulation/requirements.txt
python3 -c "import fastmcp, mcp, shapely, pymavlink; print('ok')"
```

`shapely` 최신 2.2는 Python 3.11 이상이 필요하므로 pip가 2.1.x를 설치해야 정상입니다.

### 오케스트레이터 가상환경

Windows PowerShell (저장소 루트):

```powershell
python -m venv services\orchestrator\venv
services\orchestrator\venv\Scripts\pip install -r services\orchestrator\requirements.txt
services\orchestrator\venv\Scripts\python -c "import importlib.metadata as m; print(m.version('mcp'))"
```

WSL에서 실행할 때는 `python3 -m venv services/orchestrator/venv`(필요하면 `sudo apt install
python3.10-venv`) 후 `services/orchestrator/venv/bin/pip install -r services/orchestrator/requirements.txt`.

마지막 줄이 `2.`로 시작해야 합니다. 1.x가 설치되어 있으면 접속할 때 `too many values to unpack`
오류가 나므로 `pip install "mcp>=2"`로 올립니다.

LLM 키는 터미널에서만 지정합니다. 저장소 파일에 적지 않습니다.

## 0. FakeDrone으로 리허설

SITL 없이 같은 순서를 먼저 확인합니다. macOS나 WSL 어디서든 됩니다. 드론은
`config/drones.fake.json`의 FakeDrone 2대입니다.

```bash
# 터미널 A: 미들웨어 HTTP 서버 (MIDDLEWARE_DRONES_CONFIG를 지정하지 않으면 FakeDrone)
MCP_TRANSPORT=http PYTHONPATH=services/middleware python3 -m app.mcp_server
# → [mcp_server] listening on http://127.0.0.1:9000/mcp

# 터미널 B: LLM 없이 전 구간 확인 (약 30초)
python3 services/middleware/scripts/check_mvp_pipeline.py --url http://127.0.0.1:9000/mcp
# → [6] this run's tasks succeeded ...
```

그다음 아래 5~6단계(오케스트레이터와 `/command`)를 그대로 해 봅니다. 끝나면 터미널 A에서 Ctrl+C로
서버를 내립니다.

## 1. AirSim 실행 (Windows)

2기체 구성 `settings.json`으로 Blocks를 실행합니다. 구성 배포 방법은
`services/simulation/scripts/start_hetero_sim.sh` 첫 주석을 따릅니다.

## 2. SITL 2대와 DDS Agent 기동 (WSL 터미널 1)

```bash
bash ~/skybuddy/start_hetero_sim.sh
```

저장소 사본은 `services/simulation/scripts/start_hetero_sim.sh`이지만, 스크립트 안의 경로(튜닝
스크립트, 로그, AirSim 설정)가 `~/skybuddy`와 시뮬레이션 담당 계정 기준으로 적혀 있으므로 평소
실행하던 위치의 스크립트를 씁니다. GPS 준비, DDS 초기화, ROS 2 서비스 확인까지 끝날 때까지
기다립니다. 실패 메시지가 나오면 `services/simulation/INTERFACE.md`와
`services/simulation/docs/engineering-notes.md`를 따릅니다.

## 3. 미들웨어 HTTP 서버 (WSL 터미널 2, 새 터미널)

```bash
cd <저장소 루트>
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash

export MCP_TRANSPORT=http
export MIDDLEWARE_DRONES_CONFIG=services/middleware/config/drones.real.example.json
PYTHONPATH=services/middleware python3 -m app.mcp_server
```

- `python3`가 시스템 Python(`/usr/bin/python3`, 3.10.12)인지 확인합니다. 가상환경이 활성화되어
  있으면 `deactivate` 후 실행합니다.
- 서버는 시작할 때 두 드론에 연결합니다. 어댑터 출력(`[연결됨] system=1 component=1`,
  `[연결됨] ROS 2 namespace=/ap ...`) 뒤에 `[mcp_server] listening on http://127.0.0.1:9000/mcp`가
  나오면 준비된 것입니다.
- 연결에 실패한 드론이 있어도 서버는 뜹니다. 그 드론은 `disconnected`로 보고되고 계획 검증에서
  `DRONE_DISCONNECTED`로 거부되므로, SITL 상태를 확인하고 서버를 다시 시작합니다.
- 이벤트는 `services/middleware/logs/mission-events.jsonl`에 쌓입니다(`MIDDLEWARE_EVENT_LOG`로 변경).
  `server_started`에 `transport: "http"`, `host`, `port`와 드론별 연결 결과가 남습니다.
- 이 터미널은 서버를 내릴 때까지 그대로 둡니다.

## 4. LLM 없이 확인 (WSL 터미널 3)

```bash
python3 services/middleware/scripts/check_mvp_pipeline.py --url http://127.0.0.1:9000/mcp
```

**이 단계에서 실제로 두 드론이 이륙합니다.** mission-001을 수행하고 착륙까지 약 2분 걸립니다.
`[6] this run's tasks succeeded`와 종료 코드 0이면 미들웨어 → SITL 구간은 정상입니다.

- 스크립트는 이번에 수락된 작업만 보고 판정합니다. 같은 서버에서 앞서 실패한 작업이 있으면
  `mission state`는 `failed`로 보이지만 이번 판정에는 영향이 없습니다.
- 실패해서 드론이 `unavailable`이 되면 서버를 다시 시작해야 다시 배정할 수 있습니다.
- `collect_integration_evidence.py`는 stdio로 서버를 따로 띄우므로, 실제 모드에서는 HTTP 서버를
  내린 뒤에 실행합니다(같은 UDP 포트를 동시에 쓸 수 없음).

## 5. 오케스트레이터 실행

Windows PowerShell:

```powershell
cd services\orchestrator
$env:MCP_SERVER_URL = "http://127.0.0.1:9000/mcp"
$env:GOOGLE_API_KEY = "<본인 키>"
venv\Scripts\python -m uvicorn app.main:app --host 127.0.0.1 --port 8001
```

WSL에서 실행할 때:

```bash
cd services/orchestrator
export MCP_SERVER_URL=http://127.0.0.1:9000/mcp
export GOOGLE_API_KEY=<본인 키>
venv/bin/python -m uvicorn app.main:app --host 127.0.0.1 --port 8001
```

`MCP_SERVER_URL`이 없으면 오케스트레이터는 미들웨어가 아니라 mock 서버를 stdio로 실행합니다.
mock 서버도 `MCP_TRANSPORT=http`로 같은 주소에 띄울 수 있으므로, 주소 하나로 둘을 바꿔 끼울 수
있습니다(동시에 같은 포트로 띄울 수는 없음).

## 6. 자연어 명령

```powershell
$msg = "mission-001 임무: 북쪽과 남쪽 구역을 드론 두 대로 나눠 수색해 줘"
Invoke-RestMethod -Method Post -Uri ("http://127.0.0.1:8001/command?message=" + [uri]::EscapeDataString($msg))
```

```bash
curl -s -X POST -G http://127.0.0.1:8001/command \
  --data-urlencode "message=mission-001 임무: 북쪽과 남쪽 구역을 드론 두 대로 나눠 수색해 줘"
```

브라우저에서 `http://127.0.0.1:8001/docs`로 호출해도 됩니다.

- 명령에 임무 ID를 적지 않아도 됩니다. 오케스트레이터가 `list_missions`로 임무를 확인하며,
  임무가 여러 개이고 어느 것인지 분명하지 않으면 되묻습니다.
- 방향("북쪽")은 구역 ID 이름(`area-north`)과 맞춰 고릅니다. 구역 ID가 방향을 나타내지 않으면
  `area_id`를 직접 적습니다.
- 응답이 `요청을 처리하는 데 5번의 시도로도 부족했습니다`여도 계획이 수락됐다면 비행은 계속됩니다.
  진행은 이벤트 로그(`tail -f services/middleware/logs/mission-events.jsonl`)나 "mission-001 상태
  알려줘" 같은 명령으로 확인합니다.

## 끝낼 때

1. 모든 작업이 끝났는지 확인합니다(이벤트 로그의 `task_finished`).
2. 오케스트레이터를 Ctrl+C로 내립니다.
3. 미들웨어 터미널에서 Ctrl+C. `server_stopping` 이벤트가 남고, 실행 중인 작업이 있었다면 그
   목록과 경고가 stderr에 출력됩니다.
4. 시뮬레이션 종료는 시뮬레이션 담당 절차를 따릅니다.

미들웨어는 종료할 때 자동으로 착륙시키지 않습니다. 비행 중에 내리면 아래 수동 착륙이 필요합니다.
takeoff·land처럼 오래 걸리는 명령이 진행 중이면 서버가 그 명령이 끝날 때까지 종료를 기다리며,
기다리지 않으려면 Ctrl+C를 한 번 더 누릅니다.

## 서버 중단 시 수동 착륙

서버를 비행 중에 내리면 Dispatcher 스레드가 함께 끝나 드론이 마지막 목표 지점에서 제자리 비행합니다
(K5). 어떤 드론이 남았는지는 서버 stderr의 경고나 이벤트 로그의 마지막 `server_stopping`
(`running_tasks`)에서 확인합니다.

**미들웨어 서버를 내린 상태에서** 실행합니다. 서버가 떠 있으면 MAVLink 포트를 서버가 쓰고 있습니다.

MAVLink 드론(drone-01):

```bash
cd services/simulation/app
python3 -c "from mavlink_drone import MavlinkDrone; d = MavlinkDrone('udpin:127.0.0.1:14561').connect(); print('landed:', d.land()); d.close()"
```

AP_DDS 드론(drone-02):

```bash
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 9}"   # 9 = LAND
```

- `landed: True`는 착륙 후 시동 해제까지 확인했다는 뜻입니다. AP_DDS는 `ros2 topic echo /ap/status
  --once`의 `armed`가 `false`가 될 때까지 확인합니다.
- **공중에서 disarm하지 않습니다.** 모터가 꺼져 추락합니다. 착륙이 확인되지 않을 때만, 지상에
  내려온 것을 확인하고 disarm합니다.

## Windows에서 WSL 미들웨어로 접속이 안 될 때

1. WSL 안에서 서버가 떠 있는지 확인합니다.

   ```bash
   curl -s -o /dev/null -w "%{http_code}\n" http://127.0.0.1:9000/mcp   # 406이면 정상 (MCP 클라이언트가 아니어서 거부)
   ```

2. Windows에서 포트가 보이는지 확인합니다.

   ```powershell
   Test-NetConnection 127.0.0.1 -Port 9000
   ```

   WSL2는 기본으로 Windows의 `localhost`를 WSL로 전달합니다(`localhostForwarding`). `False`이면
   `%UserProfile%\.wslconfig`에 `localhostForwarding=false`가 있는지 확인하고, 바꿨다면
   `wsl --shutdown` 후 다시 엽니다. 주소는 `localhost` 대신 `127.0.0.1`을 씁니다.

3. 그래도 안 되면 **오케스트레이터도 WSL에서 실행**하는 것을 먼저 권합니다(5단계의 WSL 명령).
   미들웨어를 외부에 열 필요가 없습니다.

4. 마지막 수단으로 미들웨어를 모든 인터페이스에 엽니다.

   ```bash
   MCP_HOST=0.0.0.0 PYTHONPATH=services/middleware python3 -m app.mcp_server   # 3단계 export는 그대로
   hostname -I   # WSL IP 확인, 예: 172.29.80.2
   ```

   Windows에서는 `MCP_SERVER_URL=http://<WSL IP>:9000/mcp`를 씁니다.

   > ⚠️ 미들웨어 MCP 서버에는 **인증이 없습니다.** `0.0.0.0`으로 열면 네트워크 구성에 따라
   > (예: WSL mirrored 네트워킹) 같은 네트워크의 다른 컴퓨터도 접속해 드론에 명령을 보낼 수
   > 있습니다. 집이나 연구실처럼 믿을 수 있는 네트워크에서만, 실행하는 동안만 사용하고 끝나면
   > 기본값(`127.0.0.1`)으로 되돌립니다. 공용 와이파이에서는 쓰지 않습니다.

## 문제 해결

| 증상 | 원인과 조치 |
|---|---|
| 서버 시작 시 `MCP_TRANSPORT must be one of stdio, http` | 오타. 서버는 드론에 연결하지 않고 종료 코드 2로 끝남 |
| `ModuleNotFoundError: rclpy` 또는 `ardupilot_msgs` | 가상환경 Python으로 실행했거나 `source` 2개를 빠뜨림. 시스템 `python3`와 같은 터미널의 `source` 확인 |
| `[Errno 98] address already in use` (포트 9000) | 이전 서버나 mock 서버가 떠 있음. `ss -ltnp \| grep 9000`으로 찾아 종료하거나 `MCP_PORT` 변경(오케스트레이터 주소도 같이 변경) |
| 드론이 `disconnected`, 계획이 `DRONE_DISCONNECTED`로 거부 | SITL·Agent가 준비되기 전에 서버를 띄움(K2, K3). 2단계 완료 후 서버 재시작 |
| 계획이 `DRONE_NOT_AVAILABLE`로 거부 | 그 드론이 아직 비행 중이거나 이전 작업 실패로 `unavailable`. 실패라면 서버 재시작 |
| 오케스트레이터 `too many values to unpack` | 오케스트레이터 가상환경의 `mcp`가 1.x. `pip install "mcp>=2"` |
| 오케스트레이터 응답에 `UNKNOWN_MISSION` | LLM이 도구 결과에 없는 임무 ID를 씀. 명령에 `mission-001`처럼 임무 ID를 직접 적어 다시 시도 |
| 스크립트·서버를 비행 중에 중단함 | [서버 중단 시 수동 착륙](#서버-중단-시-수동-착륙) |

## 검증 상태

FakeDrone으로는 HTTP 전송, 클라이언트 연결 종료 후 비행 계속, 종료 시 기록을 자동 테스트로
확인했습니다(`services/middleware/tests/test_mcp_http.py`). **실제 SITL에서 HTTP 모드와
오케스트레이터 연결은 아직 검증하지 않았습니다.** 항목별 상태는
[검증 범위와 지원 범위](../contracts/verification-status.md)를 봅니다.
