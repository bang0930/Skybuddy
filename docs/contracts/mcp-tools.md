# 미들웨어 MCP 서버 및 임무 실행 파이프라인 (MVP)

이 문서는 오케스트레이터가 호출하는 미들웨어 MCP 도구와, 계획이 드론 명령으로 실행되기까지의
경로를 정의합니다. 실행 가능한 원본은 다음 코드입니다.

- MCP 서버: `services/middleware/app/mcp_server.py`
- 도구 결과 계약: `services/middleware/app/schemas/mcp.py`
- 실행 계층: `services/middleware/app/runtime/`

## 전체 흐름

```text
오케스트레이터 (MCP Client)
  │ get_mission_context → submit_mission_plan → get_mission_status
  ▼
MCP 서버 ── MissionService
              ├ 계획 검증 (planning.validate_plan)
              ├ 웨이포인트 생성·검사, MissionTask 생성 (planning.build_mission_tasks)
              └ Dispatcher ── 드론별 스레드
                               └ DroneInterface 변환 (adapters.drone_interface)
                                   └ DroneRegistry ── FakeDrone | MavlinkDrone | Ros2Drone
                                                        └ Telemetry → DroneState
```

## MCP 도구

| 도구 | 입력 | 결과 |
|---|---|---|
| `get_mission_context` | `mission_id` | `MissionContext` |
| `submit_mission_plan` | `mission_id`, `assignments[]`, 선택 `generated_at` | `PlanSubmissionResult` |
| `get_mission_status` | `mission_id` | `MissionStatusReport` |

- `assignments`의 각 항목은 `drone_id`, `area_id`, 선택적 `priority(1~100)`입니다. LLM은
  좌표나 비행 명령을 만들지 않습니다.
- `generated_at`은 생략할 수 있습니다. LLM은 현재 시각을 알기 어려우므로 비어 있으면 서버가
  제출 시각(UTC)으로 채우고, 로그에는 제출 원본과 `generated_at_filled_by_server`를 남깁니다.
- `submit_mission_plan`은 즉시 반환합니다. 비행은 수십 초에서 수 분이 걸리므로 진행 상황은
  `get_mission_status`로 확인합니다.
- 계획이 계약·검증을 통과하지 못하면 도구 오류가 아니라 `status="rejected"`와 `errors`를
  반환합니다. 오케스트레이터는 오류 내용을 보고 계획을 고쳐 다시 제출할 수 있습니다.
- 로드되지 않은 `mission_id`로 context·status를 조회하면 MCP 도구 오류를 반환합니다.

### 검증 오류 코드

| 코드 | 의미 |
|---|---|
| `SCHEMA_VALIDATION_ERROR` | `MissionPlan` 계약 위반 (중복 배정, 형식 오류, 시간대 누락 등) |
| `UNKNOWN_MISSION` | 로드되지 않은 임무 |
| `MISSION_ID_MISMATCH` | 계획의 임무 ID가 context와 다름 |
| `UNKNOWN_DRONE` / `UNKNOWN_AREA` | context에 없는 드론·구역 |
| `DRONE_NOT_AVAILABLE` | 드론 운용 상태가 `available`이 아님 (다른 임무 수행 중 포함) |
| `DRONE_DISCONNECTED` | 최근 텔레메트리 없음 |
| `DRONE_POSITION_UNKNOWN` | 귀환 기준이 될 위치 없음 |
| `COMMAND_NOT_SUPPORTED` | `takeoff`·`goto`·`land` 중 미지원 명령 존재 |
| `UNSUPPORTED_ALTITUDE_REFERENCE` | 구역 탐색 고도가 `home_relative`가 아님 |
| `ALTITUDE_BELOW_MINIMUM` | 구역 탐색 고도가 최소 안전 고도 30m 미만 |
| `INVALID_AREA_BOUNDARY` | 구역 경계가 단순 다각형이 아님 (변끼리 교차하는 꼭짓점 순서 등). 메시지에 shapely의 판정 이유 포함 |
| `WAYPOINT_CHECK_FAILED` | 생성된 구역 경로가 검사를 통과하지 못함 (경계 밖 점, 탐색 고도와 다른 고도, `max_leg_m`보다 긴 구간, 생성 실패). 생성기 결함이므로 LLM이 고칠 수 있는 오류는 아님 |

`INVALID_AREA_BOUNDARY`까지는 `validate_plan`이 검사하고, `WAYPOINT_CHECK_FAILED`는 그다음
MissionTask를 만들 때 검사합니다. 둘 다 명령을 하나도 보내기 전에 계획 전체를 `rejected`로
돌려보냅니다.

### 임무 상태

`MissionStatusReport.state`는 수락된 작업 전체에서 계산합니다.

| 값 | 의미 |
|---|---|
| `idle` | 수락된 계획 없음 |
| `running` | 하나 이상의 작업이 실행 중 |
| `succeeded` | 모든 작업의 모든 명령이 성공 |
| `failed` | 하나 이상의 작업 실패 |

## 상태 수집

`MavlinkDrone`과 `Ros2Drone`은 모두 `DroneInterface.get_telemetry()`로 공통 `Telemetry`를
반환합니다. `Ros2Drone`이 이미 ENU→NED, AMSL→홈 상대고도 변환을 수행하므로 MVP에서는 두
프로토콜 모두 `TelemetryStateMapper` 하나로 `DroneState`를 만듭니다.

- capabilities는 프로토콜별로 유지합니다. AP_DDS는 위성 수·진동·clipping이 `unsupported`입니다.
- `MavlinkStateMapper`는 SYSID 검증 후 같은 매퍼에 위임합니다.
- `ApDdsStateMapper`는 ROS 2 원본 토픽을 직접 받는 경로를 위해 유지하며 MVP에서는 사용하지 않습니다.
- 마지막 텔레메트리 수신 후 경과 시간이 `telemetry_timeout_s`(기본 3초)를 넘으면
  `disconnected`입니다. 통신이 끊겨도 마지막 위치와 운용 상태는 유지합니다.
- heading은 `[0, 360)`으로 정규화합니다. 어댑터가 `% 360` 뒤에 반올림하면 359.995° 이상이
  360.0으로 오기 때문입니다.
- 한 드론의 샘플이 `DroneState` 계약을 위반하면 그 샘플은 받지 않은 것으로 취급하고 직전의
  정상 샘플로 상태를 보고합니다. 다른 드론과 `get_mission_context`/`get_mission_status`는
  영향을 받지 않으며, 위반이 계속되면 timeout 뒤 `disconnected`가 됩니다. 위반 필드와 이유는
  서버 stderr 경고로 남습니다.
- 어댑터의 `takeoff()`가 고도 도달을 기다리며 내부에서 읽는 텔레메트리도 캐시에 반영됩니다.
  다만 실제 `MavlinkDrone.takeoff()`의 시동·GUIDED 전환 대기(최대 30초)나 `land()`의 시동
  해제 대기(고도 1m당 4초, 최소 60초)처럼 텔레메트리를 읽지 않는 구간에는 `disconnected`로
  보일 수 있습니다. 실제 SITL 1차 실행에서는 land 구간 전체(MAVLink 약 55초, AP_DDS 약 29~36초)가
  `disconnected`로 보고되었습니다. FakeDrone의 `land()`도 텔레메트리를 읽지 않아 같은 현상이 재현됩니다.
  원인 구간과 대응안은 [검증 범위와 지원 범위](verification-status.md)의 K1에 정리했습니다.

## 계획 → 명령

수락된 배정마다 다음 순서의 `MissionTask`를 만듭니다. 고도는 구역의 탐색 고도(홈 상대)입니다.

```text
takeoff(탐색 고도) → goto(구역 웨이포인트) × N → 귀환 → land
```

구역 웨이포인트는 설정한 생성기(`runtime/waypoints.py`의 `WaypointGenerator`)가 만듭니다.

| 생성기 | `waypoints.generator` | 경로 | 필요한 값 |
|---|---|---|---|
| `CoverageGenerator` (기본) | `coverage` | `runtime/coverage.py`의 왕복(boustrophedon) 경로. 줄 간격은 `swath_width_m × (1 − side_overlap)` | 드론별 `swath_width_m` |
| `VertexTraversalGenerator` | `vertex_traversal` | 이전 임시 방식. 꼭짓점을 순서대로 돌고 첫 꼭짓점으로 돌아옴 | 없음 |

- 두 생성기 모두 인접 웨이포인트 사이가 `max_leg_m`(기본 100m)보다 길면 같은 간격으로 나눕니다.
  꼭짓점 순회는 변이 100m 이하인 구역에서 이전과 같은 경로입니다.
- 만든 경로는 명령으로 바꾸기 전에 검사합니다. 모든 점이 구역 경계 안 또는 경계 위(좌표 반올림
  허용 오차 0.05m)에 있고, 고도가 구역 탐색 고도·`home_relative`와 같고, 인접 점 사이가
  `max_leg_m` 이하여야 합니다. 홈 → 구역, 구역 → 홈 구간은 검사 대상이 아닙니다.
- 생성 결과는 입력으로만 정해집니다. 같은 `MissionContext`와 `MissionPlan`이면 시각 필드를 뺀
  웨이포인트, MissionTask, `command_id` 순서, 명령 payload가 같습니다(`tests/test_waypoints.py`).
  `task_id` 번호는 서버가 시작된 뒤 수락한 작업 순서로 매기므로, 같은 계획이라도 서버를 다시
  시작하지 않고 두 번 제출하면 번호가 달라집니다.
- mission-001(FakeDrone, 임시 `swath_width_m` 10m)에서 드론별 명령은 8개 → 9개입니다. 구역마다
  꼭짓점 5개(둘레 80m) 대신 3줄 왕복 6개(70m)를 돌며, 홈 왕복을 포함한 수평 이동 거리는
  drone-01 113m → 124m, drone-02 154m → 124m입니다.

| 명령 | `DroneInterface` 호출 | 결과 상태 |
|---|---|---|
| `takeoff` | `takeoff(alt)` — 목표 고도 95% 도달 후 반환 | `sent → succeeded` |
| `goto` | `goto(lat, lon, alt)` — 즉시 반환 | `sent → executing → succeeded` (도달 반경 이내), 시간 초과 시 `timed_out` |
| `land` | `land()` — 시동 해제 확인 시 True. 대기 시간은 어댑터가 현재 고도로 정함(1m당 4초, 최소 60초) | `sent → succeeded`, False면 `timed_out` |
| `disarm` | `disarm()` | `land`와 동일 |

- `DroneInterface`는 프로토콜 ACK를 노출하지 않으므로 `accepted`를 임의로 만들지 않습니다.
- `goto` 구간의 제한 시간은 `max(waypoint_timeout_s, 구간 거리 ÷ min_ground_speed_m_s)`입니다.
  홈 → 구역, 구역 → 홈처럼 긴 이동 구간도 거리에 비례한 시간을 받습니다. 기본값은 60초,
  도달 반경 3m, 최저 속도 2.0m/s(실측 2.7~4.8m/s보다 보수적)이며 설정 파일로 바꿀 수 있습니다.
- 30m 미만 또는 `msl` 기준 고도는 변환 단계에서 거부합니다. 시뮬레이션의 `run_mission()`도
  같은 하한을 검사하는 이중 방어입니다.
- 명령이 실패하면 남은 명령은 `failed(TASK_ABORTED)`가 되고, 비행 중이었다면 착륙을 먼저
  시도한 뒤 확인되지 않으면 disarm합니다. 실패한 드론은 `unavailable`로 바뀌어 이후 배정에서
  제외됩니다.
- 드론 운용 상태: 수락 시 `assigned`, 귀환 구간 `returning`, 성공 후 `available`.

## 실행 로그

`MIDDLEWARE_EVENT_LOG`(기본 `services/middleware/logs/mission-events.jsonl`)에 한 줄당 한
이벤트를 남깁니다.

| event | 주요 필드 |
|---|---|
| `server_started` | 드론 구성, 연결 문자열, 연결 여부와 실패 이유, binding, Dispatcher 설정, 웨이포인트 설정(`waypoints`: 생성기, `side_overlap`, `max_leg_m`, 드론별 `swath_width_m`), 로드된 임무 |
| `plan_submitted` | 제출 원본 `plan`, `attempt`(해당 임무 누적 제출 횟수), `generated_at_filled_by_server` |
| `plan_rejected` | `errors` |
| `plan_accepted` | 작업 목록, 생성된 명령 전체 |
| `command_transition` | 명령 ID·종류, 이전·현재 상태, 시각, `elapsed_since_sent_s`, 오류. `sent`에는 실제로 호출한 클라이언트(`target`: drone_id, protocol, binding) |
| `task_finished` | 작업 최종 상태 |
| `emergency_stop` | 비상 정지 대상과 착륙 확인 여부 |

LLM 원본 출력과 모델 설정은 오케스트레이터에서 기록해야 합니다. 미들웨어는 도구 호출로
전달받은 계획만 볼 수 있습니다.

## 설정

| 파일 | 내용 |
|---|---|
| `services/middleware/config/drones.fake.json` | FakeDrone 2대 (`drone-01` MAVLink, `drone-02` AP_DDS). `swath_width_m` 10m는 테스트용 **임시값** |
| `services/middleware/config/drones.real.example.json` | 실제 SITL 연결 예시. `swath_width_m`이 정해질 때까지 **임시로** `vertex_traversal` 사용 (아래 참고) |
| `services/middleware/config/scenarios/mission-001.json` | AirSim 원점 인근 20×20m 구역 2개, 탐색 고도 30m / 50m |

환경 변수 `MIDDLEWARE_DRONES_CONFIG`, `MIDDLEWARE_SCENARIOS_DIR`, `MIDDLEWARE_EVENT_LOG`로
경로를 바꿀 수 있습니다. 실제 모드는 `services/simulation/app`을 import 경로에 추가해
`MavlinkDrone`/`Ros2Drone`을 그대로 사용합니다.

드론 설정 파일의 `dispatcher` 항목으로 도착 판정 규칙을 바꿀 수 있습니다. 생략하면 기본값입니다.

```json
"dispatcher": {
  "reach_threshold_m": 3.0,
  "waypoint_timeout_s": 60.0,
  "min_ground_speed_m_s": 2.0
}
```

`waypoints` 항목으로 웨이포인트 생성 방식을 고릅니다. 생략하면 아래 기본값입니다.
`side_overlap`과 `max_leg_m`의 기본값은 `generate_coverage_waypoints()`의 기본값과 같습니다.

```json
"waypoints": {
  "generator": "coverage",
  "side_overlap": 0.2,
  "max_leg_m": 100.0
}
```

`swath_width_m`(한 줄이 탐색 고도에서 덮는 지면 폭, m)은 드론 항목마다 적습니다. 코드에 기본값이
없으며, `coverage` 생성기를 쓸 때 하나라도 빠져 있으면 구성 파일을 읽는 단계에서 서버가 시작되지
않습니다. `vertex_traversal`에서는 쓰지 않습니다.

```json
{"drone_id": "drone-01", "protocol": "mavlink", "mode": "real",
 "connection_string": "udpin:127.0.0.1:14561", "swath_width_m": 10.0}
```

- `drones.fake.json`의 10m는 mission-001 구역(약 20×20m)에서 여러 줄 왕복이 생기도록 정한
  **임시값**입니다. 실제 카메라 사양에서 나온 값이 아닙니다.
- `drones.real.example.json`은 실제 값이 정해질 때까지 **임시로** `"waypoints": {"generator":
  "vertex_traversal"}`을 지정해 이전 방식(꼭짓점 순회)으로 비행합니다. 카메라 화각과 탐색 고도로
  계산한 `swath_width_m`을 드론마다 넣은 뒤 `waypoints` 항목을 지우면 coverage로 바뀝니다.
  `swath_width_m` 없이 `waypoints` 항목만 지우면 서버가 시작되지 않습니다.
- 서버 시작 시 `waypoints.max_leg_m ≤ dispatcher.waypoint_timeout_s × min_ground_speed_m_s`
  (기본 60초 × 2.0m/s = 120m)인지 검사하고, 어긋나면 시작하지 않습니다. 구역 안의 한 구간은 가장
  느린 속도로도 구간 기본 제한 시간 안에 끝나야 한다는 조건입니다.

연결에 실패한 드론이 있어도 서버는 시작합니다. 실패한 드론은 텔레메트리 없이 등록되어
`disconnected`로 보고되고, 계획 검증에서 `DRONE_DISCONNECTED`로 거부됩니다. 실패 이유는
`server_started` 로그와 stderr에 남습니다.

## 실행

저장소 루트에서 실행합니다.

```bash
pip install -r services/middleware/requirements-dev.txt

# 테스트
PYTHONPATH=services/middleware pytest -q services/middleware/tests

# MCP 서버 (stdio)
PYTHONPATH=services/middleware python -m app.mcp_server

# LLM 없이 서버를 띄워 전 구간 확인
python services/middleware/scripts/check_mvp_pipeline.py
```

### 실제 모드 실행 조건

실제 모드는 미들웨어가 시뮬레이션 어댑터를 같은 프로세스에서 import하므로 시뮬레이터와 같은
환경에서 실행해야 합니다(`services/simulation/INTERFACE.md` 2-1-2).

- WSL2 Ubuntu 22.04의 **시스템 Python 3.10**. ROS 2 Humble의 `rclpy`는 3.10에서만 동작하므로
  다른 버전의 가상환경으로는 `Ros2Drone`을 불러올 수 없습니다. 미들웨어 코드는 3.10에서 동작하도록
  3.11 이상 전용 `typing` 기능을 `typing_extensions`에서 가져오며, `tests/test_python310_compat.py`가
  이를 검사합니다.
- 실행 전 ROS 환경 source: `source /opt/ros/humble/setup.bash`, `source ~/ardu_ws/install/setup.bash`
- `pymavlink` 설치, Micro XRCE-DDS Agent와 SITL 2대 기동(`services/simulation/scripts/start_hetero_sim.sh`)

```bash
MIDDLEWARE_DRONES_CONFIG=services/middleware/config/drones.real.example.json \
  python3 services/middleware/scripts/check_mvp_pipeline.py
```

### 통합 검증 증거 수집

`scripts/collect_integration_evidence.py`는 `check_mvp_pipeline.py`와 같은 흐름(context →
잘못된 계획 → 정상 계획 → 상태 조회)을 stdio MCP로 실행하고, 결과를 증거 폴더에 남깁니다.
`--drones`로 고른 드론만 서버에 등록하고 계획에 넣습니다. 서버에는 구성 파일에서 선택한
드론만 남긴 사본(`drones.selected.json`)을 넘깁니다. `MIDDLEWARE_DRONES_CONFIG`,
`MIDDLEWARE_SCENARIOS_DIR`, `MIDDLEWARE_EVENT_LOG`는 서버와 같은 의미로 사용합니다.

```bash
export MIDDLEWARE_DRONES_CONFIG=services/middleware/config/drones.real.example.json

# A. MAVLink 1대
python3 services/middleware/scripts/collect_integration_evidence.py --drones drone-01
# B. AP_DDS 1대
python3 services/middleware/scripts/collect_integration_evidence.py --drones drone-02
# C. 2대 동시
python3 services/middleware/scripts/collect_integration_evidence.py --drones drone-01,drone-02
```

FakeDrone으로 확인할 때는 `MIDDLEWARE_DRONES_CONFIG`를 지정하지 않습니다. 결과는
`services/middleware/evidence/<시각>_<시나리오>/`에 저장되며 저장소에는 올리지 않습니다
(`.gitignore`).

| 파일 | 내용 |
|---|---|
| `summary.md` | 임무 결과, 실행 환경, binding, 도구 호출, 명령별 결과, `disconnected` 구간, 식별자 혼선 검사 |
| `tool_calls/NNN_<tool>.json`, `tool_calls.csv` | 도구 호출별 요청, 응답, 소요 시간(ms) |
| `command_transitions.csv`, `command_summary.csv` | 명령 상태 전이 전체와 명령별 전이 경로, `sent` 후 경과 시간 |
| `events.jsonl` | 이번 실행에서 EventLog에 추가된 줄만 잘라 낸 사본 |
| `bindings.json` | `server_started`의 drone_id, protocol, 연결 문자열, SYSID/COMPID 또는 namespace, 연결 결과 |
| `environment.json` | Python, OS, WSL 여부, `ROS_DISTRO`, 패키지 버전, 커밋, 구성 파일 경로, fake/real 모드 |
| `connection_gaps.json`, `identifier_check.json` | 아래 두 검사의 원본 |
| `server.stderr.log` | 서버와 시뮬레이션 어댑터 출력 |

- **disconnected 구간**: 상태 조회(기본 1초 간격) 응답에서 드론이 `disconnected`로 보고된 연속
  구간과 그때 실행 중이던 명령을 표시합니다. 조회 간격보다 짧은 구간은 빠질 수 있습니다.
- **식별자 혼선 검사**: EventLog만으로 판정합니다. 등록된 드론끼리 연결 문자열, ROS 2 namespace,
  같은 링크의 SYSID가 겹치지 않는지, 계획된 명령이 작업의 드론과 그 프로토콜을 가리키는지,
  `sent` 전이의 `target`이 `server_started`의 drone_id·protocol·binding과 같은지 확인합니다.
  명령이 하나도 전송되지 않았으면 `N/A`입니다.
- 종료 코드는 임무 `succeeded`이고 검사가 `PASS`일 때 0입니다. 기본 대기 시간은 900초(`--timeout`)입니다.
- 실행 중 스크립트를 중단하면 서버가 함께 종료되어 비행 중인 기체는 마지막 목표에서 대기합니다.
  이때는 MAVProxy나 ROS 2 서비스로 직접 착륙시켜야 합니다.
- EventLog 사본은 시작 시점의 파일 크기 이후만 복사하므로, 같은 로그 경로를 쓰는 서버를 동시에
  실행하지 않습니다.

## 임시 구현과 교체 지점

| 임시 구현 | 위치 | 교체 대상 |
|---|---|---|
| FakeDrone `swath_width_m` 10m | `config/drones.fake.json` | 시뮬레이션 카메라 화각과 탐색 고도로 계산한 드론별 값 |
| 실제 구성의 `vertex_traversal` 지정 | `config/drones.real.example.json`의 `waypoints` | 드론별 `swath_width_m`을 넣고 `waypoints` 항목을 지워 coverage 사용 |
| 드론별 고정 `swath_width_m` | `runtime/registry.py`의 `DroneConfig` | 촬영 폭은 탐색 고도에 비례하므로 구역 고도가 다르면 값도 달라야 함. 화각을 받아 구역 고도로 계산하는 방식 |
| `goto(home)` 귀환 | `runtime/planning.py`의 `build_mission_task()` | `DroneInterface.return_home()`. 클라이언트에 메서드가 생기면 capabilities에 `return_home`이 자동 추가되어 그 명령을 사용합니다. |
| FakeDrone | `runtime/fake_drone.py`, `registry.py`의 `ClientFactory` | 시뮬레이션 파트의 정식 FakeDrone |

## 검증 범위

항목별 실제 SITL 검증, Mock(FakeDrone) 검증, 검증 불가 구분과 지원 범위, 버전 제약, 알려진 실패
지점은 [검증 범위와 지원 범위](verification-status.md)에 정리합니다.

FakeDrone은 일정 속도로 목표를 향해 직선 이동하는 모델이며 물리, 배터리 소모, 바람이 없습니다.
FakeDrone 결과는 실제 비행 성공의 근거가 아닙니다.

## 제외 범위

- 오케스트레이터 연결 변경과 프롬프트
- 드론 간 수직 분리 강제 (구역 할당 계층에서 예정)
- 장애 주입, Fallback, 재배정
- 커버리지 등 KPI 계산
- HTTP API 라우트, Docker Compose 통합, 상태 영속화
