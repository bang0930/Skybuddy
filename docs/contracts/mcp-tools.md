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
              ├ MissionTask 생성 (planning.build_mission_task)
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
  보일 수 있습니다. 명령 실행 중 연결 판정 방식은 실환경 검증 때 함께 정리해야 합니다.

## 계획 → 명령

수락된 배정마다 다음 순서의 `MissionTask`를 만듭니다. 고도는 구역의 탐색 고도(홈 상대)입니다.

```text
takeoff(탐색 고도) → goto(구역 웨이포인트) × N → 귀환 → land
```

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
| `server_started` | 드론 구성, 연결 여부와 실패 이유, binding, Dispatcher 설정, 로드된 임무 |
| `plan_submitted` | 제출 원본 `plan`, `attempt`(해당 임무 누적 제출 횟수), `generated_at_filled_by_server` |
| `plan_rejected` | `errors` |
| `plan_accepted` | 작업 목록, 생성된 명령 전체 |
| `command_transition` | 명령 ID·종류, 이전·현재 상태, 시각, `elapsed_since_sent_s`, 오류 |
| `task_finished` | 작업 최종 상태 |
| `emergency_stop` | 비상 정지 대상과 착륙 확인 여부 |

LLM 원본 출력과 모델 설정은 오케스트레이터에서 기록해야 합니다. 미들웨어는 도구 호출로
전달받은 계획만 볼 수 있습니다.

## 설정

| 파일 | 내용 |
|---|---|
| `services/middleware/config/drones.fake.json` | FakeDrone 2대 (`drone-01` MAVLink, `drone-02` AP_DDS) |
| `services/middleware/config/drones.real.example.json` | 실제 SITL 연결 예시 (미검증) |
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

## 임시 구현과 교체 지점

| 임시 구현 | 위치 | 교체 대상 |
|---|---|---|
| 구역 꼭짓점 순회 웨이포인트 | `runtime/planning.py`의 `area_waypoints()` | boundary → 웨이포인트 경로 생성 함수 (#26 진행 중, 담당 정리 필요) |
| `goto(home)` 귀환 | `runtime/planning.py`의 `build_mission_task()` | `DroneInterface.return_home()`. 클라이언트에 메서드가 생기면 capabilities에 `return_home`이 자동 추가되어 그 명령을 사용합니다. |
| FakeDrone | `runtime/fake_drone.py`, `registry.py`의 `ClientFactory` | 시뮬레이션 파트의 정식 FakeDrone |

## 검증 범위

| 항목 | 상태 |
|---|---|
| FakeDrone 2대로 context 조회 → 계획 제출 → 동시 비행 → 완료 | 검증 (pytest, stdio 확인 스크립트) |
| 계획 교차 검증, 명령 변환, 상태 전이, 통신 두절 판정, 비상 정지 | 검증 (pytest, FakeDrone) |
| 실제 `MavlinkDrone` / `Ros2Drone` 연결 | **미검증** — WSL, ArduPilot SITL, ROS 2 Humble 환경 필요 |
| 실제 SITL 비행 중 stdout 격리, 스레드 동시 접근 | **미검증** |
| Python 3.10 호환 | 정적 검사만 (`test_python310_compat.py`). 실제 3.10 실행은 WSL에서 확인 필요 |

FakeDrone은 일정 속도로 목표를 향해 직선 이동하는 모델이며 물리, 배터리 소모, 바람이 없습니다.
FakeDrone 결과는 실제 비행 성공의 근거가 아닙니다.

## 제외 범위

- 오케스트레이터 연결 변경과 프롬프트
- 드론 간 수직 분리 강제 (구역 할당 계층에서 예정)
- 장애 주입, Fallback, 재배정
- 커버리지 등 KPI 계산
- HTTP API 라우트, Docker Compose 통합, 상태 영속화
