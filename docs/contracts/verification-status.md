# 검증 범위와 지원 범위

미들웨어 MCP 파이프라인(오케스트레이터 → 미들웨어 → `DroneInterface` → SITL)을 어디까지 어떤
방법으로 확인했는지 정리합니다. #31(실제 SITL 1차 통합 검증)과 #20(실제·Mock·검증 불가 구분,
지원 범위·버전 제약·알려진 실패 지점)의 기준 문서입니다.

기준일: 2026-10-14. 실제 SITL 칸은 같은 날 시뮬레이션 담당이 실행한 1차 결과(실행 A, B, C)로
채웠습니다. 실행별 결과와 증거 폴더는 [5절](#5-실제-sitl-1차-검증-결과-2026-10-14)에 있습니다.

## 구분

| 구분 | 의미 | 근거 |
|---|---|---|
| 실제 SITL 검증 | WSL2 + ArduPilot SITL + AirSim에서 확인 | `collect_integration_evidence.py` 증거 폴더 |
| Mock 검증 | FakeDrone으로 확인. 실제 비행 성공의 근거가 아님 | pytest, 증거 스크립트 FakeDrone 실행 |
| 검증 불가 | 현재 환경이나 시뮬레이터 특성상 재현할 수 없음 | 사유를 함께 적음 |

표의 값: **검증**, **부분**(일부 조건만 확인), **미검증**, **검증 불가**, **범위 밖**.

## 1. 항목별 검증 상태

실제 SITL 칸의 A, B, C는 5절의 실행을 가리킵니다.

### 1-1. 통합 흐름

| # | 항목 | Mock (FakeDrone) | 실제 SITL | Mock 근거 |
|---|---|---|---|---|
| F1 | stdio MCP로 context → 잘못된 계획 거부 → 정상 계획 수락 → 상태 조회 | 검증 | 검증 (A, B, C) | `test_mcp_server.py`, 증거 스크립트 |
| F2 | MAVLink 1대 단일 흐름 (`--drones drone-01`) | 검증 | 검증 (A, 8/8 성공) | 증거 스크립트 시나리오 A |
| F3 | AP_DDS 1대 단일 흐름 (`--drones drone-02`) | 검증 | 검증 (B, 8/8 성공) | 증거 스크립트 시나리오 B |
| F4 | 2대 동시 실행 (`--drones drone-01,drone-02`) | 검증 | 검증 (C, 16/16 성공) | 증거 스크립트 시나리오 C, `test_mission_service.py` |
| F5 | 명령이 등록된 drone_id·protocol·binding으로만 실행됨 (식별자 혼선 없음) | 검증 | 검증 (A, B, C 모두 PASS, 불일치 0) | `test_integration_evidence.py`, summary의 검사 |
| F6 | 프로토콜별 상위 제어 경로 사용 (MAVLink `SET_POSITION_TARGET_GLOBAL_INT`, AP_DDS `/ap/cmd_gps_pose`) | 검증 불가 | 부분 (C) | FakeDrone에는 프로토콜이 없음. 실제: AP_DDS는 토픽 캡처로 확인, MAVLink는 binding과 어댑터 출력만 있고 메시지 캡처 없음 (5절) |

### 1-2. 연결과 상태 수집

| # | 항목 | Mock (FakeDrone) | 실제 SITL | Mock 근거 |
|---|---|---|---|---|
| S1 | MAVLink 연결과 SYSID/COMPID binding 획득 | 부분 | 검증 (A, C: SYSID 1, COMPID 1) | FakeDrone은 설정값의 SYSID를 돌려줌. 실제는 HEARTBEAT에서 얻음 |
| S2 | AP_DDS 연결과 namespace binding | 부분 | 검증 (B, C: `/ap`) | FakeDrone 기본 namespace `ap` |
| S3 | `Telemetry` → `DroneState` 변환, 프로토콜별 capabilities | 검증 | 부분 (A, B, C: 샘플 거부 없음. AP_DDS `gps_fix_type`·`gps_eph`가 항상 `null`, K9) | `test_telemetry_mapper.py`, `test_runtime_registry.py` |
| S4 | 통신 두절 판정 (`telemetry_timeout_s` 3초) | 검증 | 미검증 (링크 끊기 시험 안 함) | FakeDrone `link_up=False`. 실제는 MAVProxy·Agent를 내려 링크만 끊어야 함 |
| S5 | 연결 실패 드론이 있어도 서버 계속 실행, 계획은 `DRONE_DISCONNECTED`로 거부 | 검증 | 미검증 (모든 연결 성공) | `test_runtime_registry.py` |
| S6 | 블로킹 명령(takeoff, land) 중 `disconnected` 표시 (K1) | 부분 | 검증 (A, B, C: land 구간에서 발생, takeoff 구간은 미발생) | FakeDrone `land()` 구간에서 재현. takeoff 구간은 FakeDrone이 텔레메트리를 읽어 재현 안 됨 |
| S7 | 연결 유지 상태로 상태 조회를 반복해도 최신 위치 | 미검증 | 검증 (C: 119회 조회 동안 위치가 비행 경로를 따라 갱신) | FakeDrone은 버퍼가 없음. MAVLink `_drain()` 동작은 실제 SITL에서만 의미가 있음 |

### 1-3. 명령 실행

| # | 항목 | Mock (FakeDrone) | 실제 SITL | Mock 근거 |
|---|---|---|---|---|
| C1 | 계획 → MissionTask → `DroneInterface` 호출 변환, 30m 미만·`msl` 거부 | 검증 | 부분 (정상 변환만 실행. 거부 경로는 실제 비행과 무관) | `test_drone_interface_commands.py`, `test_plan_validation.py` |
| C2 | takeoff 95% 고도 도달 후 반환 → `sent → succeeded` | 검증 | 검증 (4회, 12.8~20.6초) | `test_mission_service.py` (상태 전이), FakeDrone이 같은 완료 기준 |
| C3 | goto 도달 판정(반경 3m), 구간 거리 비례 제한 시간 | 검증 | 부분 (24회 모두 도달, 3.9~7.5초. 시간 초과 경로는 발생 안 함) | `test_mission_service.py` (`test_long_transit_legs_...`, `test_lost_link_...`) |
| C4 | land 고도 비례 대기(1m당 4초, 최소 60초), 시동 해제 확인 | 검증 | 검증 (4회, 32.6~60.1초. 대기 한도 120초, 200초 이내) | `test_fake_drone_contract.py` (`test_landing_timeout_matches_simulation_rule`) |
| C5 | 실패 시 남은 명령 `TASK_ABORTED`, 착륙 후 미확인 시 disarm, 드론 `unavailable` | 검증 | 미검증 (실패 없음) | `test_mission_service.py` |
| C6 | 두 드론 스레드의 동시 어댑터 접근 직렬화 (pymavlink, rclpy) | 검증 불가 | 부분 (C 1회, 약 2분 동시 비행 중 오류 없음) | FakeDrone은 스레드 문제가 없음 |
| C7 | 어댑터 `print()`가 MCP stdio를 오염시키지 않음 | 미검증 | 검증 (A, B, C: 어댑터 출력이 `server.stderr.log`에만 있고 도구 응답 정상) | FakeDrone은 출력하지 않음. 실제 실행이 끝까지 되고 `server.stderr.log`에 어댑터 출력이 남으면 확인 |
| C8 | `return_home()` 사용 | 범위 밖 | 범위 밖 | 시뮬레이션 어댑터에 메서드가 아직 없음. `goto(home)`으로 대체 |

### 1-4. 실행 환경

| # | 항목 | Mock (FakeDrone) | 실제 SITL | Mock 근거 |
|---|---|---|---|---|
| E1 | Python 3.10에서 미들웨어 실행 | 부분 | 검증 (시스템 Python 3.10.12) | 정적 검사만 (`test_python310_compat.py`). 개발 PC는 3.13 |
| E2 | 미들웨어와 어댑터를 같은 프로세스에서 import (`simulation_app_path`) | 미검증 | 검증 (A, B, C) | |

### 1-5. 검증 불가와 범위 밖

| 항목 | 구분 | 사유 |
|---|---|---|
| 실제 비행 시간, 경로 효율 등 KPI | 검증 불가 (Mock) | FakeDrone은 등속 직선 이동. 실제 SITL 값만 의미가 있음 |
| 배터리 부족 | 검증 불가 | SITL 배터리는 항상 100% (`INTERFACE.md` 2-4 ⑤). 값 주입 기능이 필요 |
| GPS 저하 | 검증 불가 | SITL GPS가 항상 최상급 fix |
| 드론 이탈 (SITL 종료) | 검증 불가 | AirSim lock-step이라 SITL 하나를 끄면 시뮬레이션 전체가 멈춤 |
| 드론 간 수직 분리 강제 | 범위 밖 | 구역 할당 계층에서 예정. 현재 아무도 강제하지 않음 |
| Fallback, 재배정 | 범위 밖 | 9~10주차 |
| 구역 경계 → 탐색 경로 (coverage 모듈, #30·#32) | 부분 | 파이프라인 기본 생성기로 연결(#32). FakeDrone(임시 `swath_width_m` 10m)으로만 검증. 실제 구성은 드론별 `swath_width_m`이 정해질 때까지 임시로 꼭짓점 순회(`vertex_traversal`)를 사용하므로 실제 SITL에서 coverage는 미실행 |

## 2. 지원 범위

| 항목 | 지원 |
|---|---|
| 프로토콜 | MAVLink (`MavlinkDrone`, pymavlink), AP_DDS (`Ros2Drone`, ROS 2 Humble) |
| 드론 수 | 설정 파일에 등록한 수. 검증 구성은 프로토콜별 1대씩 2대 |
| 명령 | `takeoff`, `goto`, `land`, `disarm`. `return_home`은 클라이언트에 메서드가 있을 때만 |
| 고도 | 홈 기준 상대 고도(`home_relative`), 30m 이상 |
| 텔레메트리 | 공통 `Telemetry` 필드. AP_DDS는 위성 수, 진동 3축, clipping을 제공하지 않음 |
| 배치 | 미들웨어가 어댑터를 같은 프로세스에서 import. 시뮬레이터와 같은 WSL 환경에서 실행해야 함 |
| MCP 전송 | stdio |

## 3. 버전 제약

| 구성 요소 | 버전 | 이유 |
|---|---|---|
| OS | WSL2 Ubuntu 22.04 | ROS 2 Humble 지원 배포판. AirSim은 Windows |
| Python | **시스템 Python 3.10** (검증 환경 3.10.12) | Humble의 `rclpy`가 3.10에서만 동작. 다른 버전의 venv에서는 `Ros2Drone`을 import할 수 없음 |
| 미들웨어 코드 | 3.10 호환 | 3.11+ `typing` 기능은 `typing_extensions`에서 가져옴 (`test_python310_compat.py`) |
| ROS 2 | Humble | ArduPilot 공식 지원 배포판 |
| ArduPilot | drone1 ArduCopter 4.3.0, drone2 4.7.1 (`--enable-DDS`) | AP_DDS 이륙 서비스는 4.6 이상 |
| pymavlink | `>=2.4.37` | `services/simulation/requirements.txt` |
| ardupilot_msgs, Micro XRCE-DDS Agent | ArduPilot 4.7.1 소스 빌드, UDP 2019 | `INTERFACE.md` 0-1 |
| mcp, fastmcp | `mcp>=1.29,<2.0`, `fastmcp>=3.4,<4.0` | Python 3.10 이상 지원 |
| pydantic | `>=2.10,<3.0` | |
| shapely | `>=2.0,<3.0` | 최신 2.2는 Python 3.11 이상이 필요하므로 3.10에서는 pip가 2.1.x를 설치해야 함. #32부터 서버 런타임(`runtime/waypoints.py`, `runtime/coverage.py`)이 import하므로 실제 모드의 시스템 Python 3.10에도 설치되어 있어야 함 (3.10 + shapely 2.1.x 조합은 미검증) |

## 4. 알려진 실패 지점

| # | 지점 | 증상 | 대응 |
|---|---|---|---|
| K1 | 블로킹 명령 중 텔레메트리 미수집 | land 전체 동안 상태 조회가 `disconnected`로 보고됨(실측 MAVLink 약 55초, AP_DDS 약 29~36초). takeoff 앞부분도 같은 구조지만 1차 실행에서는 GUIDED 전환이 즉시 끝나 발생하지 않음 | 4-1 참고. 현재는 증거 summary의 `disconnected` 구간과 명령 단계로 구분 |
| K2 | 부팅 직후 GUIDED 전환 거부, NAV_TAKEOFF 거부 | takeoff 예외 → 작업 `failed`, 드론 `unavailable` | SITL 준비 완료(`start_hetero_sim.sh`) 후 실행. 실패한 드론은 서버를 다시 시작해야 재배정 가능 |
| K3 | DDS Agent 미기동, ROS 환경 미소싱 | `Ros2Drone` 연결이 30초 뒤 실패하거나 import 실패 → `connect_error` | 기동 순서 AirSim → Agent → SITL, `source` 2개 확인 |
| K4 | AirSim lock-step | SITL 하나라도 꺼지면 시뮬레이션 정지 | 1대 시나리오에서도 SITL 2대를 모두 띄움. 선택하지 않은 드론은 미들웨어에 등록하지 않을 뿐임 |
| K5 | 실행 중 스크립트·서버 중단 | Dispatcher 스레드가 함께 종료되어 기체가 마지막 목표에서 대기 | MAVProxy 또는 ROS 2 서비스로 직접 착륙 |
| K6 | goto 제한 시간 | 실측 속도(2.7~4.8m/s)보다 느리면 `WAYPOINT_TIMEOUT` | 구간 제한 시간 = max(60초, 거리 ÷ 2.0m/s). 설정 파일 `dispatcher`로 조정 |
| K7 | Python 버전 불일치 | 3.11+ venv에서 `rclpy` import 실패 | 시스템 `python3`(3.10) 사용 |
| K8 | 증거 수집 동시 실행 | 같은 EventLog 경로를 쓰는 서버가 둘이면 사본에 다른 실행의 이벤트가 섞임 | 한 번에 하나씩 실행하거나 `MIDDLEWARE_EVENT_LOG`를 분리 |
| K9 | AP_DDS `gps_fix_type`, `gps_eph`가 항상 `null` (1차 실행에서 발견) | `Ros2Drone`이 `/ap/navsat/navsat0`를 받지 못한 것으로 보임. capabilities에는 지원 필드로 올라가 있어 계속 `temporarily_unavailable`로 보고됨 | 시뮬레이션 담당이 `ros2 topic list`로 토픽 이름과 발행 여부를 확인. 발행되지 않는 것이 맞다면 `AP_DDS_CAPABILITIES`에서 두 필드를 빼서 `unsupported`로 보고 |
| K10 | AP_DDS 좌표 정밀도 (1차 실행에서 발견) | AP_DDS 위경도가 모두 float32로 표현되는 값. 이 위치에서 해상도는 위도 약 0.42m, 경도 약 1.4m (MAVLink는 약 1cm) | 도달 반경 3m 판정에는 영향 없음. 커버리지·경로 효율 KPI와 #20 정합성 허용 오차를 정할 때 반영. 원인(ArduPilot AP_DDS 발행 형식) 확인 필요 |

### 4-1. K1 블로킹 명령 중 연결 판정

`connection_status`는 마지막으로 기록된 `Telemetry` 샘플의 나이로 판정합니다
(`telemetry_timeout_s` 3초). 샘플은 `get_telemetry()`가 불릴 때만 기록되고, 명령 실행 중에는
Dispatcher가 클라이언트 잠금을 쥐고 있어 상태 조회가 새 샘플을 읽지 못하고 캐시를 씁니다.
어댑터가 블로킹 명령 안에서 `get_telemetry()`를 부르지 않는 구간은 다음과 같습니다.

| 어댑터 | 메서드 | 텔레메트리를 읽지 않는 구간 | 최대 길이 |
|---|---|---|---|
| `MavlinkDrone` | `takeoff()` | `set_mode('STABILIZE')`(최대 10초) → `arm()` ACK·STATUSTEXT(최대 6초) → `set_mode('GUIDED', timeout=30)` → 1초 대기 → `NAV_TAKEOFF` 재시도(최대 20초). 그 뒤 고도 대기에서만 `get_telemetry()` 호출 | 약 67초 |
| `MavlinkDrone` | `land()` | `NAV_LAND` 전송 후 HEARTBEAT의 armed 플래그만 확인 | `landing_timeout_s` (최소 60초) |
| `MavlinkDrone` | `disarm()` | HEARTBEAT만 확인 | 5초 |
| `Ros2Drone` | `takeoff()` | prearm, GUIDED 전환, 시동, 이륙 서비스 호출(각각 서비스 대기 10초 + 응답 10초) | 약 80초 (보통 수 초) |
| `Ros2Drone` | `land()` | LAND 모드 전환 후 `/ap/status`의 armed만 확인 | 서비스 20초 + `landing_timeout_s` |
| FakeDrone | `land()` | 내부 상태만 갱신 | 하강 시간 (30m에서 5초) |

이 구간에도 링크 자체는 살아 있습니다. pymavlink는 `recv_match()`로 읽은 모든 메시지를
`master.messages`에 남기고, `Ros2Drone`은 별도 스레드의 구독 콜백이 `_geopose`·`_status`를 계속
갱신합니다. 미들웨어가 이 신호를 보지 못할 뿐입니다.

미들웨어 쪽 대응안(구현 전, 결정 필요):

| 안 | 내용 | 장점 | 한계 |
|---|---|---|---|
| A. 실행 중 판정 기준 분리 | Dispatcher가 블로킹 명령을 호출하는 동안 해당 드론의 판정 시간을 어댑터 자체 제한 시간(takeoff 약 67초 + 고도 대기, land `landing_timeout_s`)으로 늘림 | 미들웨어만 수정. 계약 변경 없음. 실제 두절은 어댑터가 제한 시간에 예외나 False로 알려 줌 | 그 구간의 진짜 두절을 최대 60초 이상 늦게 앎. 9~10주차 통신 두절 Fallback의 감지 시간이 길어짐 |
| B. 실행 중 상태를 따로 표시 | `DroneState`에 선택 필드(예: `active_command`와 `telemetry_age_s`)를 추가하고 `connection_status`는 그대로 둠. 오케스트레이터와 Fallback이 "블로킹 명령 중 샘플 없음"을 구분 | 사실을 숨기지 않음. 판정은 상위 계층이 정함 | 데이터 계약과 오케스트레이터 프롬프트 변경 필요(배성열 합의). `disconnected` 표시 자체는 남음 |
| C. 링크 생존 신호 분리 | 연결 판정을 "마지막 `Telemetry` 샘플"이 아니라 "기체에서 마지막으로 메시지를 받은 시각"으로 바꿈. 시뮬레이션 어댑터에 잠금 없이 부를 수 있는 `last_message_at()` 같은 메서드 추가가 필요 | 블로킹 구간에도 실제 두절만 `disconnected`가 됨. 프로토콜과 무관한 같은 기준 | `services/simulation` 변경 필요(이태우). 어댑터가 스레드 안전하지 않으므로 이 메서드만은 동시 호출에 안전해야 함 |

권장: C를 목표로 시뮬레이션 담당과 메서드 형태를 맞추고, 그전까지는 A로 오탐을 막되 판정 시간을
늘린 사실을 EventLog에 남깁니다. B는 Fallback 설계(9~10주차)에서 필요하면 함께 검토합니다.
`master.messages`나 `_geopose` 같은 어댑터 내부 속성을 미들웨어가 직접 읽는 방식은 시뮬레이션
코드 변경에 쉽게 깨지므로 쓰지 않습니다.

## 5. 실제 SITL 1차 검증 결과 (2026-10-14)

시뮬레이션 담당이 `collect_integration_evidence.py`(커밋 `3470fe1`)를 실제 모드
(`drones.real.example.json`)로 실행한 결과입니다. 증거 폴더는 저장소에 올리지 않고
`sitl_evidence_31.zip`으로 보관합니다.

| 실행 | 증거 폴더 | 드론 | 결과 | 소요 | 식별자 검사 | `disconnected` 구간 |
|---|---|---|---|---|---|---|
| A | `20261014-115011_single-drone-01-mavlink` | drone-01 (MAVLink, `udpin:127.0.0.1:14561`, SYSID 1) | succeeded, 8/8 | 123.7초 | PASS | land 중 약 55초 (조회 56회) |
| B | `20261014-115223_single-drone-02-ap_dds` | drone-02 (AP_DDS, `ros2:ap`) | succeeded, 8/8 | 74.5초 | PASS | land 중 약 29초 (조회 30회) |
| C | `20261014-115610_multi-drone-01-drone-02` | 두 대 동시 | succeeded, 16/16 | 123.6초 | PASS | drone-01 land 중 약 56초, drone-02 land 중 약 36초 |

실행 환경: WSL2 (Linux 6.18 microsoft-standard), 시스템 Python 3.10.12, `ROS_DISTRO=humble`.
AirSim 실행 여부는 증거 폴더에 기록되지 않으므로 시뮬레이션 담당의 실행 기록으로 확인합니다.

명령별 실측 (A, B, C 합계):

| 명령 | MAVLink | AP_DDS |
|---|---|---|
| takeoff (`sent` → `succeeded`) | 19.4~20.6초 | 12.8~17.6초 |
| goto 한 구간 | 5.5~7.5초 | 3.9~5.8초 |
| land (시동 해제 확인까지) | 59.4~60.1초 | 32.6~40.6초 |
| `get_mission_context` 응답 | 231~253ms | 241~253ms |

확인 방법:

- **흐름**: 세 실행 모두 잘못된 계획이 `UNKNOWN_AREA`로 거부된 뒤 정상 계획이 수락되고, 모든
  명령이 `pending → sent → (executing) → succeeded`로 끝났습니다. 오류 전이와 비상 정지는 없습니다.
- **식별자**: summary의 검사 결과를 받은 `events.jsonl`과 도구 응답 원본으로 다시 계산해 같은
  결과(PASS, 불일치 0)를 확인했습니다. 모든 `sent` 전이의 `target`이 등록된 binding과 같습니다.
- **AP_DDS 제어 경로**: 실행 C 동안 `ros2 topic echo /ap/cmd_gps_pose`로 받은 메시지 62개의 목표
  변화 6번(꼭짓점 4개 → 첫 꼭짓점 복귀 → 귀환점)이 drone-02에 계획된 goto 6개와 값과 순서까지
  일치하고, drone-01의 목표는 이 토픽에 나타나지 않았습니다. 메시지는 `frame_id: map`, `coordinate_frame: 6`,
  `type_mask: 3576`으로 어댑터 계약과 같습니다.
- **MAVLink 제어 경로**: HEARTBEAT로 얻은 SYSID 1 binding과 어댑터 출력(`[연결됨] system=1
  component=1`, `[GUIDED 전환]`)으로 간접 확인했습니다. `SET_POSITION_TARGET_GLOBAL_INT`
  메시지 캡처(MAVProxy 로그 등)는 없습니다.

이번 실행에서 발생하지 않은 것: K2~K8의 실패, goto 시간 초과, 연결 실패, 샘플 계약 위반 경고.
새로 발견한 것: K9, K10.
