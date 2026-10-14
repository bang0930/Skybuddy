# MAVLink 미들웨어 변환 계약

이 문서는 시뮬레이션 서비스의 `MavlinkDrone` 출력과 미들웨어 공통 계약 사이의 변환
경계를 정의합니다. 미들웨어는 `pymavlink` 연결을 새로 만들지 않으며 아래 구현을
재사용합니다.

- `services/simulation/app/drone_interface.py`
- `services/simulation/app/mavlink_drone.py`
- `services/middleware/app/adapters/mavlink.py`

## 드론 식별자 연결

`MavlinkDroneBinding`은 미들웨어 `drone_id`와 MAVLink `system_id`, `component_id`의
고정 연결을 표현합니다. 텔레메트리를 변환할 때 원본 SYSID가 binding과 다르면 다른
기체의 상태가 섞이지 않도록 즉시 거부합니다.

```json
{
  "drone_id": "drone-01",
  "system_id": 1,
  "component_id": 1
}
```

binding은 연결 시 수신한 Heartbeat의 source system/component로 생성하고 Drone Registry가
도입되면 그곳에서 관리합니다. 이번 프로토타입은 영속 Registry를 구현하지 않습니다.

## 텔레메트리 변환

`MavlinkStateMapper`는 `MavlinkDrone.get_telemetry()`의 dict를 받아 `DroneState`를
생성합니다.

| MAVLink Adapter 필드 | 공통 계약 | 규칙 |
|---|---|---|
| `lat`, `lon`, `relative_alt_m` | `position` | WGS84와 홈 상대고도를 유지하고 `home_relative`를 명시 |
| `vx`, `vy`, `vz` | `velocity_ned_m_s` | NED 축과 m/s 단위를 그대로 유지 |
| `heading_deg` | `heading_deg` | 진북 기준 시계방향 각도 |
| `gps_fix_type` | `gps_fix_type` | MAVLink fix type 유지 |
| `satellites_visible` | `satellites_visible` | 정수 위성 수 |
| `gps_eph` | `gps_eph_m` | 미터 단위 |
| `battery_voltage_v` | `battery_voltage_v` | 볼트 단위 |
| `battery_remaining_pct` | `battery_percent` | 0~100 백분율 |
| `vibration_x/y/z` | `vibration` | 세 축을 하나의 값 객체로 묶음 |
| `clipping` | `clipping_count` | 세 가속도계 clipping 합계 |
| `timestamp` | `observed_at` | Unix epoch를 UTC datetime으로 변환 |

필드 변환은 프로토콜 공통 `TelemetryStateMapper`(`app/adapters/telemetry.py`)가 수행하며,
`MavlinkStateMapper`는 SYSID binding 검증과 Heartbeat age 판정만 추가합니다.

원본의 `None`과 과거 샘플의 빈 문자열은 공통 계약의 `None`으로 정규화합니다. 위치,
속도, 진동처럼 여러 값으로 구성된 그룹은 하나라도 없으면 그룹 전체를 `None`으로 둡니다.
MAVLink capabilities에는 해당 필드를 계속 포함하므로 이는 `unsupported`가 아니라
`temporarily_unavailable`로 판정됩니다.

Heartbeat age가 설정한 timeout 이하이면 `connected`, 초과하거나 Heartbeat가 없으면
`disconnected`로 변환합니다. 운용 상태인 `DroneStatus`는 통신 상태와 독립적이므로 호출자가
현재 상태 관리 결과를 명시적으로 전달합니다.

## Takeoff 명령 프로토타입

이번 이슈에서는 제어 요청 한 개로 `takeoff`를 연결합니다.

`to_mavlink_takeoff_request()`는 다음을 검증합니다.

- 명령 protocol이 `mavlink`인지
- 명령의 `drone_id`가 binding과 일치하는지
- payload가 `takeoff`인지
- 현재 `MavlinkDrone.takeoff()`가 지원하는 `home_relative` 고도인지

변환 결과의 `invoke(client)`는 주입받은 `MavlinkDrone.takeoff(altitude_m)`만 호출합니다.
따라서 미들웨어에 `pymavlink` 연결 코드나 MAVLink 메시지 송신 코드를 복제하지 않습니다.

## 명령 결과 상태

`MavlinkCommandLifecycle`은 프로토콜 관찰 결과를 다음과 같이 변환합니다.

| 관찰 | `CommandResult.status` | 오류 코드 |
|---|---|---|
| 전송 | `sent` | 없음 |
| `MAV_RESULT_ACCEPTED` | `accepted` | 없음 |
| `MAV_RESULT_IN_PROGRESS` | `executing` | 없음 |
| 임시 거부 | `failed` | `MAVLINK_ACK_TEMPORARILY_REJECTED` |
| 거부·실패·취소 | `failed` | ACK 값별 코드 |
| 미지원 | `unsupported` | `MAVLINK_COMMAND_UNSUPPORTED` |
| 알 수 없는 ACK | `failed` | `MAVLINK_UNKNOWN_ACK` |
| ACK timeout | `timed_out` | `MAVLINK_ACK_TIMEOUT` |
| 완료 확인 timeout | `timed_out` | `MAVLINK_COMPLETION_TIMEOUT` |
| 텔레메트리로 목표 달성 확인 | `succeeded` | 없음 |

ACK 수락은 실제 기체 동작 완료가 아니므로 `accepted`에 머뭅니다. `succeeded`는 고도 등
텔레메트리로 목표 달성을 별도 확인한 뒤에만 기록합니다. ACK를 제공하지 않는 명령은
`accepted`를 만들지 않고 `sent`에서 `executing` 또는 `succeeded`로 이동할 수 있습니다.

현재 시뮬레이션 `MavlinkDrone.takeoff()`는 ACK 대기와 고도 도달 확인을 메서드 내부에서
수행합니다. `invoke()`를 통한 실제 호출은 최종 성공 또는 예외를 제공하며, 개별 ACK 단계
Mapper는 Mock ACK와 향후 관찰 hook에서 재사용합니다. 이번 이슈에서는 시뮬레이션 Adapter의
소유권과 공개 메서드 계약을 변경하지 않습니다.

## 목표점 전달 방식

> 상태: **초안 (#33)**. 결정 항목은 **합의 필요**로 표시했습니다. 결정은 미들웨어 담당과
> 시뮬레이션 담당이 함께 내리며, 합의 전까지 구현은 현행(goto 순차)을 유지합니다.

신청계획서는 "MAVLink Adapter는 목표점 목록을 Mission으로, AP_DDS Adapter는 순차 전역 위치
목표로 변환"하도록 적고 있습니다. 현재 구현은 두 프로토콜 모두 `DroneInterface.goto()`를
순차 호출하고, 미들웨어 Dispatcher가 텔레메트리로 도달을 판정합니다. 이 절은 두 방식을
비교하고 추천안을 정리합니다.

### 현재 구조

`MavlinkDrone`(`services/simulation/app/mavlink_drone.py`)이 쓰는 메시지와 모드:

| 메서드 | MAVLink 메시지 | 모드 | 완료 확인 |
|---|---|---|---|
| `connect()` | `HEARTBEAT` 수신 | - | 상대 SYSID/COMPID 확정 |
| `takeoff(alt)` | `SET_MODE`(STABILIZE) → `COMMAND_LONG` `MAV_CMD_COMPONENT_ARM_DISARM`(param2 21196) → `SET_MODE`(GUIDED) → `COMMAND_LONG` `MAV_CMD_NAV_TAKEOFF`(20초 재시도) | STABILIZE → GUIDED | `COMMAND_ACK` 후 `GLOBAL_POSITION_INT` 고도 95% |
| `goto(lat, lon, alt)` | `SET_POSITION_TARGET_GLOBAL_INT`(`MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`, 위치만 쓰는 type_mask) **1회 송신** | GUIDED | 없음 (응답·재전송 없음) |
| `land()` | `COMMAND_LONG` `MAV_CMD_NAV_LAND` | LAND | `HEARTBEAT` armed 해제 |
| `disarm()` | `COMMAND_LONG` `MAV_CMD_COMPONENT_ARM_DISARM`(param1 0) | - | `HEARTBEAT` armed 해제 |
| `get_telemetry()` | `GLOBAL_POSITION_INT` 대기, `GPS_RAW_INT`·`SYS_STATUS`·`VIBRATION` 캐시 | - | - |

- `MISSION_*` 메시지는 송수신하지 않고 AUTO 모드도 쓰지 않습니다.
- `_drain()`은 읽기·명령 전에 수신 버퍼를 비우고 `master.messages`에 **메시지 종류별 최신값만**
  남깁니다.
- `Ros2Drone`의 `goto()`는 `/ap/cmd_gps_pose`에 같은 목표를 2Hz로 재발행합니다. 현재 어댑터가
  쓰는 AP_DDS 토픽·서비스에는 미션 업로드 경로가 없습니다.

미들웨어 Dispatcher(`runtime/dispatcher.py`, `runtime/config.py`):

- `goto` 호출 후 `get_telemetry()`를 반복 호출합니다. 첫 샘플에서 `executing`, 목표 반경
  `reach_threshold_m`(3m) 안에 들어오면 `succeeded`가 됩니다.
- 구간 제한 시간은 `max(waypoint_timeout_s 60초, 구간 거리 ÷ min_ground_speed_m_s 2.0m/s)`이고,
  넘기면 `timed_out(WAYPOINT_TIMEOUT)`입니다. 남은 명령은 `TASK_ABORTED`, 기체는 land → disarm
  순서로 정지합니다.
- 두 프로토콜이 같은 코드 경로와 같은 판정 기준을 씁니다.

웨이포인트 수(#32 coverage 생성기, `side_overlap` 0.2, `max_leg_m` 100m, 정사각형 구역 계산값):

| 구역 | swath 10m (현재 FakeDrone 임시값) | swath 30m |
|---|---|---|
| mission-001 구역 (약 20m × 20m) | 6개 (구간 5~20m) | - |
| 100m × 100m | 39개 | 12개 |
| 200m × 200m | 100개 | 36개 |
| 500m × 500m | 441개 | 147개 |
| 1000m × 1000m | 1,500개 | 504개 |

현행 방식에서 웨이포인트 하나는 `DroneCommand` 하나입니다. mission-001의 드론별 명령은 9개지만,
500m 구역이면 444개(이륙·귀환·착륙 포함)가 됩니다. 1차 실측의 MAVLink goto 한 구간은
5.5~7.5초였습니다(mission-001 구간은 모두 약 20m 이하,
[검증 상태 5절](verification-status.md#5-실제-sitl-1차-검증-결과-2026-10-14)).

### Mission 업로드 방식의 메시지 흐름

MAVLink Mission 프로토콜로 바꾸면 `MavlinkDrone`에 다음 흐름이 추가됩니다.

```text
(0) GCS → FC  MISSION_CLEAR_ALL → FC → GCS MISSION_ACK         # 이전 실행의 미션 제거
(1) GCS → FC  MISSION_COUNT(count = N + 1)                     # ArduPilot은 seq 0을 홈으로 예약
(2) FC → GCS  MISSION_REQUEST_INT(seq = 0)
(3) GCS → FC  MISSION_ITEM_INT(seq = 0, ...)
    ... seq = N까지 반복. 응답이 없으면 FC가 같은 seq를 다시 요청
(4) FC → GCS  MISSION_ACK(type = MAV_MISSION_ACCEPTED)
(5) 이륙: 현행 takeoff() (GUIDED + NAV_TAKEOFF, 고도 95% 확인)
(6) GCS → FC  SET_MODE(AUTO) 또는 COMMAND_LONG MAV_CMD_MISSION_START
(7) FC → GCS  MISSION_CURRENT(seq)       # 주기 스트림(SRx_EXT_STAT)
    FC → GCS  MISSION_ITEM_REACHED(seq)  # 항목 도달 시 1회
(8) 마지막 항목 도달 후: 현행 land() (MAV_CMD_NAV_LAND → LAND 모드)
```

각 항목은 `MAV_CMD_NAV_WAYPOINT`, `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`(현행 goto와 같은 홈 상대
고도), `x/y` = 위경도 × 1e7, `z` = 고도(m)입니다.

**이륙과 착륙을 미션 항목으로 넣을지 (합의 필요).** 넣지 않는 것을 추천합니다. 위 흐름처럼
이륙은 `takeoff()`, 착륙은 `land()`로 두고 탐색 웨이포인트만 미션으로 보냅니다.

- `takeoff()`(고도 95%)와 `land()`(시동 해제 확인)의 완료 기준은 두 프로토콜에서 같고, 실제
  SITL에서 검증되었습니다(C2, C4).
- `MAV_CMD_NAV_TAKEOFF`를 미션에 넣으면 지상에서 AUTO로 시동해야 합니다. ArduCopter에서는
  `AUTO_OPTIONS` 설정이 필요하고, 이륙 완료 판정이 FC 안으로 들어갑니다.
- `NAV_LAND`·`RETURN_TO_LAUNCH`를 넣으면 Dispatcher의 `land` 명령과 실패 시 land → disarm
  처리가 중복됩니다.

### 비교

| 기준 | goto 순차 (현행, 두 프로토콜 공통) | Mission 업로드 (MAVLink만, AP_DDS는 goto 유지) |
|---|---|---|
| 두 프로토콜의 판정 기준 | 같은 코드 경로, 같은 기준(미들웨어 텔레메트리, 반경 3m, 같은 제한 시간) | MAVLink는 FC 판정(`MISSION_ITEM_REACHED`, `WPNAV_RADIUS` 기준), AP_DDS는 미들웨어 판정. 미들웨어가 3m 판정을 병행해도 비행 방식이 다름. ArduCopter 4.1 이후 AUTO는 중간 웨이포인트에서 멈추지 않고 곡선으로 통과하는 것으로 알려져 있음(SITL 실측 필요). 구간 시간 차이가 프로토콜 차이인지 실행 방식 차이인지 분리되지 않음 |
| 웨이포인트 수 증가: 명령 수 | N개 `SET_POSITION_TARGET_GLOBAL_INT`를 비행 중에 하나씩 보냄. `DroneCommand`·tracker N개 | 비행 전 핸드셰이크 약 2N+3개 메시지를 한 번 주고받음. 비행 중 지상 송신 없음. 기체의 미션 저장 한도 확인 필요(1000m 구역, swath 10m면 1,500개) |
| 웨이포인트 수 증가: 지연 | 웨이포인트마다 목표 근처에서 감속하고, 도달 판정(텔레메트리 약 4Hz) 뒤 다음 목표를 보냄. 웨이포인트당 추가 지연은 미측정. 441개에서 1초씩이면 약 7분 | 웨이포인트 사이 지상 왕복 없음. 업로드 시간은 로컬 UDP라 짧을 것으로 예상하지만 미측정 |
| 실패 판정 단위 | 구간 하나. 어느 웨이포인트에서 실패했는지 바로 앎 | 업로드 전체(실패하면 비행 전 거부) + 진행 정체(`MISSION_CURRENT` seq가 정해진 시간 안에 바뀌지 않음) |
| 구간별 제한 시간 | `max(60초, 거리 ÷ 2.0m/s)`, 초과 시 `WAYPOINT_TIMEOUT` | 같은 공식을 seq 전환 시점 기준으로 적용할 수 있음. 시간 초과 시 AUTO를 벗어나는 처리를 비상 정지 경로에 추가해야 함 |
| 통신 단절 | GUIDED는 마지막 목표에서 제자리 비행(K5와 같은 상태). 그 뒤 동작은 FC failsafe 파라미터(`FS_GCS_ENABLE` 등)를 따름. 현재 SITL 값은 확인하지 않음 | GCS failsafe가 꺼져 있거나 AUTO 계속 옵션이면 FC가 업로드된 경로를 끝까지 비행한 뒤 마지막 지점에서 대기(마지막 항목이 RTL·LAND가 아닐 때). 탐색은 계속되지만 그동안 미들웨어는 상태를 모름. 통신 악화 시 귀환(장애 시나리오 1순위)은 두 방식 모두 failsafe 설정으로 정해짐 |
| 전달 신뢰성 | MAVLink `goto()`는 1회 송신, 응답·재전송 없음. 손실은 `WAYPOINT_TIMEOUT`으로만 드러남. AP_DDS는 2Hz 재발행 | FC가 빠진 항목을 다시 요청하고 `MISSION_ACK`로 수락을 알림. `accepted`를 실제 프로토콜 응답으로 기록할 수 있음 |
| K1(블로킹 중 `disconnected`) | goto 대기 중 Dispatcher가 `get_telemetry()`를 계속 부르므로 goto 구간은 K1 대상이 아님(실측 A, B, C에서도 land 구간에서만 발생) | 진행 확인을 미션 종료까지 반환하지 않는 블로킹 메서드로 만들면 탐색 전체가 K1 구간이 됨. 비블로킹 진행 조회와 `get_telemetry()` 폴링이면 현행과 같음. `_drain()`이 종류별 최신값만 남기므로 1회성 `MISSION_ITEM_REACHED`는 폴링 사이에 덮어써질 수 있음. 진행은 `MISSION_CURRENT` 기준으로 보거나 어댑터가 도달 이벤트를 따로 쌓아야 함. 업로드 중에는 텔레메트리를 읽지 않음 |
| 재배정(9~10주차) | 새 goto로 진행 중 목표를 덮어쓰면 바로 교체됨. 두 프로토콜이 같은 방식 | AUTO를 멈추고(GUIDED 전환) goto로 교체하거나, 새 미션을 다시 올리고 현재 항목을 지정(`MAV_CMD_DO_SET_MISSION_CURRENT`)해야 함. AP_DDS와 경로가 달라 재배정 로직이 프로토콜별로 갈림 |
| 잔여 상태 | 기체에 남는 상태 없음 | 미션이 FC에 저장되어 다음 실행까지 남음. 업로드 전 삭제(0단계)를 빼먹고 AUTO로 전환하면 이전 경로를 비행할 수 있음 |
| 테스트 | 현행 pytest(FakeDrone)와 증거 스크립트 A, B, C. MAVLink 메시지 캡처(F6)만 남음 | 핸드셰이크 단위 테스트(가짜 MAVLink 상대: 재요청, 거부, 시간 초과), FakeDrone 미션 흉내, SITL에서 MAVProxy `wp list`로 업로드 내용 대조, `MISSION_ITEM_INT`·`MISSION_CURRENT` 캡처, 링크를 끊고 계속 비행하는지 확인 |

두 방식에 공통인 문제: 웨이포인트마다 `DroneCommand`를 두면 `get_mission_status` 응답과
EventLog가 웨이포인트 수에 비례해 커집니다(500m 구역이면 드론당 명령 444개). 전달 방식과 별개로
상태 보고 요약이 필요합니다.

### 변경 범위

goto 순차 유지: 이번 결정으로 바뀌는 코드는 없습니다. 결정과 별개로 남는 일은 MAVLink 제어
메시지 캡처(F6), K1 대응([검증 상태 4-1](verification-status.md#4-1-k1-블로킹-명령-중-연결-판정)),
`return_home()`입니다.

Mission 업로드 채택 시:

| 위치 | 담당 | 변경 |
|---|---|---|
| `services/simulation/app/mavlink_drone.py` | 시뮬레이션 | 미션 업로드(0~4단계, 재요청·거부·시간 초과 처리, 핸드셰이크 중 `_drain()` 금지), 미션 시작(AUTO 전환, 시작 항목 지정), 비블로킹 진행 조회(현재 seq, 도달 seq, 모드) |
| `services/simulation/app/drone_interface.py`, `INTERFACE.md` | 시뮬레이션 | 선택 기능으로 정의. 추상 메서드로 넣으면 `Ros2Drone`이 구현할 수 없음 |
| `services/simulation/app/ros2_drone.py` | 시뮬레이션 | 변경 없음 |
| `app/adapters/drone_interface.py` | 미들웨어 | `DroneInterfaceClient`에 선택 메서드 추가, 연속 goto를 미션 요청 하나로 묶는 변환 |
| `runtime/registry.py` `_capabilities_for` | 미들웨어 | 클라이언트에 메서드가 있으면 미션 지원 표시(`return_home`과 같은 방식) |
| `runtime/dispatcher.py` | 미들웨어 | MAVLink 묶음 실행 경로: 업로드 시 각 goto `sent`, `MISSION_ACK`에 `accepted`, seq 전환과 3m 판정으로 `executing`·`succeeded`, 진행 정체 시간 초과, 비상 정지 시 AUTO 해제 |
| `CommandResult`, `DroneCommand` 계약 | 미들웨어 | goto 명령 단위를 유지하면 명령 종류와 전이 규칙은 그대로(`sent → accepted`는 이미 허용). 오류 코드 추가(예: `MAVLINK_MISSION_UPLOAD_FAILED`, `MISSION_STALLED`). 위 명령 결과 상태 표 갱신 |
| `runtime/fake_drone.py` | 미들웨어 | `mavlink` flavor에만 미션 메서드 흉내. `ap_dds` flavor는 그대로 |
| `scripts/collect_integration_evidence.py` | 미들웨어 | 업로드 이벤트와 실행 방식 기록, 항목별 전이 집계 |
| `docs/contracts/mcp-tools.md`, `verification-status.md` | 미들웨어 | 계획 → 명령 표, 검증 항목 추가 |

### 추천안

**goto 순차를 두 프로토콜 공통 방식으로 유지하고, MAVLink Mission 업로드는 이번 학기 구현
범위에서 보류합니다 (합의 필요).**

1. 이 프로젝트의 핵심 주장은 "미들웨어가 프로토콜을 몰라도 같은 판정으로 이기종 드론을 다룬다"이고,
   프로토콜 간 KPI를 비교합니다. Mission을 MAVLink에만 쓰면 완료 기준과 비행 방식이 프로토콜마다
   달라져 비교가 흐려집니다. AP_DDS에는 대응하는 Mission 경로가 없습니다.
2. 9~10주차 재배정과 Fallback은 진행 중 목표를 바꿀 수 있어야 합니다. goto는 두 프로토콜이 같은
   방법으로 교체되지만, Mission은 AUTO 중단이나 재업로드가 필요하고 경로가 프로토콜별로 갈립니다.
3. 현행 방식은 실제 SITL에서 세 실행 모두 성공했습니다(F2~F4). Mission은 시뮬레이션 어댑터,
   Dispatcher, FakeDrone, 증거 스크립트를 함께 바꿔야 하고 다시 검증해야 합니다.
4. Mission의 장점(웨이포인트 사이 지연 없음, 통신 단절 중 계속 비행, 전달 확인)은 실제로
   있지만 아직 측정되지 않았습니다. 통신 단절 대응은 두 방식 모두 FC failsafe 설정으로 정해집니다.

재검토 조건 (합의 필요): 아래 중 하나가 확인되면 Mission을 다시 검토합니다.

- 같은 coverage 경로를 SITL에서 MAVProxy `wp load` + AUTO와 현행 goto로 각각 비행해, 웨이포인트당
  지연이 탐색 시간 KPI에 의미 있는 차이를 만든다(코드 변경 없이 측정 가능).
- 통신 단절 중에도 탐색을 계속해야 한다는 요구가 장애 시나리오에 추가된다.
- 구역이 커져 상태 보고와 EventLog 크기 문제가 명령 단위로는 해결되지 않는다.

### 결정 항목

| # | 항목 | 추천 | 상태 |
|---|---|---|---|
| D1 | 목표점 전달 방식 | goto 순차 유지, Mission 보류 | 합의 필요 |
| D2 | Mission 채택 시 이륙·착륙을 미션 항목에 넣을지 | 넣지 않음 (`takeoff()`, `land()` 유지) | 합의 필요 |
| D3 | 통신 단절 시 FC failsafe 파라미터(`FS_GCS_ENABLE` 등) | 두 방식 공통. 현재 SITL 값 확인 후 결정 | 합의 필요 |
| D4 | `MavlinkDrone.goto()` 재전송 | `Ros2Drone`처럼 목표를 주기적으로 다시 보낼지 검토 | 합의 필요 |
| D5 | Mission 재검토 조건 | 위 재검토 조건 | 합의 필요 |

### 계획서와 다르게 갈 경우의 근거

신청계획서는 MAVLink Adapter가 목표점 목록을 Mission으로 변환하도록 했으나, 구현 단계에서
두 프로토콜 모두 전역 위치 목표를 순차 전달하고 미들웨어가 텔레메트리로 도달을 판정하는 방식으로
통일했다. AP_DDS에는 Mission에 대응하는 경로가 없어, MAVLink만 Mission을 쓰면 같은 명령의 완료
기준(비행 제어기 판정과 미들웨어 판정)과 비행 방식이 프로토콜마다 달라진다. 그러면 프로토콜에
의존하지 않는 공통 제어라는 미들웨어의 목적과 프로토콜 간 성능 비교의 공정성이 함께 약해진다.
또한 이후 단계의 재배정과 장애 대응은 진행 중인 목표를 즉시 바꿀 수 있어야 하는데, 순차 목표
방식은 두 프로토콜에서 같은 방법으로 목표를 교체할 수 있다. 이 방식으로 실제 SITL에서 MAVLink 1대,
AP_DDS 1대, 2대 동시 실행이 모두 성공했다. Mission 방식의 장점인 웨이포인트 간 지연 감소와 통신
단절 중 자율 비행은 같은 경로의 비행 시간 비교 실험으로 효과를 측정한 뒤 다시 검토한다.

## 제외 범위

- MAVLink 연결·수신 루프 중복 구현
- 여러 드론의 binding을 저장하는 Registry
- 전체 임무 Dispatcher
- AirSim·SITL 환경 구성
- AP_DDS 변환
