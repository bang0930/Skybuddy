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

## 제외 범위

- MAVLink 연결·수신 루프 중복 구현
- 여러 드론의 binding을 저장하는 Registry
- 전체 임무 Dispatcher
- AirSim·SITL 환경 구성
- AP_DDS 변환
