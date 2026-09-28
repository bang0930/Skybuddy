# AP_DDS Adapter 계약

이 문서는 AP_DDS의 ROS 2 토픽·서비스 데이터를 미들웨어 공통 계약으로 변환하는
프로토타입의 경계를 정의합니다. 실행 가능한 구현은
`services/middleware/app/adapters/ap_dds.py`입니다.

## 책임 경계

- simulation의 `Ros2Drone`이 ROS 2 노드, QoS, 구독, 서비스 클라이언트를 소유합니다.
- middleware adapter는 ROS 2 패키지를 직접 import하거나 별도 노드를 만들지 않습니다.
- middleware는 평탄화된 토픽 값과 주입된 `Ros2Drone` 호환 클라이언트만 사용합니다.
- `drone_id`와 ROS 2 namespace의 연결은 `ApDdsBinding`에 명시합니다.

따라서 ROS 2가 없는 개발 환경에서도 변환 계약을 테스트할 수 있으며, ROS 2 연결과
실제 비행 검증은 simulation 계층에서 별도로 수행합니다.

## 상태 변환

| ROS 2 입력 | 공통 `DroneState` | 변환 |
|---|---|---|
| `GeoPoseStamped.position.latitude/longitude` | `position.latitude/longitude` | WGS84 degree 유지 |
| `GeoPoseStamped.position.altitude` | `position.altitude_m` | AMSL 유지 또는 홈 AMSL 차감 |
| `TwistStamped.linear.x/y/z` | `velocity_ned_m_s` | ENU `(east, north, up)` → NED `(north, east, down)` |
| `GeoPoseStamped.orientation` | `heading_deg` | ENU yaw → 진북 기준 시계방향 방위 |
| `NavSatFix.status.status` | `gps_fix_type` | fix 있음 `3`, 없음 `1` |
| `NavSatFix.position_covariance[0]` | `gps_eph_m` | 음수가 아닌 분산의 제곱근 |
| `BatteryState.voltage` | `battery_voltage_v` | V 유지 |
| `BatteryState.percentage` | `battery_percent` | `0..1` → `0..100` |

ROS sensor message에서 미상 값을 나타내는 `NaN`은 `None`으로 변환합니다. 일부 값만
도착한 position, velocity, quaternion 그룹도 잘못 조합하지 않고 그룹 전체를 `None`으로
둡니다. 빈 문자열은 이전 CSV 경계와의 호환 목적으로만 `None`으로 정규화합니다.

### ENU → NED

ROS REP-103의 ENU 속도는 아래처럼 변환합니다.

```text
north = enu.y
east  = enu.x
down  = -enu.z
```

ROS ENU yaw는 동쪽 0도·반시계 방향이고 공통 heading은 진북 0도·시계 방향이므로
`heading = (90 - yaw_enu_deg) % 360`을 사용합니다. quaternion은 계산 전에 정규화하며
길이가 0이면 입력 오류로 처리합니다.

### AMSL과 홈 상대고도

`GeoPoseStamped` 고도는 AMSL입니다.

- 홈 AMSL을 알고 있으면 `altitude_amsl_m - home_altitude_amsl_m`로 변환하고
  `altitude_reference=home_relative`를 기록합니다.
- 홈 AMSL을 모르면 원본 고도를 보존하고 `altitude_reference=msl`을 기록합니다.

기준점을 모르는 AMSL을 상대고도로 가장하지 않습니다. simulation의 `Ros2Drone`은 연결
직후 첫 geopose의 AMSL을 홈 기준점으로 저장합니다.

### 미지원 필드

현재 AP_DDS 입력에서 제공하지 않는 아래 필드는 capabilities에서 제외하며 값은 `None`입니다.

- `satellites_visible`
- `vibration`
- `clipping`

따라서 `DroneState.telemetry_availability()`은 이 필드를
`unsupported`로 반환합니다. capabilities에는 있지만 이번 샘플에 값이 없는 필드는
`temporarily_unavailable`입니다.

### 연결 상태

최신 토픽 수신 후 경과 시간이 `topic_timeout_s` 이하이면 `connected`, 초과하거나 수신
기록이 없으면 `disconnected`입니다. 기본 timeout은 3초입니다.

## 이륙 명령 변환

이번 프로토타입은 공통 `takeoff` 한 종류를 AP_DDS
`/{namespace}/experimental/takeoff` 서비스 요청으로 변환합니다.

- command protocol이 `ap_dds`여야 합니다.
- command의 `drone_id`가 namespace binding과 일치해야 합니다.
- AP_DDS Takeoff의 `alt`는 홈 상대고도이므로 `altitude_reference=home_relative`만 허용합니다.
- 다른 명령 및 MSL 이륙 요청은 묵시적으로 바꾸지 않고 미지원 오류로 거부합니다.

`ApDdsTakeoffRequest.invoke()`는 주입된 `Ros2Drone.takeoff(altitude_m)` 호환 메서드를
호출합니다. ROS 2 클라이언트나 simulation 구현을 middleware에 복제하지 않습니다.

## 명령 결과 생명주기

AP_DDS 서비스 호출과 실제 기체 동작 완료는 별개의 상태입니다.

```text
pending -> sent -> accepted -> executing -> succeeded
                   |              |
                   +-> failed     +-> timed_out
```

- `sent`: 서비스 요청을 전송함
- `accepted`: 서비스 응답의 `status`가 참임
- `executing`: 텔레메트리로 실제 동작 시작을 확인함
- `succeeded`: 텔레메트리로 목표 고도 도달 등 완료 조건을 확인함
- `failed`: 서비스가 요청을 거부함
- `timed_out`: 서비스 응답 또는 동작 완료가 각각의 기한을 넘김

서비스 수락만으로 `succeeded`를 만들지 않습니다. 서비스 응답 timeout은
`AP_DDS_SERVICE_TIMEOUT`, 완료 확인 timeout은 `AP_DDS_COMPLETION_TIMEOUT`으로 구분합니다.

현재 simulation의 `Ros2Drone.takeoff()`는 내부에서 서비스 수락과 목표 고도 도달을 모두
처리한 뒤 반환합니다. 이 프로토타입의 세분화된 lifecycle은 향후 서비스 응답 hook 또는
dispatcher가 연결할 계약이며, 현재는 Mock으로 검증합니다.

## 검증 범위와 환경 제약

### 현재 완료된 Mock 검증

- AP_DDS 샘플 → `DroneState`
- ENU/NED 축 및 부호 변환
- ENU quaternion → 공통 heading
- AMSL 유지 및 홈 상대고도 변환
- 정상, 부분 누락, 미지원, NaN 입력
- namespace binding과 잘못된 수치 입력 거부
- 서비스 수락·거부 및 서비스/완료 timeout 구분
- 서비스 수락과 실제 완료 상태 분리

### 실제 환경 검증

이번 변경에서는 수행하지 않았습니다. 실제 검증에는 다음 환경이 필요합니다.

- ROS 2 Humble
- ArduPilot 4.7.1 이상, DDS 활성화 빌드
- `ardupilot_msgs`가 설치된 workspace
- Micro XRCE-DDS Agent
- ArduPilot 토픽과 맞는 BEST_EFFORT subscription QoS

실제 연결·비행 절차는 `services/simulation/README.md`와
`services/simulation/INTERFACE.md`를 따릅니다.

## 테스트

저장소 루트에서 실행합니다.

```bash
PYTHONPATH=services/middleware pytest -q services/middleware/tests/test_ap_dds_adapter.py
```

테스트는 ROS 2 설치 없이 실행되며 실제 ROS 2 연결 성공을 의미하지 않습니다.
