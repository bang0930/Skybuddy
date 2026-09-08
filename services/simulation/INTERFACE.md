# SkyBuddy 시뮬레이션 파트 인터페이스 명세

작성: 이태우 (AirSim/ArduPilot 다중 SITL, MAVLink 및 AP_DDS 연동)
기준일: 2026-09-07
대상: 박병언(미들웨어), 배성열(LLM 오케스트레이션)

---

## 0. 현재 상태

**서로 다른 통신 프로토콜을 쓰는 드론 2대가 같은 AirSim 씬에서 동시에 임무를 수행하고 각자 KPI를 수집**하는 단계까지 완료했습니다.

| 항목 | 상태 |
|---|---|
| 2대 이상 동시 운용 | 완료 |
| MAVLink 드론 제어·상태 수집 | 완료 |
| AP_DDS(ROS 2) 드론 제어·상태 수집 | 완료 |
| 프로토콜 공통 추상화 | 완료 (`DroneInterface`) |
| 격자 탐색 / 구역 분할 / 자동 복귀 | 완료 |
| **미들웨어 연동** | **미완 — 이 문서의 목적** |
| 탐색 커버리지 지표 | 미완 |
| 장애 상황 재현 | 미완 |

두 드론 모두 ArduPilot을 쓰고 **통신 인터페이스만 다릅니다.** 기체·펌웨어가 아니라 프로토콜이 이기종입니다.

---

## 0-1. 환경 버전

아래 조합에서 검증했습니다. 미들웨어 쪽 라이브러리 버전을 맞출 때 참고하십시오.

| 구성 요소 | 버전 | 비고 |
|---|---|---|
| OS | WSL2 Ubuntu 22.04.5 LTS | AirSim만 Windows에서 실행 |
| Python | 3.10.12 | |
| ArduPilot (drone1) | ArduCopter 4.3.0 | MAVLink 전용 |
| ArduPilot (drone2) | ArduCopter 4.7.1 | `--enable-DDS`로 빌드 |
| MAVLink 라이브러리 | pymavlink | `requirements.txt` |
| ROS 2 | Humble | ArduPilot이 공식 지원하는 유일한 배포판 |
| ROS 2 메시지 | `ardupilot_msgs` | ArduPilot 4.7.1 소스에서 colcon 빌드 |
| DDS Agent | micro-XRCE-DDS Agent | UDP 2019 |
| IDL 코드 생성기 | ardupilot 포크 `v4.7.0` | eProsima 원본 아님 |
| 시뮬레이터 | AirSim (Blocks / LandscapeMountains) | 패키지 빌드, 버전 고정 안 함 |

**버전에 관해 반드시 알아야 할 것**

- **AP_DDS의 이륙 서비스는 ArduPilot 4.6 이상에만 있습니다.** 4.5에는 서비스가
  `arm_motors`와 `mode_switch` 둘뿐이라 위치 명령만으로는 이륙시킬 수 없습니다.
- **코드 생성기 버전이 ArduPilot 버전과 맞아야 합니다.** 구버전 `microxrceddsgen`은
  IDL 안의 상수를 생성하지 않아 빌드가 실패합니다. 4.7 이상은 ardupilot 포크의
  `v4.7.0`을 써야 합니다.
- **ROS 2는 Humble만 지원합니다.** ArduPilot 공식 문서 기준이며 다른 배포판은
  메시지 직렬화가 어긋날 수 있습니다.
- 현재 두 드론의 ArduPilot 버전(4.3 / 4.7.1)이 다릅니다. 이기종 프로토콜 주장에는
  영향이 없지만 **성능 비교 수치에는 영향이 있습니다** (3-3 참고). 통일 예정입니다.

환경 구축 절차 전체는 `docs/engineering-notes.md`에 있습니다.

---

## 1. 구조

핵심 원칙은 하나입니다.

> **상위 계층은 드론이 어떤 프로토콜을 쓰는지 몰라도 된다.**

```
kpi_mission_multi.py      오케스트레이션 — 몇 대를, 어느 구역·고도로 띄울지
        |                 (← 최종적으로 미들웨어가 대체할 자리)
kpi_mission.py            임무 — 경로 생성, 실행, KPI 측정
        |
DroneInterface            공통 계약 (drone_interface.py)
        |
   +----+----+
   |         |
MavlinkDrone  Ros2Drone
pymavlink     rclpy
   |            |
MAVProxy    XRCE-DDS Agent
UDP 14561     UDP 2019
   |            |
ArduCopter   ArduCopter
  -I1          -I2
   |            |
   +--- AirSim -+
```

`kpi_mission.py`에서 프로토콜을 아는 곳은 `make_drone()` 팩토리 한 곳뿐입니다.
미션·도달판정·KPI 계산·CSV 저장 로직은 두 프로토콜에 대해 **완전히 동일한 코드**가 돕니다.
`tests/test_adapters.py`가 이를 자동 검사합니다 (`run_mission()` 본문에 프로토콜 관련 단어가 있으면 실패).

### AP_DDS는 별도 프로세스가 아닙니다

AP_DDS는 ArduPilot 펌웨어 안에 포함된 XRCE-DDS 클라이언트 라이브러리입니다.
ArduPilot이 임베디드 보드에서도 동작해야 해서 완전한 DDS 스택 대신 경량 XRCE-DDS를 쓰고,
`MicroXRCEAgent`(별도 프로세스)가 이를 일반 DDS로 번역해 ROS 2가 평범한 노드로 보게 합니다.

---

## 2. 박병언(미들웨어)에게

### 2-1. 제어 인터페이스

`DroneInterface`의 7개 메서드입니다. 어느 프로토콜이든 시그니처가 동일합니다.

| 메서드 | 설명 |
|---|---|
| `connect(timeout=30)` | 연결. 성공 시 자기 자신을 반환 |
| `takeoff(altitude_m)` | 시동 → 이륙 → 목표 고도 도달까지. 실패 시 예외 |
| `goto(lat, lon, alt_m)` | 목표 지점 지정. 도달을 기다리지 않음 |
| `land(timeout=60)` | 착륙 + 시동 꺼짐 확인. bool 반환 |
| `disarm(timeout=5)` | 강제 시동 해제 (안전장치). bool 반환 |
| `get_telemetry(timeout=2)` | 현재 상태 1건. dict 또는 None |
| `close()` | 연결 정리 |

`distance_m(lat1, lon1, lat2, lon2)`는 정적 메서드로 제공됩니다.

### 2-2. 공통 텔레메트리 스키마

`get_telemetry()`가 반환하는 dict입니다. **필드는 항상 전부 존재하고, 값이 없으면 빈 문자열 `""`입니다.**

| 필드 | 타입 | 단위 / 기준 | MAVLink | AP_DDS |
|---|---|---|:---:|:---:|
| `lat` | float | WGS84 위도(도) | O | O |
| `lon` | float | WGS84 경도(도) | O | O |
| `relative_alt_m` | float | **홈 기준 상대 고도(m)** | O | O |
| `vx`, `vy`, `vz` | float | NED 속도(m/s), vz는 아래가 양수 | O | O |
| `heading_deg` | float | 진북 기준 시계방향 0~360 | O | O |
| `gps_fix_type` | int | 3 = 3D Fix 이상이 정상 | O | O |
| `satellites_visible` | int | 위성 수 | O | **X** |
| `gps_eph` | float | 수평 위치 정밀도(m) | O | O |
| `battery_voltage_v` | float | 전압(V) | O | O |
| `battery_remaining_pct` | float | 잔량(%) | O | O |
| `vibration_x/y/z` | float | 진동. 30 이상이면 이상 | O | **X** |
| `clipping` | int | 가속도계 포화 횟수. 0이 정상 | O | **X** |
| `timestamp` | float | Unix epoch | O | O |

**X 표시는 구현 부족이 아니라 프로토콜 간 실제 기능 격차입니다.**
AP_DDS에는 해당 토픽이 존재하지 않습니다. 미들웨어는 이 필드들을 필수로 가정하면 안 됩니다.

**갱신 주기: 두 드론 모두 약 4Hz로 맞춰 제공합니다.**
DDS 원본은 약 28Hz까지 나오지만, 두 드론의 로그를 나란히 비교하기 위해 샘플링을 맞췄습니다.
더 높은 주기가 필요하면 `Ros2Drone(telemetry_interval_s=...)`로 조정 가능합니다.

### 2-3. 프로토콜 간 실측 차이

| 항목 | MAVLink | AP_DDS |
|---|---|---|
| 원본 갱신 주기 | 약 4Hz | 약 28Hz |
| 고도 원본 기준 | 홈 기준 AGL | **AMSL** (어댑터가 상대고도로 환산) |
| 진동·클리핑 지표 | 제공 | 없음 |
| 위성 수 | 제공 | 없음 |
| 이륙 명령 | `MAV_CMD_NAV_TAKEOFF` | `/ap/experimental/takeoff` 서비스 |
| 위치 명령 | `SET_POSITION_TARGET_GLOBAL_INT` | `/ap/cmd_gps_pose` 토픽 |

고도 기준 차이는 평지에서는 문제가 없지만 **산악 지형에서는 수십 m 차이로 벌어집니다.**
어댑터가 연결 시점의 AMSL을 홈 고도로 잡아 환산하므로 상위 계층은 신경 쓰지 않아도 됩니다.

### 2-4. 반드시 알아야 할 제약 6가지

**① `goto()`는 fire-and-forget**
목표만 던지고 즉시 리턴합니다. 도달 여부는 텔레메트리를 폴링해 판단해야 합니다.
현재 기준: 목표 반경 **3m** 안에 들어오면 도달로 간주.

**② 이륙은 즉시 되지 않음**
모드 전환과 모터 스풀업에 시간이 걸리고, 부팅 직후에는 EKF가 위치를 확정하지 못해 실패할 수 있습니다.
MAVLink 쪽은 이륙 명령이 거부되면 20초간 재시도합니다.

**③ ⚠️ AirSim은 lock-step — Fallback 설계에 직접 영향**
`settings.json`에 정의된 드론의 SITL이 **하나라도 끊기면 시뮬레이션 시간 자체가 멈춥니다.**
따라서 "드론 이탈" 장애를 SITL 프로세스 종료로 재현하면 **시뮬 전체가 정지**합니다.
→ **통신 링크만 끊는 방식으로 모사해야 합니다.**

**④ arming check를 끈 상태로 운용**
AirSim의 시뮬레이션 GPS가 ArduPilot prearm 기준을 통과하지 못합니다.
`ARMING_SKIPCHK -1`(4.7 기준, 모든 검사 생략)로 운용합니다. 실기체 전환 시 달라지는 부분입니다.

**⑤ 배터리는 항상 100%**
SITL에는 소모 모델이 없습니다. 배터리 기반 재배정 로직을 검증하려면
텔레메트리에 값을 인위적으로 주입하는 기능이 별도로 필요합니다. (요청 시 구현 가능)

**⑥ DDS 드론은 Agent가 먼저 떠 있어야 함**
`MicroXRCEAgent`가 실행 중이 아니면 ArduPilot의 DDS 클라이언트가 붙지 못합니다.
기동 순서: AirSim → Agent → SITL.

### 2-5. 아직 정하지 못한 것

- [ ] **연결 방식** — 파이썬 모듈 직접 호출 / REST / WebSocket / gRPC
- [ ] **명령 단위** — 웨이포인트 단위 / 구역 단위 / 미션 단위
- [ ] 좌표계를 위경도로 통일할지 (현재 시뮬 내부는 홈 기준 오프셋도 사용)
- [ ] 역할 경계 — 이태우 = "프로토콜 → 공통 스키마", 박병언 = "공통 스키마 → MCP Context"

**현재 구조상의 제약**: `run_mission()`이 연결·이륙·순회·귀환·착륙·CSV저장을 한 함수에 담고 있어
밖에서 "웨이포인트 하나만 실행" 같은 요청을 받을 수 없습니다.
연결 방식이 무엇으로 정해지든 **명령 단위 분해가 선행되어야 합니다.** 이 작업은 다음 차례입니다.

---

## 3. 배성열(LLM)에게

### 3-1. LLM 출력이 최종적으로 변환되는 형태

```
drone_id + 웨이포인트 목록 [(위도, 경도), ...] + 순항 고도
```

구역 분할 결과가 이 형태로만 떨어지면 그대로 실행됩니다.

### 3-2. 물리적 제약 — 이를 벗어나면 실행 불가능한 명령이 됩니다

| 항목 | 값 | 의미 |
|---|---|---|
| 드론 수 | 2대 | drone1(MAVLink), drone2(AP_DDS) |
| 출발 위치 간격 | 6m | 두 기체 스폰 간격 |
| **최소 수직 분리** | **10m 이상** | 구역이 겹칠 때 필수 |
| 이동 속도 | 약 2.7~4.8 m/s | 기체별로 다름 (아래 참고) |
| 도달 판정 반경 | 3m | 이보다 촘촘한 웨이포인트는 무의미 |
| 웨이포인트 타임아웃 | 60초 | 초과 시 실패 처리 |
| **최소 안전 고도** | **30m 이상** | 저고도(10~20m)는 지형지물 충돌 발생 |

> ⚠️ **가장 중요**: 두 드론에게 겹치는 구역을 같은 고도로 배정하면 충돌합니다.
> **구역을 나누거나 고도를 나누거나, 둘 중 하나는 반드시** 해야 합니다.

### 3-3. 비교 기준선(baseline)

알고리즘 고정 분할(20×20m 정사각형 5개 웨이포인트)로 측정한 값입니다.
**두 대를 동시에 띄운 상태에서 측정했습니다.**

| 지표 | drone1 (MAVLink) | drone2 (AP_DDS) |
|---|---|---|
| 웨이포인트 도달률 | 5/5 | 5/5 |
| 구간 이동 시간 | 7.0~7.3초 | 4.2~4.7초 |
| 경로 효율 | 0.93~1.00 (평균 0.957) | 0.73~0.92 (평균 0.863) |
| 총 소요 | 112.4초 | 86.5초 |
| 총 이동거리 | 84.0m | 97.5m |
| 진동(vibration_z) | 0.046~0.057 | (지표 없음) |

> **경로 효율 정의**: 실제로 도달한 지점까지의 직선거리 ÷ 실제 이동 경로 길이.
> 목표 도달 실패 시에도 1을 넘지 않도록 이렇게 정의했습니다.

> ⚠️ **이 속도·효율 차이를 프로토콜 탓으로 해석하면 안 됩니다.**
> 측정 시점에 두 기체의 ArduPilot 버전(4.3 / 4.7.1)과 자세 루프 튜닝이 달랐습니다.
> 버전을 통일한 뒤 재측정할 예정이며, 그 전까지 이 수치는 **각 드론의 개별 기준선**으로만 쓰십시오.

### 3-4. 현재 재현 불가능한 장애 상황

| 장애 상황 | 상태 |
|---|---|
| 배터리 부족 | 불가 — 항상 100%. 값 주입 기능 필요 |
| GPS 저하 | 불가 — 항상 최상급 fix. 인위적 주입 필요 |
| 통신 지연 | 가능 — 링크 끊기로 모사 |
| 드론 이탈 | 주의 — SITL 종료 시 **시뮬 전체 정지** (lock-step) |

→ 현재 상태로는 "배터리 부족으로 임무 재배정" 시나리오를 **검증할 수 없습니다.**
필요하다면 텔레메트리 값 주입 기능을 만들겠습니다.

### 3-5. 아직 정하지 못한 것

- [ ] **탐색 구역 커버리지 지표 정의** (셀 크기, "훑었다"의 판정 반경)
      → 계획서 핵심 평가항목인데 현재 없음. 중복 탐색 감소를 증명하려면 필수
- [ ] 장애 상황 값 주입 기능이 필요한지
- [ ] LLM 출력에 고도를 포함할지, 시뮬에서 자동 분리할지

---

## 4. 실행 방법 (참고)

팀원 대부분이 macOS라 직접 실행은 어렵습니다. 구조 참고용으로만 기재합니다.
AirSim은 Windows, ArduPilot SITL과 ROS 2는 WSL2(Ubuntu 22.04)에서 동작합니다.

```bash
# 1) Windows: 2기체용 settings.json 배포 후 AirSim 실행
# 2) WSL: 환경 기동 (Agent + SITL 2기 + 준비 대기 + 튜닝)
bash scripts/start_hetero_sim.sh

# 3) WSL: 이기종 2대 동시 임무
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
python3 app/kpi_mission_multi.py
```

결과물:

- `kpi_summary_drone1.csv` / `kpi_summary_drone2.csv` — 웨이포인트별 지표
- `kpi_log_drone1.csv` / `kpi_log_drone2.csv` — 원시 텔레메트리

실제 출력 예시는 `samples/` 폴더에 있습니다.

시뮬레이터 없이 어댑터 계약만 검증하려면:

```bash
python3 tests/test_adapters.py
```

환경 구축 절차와 알려진 문제(조용히 실패하는 함정 포함)는 `docs/engineering-notes.md`를 참고하십시오.
