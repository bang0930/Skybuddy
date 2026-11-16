# SkyBuddy 시뮬레이션 파트 인터페이스 명세

작성: 이태우 (AirSim/ArduPilot 다중 SITL, MAVLink 및 AP_DDS 연동)
기준일: 2026-09-07 (최종 갱신 2026-11-09, 11주차 장애 주입 2-9 — 배터리 연속 감소·링크 품질·텔레메트리 기록 반영)
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
| 격자 탐색 / 구역 오프셋 분리 / 자동 복귀 | 완료 |
| 구역 boundary → 웨이포인트 생성 | #26에서 진행 중 (3-1) |
| **미들웨어 연동** | **#24에서 진행 중.** 시뮬레이션 쪽 대비는 2-1-2, 남은 항목은 2-6 |
| 탐색 커버리지 지표 | 미완 |
| 장애 상황 재현 | 주입 구현·SITL 비행 검증 (2-9): 통신 두절(시간·거리, 링크 품질), 웨이포인트 미도달, 배터리 연속 감소 |

두 드론 모두 ArduPilot을 쓰고 **통신 인터페이스가 다릅니다.**

> ⚠️ **이기종 프로토콜 대상 변경은 교학팀 승인 대기 중입니다.**
> 계획서에 명시된 ISO/IEC 4005가 시뮬레이션에서 구현 불가능하다고 판단해 AP_DDS로
> 대체했고(1주차), 교학팀에 변경 가능 여부를 문의한 상태입니다.
> 불가 통보 시 대안을 다시 검토해야 하므로, 이 문서의 내용은 승인 전까지 잠정입니다.

**이기종의 정의는 두 갈래입니다** (회의 정리).

| 축 | 내용 | 현재 상태 |
|---|---|---|
| 프로토콜 | 동일 펌웨어에서 2개 프로토콜 동시 운영 | 검증 완료 |
| 펌웨어 | 서로 다른 펌웨어 운영 | 현재 4.3 / 4.7.1 로 우연히 충족 |

어느 축을 주장으로 쓸지에 따라 버전 통일 여부가 갈립니다(3-3 참고).

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
| `takeoff(altitude_m)` | 시동 → 이륙 → **목표 고도 도달까지** 기다린 뒤 반환. 실패 시 예외 |
| `goto(lat, lon, alt_m)` | 목표 지점 지정. 도달을 기다리지 않음 |
| `land(timeout=None)` | 착륙 + 시동 꺼짐 확인. bool 반환. timeout을 비우면 현재 고도로 정함(1m당 4초, 최소 60초). 그 시간이 지나도 내려가는 중이면 최대 300초 더 기다림 |
| `disarm(timeout=5)` | 강제 시동 해제 (안전장치). bool 반환 |
| `get_telemetry(timeout=2)` | 현재 상태 1건. `Telemetry` 또는 None |
| `close()` | 연결 정리 |

`distance_m(lat1, lon1, lat2, lon2)`는 정적 메서드로 제공됩니다.

**`takeoff()`의 완료 조건은 두 구현이 동일합니다.** 명령이 수락된 시점이 아니라
목표 고도의 95%에 도달한 시점에 반환합니다. 그래서 `takeoff()`가 돌아온 직후에
바로 `goto()`를 보내도 기체가 아직 지상 근처에 있는 상황이 생기지 않습니다.

### 2-1-1. 미션 실행 진입점

어댑터 위에 두 개의 진입점이 있습니다. 둘 다 `kpi_mission.py`에 있습니다.

| 함수 | 용도 |
|---|---|
| `run_mission(...)` | 연결 → 이륙 → 웨이포인트 순회 → 귀환 → 착륙 → CSV 저장까지 전 과정 |
| `execute_waypoint(drone, lat, lon, alt, ...)` | **웨이포인트 하나만** 실행하고 지표를 반환 |

웨이포인트 하나를 실행하고 그 구간 지표까지 받고 싶으면 `execute_waypoint()`를
직접 호출하면 됩니다. `run_mission()`은 이 함수를 반복 호출하는 구조입니다.
#24 Dispatcher는 이 함수 대신 `goto()`와 도달 판정을 직접 수행합니다.

`run_mission()`의 실패 처리는 미들웨어 Dispatcher와 같은 정책입니다.

| 상황 | 동작 |
|---|---|
| 어느 지점에서 실패하든 | `close()` 보장, 수집된 로그는 CSV로 남김 |
| 이륙 명령 **이후**의 실패 (상승 도중 시간 초과 포함) | 공중일 수 있으므로 **착륙부터** 시도, 안 되면 강제 disarm |
| 착륙(시동 꺼짐)을 확인하지 못함 | 경고로 끝내지 않고 **미션 실패**. 착륙 재시도 → 안 되면 강제 disarm |

이 정책은 `tests/test_mission_policy.py`가 가짜 드론으로 검사합니다.

### 2-1-2. 연결을 유지한 채 쓸 때 (미들웨어 Registry)

`run_mission()`은 실험용이라 매번 새로 연결하고 끝나면 닫습니다. 미들웨어 Registry(#24)는
반대로 **드론마다 한 번만 연결해 객체를 계속 들고 있으면서** 명령마다 메서드를 부릅니다.
어댑터는 이 방식에서도 동작하도록 맞춰 두었습니다.

| `run_mission()`으로 실험할 때는 드러나지 않던 문제 | 대응 |
|---|---|
| MAVLink는 수신 버퍼에 쌓인 메시지를 **오래된 것부터** 꺼낸다. 한참 쉬었다 읽으면 그때 위치가 나온다 | 읽기·명령 전에 버퍼를 비우고 최신값만 사용 |
| ArduPilot은 모드 변경에도 `COMMAND_ACK`를 보낸다. 시동·이륙 응답과 헷갈릴 수 있다 | 명령 번호가 맞는 응답만 인정 |
| AP_DDS 홈 고도를 연결 시점에만 잡으면, 다른 곳에 내렸다가 다시 뜰 때 기준점이 어긋난다 | 이륙할 때마다 시동 직전 고도로 다시 잡음 (ArduPilot도 시동 시 홈을 새로 정함) |
| 미들웨어는 `land()`를 인자 없이 부른다. 기본값이 고정 60초면 40m에서 출발한 기체도 다 내려오기 전에 끊긴다(실측) | 기본 대기 시간을 현재 고도로 계산 (`run_mission()`이 원래 넘기던 값과 같은 규칙) |
| 미들웨어 설정의 드론 ID는 `drone-02`처럼 하이픈을 쓴다. ROS 2 노드 이름에는 하이픈을 쓸 수 없어 AP_DDS 드론 연결이 실패한다 | 노드 이름을 만들 때 영문·숫자·밑줄 외의 문자는 밑줄로 바꿈 |
| AP_DDS 드론의 `close()`가 두 번 불리면, 같은 프로세스에서 다른 DDS 드론이 쓰는 ROS 2 런타임까지 종료된다 | 몇 번 불려도 정리는 한 번만 함 |
| `land()`는 시동이 꺼질 때까지 수십 초 블로킹한다. 그동안 위치를 안 읽으면 미들웨어가 샘플이 끊긴 것으로 보고 `disconnected`로 잘못 표시한다(K1. 2026-11-09 실측: MAVLink 약 55초, AP_DDS 약 35초) | 기다리는 동안에도 `get_telemetry()`로 위치를 읽어, 미들웨어 기록 장치에 샘플이 계속 들어가게 함 (`takeoff()`와 같은 방식) |
| `land()` 대기 시간은 **홈 기준 고도**로 정한다. 착륙 지점 지형이 홈보다 낮으면 그 시간 안에 다 내려오지 못한다. 2026-11-09 배터리 시나리오에서 홈보다 20m 넘게 낮은 호수 쪽에 강제 착륙한 drone-01이 60초 뒤에도 내려가는 중이었고, `land()`가 False를 돌려주자 미들웨어(`_land_or_disarm`)가 **공중에서 강제 disarm**했다 | 기본 시간이 지나도 아직 내려가는 중(vz > 0.15m/s)이면 계속 기다림 (`LandingWait`, 최대 300초 더). 내려가지도 않고 시동도 안 꺼지면 예전처럼 기본 시간에 False. 같은 시나리오 재실행에서 홈 기준 −27m 지면까지 내려와 `landed: true` 확인 |

실제 SITL에서 "연결 한 번으로 임무 두 번"이 되는지는 `scripts/session_check.py`로 확인합니다.

알아 둘 제약은 두 가지입니다.

- **어댑터는 스레드 안전하지 않습니다.** 한 드론에 대한 호출은 한 번에 하나씩이어야 하며,
  Registry의 `DroneHandle.exclusive()`가 이를 맞춰 주고 있습니다.
- **실제 모드로 돌리려면 미들웨어가 시뮬레이터와 같은 환경에 있어야 합니다.** Registry가 어댑터를
  직접 import하므로 그 프로세스에 pymavlink, ROS 2 Humble, `ardupilot_msgs`가 있어야 하고,
  SITL의 UDP 포트와 DDS에 직접 닿아야 합니다. 미들웨어만 AWS에 두고 시뮬레이터는 노트북에서
  돌리는 구성은 그대로는 연결되지 않습니다. Fake 모드는 어디서든 동작합니다.

### 2-2. 공통 텔레메트리 스키마

`get_telemetry()`가 반환하는 `Telemetry`(TypedDict)입니다.
**필드는 항상 전부 존재하고, 값이 없으면 `None`입니다.**

값이 없을 때 빈 문자열을 쓰면 같은 필드가 상황에 따라 숫자와 문자열 두 타입을 갖게 되어
Pydantic 모델로 바로 옮기기 어렵습니다. 그래서 어댑터 반환값은 항상 **숫자 아니면 `None`**이고,
빈 문자열로 바뀌는 것은 CSV로 직렬화할 때뿐입니다
(`drone_interface.telemetry_to_csv_row()`가 담당).

타입 정의는 `app/drone_interface.py`의 `Telemetry`를 그대로 참고하시면 됩니다.

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
| `link_quality_pct` | float | 링크 품질 0~100(%) | △ | △ |
| `timestamp` | float | Unix epoch | O | O |

**X 표시는 구현 부족이 아니라 프로토콜 간 실제 기능 격차입니다.**
AP_DDS에는 해당 토픽이 존재하지 않습니다. 미들웨어는 이 필드들을 필수로 가정하면 안 됩니다.

**△ `link_quality_pct`** (2026-11-09 추가): 시뮬레이션에서는 장애 주입이 채웁니다(장애 없음 = 100, 2-9-2).
실기체 어댑터는 아직 `None`입니다. 미들웨어 매퍼는 모르는 키를 무시하므로 추가돼도 기존 동작은 그대로입니다.

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
어댑터가 이륙할 때마다 시동 직전의 AMSL을 홈 고도로 잡아 환산하므로 상위 계층은 신경 쓰지 않아도 됩니다.

### 2-4. 반드시 알아야 할 제약 7가지

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
SITL에는 소모 모델이 없습니다(비행 내내 100%로 실측). 배터리 부족은 2-9의 주입 기능으로
만듭니다. 미들웨어가 받는 `battery_remaining_pct`가 주입값으로 바뀝니다.

**⑥ DDS 드론은 Agent가 먼저 떠 있어야 함**
`MicroXRCEAgent`가 실행 중이 아니면 ArduPilot의 DDS 클라이언트가 붙지 못합니다.
기동 순서: AirSim → Agent → SITL.

**⑦ 실행 경계에서도 최소 안전 고도를 검증합니다**
`run_mission()`은 연결 전에 `validate_altitude()`로 순항 고도를 확인하고,
**30m 미만이면 `ValueError`를 던지고 기체를 띄우지 않습니다.**
미들웨어에서도 검증하시겠지만, 실제로 명령을 보내는 쪽이 마지막 방어선이라 이중으로 둡니다.
상위 계층의 버그나 LLM의 잘못된 출력이 그대로 비행 명령이 되는 것을 막는 목적입니다.

### 2-5. 웨이포인트 입력 경로

미들웨어가 `SearchArea.boundary`로 계산한 경로를 그대로 주입할 수 있습니다.

```python
run_mission(connection_string='ros2:ap', drone_id='drone2', protocol='ros2',
            target_alt=30,
            waypoints=[(-35.36325, 149.16512), (-35.36307, 149.16534)])
```

`waypoints`는 **절대 위경도 (lat, lon) 튜플 리스트**입니다. 홈 기준 오프셋이 아닙니다.

내부 생성 경로와 외부 주입 경로는 **배타적**입니다.
`waypoints`를 주면서 `pattern` / `reverse` / `zone_offset_*`을 함께 주면 `ValueError`입니다.
둘을 섞으면 어느 경로가 실제로 비행되는지 호출 측이 알 수 없기 때문입니다.

| 입력 | 동작 |
|---|---|
| `waypoints=[...]` | 준 좌표를 그대로 비행 |
| `waypoints` 없음 | `pattern`으로 내부 생성 후 홈 기준 변환 (`pattern` 기본값 `'grid'`) |
| 둘 다 | `ValueError` |

요약 CSV의 `north_m` / `east_m` 컬럼은 어느 경로든 홈 기준으로 역산해 채우므로
컬럼 구조는 두 방식이 동일합니다.

### 2-6. 정해진 것과 남은 것

#24(미들웨어 MVP, 진행 중)에서 연결 방식이 정해졌습니다.

- [x] **전송 계층** — 오케스트레이터 ↔ 미들웨어는 MCP, 미들웨어 ↔ 시뮬레이션은
      미들웨어가 어댑터를 **직접 import해 `DroneInterface` 메서드를 호출**하는 방식 (2-1-2)
- [x] **명령 단위 분해** — 미들웨어 Dispatcher가 `takeoff` / `goto` / `land` / `disarm`을
      하나씩 호출하므로 시뮬레이션 쪽 추가 분해는 필요 없음. `run_mission()`은 실험용으로 유지
- [x] **역할 경계** — 시뮬레이션 = 프로토콜 → `Telemetry`, 미들웨어 = `Telemetry` → `DroneState`
- [ ] **`return_home()`** — #24의 시뮬레이션 의존 항목. 그전까지 미들웨어는 `goto(home)`으로 대체
- [ ] **정식 FakeDrone** — #24의 시뮬레이션 의존 항목
- [x] 배터리 부족·통신 악화/두절·웨이포인트 미도달 주입 (2-9). 배터리는 비행 중에만 연속 감소, 통신은
      링크 품질 값까지 샘플마다 제공. GPS 저하 주입은 아직 없음

**완료됨**: 외부 waypoints 주입(2-5), 웨이포인트 단위 실행 분리(`execute_waypoint`),
최소 안전 고도 검증(2-4 ⑦), 텔레메트리 타입 고정(2-2), 실패 처리 정책(2-1-1),
연결 유지 운용 대비(2-1-2).

### 2-7. #17 명령 계약과의 대조

`feat/17-command-contracts-api-schema`의 `docs/contracts/command-api.md`와
`services/middleware/app/schemas/command.py`를 확인하고 대조했습니다.
**서로 상의 없이 각자 설계했는데 명령 단위가 거의 그대로 맞았습니다.**

| #17 `DroneCommandType` | `DroneInterface` | 상태 |
|---|---|---|
| `takeoff` (`altitude_m`, `altitude_reference`) | `takeoff(altitude_m)` | 고도 기준 인자만 없음 |
| `goto` (`target`) | `goto(lat, lon, alt_m)` | 일치 |
| `land` | `land(timeout=None)` | 일치 |
| `disarm` | `disarm(timeout)` | 일치 |
| `return_home` | **없음** | `run_mission()` 안에서 `goto(home)`으로 처리 중 |

대조 당시 맞춰야 할 것으로 세 가지를 꼽았고, #24에서 다음과 같이 정리되었습니다.

1. **`return_home()`** — 아직 없음. #24가 기다리는 항목이라 다음 작업으로 진행합니다.
   미들웨어 Dispatcher는 `return_home()`이 정상 반환하면 바로 성공으로 처리하므로,
   `takeoff()`처럼 **복귀를 마칠 때까지 기다렸다가 반환**하도록 만들어야 합니다.
2. **`altitude_reference`** — 미들웨어가 `home_relative`만 허용하고 나머지는 앞에서
   거부하기로 해서, 어댑터는 지금처럼 홈 기준 상대고도만 받으면 됩니다.
3. **필드 이름** — 미들웨어의 공통 매퍼가 `Telemetry`(`lat`, `lon`, ...)를 `DroneState`로
   옮기기로 해서 시뮬레이션 쪽 이름은 그대로 둡니다.

`rotate`는 목 서버에만 있고 계약 문서에는 없습니다. 현재 어댑터에 yaw 제어가 없으니
정식 명령으로 확정되면 추가하겠습니다.

### 2-8. 목 서버를 실제 연동으로 바꿀 때 주의할 점

`sandbox/mock-orchestrator/mock_drone_server.py` 머리말에 이런 안내가 있습니다.

> 실제 AirSim 연동 시 각 함수 안의 상태 관리 코드를 `airsim.MultirotorClient()`
> 호출로 교체하면 됨

**현재 시뮬레이션 구성에서는 이 방법이 동작하지 않습니다.** 미리 공유합니다.

`config/airsim_2vehicle.json`의 기체 타입은 `ArduCopter`입니다. 이 설정에서
AirSim은 **물리 연산과 렌더링만** 담당하고 비행 제어는 전적으로 ArduPilot SITL이 합니다.
그래서 `MultirotorClient`의 이동 API는 기체를 움직이지 못합니다.

기체 타입을 `SimpleFlight`로 바꾸면 `MultirotorClient`가 동작하긴 합니다.
하지만 그 순간 ArduPilot이 빠지므로 **MAVLink도 AP_DDS도 없어집니다.**
이기종 프로토콜 검증이라는 주제 자체가 성립하지 않게 됩니다.

```
[지금]  미들웨어 -> DroneInterface -> MavlinkDrone / Ros2Drone -> ArduPilot SITL -> AirSim(물리·렌더)
[대안]  미들웨어 -> MultirotorClient -> AirSim 내장 비행 모델        (ArduPilot 없음, 프로토콜 없음)
```

**목 함수가 교체될 자리는 `MultirotorClient`가 아니라 `DroneInterface`입니다.**
목 서버의 `takeoff` / `move_to` / `land` / `return_home` / `get_status`는
각각 `takeoff()` / `goto()` / `land()` / (2-7의 1번) / `get_telemetry()`로 바로 연결됩니다.

### 2-9. 장애 주입 — 미들웨어로 전달되는 값 (11주차)

SITL은 배터리가 비행 내내 100%이고 통신도 끊기지 않습니다. 그래서 장애는 **어댑터 안에서**
만듭니다(`app/fault_injection.py`). 값은 **텔레메트리 샘플마다(약 4Hz) 이어서 바뀌고**, 미들웨어가
`get_telemetry()`로 받는 경로 그대로 들어갑니다. 미들웨어 코드를 바꾸지 않아도 값은 들어오고,
그 값으로 귀환 등을 판단하는 것이 미들웨어·LLM 쪽 작업입니다. 비행 컨트롤러(ArduPilot)는 이 장애를 모릅니다.

| 장애 | 거는 법 (`scripts/fault.py`) | 샘플마다 바뀌는 값 | 미들웨어에 보이는 것 |
|---|---|---|---|
| 배터리 소진 | `battery drone-01 0 --drain 120` | `battery_remaining_pct`·`battery_voltage_v`가 **비행 중에만** 줄어듦. 0%면 그 자리 강제 착륙 | `DroneState.battery_percent`, `battery_voltage_v` |
| 배터리 부족(고정) | `battery drone-02 25` | 위 두 값이 25%와 그에 맞는 전압 | 같음 |
| 통신 거리 | `link drone-02 range --degraded-m 250 --down-m 330` | `link_quality_pct`가 거리에 따라 100→0. 250m부터 샘플 2초에 1개, 330m부터 샘플·명령 끊김. 돌아오면 회복 | 링크 품질(매퍼에 추가 필요, 2-9-2). `observed_at` 지연 → `disconnected` |
| 통신 악화(시간) | `link drone-01 degraded` | 샘플을 2초에 1개만, `link_quality_pct` 20. 명령은 정상 전달 | `observed_at`이 최대 2초 늦음. `connected` 유지 |
| 통신 두절(시간) | `link drone-01 down --for 15` | 샘플 없음. 명령·모드 전환·ACK 모두 기체에 안 닿음. 5초 뒤 기체 failsafe(RTL)를 흉내 냄 | 3초 뒤 `disconnected`. goto 중 5초 무응답이면 `LINK_LOST`(#42) → 복구 대기 → 복구되면 귀환·착륙 |
| 웨이포인트 미도달 | `stall drone-01` (다음 goto 1회 무시) | `goto()`를 기체에 보내지 않음. 위치가 목표 쪽으로 움직이지 않음 | `WAYPOINT_TIMEOUT` |

시작 시점은 공통 옵션으로 정합니다: `--after 초`(지금부터), `--after-takeoff 초`(다음 이륙 완료부터),
`--for 초`(지속 시간). 같은 드론에 같은 종류를 다시 걸면 이전 설정을 대체하고, `fault.py clear`로 해제합니다.

#### 2-9-1. 배터리 값이 바뀌는 규칙

- **`--drain S`는 시동이 걸린 동안만 줄어듭니다.** 걸 때의 실제 값(SITL은 100%, `--from`으로 지정 가능)에서
  비행 S초에 목표 값이 되도록 일정하게 줄고, 지상·착륙 뒤에는 멈춥니다. 감소율 = (시작 − 목표) / S %/초
  (예: `0 --drain 120`이면 0.83%/초). 실제 배터리(수십 분)보다 빨리, 임무 한 번 안에 보이도록 압축한 값입니다.
  `--ramp S`는 시계 기준(지상에서도 줄어듦), 숫자만 주면 그 값으로 고정입니다.
- **전압도 함께 움직입니다.** 셀 수(처음 받은 실제 전압으로 추정. SITL 12.6V → 3셀) × LiPo 셀 전압표(휴지 전압 근사).

  | 잔량 | 100% | 50% | 30% | 20% | 10% | 0% |
  |---|---|---|---|---|---|---|
  | 3셀 전압 | 12.60V | 11.52V | 11.31V | 11.19V | 11.07V | 9.81V |

- **0%가 되면 기체는 더 날 수 없습니다.** 시뮬레이션에서는 그 자리 착륙(LAND)으로 흉내 내고(실제라면 추락·불시착),
  이후 `goto()`는 기체에 보내지 않고 `takeoff()`는 시동 거부로 실패합니다.
- **0% 전에 기체가 스스로 돌아오지는 않습니다.** ArduPilot 배터리 failsafe 기본값(`BATT_FS_LOW_ACT`,
  `BATT_FS_CRT_ACT` = 0)이 "경고만"이라서입니다. **값을 보고 귀환시키는 것이 미들웨어·LLM의 몫입니다.**

`scenario_check.sh battery`는 drone-01에 sector-0-0(30m)을 맡기고 `battery drone-01 0 --drain 120`을 겁니다.
**2026-11-09 SITL 결과** (계획 수락 후 시간, 미들웨어 `get_mission_status`의 `battery_percent`로 확인):

| 시각 | 잔량 | 위치 |
|---|---|---|
| 60초 | 50% | 첫 줄 끝 (홈에서 약 183m) |
| 85초 | 30% | 섹터 한가운데 직전 (홈에서 약 169m) |
| 121초 | 0% | 마지막 줄 (홈에서 181m) → 그 자리 강제 착륙, 이후 goto 무시 |

미들웨어에는 2초 폴링마다 줄어든 값이 그대로 보였고(100 → 99.1 → 97.4 → …), 10% 단위 기록이 모두 남았습니다.
drone-01 작업은 `WAYPOINT_TIMEOUT`(60초)으로 실패, 장애 없는 drone-02는 성공했습니다.
강제 착륙 지점은 지형이 홈보다 낮아 내려오는 데 2분 넘게 걸렸고, 그 사이 미들웨어 비상 정지가 공중 disarm을
보낸 문제는 어댑터 `land()`에서 고쳤습니다(2-1-2). 고친 뒤 같은 시나리오를 다시 돌려 강제 착륙 지점(홈 기준 −27m
지면)에 닿을 때까지 기다린 뒤 `emergency_stop`이 `landed: true`로 끝나는 것을 확인했습니다(시간과 잔량은 첫 실행과 같음).

**귀환 판단에 쓸 실측값** (2026-11-09 SITL 섹터 비행, 미들웨어 이벤트 기록에서 계산)

| | drone-01 (MAVLink) | drone-02 (AP_DDS) |
|---|---|---|
| 섹터 안 62.5m 구간 | 약 13초 | 약 7초 |
| 섹터 끝 → 홈 | 227m 31초 | 330m 23초 |
| 착륙 명령 → 시동 꺼짐 | 59초 (30m) | 36초 (40m) |

귀환에 필요한 잔량 ≈ (홈까지 비행 시간 + 착륙 시간) × 감소율입니다. 예를 들어 `--drain 120`(0.83%/초)에서
drone-01이 섹터 끝에서 돌아오려면 (31 + 59)초 × 0.83 ≈ 75%가 필요합니다. 압축한 감소율이라 귀환에 드는 몫도
크다는 뜻이므로, **귀환에 성공하는 시연**을 하려면 `--from`으로 시작값을 낮추고 `--drain`을 늘려 감소율을
줄인 뒤 "남은 잔량 < 귀환에 필요한 잔량 + 여유" 같은 기준으로 판단해야 합니다.

#### 2-9-2. 링크 품질 `link_quality_pct` (텔레메트리 새 필드)

| 상황 | 값 |
|---|---|
| 통신 장애 없음 | 100 |
| `range` | 기준점(처음 관측한 위치 = 홈)에서 0m일 때 100, `--down-m`에서 0. 거리에 비례해 줄어듦 |
| `degraded` (시간 기준) | `--quality` (기본 20) |
| `down` / range 두절 구간 | 샘플 자체가 없음 (→ `disconnected`) |

예: `range --degraded-m 250 --down-m 330`이면 100m 69.7, 200m 39.4, 250m 24.2(여기부터 샘플 2초에 1개), 330m 0(두절).
실기체의 RSSI·패킷 수신율에 해당하는 모의값입니다. 실기체 어댑터는 아직 `None`을 넣습니다
(MAVLink는 텔레메트리 라디오의 `RADIO_STATUS`로 채울 수 있는 자리, AP_DDS는 해당 토픽 없음).

미들웨어 매퍼(`TelemetryStateMapper`)는 모르는 키를 무시하므로 지금은 `DroneState`에 안 들어갑니다.
LLM에 보여 주려면 `DroneState`에 필드를 하나 두고 `optional_float(telemetry, "link_quality_pct")`로 옮기면 됩니다.

**2026-11-09 SITL 결과** (`scenario_check.sh link`: drone-02 → sector-1-1, `range --degraded-m 250 --down-m 330`,
계획 수락 후 시간)

| 시각 | 홈 거리 | 링크 품질 | 일어난 일 |
|---|---|---|---|
| 0~36초 | 0 → 250m | 100 → 24% | 샘플마다 줄어듦 |
| 36초 | 250m | 24% | degraded: 샘플 2초에 1개 (`connected` 유지) |
| 51초 | 330m | 0% | 두절. 미들웨어는 3초 뒤 `disconnected` |
| 56~57초 | | | 기체 failsafe RTL (두절 5초), 미들웨어 `LINK_LOST` (6.1초 무응답) |
| 65초 | 320m | 3% | 복구 (두절 8초, 그동안 미들웨어가 보낸 명령 0건) → 미들웨어 귀환 구간 시작 |
| 126초 | 0m | 100% | 홈 착륙 (`link_recovery_finished: landed`) |

두절 전 약 15초(250m → 330m) 동안 품질이 24%에서 0으로 내려갔고, 그 전에도 이륙부터 계속 줄었습니다.
이 값으로 미리 귀환시키면 두절 자체를 피할 수 있습니다 (회의의 "통신 악화 시 return home").

**쓰는 법.** 미들웨어 서버를 띄운 채 다른 터미널에서 겁니다. 어댑터가 제어 파일
(`/tmp/skybuddy_faults.json`)을 0.2초마다 읽습니다. 서버 터미널에
`[장애 주입] drone-01 battery 시작: 15%` 같은 줄이 나오면 적용된 것입니다.
`fault.py status`로 걸린 장애와 연결된 어댑터를 확인할 수 있습니다.
미들웨어는 MavlinkDrone에 drone_id를 넘기지 않으므로 `fault.py`가 drone-01을 연결 문자열
(`udpin:127.0.0.1:14561`)까지 묶어 기록합니다. 드론 구성이 기본(drone-01 MAVLink, drone-02 AP_DDS)과
다르면 `--config <미들웨어 드론 설정 JSON>`을 줍니다.

**프로토콜 차이 (실제 두절에서도 같은 차이).** 두절 중에 보낸 goto는 MAVLink는 한 번만 보내므로
복구 후에도 사라진 채로 남고, AP_DDS는 2Hz 재발행이라 복구되면 기체에 전달됩니다(PR #36 D4와 관련).

**기체 failsafe 흉내 (#42와 맞물리는 부분).** 미들웨어 #42는 goto 중 텔레메트리가
`link_loss_timeout_s`(5초) 동안 없으면 `LINK_LOST`로 보고, 두절 중에는 명령을 보내지 않은 채
기체 자체 failsafe(ArduPilot `FS_GCS_ENABLE`)가 귀환시키기를 기다립니다(`link_recovery_timeout_s` 20초).
장애 주입은 지상(어댑터) 쪽에서만 링크를 끊으므로 SITL은 두절을 모릅니다. 그래서 두절이
`--failsafe-after`(기본 5초 = `FS_GCS_TIMEOUT` 기본값) 이어지면 **어댑터가 기체 대신 RTL로 모드를
바꿉니다.** 실제 기체에서는 비행 컨트롤러 안에서 일어나는 일이라 두절 차단을 우회해 보냅니다.

| `--failsafe` | 실제 기체 설정 | 동작 |
|---|---|---|
| `rtl` (기본) | `FS_GCS_ENABLE=1` | 홈으로 돌아와 착륙 |
| `land` | `FS_GCS_ENABLE=5` | 그 자리 착륙 |
| `none` | `FS_GCS_ENABLE=0` (ArduCopter 기본값) | 아무것도 안 함. 마지막 목표에서 제자리 비행 |

예: `link drone-01 down --for 15` → 5초에 `LINK_LOST`와 failsafe RTL → 15초에 복구 → 미들웨어가
귀환 구간과 착륙을 진행합니다. RTL 중에는 GUIDED 목표가 무시되므로 기체는 RTL로 홈에 오고,
미들웨어는 도착을 확인한 뒤 LAND로 착륙시킵니다. `--for` 없이 두면 20초 뒤 `link_not_restored`로
끝나고 기체는 RTL로 스스로 내려옵니다. 실제 SITL의 `FS_GCS_ENABLE` 값은 PR #36 D3에서 따로 확인합니다.

**기록과 현재 상태 (박병언 로그·개발용)**

- **미들웨어에 넘긴 텔레메트리 전부**: `~/skybuddy/logs/telemetry-stream.jsonl` (`SKYBUDDY_TELEMETRY_LOG`로 변경,
  `0`이면 끔). `get_telemetry()`가 돌려준 샘플마다 한 줄입니다. 미들웨어 판단 로직을 SITL 없이 개발할 때
  이 파일을 그대로 입력으로 쓰면 됩니다. `scenario_check.sh`는 시나리오마다 `telemetry_<시험>.jsonl`·`.csv`로 남깁니다.

  ```json
  {"logged_at": "...", "drone": "drone-01",
   "telemetry": {"lat": ..., "relative_alt_m": 30.1, "battery_voltage_v": 11.31, "battery_remaining_pct": 30.0,
                 "link_quality_pct": 100.0, "timestamp": ..., "...": "스키마 18필드 그대로 (2-2)"},
   "sim": {"armed": true, "flight_s": 84.0, "home_distance_m": 159.2, "link_state": "ok",
           "faults": {"battery": "<항목 id>"}, "raw_battery_pct": 100, "raw_battery_voltage_v": 12.6}}
  ```

  `telemetry`는 미들웨어가 받은 값 그대로, `sim`은 시뮬레이터만 아는 참값입니다(미들웨어에는 가지 않음, 검증용).
  보기: `fault.py stream -n 20 --every 4`(약 1초에 1줄), `fault.py stream --follow`(실시간), `fault.py stream --csv out.csv`(그래프용).

- 장애 이벤트: `~/skybuddy/logs/fault-events.jsonl` (`SKYBUDDY_FAULT_LOG`로 변경). 미들웨어 이벤트 로그와
  같은 `logged_at` / `event` 형식이라 시각으로 합쳐 볼 수 있습니다.

  | event | 주요 필드 |
  |---|---|
  | `fault_started` | `drone`, `kind`, `params`, `active_since`, `summary`, 배터리면 `start_pct` |
  | `fault_ended` | `drone`, `kind`, `reason` (`expired` / `cleared` / `replaced` / `count_reached`) |
  | `fault_battery_level` | 잔량이 10% 구간을 내려갈 때마다: `level`(90…0), `pct`, `voltage_v`, `flight_s`, `position`, `home_distance_m` |
  | `fault_battery_empty` | 0%에서 강제 착륙: `sent`, `mode`(`land`), 위치·홈 거리 |
  | `fault_takeoff_blocked` | 0%에서 이륙 시도: `pct` |
  | `fault_link_state` | `state_from`, `state_to`, `distance_m`(range), `link_quality_pct`, `tx_dropped`(두절 중 전달 안 된 명령 수) |
  | `fault_goto_dropped` | `reason`(`stall` / `battery_empty`), stall이면 `dropped`, `count`, `target` |
  | `fault_failsafe` | `mode`(`rtl`/`land`), `after_s`, `sent`(보냈는지), 못 보냈으면 `reason` |
  | `fault_adapter_attached` | `identities`, `pid`, 연결 시점에 이미 걸려 있던 장애 |

- 현재 상태: `client.fault_status()` — 블로킹 없음, 스레드 안전이라 `DroneHandle.exclusive` 없이 불러도
  됩니다. `{"armed", "flight_s", "battery": {잔량, 전압, drain_s, empty, forced_landing_sent, ...} 또는 None,
  "link": {"state": "ok|degraded|down", "quality_pct", ...}, "stall": {...} 또는 None}`.
- 권장: 장애 **판단**은 실제로 받은 값(`battery_percent`, `link_quality_pct`, `observed_at`/`connection_status`,
  `WAYPOINT_TIMEOUT`)으로 하고 `fault_status()`는 "언제 무엇을 주입했는지" 기록·검증용으로 씁니다.

**미들웨어·LLM 쪽에 붙일 자리** (시뮬레이션 쪽은 값을 계속 내보내는 데까지)

1. `link_quality_pct`를 `DroneState`로 옮기기 (2-9-2, 매퍼 한 줄)
2. 비행 중 상태를 계속 보고 귀환시키는 경로: 지금 오케스트레이터는 `/command`를 받을 때만 돌고, 귀환·중단
   도구가 없습니다. 배터리 감소·링크 품질 하락을 보고 귀환시키려면 이 경로가 필요합니다.
3. 판단 기준: 배터리는 2-9-1의 실측값과 감소율로 "귀환에 필요한 잔량"을 계산, 통신은 `link_quality_pct`
   하락이나 `degraded`(샘플 간격 증가)를 두절 전에 감지.

**주의 — 미들웨어 정책과 겹치는 부분**

1. #42 이후 goto 중 두절은 5초 만에 `LINK_LOST`로 처리되어, 두절 중 land·강제 disarm을 보내던
   예전 비상 정지 경로로 가지 않습니다. 단 `waypoint_timeout_s`를 `link_loss_timeout_s`보다 짧게 두면
   `WAYPOINT_TIMEOUT`이 먼저 나서 비상 정지 경로로 갑니다(#42 테스트에도 있음).
   착륙 확인 실패 시 강제 disarm(21196)하는 정책(`_land_or_disarm`)은 그대로라, `land()`가 False면
   공중이어도 시동을 끕니다. 2026-11-09 배터리 시나리오에서 실제로 일어났고(2-1-2 표 마지막 줄),
   어댑터는 내려가는 중이면 더 기다리도록 고쳤습니다. 미들웨어 쪽에서도 disarm 전에 고도·하강 속도를
   확인하는 것을 권합니다 (복구 뒤 착륙 도중 다시 끊기는 경우는 샘플이 없어 어댑터가 판단할 수 없음).
2. stall 기본값은 goto 1회 무시입니다. 무제한(`--count 0`)이면 복귀용 goto도 막힙니다.
3. 실기체에서는 반드시 끕니다: `SKYBUDDY_FAULT_INJECTION=0`.
4. 제어 파일은 서버를 다시 켜도 남습니다. 세션을 시작할 때 `fault.py clear`를 권합니다.
   연결 시점에 걸려 있는 장애가 있으면 서버 터미널에 경고가 나옵니다.

시뮬레이터 없이 검사: `python3 tests/test_faults.py`. 비행 중에만 닳는 배터리·전압·0% 강제 착륙,
링크 품질, 텔레메트리 기록을 가짜 시계와 가짜 연결로 확인하고, 미들웨어 코드가 있으면 Registry가 만든
`DroneState`의 `battery_percent`·`battery_voltage_v`·`connection_status`까지 확인합니다.

---

## 3. 배성열(LLM)에게

### 3-1. LLM 출력에서 비행 명령까지의 경로

리뷰에서 확인한 대로, LLM은 **좌표를 직접 만들지 않습니다.**
회의에서 정한 역할 분담은 다음과 같습니다.

```
  LLM / 구역 분할          구역 할당 · 경로 설정            실행
  (배성열)                  (이태우)                        (이태우)

  대수 기반으로            어느 드론에 어느 구역을 줄지     run_mission(
  구역을 N조각으로   →     정하고, 그 구역 안의        →     waypoints=[...],
  분할                     웨이포인트를 생성                 target_alt=...)
```

미들웨어는 `SearchArea.boundary` 조회와 명령 전달을 담당합니다.

> 위 그림은 당시 회의 기준입니다. 지금은 경로 생성을 #26(배성열)에서 진행하고 있고,
> 실행은 #24 미들웨어가 `DroneInterface` 메서드를 직접 호출하는 방식으로 정해졌습니다(2-6).
> 경로 생성 담당은 아래 표와 3-5에 정리 필요 항목으로 남겨 두었습니다.

LLM 출력 예시 (성열 리뷰에서 공유된 형태):

```json
{
  "mission_id": "mission-001",
  "assignments": [
    {"drone_id": "drone-1", "area_id": "area-a", "priority": 80},
    {"drone_id": "drone-2", "area_id": "area-b", "priority": 80}
  ],
  "generated_at": "2026-09-09T10:00:00+09:00"
}
```

시뮬레이션이 최종적으로 받는 형태는 이것입니다.

```
drone_id + 웨이포인트 목록 [(위도, 경도), ...] + 순항 고도
```

**주입 경로는 구현되어 있습니다**(2-5). 다만 **구역 boundary를 받아 웨이포인트를
만드는 함수는 아직 없습니다.** 지금 있는 것은 홈 기준 사각형·격자 생성뿐입니다.

| 필요한 것 | 상태 |
|---|---|
| 웨이포인트 리스트를 받아 실행 | 구현됨 (`run_mission(waypoints=...)`, 또는 미들웨어가 `goto()`를 하나씩 호출) |
| 구역 boundary → 웨이포인트 생성 | **#26에서 배성열이 진행 중.** 시뮬레이션은 촬영 폭 계산에 필요한 카메라 값을 제공 |
| 드론별 구역 할당 (고도 분리 포함) | **미구현** |

분할 결과의 형식은 #21의 `SearchArea.boundary`(위경도 좌표 리스트, 3~100점)로 정해졌습니다.
#24에는 웨이포인트 생성이 시뮬레이션 파트 의존 항목으로 적혀 있어, #26과 담당을 맞출 필요가 있습니다.

### 3-2. 물리적 제약 — 이를 벗어나면 실행 불가능한 명령이 됩니다

| 항목 | 값 | 의미 |
|---|---|---|
| 드론 수 | 2대 | drone1(MAVLink), drone2(AP_DDS) |
| 출발 위치 간격 | 6m | 두 기체 스폰 간격 |
| 수직 분리 | 현재 20m (30m / 50m) | **측정값 아님.** 아래 참고 |
| 이동 속도 | 약 2.7~4.8 m/s | 기체별로 다름 (아래 참고) |
| 도달 판정 반경 | 3m | 이보다 촘촘한 웨이포인트는 무의미 |
| 웨이포인트 타임아웃 | 60초 | 초과 시 실패 처리 |
| **최소 안전 고도** | **30m 이상** | 저고도(10~20m)는 지형지물 충돌 발생 |

> ⚠️ **가장 중요**: 두 드론에게 겹치는 구역을 같은 고도로 배정하면 충돌합니다.
> **구역을 나누거나 고도를 나누거나, 둘 중 하나는 반드시** 해야 합니다.

**두 제약을 어느 계층이 강제할지 (리뷰 논의 사항)**

| 제약 | 현재 상태 | 제안 |
|---|---|---|
| 최소 안전 고도 30m | **시뮬에서 강제 중** — `validate_altitude()`가 30m 미만이면 `ValueError` | 미들웨어에서도 검증해 이중 방어. LLM 출력 검증 단계에서 먼저 걸러주면 실패가 더 빨리 드러납니다 |
| 수직 분리 | **아직 미구현. 최소값도 미확정** | **구역 할당 계층(이태우)에서 강제 예정.** 최소값은 실측 후 확정 |

**최소 안전 고도와 수직 분리는 성격이 다릅니다.** 안전 고도는 드론 한 대의 속성이라
`run_mission()`이 자기 `target_alt`만 보고 판단할 수 있습니다. 반면 수직 분리는
드론 **두 대 사이**의 속성이라 두 배정을 동시에 봐야 하고, 한 대 단위로 동작하는
`run_mission()`은 상대 기체의 고도를 알지 못합니다. 그래서 여기서는 검증할 수 없습니다.

> ⚠️ **수직 분리 20m는 측정값이 아니라 임의로 정한 값입니다.**
> 두 기체가 같은 20×20m 경로를 돌기 때문에, 시연 화면에서 겹쳐 보이지 않고
> 부딪힐 일도 없도록 넉넉히 벌려둔 것입니다. 이 표의 다른 값들(최소 안전 고도 30m,
> 도달 판정 반경 3m 등)은 실측으로 정해졌지만 **이 항목만 다릅니다.**
>
> 같은 고도로 붙였을 때 실제로 충돌하는지, 몇 m부터 안전한지는 **시험한 적이
> 없습니다.** 구역 할당을 구현할 때 간격을 좁혀가며 측정해 정하겠습니다.
> 그전까지는 20m를 그대로 쓰되, 이 값을 물리적 한계로 신뢰하지 말아 주십시오.

회의에서 **구역 할당이 시뮬 파트 담당으로 정해졌으므로**, 여러 드론의 배정을 한꺼번에
보는 그 계층에서 수직 분리를 강제하겠습니다. 구역 할당 함수를 만들 때 함께 넣습니다.
그때까지는 **아무도 강제하지 않는 상태**이므로, 같은 고도로 겹치는 구역을 배정하지
않도록 주의가 필요합니다.

### 3-3. 비교 기준선(baseline)

알고리즘 고정 분할(20×20m 정사각형 5개 웨이포인트)로 측정한 값입니다.
**두 대를 동시에 띄운 상태에서 측정했습니다.**

**3회 측정한 범위입니다.** 실행마다 1~2% 흔들리므로 단일 값이 아니라 범위로 적습니다.
`samples/`에는 3회차 원본 CSV가 들어 있습니다.

| 지표 | drone1 (MAVLink) | drone2 (AP_DDS) |
|---|---|---|
| 웨이포인트 도달률 | 5/5 | 5/5 |
| 구간 이동 시간 | 5.3~8.0초 | 3.1~4.7초 |
| 경로 효율 (평균) | 0.954~0.955 | 0.834~0.865 |
| 총 소요 | 122.3~124.2초 | 86.5~86.7초 |
| 총 이동거리 | 84.2~84.7m | 97.2~99.6m |
| 진동(vibration_z) | 0.046~0.126 | (지표 없음) |

drone1의 첫 구간은 세 번 모두 5.28~5.31초로 일치했습니다. drone2는 경로 효율의
편차가 상대적으로 큽니다(0.686~0.934). 구간마다 오버슛 정도가 달라서인데,
프로토콜 차이가 아니라 튜닝 차이로 보입니다.

> **경로 효율 정의**: 실제로 도달한 지점까지의 직선거리 ÷ 실제 이동 경로 길이.
> 목표 도달 실패 시에도 1을 넘지 않도록 이렇게 정의했습니다.

> ⚠️ **이 속도·효율 차이를 프로토콜 탓으로 해석하면 안 됩니다.**
> 측정 시점에 두 기체의 ArduPilot 버전(4.3 / 4.7.1)과 자세 루프 튜닝이 달랐습니다.
> 이 수치는 현재 **각 드론의 개별 기준선**으로만 쓰십시오.

**이 수치는 `takeoff()` 수정 이후 재측정한 값입니다.**

수정 전에는 drone1의 첫 구간만 10.87초로 유독 길었습니다. 첫 웨이포인트는 홈에서
대각선 절반이라 14.1m로 다른 구간(20m)보다 짧은데도 그랬습니다.
원인은 `takeoff()`가 명령 수락 시점에 반환해서 **상승과 첫 구간 이동이 겹쳤기** 때문입니다.

고도 도달까지 대기하도록 고친 뒤 첫 구간이 5.3초로 줄었고, 거리에 비례하는 값이
되었습니다(14.1m/5.3초 = 2.67m/s, 다른 구간 20m/7.0초 = 2.86m/s).
drone2는 원래부터 고도 대기를 했으므로 86.5 → 86.7초로 변화가 없었습니다.
총 소요가 112.4 → 124.1초로 늘어난 것은 이륙 대기 시간이 정직하게 포함된 결과입니다.

**버전 통일 여부는 팀 논의가 필요합니다.**
회의에서 이기종의 정의가 두 갈래로 정리되었습니다.

- 프로토콜 이기종 — 동일 펌웨어에서 2개 프로토콜 동시 운영
- 펌웨어 이기종 — 서로 다른 펌웨어 운영

**두 축을 다 보여주기로 하면 현재의 4.3 / 4.7.1 구성이 오히려 자산입니다.**
프로토콜 축만 보여줄 것이라면 버전을 통일해야 성능 비교가 성립합니다.
어느 쪽으로 갈지 정해주시면 그에 맞춰 환경을 정리하겠습니다.

### 3-4. 장애 시나리오 (회의 우선순위 기준)

**재현 가능 여부**와 **대응 동작 구현 여부**는 다른 문제입니다. 나눠서 적습니다.

| 순위 | 시나리오 | 요구 동작 | 장애 재현 | 대응 동작 |
|:---:|---|---|---|---|
| 1 | 통신 두절 | 통신 악화 시 return home | 가능 (`fault.py link`, 2-9). 거리에 따른 링크 품질 값 제공 | 두절: 미들웨어 #42 (감지 → 복구 시 귀환·착륙, SITL 검증). 악화 단계 귀환: **미구현** |
| 2 | 배터리 부족 | full/부족 2대로 우선순위 처리 | 가능 (`fault.py battery`, 2-9). 비행 중 연속 감소, 0%면 강제 착륙 | **미구현** (미들웨어·LLM) |
| 3 | 웨이포인트 미도달 | return 후 신규 배정·신규 루트 | 가능 (`fault.py stall`, 2-9) | **미구현** (미들웨어·LLM) |
| - | GPS 저하 | (미정) | 불가 (항상 최상급 fix) | - |
| - | 드론 이탈 | (미정) | **주의** — SITL 종료 시 시뮬 전체 정지 |  - |

**3순위 웨이포인트 미도달** — 미들웨어 Dispatcher는 `WAYPOINT_TIMEOUT`이 나면 그 작업을 중단하고
비상 정지(그 자리 착륙)합니다. 회의에서 정한 "복귀 후 신규 배정·신규 루트"는 미들웨어·LLM 쪽
작업입니다. (실험용 `run_mission()`은 타임아웃이 나도 다음 웨이포인트로 넘어갑니다.)

**1순위 통신 두절** — 미들웨어 #42가 goto 중 5초 무응답을 `LINK_LOST`로 보고, 두절 중에는 기체
failsafe에 맡겼다가 복구되면 귀환·착륙시킵니다. 시뮬레이션에서는 그 failsafe를 장애 주입이 흉내 냅니다(2-9).

**2순위 배터리** — 2-9로 구현했습니다. `--drain`이면 시동이 걸린 동안만 잔량·전압이 샘플마다 줄고,
0%가 되면 그 자리 강제 착륙합니다. 고정값이나 시계 기준 `--ramp`도 고를 수 있습니다.
어댑터가 실제 값 대신 주입값을 내보내므로 미들웨어 쪽 코드는 바뀌지 않습니다. 귀환 판단에 쓸 실측값과
계산 예는 2-9-1에 있습니다.

#### 통신 단절을 무엇으로 감지하는가

프로토콜과 무관한 신호는 하나입니다. **`get_telemetry()`가 `None`을 반환하는 것.**

| | 링크가 끊겼을 때 | 어댑터 동작 |
|---|---|---|
| MAVLink | `GLOBAL_POSITION_INT`가 더 이상 오지 않음 (정상 시 약 4Hz) | `timeout` 안에 못 받으면 `None` 반환 |
| AP_DDS | `/ap/geopose/filtered`가 더 이상 오지 않음 | 새 메시지 없으면 `None` 반환 |

`get_telemetry(timeout=2)`가 기본값이므로, **연속 2회 `None`이면 약 4초간 상태를
받지 못한 것**입니다. 정상 주기가 4Hz(0.25초)인데 4초면 16샘플을 놓친 상태입니다.

> ⚠️ 위 4초는 **파라미터에서 계산한 이론값이며, 실제로 링크를 끊어 측정한 값이 아닙니다.**
> Fallback 임계값으로 확정하기 전에 실측이 필요합니다.
> 테스트 방법은 확인해두었습니다 — DDS는 `MicroXRCEAgent`를, MAVLink는 MAVProxy를
> 내리면 SITL과 AirSim 연결은 유지된 채 상위 링크만 끊깁니다(lock-step 정지 없음).
>
> 시나리오 시연에는 2-9의 `fault.py link`를 씁니다. 같은 `None` 신호가 나오고 시작·종료를
> 초 단위로 정할 수 있습니다. 미들웨어는 마지막 샘플이 `telemetry_timeout_s`(기본 3초)보다
> 오래되면 `disconnected`로 봅니다.

판정 기준을 시뮬 쪽에 넣기를 원하시면 `run_mission()`에 연속 None 횟수 임계값을
파라미터로 추가하겠습니다. 다만 **재배정 판단 자체는 미들웨어 몫**이라고 보고
지금은 신호만 그대로 올리고 있습니다.

주의할 점이 하나 있습니다. AirSim이 lock-step이라 **SITL 프로세스를 종료하면
시뮬레이션 전체가 멈추므로** 링크만 끊어야 합니다. MAVLink는 UDP 연결을 닫는 것으로,
DDS는 Agent를 내리는 것으로 모사할 수 있습니다.

### 3-5. 아직 정하지 못한 것

- [ ] **탐색 구역 커버리지 지표 정의** (셀 크기, "훑었다"의 판정 반경)
      → 계획서 핵심 평가항목인데 현재 없음. 중복 탐색 감소를 증명하려면 필수
- [x] **배터리 소모 곡선** → 비행 시간 기준 선형 감소(`--drain`), 고정값, 시계 기준 감소를 시나리오마다 고름.
      전압은 LiPo 셀 전압표로 함께 움직임 (2-9-1)
- [x] **구역 분할 결과를 넘기는 형식** → #21 `SearchArea.boundary`(위경도 좌표 리스트)로 확정
- [ ] **웨이포인트 경로 생성 담당** → 회의에서는 시뮬 파트로 정했으나 #26에서 배성열이 진행 중. 정리 필요
- [ ] 통신 단절 판정 임계값 — 실측 후 확정 (3-4)

**합의된 것**

- 구역 분할은 코드(Shapely)가 수행하고 LLM은 몇 조각으로 나눌지만 판단
- 고도는 어느 경로로 오든 **30m 미만이면 시뮬이 거부**(2-4 ⑦)
- 수직 분리는 **구역 할당 계층에서 강제 예정**. 간격은 측정 후 확정(3-2)
- 임무 시작·종료 지점은 home → mission → home (이미 구현됨)

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

# 4) WSL: 연결 한 번으로 임무 두 번 (미들웨어 Registry 방식 점검, 2-1-2)
python3 scripts/session_check.py
```

결과물:

- `kpi_summary_drone1.csv` / `kpi_summary_drone2.csv` — 웨이포인트별 지표
- `kpi_log_drone1.csv` / `kpi_log_drone2.csv` — 원시 텔레메트리

실제 출력 예시는 `samples/` 폴더에 있습니다.

**계약 검증은 macOS에서도 그대로 돌아갑니다.** 시뮬레이터는 물론이고 pymavlink나
ROS 2를 설치하지 않아도 됩니다. `test_adapters.py`는 10개 항목 중 어댑터 임포트가 필요한
2개만 "건너뜀"으로 표시되고, `test_mission_policy.py`는 가짜 드론을 쓰므로 전부 수행됩니다.

```bash
python3 tests/test_adapters.py
python3 tests/test_mission_policy.py
```

리뷰하실 때 코드를 직접 돌려볼 수 있는 유일한 지점이라 의존성 없이 동작하게 해두었습니다.

환경 구축 절차와 알려진 문제(조용히 실패하는 함정 포함)는 `docs/engineering-notes.md`를 참고하십시오.
