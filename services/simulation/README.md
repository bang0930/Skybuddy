# 시뮬레이션 (services/simulation)

담당: 이태우 — AirSim/ArduPilot 다중 SITL, MAVLink 및 AP_DDS 연동, KPI 실험

AirSim 위에서 ArduPilot SITL 2대를 띄우고, **한 대는 MAVLink로 다른 한 대는
AP_DDS(ROS 2)로 제어**하면서 각각 탐색 임무를 수행시키고 KPI를 수집합니다.

## 이 코드의 핵심 주장

> 상위 계층(미들웨어, LLM)은 드론이 어떤 프로토콜을 쓰는지 몰라도 된다.

`app/kpi_mission.py`의 임무·KPI 로직은 두 프로토콜에 대해 **완전히 같은 코드**가
돕니다. 프로토콜을 아는 곳은 `make_drone()` 팩토리 한 곳뿐이고,
`tests/test_adapters.py`가 이 성질이 깨지지 않았는지 자동으로 검사합니다.

## 읽는 순서

처음 보신다면 이 순서를 권합니다.

1. **`INTERFACE.md`** — 미들웨어/LLM과의 접점. 텔레메트리 스키마, 제어 인터페이스,
   반드시 알아야 할 제약사항. **가장 중요한 문서입니다.**
2. **`app/drone_interface.py`** — 모든 어댑터가 지켜야 하는 계약(`DroneInterface`)과
   텔레메트리 타입(`Telemetry`).
3. **`app/mavlink_drone.py`** / **`app/ros2_drone.py`** — 같은 계약의 두 구현.
   나란히 보면 프로토콜 차이가 어디에서 흡수되는지 보입니다.
4. **`app/kpi_mission.py`** — 임무 수행과 KPI 계산. 프로토콜을 모릅니다.
5. **`samples/`** — 실제 실행 결과 CSV. 출력 형식 참고용.

## 구조

```
app/
  drone_interface.py     프로토콜 공통 계약 (메서드 7개 + 텔레메트리 17필드)
  mavlink_drone.py       MAVLink 어댑터 (pymavlink)
  ros2_drone.py          AP_DDS 어댑터 (rclpy)
  kpi_mission.py         드론 1대의 임무 + KPI 수집
  kpi_mission_multi.py   2대 동시 운용 진입점
tests/
  test_adapters.py       계약 준수 정적 검증 (시뮬레이터 없이 실행 가능)
  test_mission_policy.py 실패 처리 정책 검증 - 가짜 드론 사용 (시뮬레이터 없이 실행 가능)
scripts/
  start_hetero_sim.sh    이기종 2대 환경 기동
  tune_damping.py        자세 루프 감쇠 튜닝
  session_check.py       연결 한 번으로 임무 두 번 (상주 운용 점검, 실제 SITL 필요)
config/
  airsim_2vehicle.json   AirSim 2기체 설정
  dds_drone.parm         DDS 드론 시동 파라미터
samples/                 실제 실행 결과 예시 CSV
docs/
  engineering-notes.md   환경 구축 절차와 알려진 함정
```

환경을 처음부터 구축하는 1회성 스크립트는 넣지 않았습니다.
절차는 `docs/engineering-notes.md`에 정리돼 있습니다.

## 실행 환경

AirSim은 Windows에서, ArduPilot SITL과 ROS 2는 WSL2(Ubuntu 22.04)에서 돕니다.
**macOS에서는 AirSim이 동작하지 않습니다.** 코드 참고용으로 봐주시면 됩니다.

```
Windows          AirSim (Blocks 또는 LandscapeMountains)
WSL2 Ubuntu      ArduPilot SITL x2, micro-XRCE-DDS Agent, ROS 2 Humble
```

**두 검사 모두 macOS에서 그대로 돌아갑니다.** 시뮬레이터는 물론이고
pymavlink나 ROS 2가 설치돼 있지 않아도 됩니다. `test_adapters.py`는 10개 항목 중
어댑터 임포트가 필요한 2개만 건너뛰고, `test_mission_policy.py`는 전부 수행됩니다.

```bash
python3 tests/test_adapters.py
python3 tests/test_mission_policy.py
```

`test_adapters.py`가 확인하는 것: 두 어댑터의 메서드·시그니처 일치, 텔레메트리 스키마와
CSV 컬럼 일치, 누락값을 빈 문자열이 아닌 `None`으로 쓰는지, 최소 안전 고도 거부,
웨이포인트 입력 경로의 배타성, `run_mission()` 본문에 프로토콜 이름이 새어 들어오지
않았는지, 그리고 연결을 유지한 채 쓸 때의 대비(10번 항목).

`test_mission_policy.py`는 정해진 대로 실패하는 가짜 드론을 넣어, 착륙 확인 실패와
이륙 도중 실패에서 착륙·시동 해제·연결 정리가 올바른 순서로 일어나는지 확인합니다.

## 측정 결과

20x20m 정사각형 5개 웨이포인트, 두 대 동시 비행 기준입니다.

| 지표 | drone1 (MAVLink) | drone2 (AP_DDS) |
|---|---|---|
| 웨이포인트 도달 | 5/5 | 5/5 |
| 구간 이동 시간 | 5.3~8.0초 | 3.1~4.7초 |
| 경로 효율 (평균) | 0.954~0.955 | 0.834~0.865 |
| 총 소요 | 122.3~124.2초 | 86.5~86.7초 |
| 진동(vibration_z) | 0.046~0.126 | 지표 없음 |

3회 측정 범위입니다. `samples/`에 3회차 원본 CSV가 있습니다.

> 속도·효율 차이는 프로토콜 차이가 아닙니다. 측정 시점에 두 기체의 ArduPilot
> 버전과 자세 루프 튜닝이 달랐습니다. 자세한 내용은 `INTERFACE.md` 3-3 참고.

프로토콜에서 실제로 오는 차이는 세 가지입니다.

- 갱신 주기 — DDS 약 28Hz, MAVLink 약 4Hz
- 제공 지표 — AP_DDS에는 진동·클리핑·위성수 토픽이 없음
- 고도 기준 — DDS는 AMSL, MAVLink는 홈 기준 AGL (어댑터가 환산)

## 미들웨어 연동 지점

#24 미들웨어는 `app/`의 어댑터를 직접 import해서, 드론마다 한 번 연결해 두고
`takeoff` / `goto` / `land`를 하나씩 호출합니다. 이렇게 연결을 오래 유지할 때 필요한
대비와 제약(스레드 안전성, 실행 환경)은 `INTERFACE.md` 2-1-2에 정리했습니다.

단독 실험에서는 경로를 `run_mission()`에 그대로 주입할 수 있습니다.

```python
run_mission(connection_string='ros2:ap', drone_id='drone2', protocol='ros2',
            target_alt=30,
            waypoints=[(-35.36325, 149.16512), (-35.36307, 149.16534)])
```

웨이포인트 하나만 실행하려면 `execute_waypoint()`를 직접 호출하면 됩니다.
실패 처리 규약은 `INTERFACE.md` 2-1-1, 경로 주입은 2-5를 참고하십시오.

## 미들웨어 명령 계약(#17)과의 대조

5개 명령 중 4개가 `DroneInterface`와 그대로 맞습니다. 대조 당시 맞출 것으로 꼽은
셋 중 고도 기준과 필드 이름은 #24 미들웨어에서 처리하기로 해서, 남은 것은
`return_home()` 하나입니다. 대조표와 조립 지점은 **`INTERFACE.md` 2-7, 2-8**에 있습니다.

> ⚠️ 목 서버의 "`airsim.MultirotorClient()`로 교체하면 됨" 안내는 현재 구성에서
> 동작하지 않습니다. 기체 타입이 `ArduCopter`라 AirSim은 물리·렌더링만 담당합니다.
> 교체 지점은 `DroneInterface`입니다. 이유는 `INTERFACE.md` 2-8 참고.

## 다음 작업

- `return_home()` 추가 (#24 의존 항목. 미들웨어는 그전까지 goto(home)으로 대체)
- 정식 FakeDrone (#24 의존 항목)
- 배터리·GPS 저하 값 주입 기능 (장애 시나리오 검증용)
- 탐색 구역 커버리지 지표 구현
- 두 기체의 펌웨어 버전과 튜닝을 통일해 성능 비교 재측정
