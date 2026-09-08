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
2. **`app/drone_interface.py`** — 모든 어댑터가 지켜야 하는 계약. 30줄 남짓.
3. **`app/mavlink_drone.py`** / **`app/ros2_drone.py`** — 같은 계약의 두 구현.
   나란히 보면 프로토콜 차이가 어디에서 흡수되는지 보입니다.
4. **`app/kpi_mission.py`** — 임무 수행과 KPI 계산. 프로토콜을 모릅니다.
5. **`samples/`** — 실제 실행 결과 CSV. 출력 형식 참고용.

## 구조

```
app/
  drone_interface.py     프로토콜 공통 계약 (메서드 7개 + 텔레메트리 16필드)
  mavlink_drone.py       MAVLink 어댑터 (pymavlink)
  ros2_drone.py          AP_DDS 어댑터 (rclpy)
  kpi_mission.py         드론 1대의 임무 + KPI 수집
  kpi_mission_multi.py   2대 동시 운용 진입점
tests/
  test_adapters.py       계약 준수 정적 검증 (시뮬레이터 없이 실행 가능)
scripts/
  start_hetero_sim.sh    이기종 2대 환경 기동
  tune_damping.py        자세 루프 감쇠 튜닝
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

시뮬레이터 없이 어댑터 계약만 검증하려면:

```bash
python3 tests/test_adapters.py
```

## 측정 결과

20x20m 정사각형 5개 웨이포인트, 두 대 동시 비행 기준입니다.

| 지표 | drone1 (MAVLink) | drone2 (AP_DDS) |
|---|---|---|
| 웨이포인트 도달 | 5/5 | 5/5 |
| 구간 이동 시간 | 7.0~7.3초 | 4.2~4.7초 |
| 경로 효율 | 평균 0.957 | 평균 0.863 |
| 총 소요 | 112.4초 | 86.5초 |
| 진동(vibration_z) | 0.046~0.057 | 지표 없음 |

> 속도·효율 차이는 프로토콜 차이가 아닙니다. 측정 시점에 두 기체의 ArduPilot
> 버전과 자세 루프 튜닝이 달랐습니다. 자세한 내용은 `INTERFACE.md` 3-3 참고.

프로토콜에서 실제로 오는 차이는 세 가지입니다.

- 갱신 주기 — DDS 약 28Hz, MAVLink 약 4Hz
- 제공 지표 — AP_DDS에는 진동·클리핑·위성수 토픽이 없음
- 고도 기준 — DDS는 AMSL, MAVLink는 홈 기준 AGL (어댑터가 환산)

## 다음 작업

- `run_mission()`을 명령 단위로 분해 — 현재는 연결부터 CSV 저장까지 한 함수라
  밖에서 "웨이포인트 하나만 실행" 같은 요청을 받을 수 없습니다.
- 미들웨어와의 연결 방식 확정 (모듈 호출 / REST / WebSocket)
- 탐색 구역 커버리지 지표 구현
- 두 기체의 펌웨어 버전과 튜닝을 통일해 성능 비교 재측정
