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
  drone_interface.py     프로토콜 공통 계약 (메서드 7개 + 텔레메트리 18필드)
  fault_injection.py     장애 주입 (배터리·통신·웨이포인트 미도달). 두 어댑터가 공통으로 씀
  mavlink_drone.py       MAVLink 어댑터 (pymavlink)
  ros2_drone.py          AP_DDS 어댑터 (rclpy)
  kpi_mission.py         드론 1대의 임무 + KPI 수집
  kpi_mission_multi.py   2대 동시 운용 진입점
tests/
  test_adapters.py       계약 준수 정적 검증 (시뮬레이터 없이 실행 가능)
  test_mission_policy.py 실패 처리 정책 검증 - 가짜 드론 사용 (시뮬레이터 없이 실행 가능)
  test_faults.py         장애 주입 → 미들웨어 DroneState 까지 검증 (시뮬레이터 없이 실행 가능)
scripts/
  start_hetero_sim.sh    이기종 2대 환경 기동 (두 기체 비행 속도 맞춤 포함)
  tune_damping.py        자세 루프 감쇠 튜닝
  apply_params.py        파라미터 파일을 MAVLink 로 넣고 확인 (drone1 비행 속도)
  speed_check.py         비행 기록으로 두 드론 실제 속도 비교 (같은 길이 구간 시간·유지 속도, 다르면 고칠 값 안내)
  mission_check.py       자연어 명령·정상 운용·장애 시나리오 3개의 결과 판정 (미들웨어 이벤트 기록), --report 로 검증 현황
  session_check.py       연결 한 번으로 임무 두 번 (상주 운용 점검, 실제 SITL 필요)
  fault.py               장애 걸기·풀기, 미들웨어에 넘긴 텔레메트리 보기 (fault.py stream)
  map_probe.py           맵 장애물 높이 측정·섹터 그리기 (AirSim 실행 중 필요)
  fault_live_check.py    실제 SITL 연결에서 장애 주입 확인 (비행 없음, 명령 안 보냄)
  make_sections.py       2x2 탐색 섹터 정의·안전 검사 -> 임무 파일
  run_plan.py            LLM 없이 임무 계획 제출 (리허설, demo --no-llm)
  track_drones.py        AirSim 화면에 드론이 지나간 길을 색으로 표시, 작은 창 배치·촬영 카메라 짐벌 (layout)
  minimap.py             섹터·드론 위치 미니맵 (브라우저, AirSim 위에 작은 창으로 띄움. 영상용)
  run_live.sh            한 번에 띄우기 (미들웨어 + LLM 오케스트레이터 + AirSim 표시 + 미니맵),
                         say 로 자연어 명령, demo normal|link|battery|timeout 로 정상 운용·장애 시나리오 3개
scenarios/
  README.md              섹터 정의, 정상 운용·장애 시나리오 3개, AirSim 화면 표시
  missions/              mission-101 (산악맵 2x2 섹터. 정상 운용·장애 시나리오 3개 모두 이 임무)
  drones.sitl.json       SITL 2대용 미들웨어 드론 설정 (지그재그 coverage, 촬영 폭 60m, 한 줄 125m 를 한 번에)
config/
  airsim_2vehicle.json   AirSim 2기체 설정 (작은 창 1=drone-01, 2=drone-02 촬영 카메라, 3=숨김. 짐벌로 흔들림 없음)
  dds_drone.parm         DDS 드론(4.7.1) 부팅 파라미터 (시동 검사 생략, 비행 속도 WP_SPD·WP_ACC·LAND_SPD_MS,
                         기수 고정 WP_YAW_BEHAVIOR 0)
  mavlink_drone.parm     MAVLink 드론(4.3) 비행 속도 (WPNAV_*·LAND_SPEED. 4.3 은 실제 속도가 설정의 약 0.555 배라
                         drone2 와 실제 속도가 같게 맞춘 값, INTERFACE.md 3-2), 기수 고정 WP_YAW_BEHAVIOR 0
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
python3 tests/test_faults.py      # pymavlink 필요. ROS 2·미들웨어 코드가 없으면 그 부분만 건너뜀
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

## 장애 주입 (11주차)

SITL은 배터리가 항상 100%이고 통신도 끊기지 않으므로, 장애는 어댑터 안에서 만듭니다.
값은 텔레메트리 샘플마다(약 4Hz) 이어서 바뀌고 미들웨어가 받는 경로 그대로 들어갑니다.
미들웨어 서버를 띄운 채 다른 터미널에서 겁니다. 값과 규칙은 `INTERFACE.md` 2-9에 있습니다.

```bash
python3 scripts/fault.py link drone-02 range --degraded-m 250 --down-m 330
                                                          # 거리에 따라 링크 품질 100→0, 악화 → 두절 → 기체 RTL
python3 scripts/fault.py battery drone-01 0 --from 40 --drain 140 --rtl-at 20
                                                          # 40% 에서 비행 중 줄다가 20% 에서 기체 배터리 failsafe RTL
python3 scripts/fault.py stall drone-02 --after-takeoff 20 # 다음 goto 1번 무시 → WAYPOINT_TIMEOUT
python3 scripts/fault.py stream --every 4                 # 미들웨어에 넘긴 텔레메트리 (배터리·링크 품질)
python3 scripts/fault.py status
python3 scripts/fault.py clear
```

## 정상 운용 + 장애 시나리오 3개 (11/9 회의)

넷 다 **같은 임무(mission-101)·같은 섹터(남·북)·같은 미니맵·같은 첫 명령**으로 돕니다(시뮬 → LLM → 시뮬).
첫 명령: "남쪽과 북쪽 섹터를 드론 두 대로 하나씩 맡아 수색해 줘. 어느 드론이 어느 섹터를 맡을지는 드론 상태를 보고
정해 줘." LLM 은 mission-101 설명의 규칙(배터리 적은 드론에 가까운 섹터, 비슷하면 drone-01 남·drone-02 북)으로 정합니다.

| # | 시나리오 | 흐름 | 명령 |
|---|---|---|---|
| 0 | 정상 운용 (기준) | 장애 없음 → drone-01 남·drone-02 북 끝까지 수색(이동 7/7) → 둘 다 홈 착륙. 통신 두절·미도달·비상 정지 없음 | `run_live.sh demo normal` |
| 1 | 통신 두절 | 북으로 간 drone-02 링크가 250m 부터 악화, 330m 에서 두절 → 기체 귀환(RTL) → 미들웨어 `LINK_LOST` → 복구 → 귀환·착륙. drone-01 은 성공 | `run_live.sh demo link` |
| 2 | 배터리 부족 | drone-01 40%·drone-02 100% → LLM 이 drone-01 에 가까운 남 → 비행 중 20% 에서 기체 귀환 → 미들웨어가 timeout 처럼 처리(`WAYPOINT_TIMEOUT` → 귀환 → 홈 착륙) → "못 끝낸 섹터 다시" → drone-02 가 남 수색 | `run_live.sh demo battery` |
| 3 | 웨이포인트 미도달 | drone-02 가 북 섹터 입구에서 웨이포인트를 못 받음 → 60초 뒤 `WAYPOINT_TIMEOUT` → 귀환 → 홈 착륙 → "못 끝낸 섹터 다시" → drone-01 이 북 수색 | `run_live.sh demo timeout` |

```bash
bash scripts/run_live.sh                  # 터미널 1: 미들웨어 + LLM + 미니맵 (LLM 키 입력)
bash scripts/run_live.sh demo link        # 터미널 2: 장애 걸기 → 명령 → 끝까지 따라가며 판정 (LLM 없이: 뒤에 --no-llm)
python3 scripts/mission_check.py --report # 4개가 지금 설정으로 검증됐는지, 안 됐으면 무엇을 돌릴지
```

장애 드론의 작업 실패는 정상이라, 대신 장애가 실제로 걸렸는지 → 미들웨어가 알아채 멈췄는지 → 장애 드론이 홈으로
돌아와 내렸는지 → 다른 드론은 성공했는지 → (2·3) 재배정이 성공했는지를 봅니다. 장애 드론은 `unavailable` 로
남으므로 장애 시나리오마다 AirSim·SITL·`run_live.sh` 를 다시 켭니다 (`demo` 가 먼저 확인합니다). 정상 운용은 실패한
드론이 없어서 끝나고 바로 장애 시나리오 하나를 이어서 할 수 있습니다 (시연: 정상 → 장애).

**미들웨어 변경 (이 PR 에 함께)**: `services/middleware/app/runtime/dispatcher.py` 에서 `WAYPOINT_TIMEOUT` 이고
텔레메트리가 들어오면(기체와 통신은 되는데 그 구간을 못 간 경우) 그 자리에 내리지 않고 작업의 귀환 구간(`goto(home)`)을
보낸 뒤 착륙합니다(`timeout_return_finished` 기록). 시나리오 2·3 의 "return 시키고" 부분입니다. 텔레메트리가 없거나
귀환 구간 자체가 시간 초과면 예전처럼 그 자리 비상 착륙입니다. 시험: `services/middleware/tests/test_timeout_return.py`.
함께 고친 두 가지(main 에서도 실패하던 시험 3개):
- `fake_drone.py`: 텔레메트리에 `link_quality_pct`(100) 추가 — #43 에서 스키마가 18필드가 된 뒤 `test_fake_drone_contract` 실패
- `mcp_server.py`: Python 3.10(ROS 2 Humble)에서 Ctrl+C 가 `KeyboardInterrupt` 가 아니라 `CancelledError` 로 올라와
  traceback 과 종료 코드 1 → 같이 잡아 0 으로 끝냄 (`test_mcp_http` ctrl_c 시험 2개. Windows 에서는 건너뛰는 시험)

아직 안 하는 것: 실패한 섹터 자동 재배정(지금은 사용자가 LLM 에 "못 끝낸 섹터 다시"라고 말함).

## 시연 화면 (11주차)

```bash
python3 scripts/track_drones.py layout       # 한 번만: AirSim 작은 창 1=drone-01, 2=drone-02 촬영 카메라 (AirSim 재시작)
bash scripts/start_hetero_sim.sh             # SITL 2대 (두 기체 실제 속도 수평 10 m/s 로 맞춤)
bash scripts/run_live.sh                     # 미들웨어 + LLM + 섹터 선 + 지나간 길 + 미니맵 (--no-llm 이면 LLM 없이)
bash scripts/run_live.sh check               # 다른 터미널: 비행 없이 오케스트레이터·미들웨어·LLM 도구 호출 점검
bash scripts/run_live.sh say "북쪽과 남쪽 섹터를 드론 두 대로 하나씩 맡아 수색해 줘"
                                             # 자연어 명령 -> 계획 수락 -> 이륙·이동·착륙까지 따라가며 '판정: 통과/실패'
python3 scripts/speed_check.py               # 비행 뒤: 두 드론 실제 속도 비교
```

두 기체 모두 기수 방향을 고정합니다(`WP_YAW_BEHAVIOR 0`). 기본값(다음 목표를 바라봄)에서는 drone1(4.3)의 앞 카메라가
비행 중 좌우로 반복해 돌았습니다. AirSim 화면 끊김을 줄이려고 `track_drones.py`(0.5초·5m 마다)와 `minimap.py`(1초에 3번)의
AirSim API 호출도 줄였습니다.

`say` 는 LLM 답을 받은 뒤 `mission_check.py` 로 미들웨어 이벤트 기록을 끝까지 따라가 판정합니다 (계획 거부 이유,
명령별 실패 코드, 작업 결과). 섹터는 동서남북으로 말해도 됩니다 (mission-101 설명에 sector-0-0 남, 0-1 동, 1-0 서,
1-1 북이 적혀 있어 LLM 이 같은 이름을 씁니다).

화면 아래 줄은 왼쪽부터 drone-01 카메라(초록 테두리), drone-02 카메라(연노랑), 미니맵입니다. AirSim 작은 창은
자리와 테두리 색이 AirSim 안에 고정돼 있어(1 왼쪽 초록, 2 가운데 연노랑, 3 오른쪽 보라) 드론 색을 그 테두리 색에
맞췄습니다(지나간 길·미니맵 핀). 섹터 선은 북 빨강·남 파랑·동 주황·서 하늘입니다(미니맵도 같은 색).

미니맵(`scripts/minimap.py`)은 섹터(서·북·남·동)·지그재그 경로·드론 위치와 고도·배터리·링크를 보여 주고,
Windows에서는 화면 오른쪽 아래(숨긴 작은 창 3 자리)에 항상 위로 저절로 붙습니다. 자세한 것은 `scenarios/README.md`.

## 다음 작업

- 실패한 섹터 자동 재배정 (지금은 사용자가 LLM 에 "못 끝낸 섹터 다시"라고 말함)
- `return_home()` 추가 (#24 의존 항목. 미들웨어는 그전까지 goto(home)으로 대체)
- 정식 FakeDrone (#24 의존 항목)
- GPS 저하 값 주입
- 탐색 구역 커버리지 지표 구현
- 두 기체의 펌웨어 버전과 튜닝을 통일해 성능 비교 재측정
