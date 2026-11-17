# 탐색 섹터와 장애 시나리오

담당: 이태우 (시뮬레이션). 미들웨어는 설정 파일과 환경 변수로 붙는다. 미들웨어 코드는 장애 시나리오 3 의
"return 시키고" 한 곳만 바꿨다 (`dispatcher.py`, 아래 장애 시나리오).

경로와 명령은 개발 환경(`~/skybuddy`, 평면 구조) 기준이다. 팀 저장소에서는 이 폴더가
`services/simulation/scenarios/`, 스크립트(`fault.py`, `map_probe.py`, `run_plan.py`, `track_drones.py`,
`minimap.py`, `run_live.sh`, `mission_check.py`)가 `services/simulation/scripts/` 에 있다. `drones.sitl.json` 은 팀 저장소
사본에서 `simulation_app_path` 를 빼서 미들웨어 기본값(같은 저장소의 `services/simulation/app`)을 쓴다.

## 파일

| 파일 | 내용 |
|---|---|
| `missions/mission-101.json` | 산악맵 홈 북동쪽 2x2 섹터 (`make_sections.py`가 만든다). 정상 운용·장애 시나리오 3개 모두 이 임무 |
| `drones.sitl.json` | 미들웨어 드론 설정. 지그재그 수색(`coverage`), 촬영 폭 60m, 한 줄(125m)을 한 번에(`max_leg_m 130`, 구간 제한 시간 기준 `min_ground_speed_m_s 2.5`), 장애 주입이 들어간 어댑터 경로 |

미들웨어 서버를 켤 때:

```bash
export MIDDLEWARE_DRONES_CONFIG=$HOME/skybuddy/scenarios/drones.sitl.json
export MIDDLEWARE_SCENARIOS_DIR=$HOME/skybuddy/scenarios/missions
```

## 비행 흐름

LLM은 드론마다 섹터를 하나씩 배정만 한다(`submit_mission_plan`). 미들웨어가 배정받은 섹터를
웨이포인트로 바꿔 이륙 → 섹터 수색 → 홈 복귀 → 착륙 순서로 비행시킨다.

- 섹터를 웨이포인트로 바꾸는 함수는 미들웨어의 `coverage.py`(지그재그)와 `VertexTraversalGenerator`
  (섹터 경계만 한 바퀴) 두 가지이고, 드론 설정의 `waypoints.generator`로 고른다.
- **팀 기본 설정(`drones.real.example.json`)은 `vertex_traversal`이라 섹터 안을 훑지 않고 경계만 돈다.**
  지그재그로 수색하려면 `"waypoints": {"generator": "coverage"}`와 드론마다 `swath_width_m`이 필요하다.
- 촬영 폭 60m: 아래 카메라 화각 90도에서 고도 30m일 때 바닥에 찍히는 폭(5주차 실측). 섹터 고도 중
  가장 낮은 30m 기준으로 잡아, 어느 섹터를 맡아도 빈틈없이 훑는다.
- 한 줄을 한 번에 난다: `coverage` 기본값(`max_leg_m` 100m)은 125m 줄을 가운데서 한 번 끊어(같은 직선 위 점)
  멈췄다 다시 가속한다. `waypoints.max_leg_m` 130 으로 줄마다 한 구간이 된다(섹터당 멈춤 8번 → 5번). 미들웨어는
  `max_leg_m` 이 `waypoint_timeout_s(60) × min_ground_speed_m_s` 를 넘으면 서버를 안 띄우므로
  `dispatcher.min_ground_speed_m_s` 를 2.0 → 2.5 로 같이 올렸다 (드론은 10 m/s 라 구간 제한 시간 여유는 그대로 크다).

## 섹터 (mission-101)

2026-11-09 산악맵(LandscapeMountains)에서 `map_probe.py scan`으로 홈 반경 300m의 지형·물체 높이를
25m 간격으로 잰 결과로 정했다. 홈 북동쪽은 호수 쪽으로 지형이 홈보다 20~45m 낮아지는 열린 지대이고,
서쪽은 30~60m, 홈 남쪽은 출발점보다 20m 이상 높은 언덕이다.

이름은 `sector-행-열`이다. 행 0은 홈에 가까운 남쪽 줄, 1은 먼 북쪽 줄 / 열 0은 서쪽, 1은 동쪽.

| 섹터 | 위치 (홈 기준) | 홈→중심 | 탐색 고도 | 지그재그 |
|---|---|---|---|---|
| sector-0-0 | 북 50~175m, 동 50~175m | 약 159m | 30m | 3줄 |
| sector-0-1 | 북 50~175m, 동 175~300m | 약 263m | 40m | 3줄 |
| sector-1-0 | 북 175~300m, 동 50~175m | 약 263m | 50m | 3줄 |
| sector-1-1 | 북 175~300m, 동 175~300m | 약 336m | 60m | 3줄 |

- 한 변 125m: 촬영 폭 60m로 지그재그 3줄이 나온다 (100m면 2줄뿐이다).
- 검사 기준: 섹터 안과 두 기체 출발점에서 섹터 꼭짓점까지 직선 경로 주변의 가장 높은 물체보다 탐색 고도가
  15m 이상 높을 것. 결과는 `python3 make_sections.py` 출력에 나온다.
- 섹터마다 탐색 고도가 10m씩 달라, 어느 드론이 어느 섹터를 맡아도 두 드론의 비행 고도가 겹치지 않는다.
- AirSim 화면에 섹터 경계 그리기: `python3 map_probe.py draw scenarios/missions/mission-101.json`
  (선만. 지그재그 경로는 `--paths`, 이름 글씨는 `--labels`. AirSim을 다시 켜면 지워지므로 시나리오를 돌릴 때마다
  그린다. `run_live.sh`는 자동으로 그린다)
- 미니맵 이름: 스폰(홈)에서 가장 먼 섹터를 북으로 잡아 **서 북 / 남 동** (sector-1-0 서, sector-1-1 북,
  sector-0-0 남, sector-0-1 동). 미니맵 글씨만이고 임무 파일·LLM 이 쓰는 이름은 sector-행-열 그대로다.

## 정상 운용 + 장애 시나리오 3개 (11/9 팀 회의)

넷 다 **같은 임무(mission-101)·같은 섹터(남 sector-0-0, 북 sector-1-1)·같은 미니맵(섹터 4개)·같은 첫 명령**으로 돈다.
흐름은 모두 시뮬(장애) → LLM(배정·재배정) → 시뮬(비행·귀환)이다. 정상 운용(0)은 장애 없이 같은 명령을 돌린 기준이다.

첫 명령: "남쪽과 북쪽 섹터를 드론 두 대로 하나씩 맡아 수색해 줘. 어느 드론이 어느 섹터를 맡을지는 드론 상태를 보고
정해 줘." LLM 은 mission-101 설명(instruction)의 규칙으로 정한다: 배터리가 적은 드론일수록 가까운 섹터, 배터리가
비슷하면(10%p 안) drone-01 에 남·drone-02 에 북. 그래서 배터리 장애가 없으면 drone-01 남, drone-02 북이다.

| # | 시나리오 (회의 메모) | 장애 걸기 (`fault.py`) | 흐름 | 명령 |
|---|---|---|---|---|
| 0 | 정상 운용 (기준) | 없음 (`fault.py clear`) | drone-01 남, drone-02 북 → 둘 다 끝까지 수색(이동 7/7) → 홈 착륙. 통신 두절·미도달·비상 정지 없음 | `run_live.sh demo normal` |
| 1 | 통신 두절: 거리가 멀면 드론을 다루지 않는다, 악화 → return home | 두 드론 모두 `link <드론> range --degraded-m 250 --down-m 330` (지상국에서 멀수록 나빠지는 환경) | 북으로 간 drone-02 만 250m 부터 링크 품질이 내려가고 330m 에서 끊김 → 5초 뒤 기체 귀환(RTL) → 미들웨어 `LINK_LOST` → 복구되면 귀환 구간·착륙. 남의 drone-01 (최대 230m)은 성공 | `run_live.sh demo link` |
| 2 | 배터리 부족: timeout 처럼 처리, 한 대만 부족, full·부족 두 대 → 우선순위 기준 업무 처리 | `battery drone-01 0 --from 40 --drain 140 --rtl-at 20` (계획 전) | LLM 이 40% drone-01 에 가까운 남, 100% drone-02 에 먼 북 → drone-01 이 비행 중 20% 에서 기체 배터리 failsafe 귀환 → 미들웨어는 목표에 못 가는 것으로 보고 `WAYPOINT_TIMEOUT` → 귀환 구간·홈 착륙 → "못 끝낸 섹터 다시" → drone-02 가 남 수색 | `run_live.sh demo battery` |
| 3 | 구역 나눠 수색 중 웨이포인트에 일정 시간 안에 못 가면 return, 신규 배정 | `stall drone-02 --after-takeoff 20` (다음 goto 1번 무시) | drone-02 가 북 섹터 입구에서 다음 웨이포인트를 못 받음 → 60초 뒤 `WAYPOINT_TIMEOUT` → 미들웨어가 귀환 구간·홈 착륙 → "못 끝낸 섹터 다시" → drone-01 이 북 수색 | `run_live.sh demo timeout` |

```bash
bash run_live.sh                 # 터미널 1: 미들웨어 + LLM + AirSim 섹터 선·지나간 길 + 미니맵 (LLM 키 입력)
bash run_live.sh demo link       # 터미널 2: 장애 걸기 → 명령 → 끝까지 따라가며 판정 (normal, battery, timeout 도 같다)
bash run_live.sh demo link --no-llm   # LLM 없이 같은 배정으로 (터미널 1 은 bash run_live.sh --no-llm 이어도 된다)
python3 mission_check.py --report     # 4개가 지금 설정으로 검증됐는지, 안 됐으면 무엇을 돌릴지
```

- `demo normal` 은 장애를 지우고 같은 명령을 보내, 배정 규칙(drone-01 남·drone-02 북) → 두 드론 모두 이동 7/7 →
  홈 착륙 → 통신 두절·미도달·비상 정지 없음을 본다. 실패한 드론이 없으므로 끝나고 다시 켜지 않고 장애 시나리오
  하나를 이어서 할 수 있다 (시연: 정상 → 장애)
- `demo` 는 끝에 `판정(1. 통신 두절): 통과/실패` 처럼 낸다. 장애 드론의 작업 실패는 정상이고, 대신 이것을 본다:
  장애가 실제로 걸렸다(`fault-events`) → 미들웨어가 알아채 멈췄다(`LINK_LOST`·`WAYPOINT_TIMEOUT`) → 장애 드론이 홈으로
  돌아와 내렸다(두절 처리 끝 `landed` / `timeout_return_finished`) → 다른 드론은 성공 → (2·3) 재배정 계획이 수락돼 성공
- 2·3 은 1단계(장애 → 귀환·착륙, 다른 드론 성공)가 맞을 때만 재배정 명령("장애로 못 끝낸 섹터를 지금 쓸 수 있는 드론에
  다시 배정해서 수색해 줘")을 보낸다. LLM 은 `get_mission_status` 에서 failed 로 끝난 작업의 area_id 를 찾아
  available 인 드론에 준다 (mission-101 설명). 실패한 드론은 `unavailable` 이라 배정되지 않는다
- 웨이포인트 미도달·배터리 부족에서 "return 시키고" 는 팀 미들웨어 `dispatcher.py` 가 한다: `WAYPOINT_TIMEOUT` 이고
  텔레메트리가 들어오면 그 자리에 내리지 않고 귀환 구간(`goto(home)`)을 보낸 뒤 착륙 (이 PR 에서 넣음)
- 장애 시나리오마다 장애 드론이 `unavailable` 로 남으므로 다음 시나리오 전에 `run_live.sh` 를 끄고 AirSim →
  `start_hetero_sim.sh` → `run_live.sh` 를 다시 켠다. `demo` 가 시작 전에 확인하고 알려 준다
- 아직 안 하는 것: 실패한 섹터 자동 재배정 (지금은 사용자가 LLM 에 "못 끝낸 섹터 다시"라고 말한다)

장애 값은 텔레메트리 샘플마다(약 4Hz) 이어서 바뀐다. 미들웨어가 받는 값과 규칙은 `INTERFACE.md` 2-9.
LLM 없이 리허설: `python3 run_plan.py mission-101 drone-01=sector-0-0 drone-02=sector-1-1`
(상태 줄에 미들웨어가 받은 배터리 값이 바뀔 때마다 나온다)

### 실제 운용 (`run_live.sh`, 시험 아님)

AirSim(산악맵)과 `start_hetero_sim.sh` 가 떠 있는 상태에서 한 번에 띄운다: 장애 주입 지우기 → AirSim 섹터 경계 →
미들웨어 서버(127.0.0.1:9000) → 드론 지나간 길 → 미니맵(섹터 4개 전부) → LLM 오케스트레이터(127.0.0.1:8001).

```bash
bash run_live.sh                 # LLM 키(GOOGLE_API_KEY)를 물으면 입력 (화면에 안 보임)
bash run_live.sh check           # 다른 터미널: 비행 없이 점검 (오케스트레이터·미들웨어 응답, LLM 이 도구로 두 드론을 읽는지)
bash run_live.sh say "북쪽과 남쪽 섹터를 드론 두 대로 하나씩 맡아 수색해 줘"   # 자연어 명령 + 끝까지 판정
bash run_live.sh demo link        # 정상 운용·장애 시나리오 (normal | link | battery | timeout, 위 표)
```

- `say` 는 LLM 답을 보여 준 뒤 미들웨어 이벤트 기록을 따라가며 계획 수락·이륙·이동 k/N·착륙을 한 줄씩 보여 주고
  `판정: 통과` 또는 `판정: 실패` 와 이유(거부 code, 명령 실패 code)를 낸다 (`mission_check.py`). 통과 조건: 계획 수락,
  모든 명령 succeeded, 작업 succeeded. Ctrl+C 로 따라가기를 멈춰도 비행은 계속되고, 나중에 `python3 mission_check.py`
  로 판정만 볼 수 있다 (지난 기록: `python3 mission_check.py ~/skybuddy/logs/live_<시각>`)
- 섹터는 동서남북으로 말해도 된다. mission-101 설명(instruction)에 sector-0-0 남, 0-1 동, 1-0 서, 1-1 북이 적혀 있어
  LLM 이 미니맵과 같은 이름으로 고른다
- `check` 는 LLM 에 읽기만 하는 질문을 한 번 보낸다 (계획 제출 안 함). 시작할 때 오케스트레이터 가상환경의 `mcp` 가
  2.x 인지, `google-genai` 가 있는지도 본다
- 말에 임무 ID 가 없으면 `mission-101 임무: ` 를 붙여 보낸다 (시스템 프롬프트(#45)가 어느 임무인지 되묻지 않게)
- LLM 없이: `bash run_live.sh --no-llm` 후 `bash run_live.sh demo <시나리오> --no-llm` 또는
  `python3 run_plan.py mission-101 drone-01=sector-0-0 drone-02=sector-1-1`
- 오케스트레이터 가상환경이 없으면 설치 명령을 알려 주고 끝난다 (팀 문서 `docs/development/local-full-pipeline.md`)
- 착륙한 뒤 Ctrl+C 로 전부 끈다. 기록: `~/skybuddy/logs/live_<시각>/`

지난 기록 다시 판정: `python3 mission_check.py ~/skybuddy/logs/live_<시각> --scenario battery`.

미들웨어에 넘긴 값 보기: `python3 fault.py stream --every 4` (약 1초에 1줄), 실시간은 `--follow`,
그래프용은 `--csv 파일`. 기록 폴더(`~/skybuddy/logs/live_<시각>/`)에는 `telemetry.jsonl` 로 남는다.

## AirSim 화면 (영상용)

- 섹터 경계: `python3 map_probe.py draw scenarios/missions/mission-101.json` — 선만 (두께 15, `--thickness`).
  글씨·지그재그 경로는 드론을 가려서 기본으로 안 그린다 (`--labels`, `--paths`). 이미 떠 있는 글씨는
  AirSim 을 다시 켜야 지워진다
- 섹터 선 색은 동서남북 이름으로 정한다: **북 빨강, 남 파랑, 동 주황, 서 하늘** (미니맵도 같은 색. 초록·연노랑은
  드론 색이라 안 쓴다. 2x2 가 아닌 임무는 순서대로)
- 드론이 지나간 길: `python3 track_drones.py` — drone-01 초록, drone-02 연노랑 가는 선 (두께 6, `--trail-thickness`).
  드론 위 기둥·이름표는 기본으로 안 그린다 (`--beacon 50`, `--labels` 로 켤 수 있다). 배터리·링크는 미니맵에서 본다
- AirSim 키 (1.8.1 소스 기준): `B` 뒤따라가기, `F` 1인칭, `\` 지상 관찰자, `M` 수동 카메라(화살표·W/A/S/D로 이동),
  `1`·`2`·`3` 작은 창. `B`·`F`는 FPV 기체 한 대만 따라가고 실행 중에 바꾸는 키는 없다. 기본은 첫 기체(drone-01).
- **작은 창 배치**: `python3 track_drones.py layout` → AirSim 완전히 껐다 켜기. `start_hetero_sim.sh` 가 시작할 때
  settings.json 을 검사해 배치가 다르면 고치고 AirSim 을 다시 켜라고 멈춘다 (`layout --check` 로 보기만 할 수 있다)
  - 화면 아래 줄: **왼쪽 = drone-01 카메라(초록 테두리), 가운데 = drone-02 카메라(연노랑), 오른쪽 = 미니맵**.
    AirSim 작은 창 1(왼쪽) = drone-01, 2(가운데) = drone-02 촬영 카메라, 3(오른쪽) = 숨김(미니맵 자리)
  - 작은 창의 자리와 테두리 색은 AirSim 안에 고정돼 있다 (1 왼쪽 초록, 2 가운데 연노랑, 3 오른쪽 보라.
    settings.json 에는 어느 카메라를 띄울지만 있다). 그래서 **드론 색을 테두리 색에 맞췄다: drone-01 초록,
    drone-02 연노랑** (지나간 길·미니맵 핀 모두)
  - 촬영 카메라 `search_cam`: 기체 앞쪽, 30도 숙임(`--pitch`), 640x360. **짐벌(Stabilization 1)이 숙임·옆기울기를
    고정해 기체가 가감속으로 기울어도 화면이 위아래로 흔들리지 않는다** (AirSim 1.8.1 `APIPCamera::Tick`)
  - settings.json 을 백업한 뒤 고친다 (ApiServerPort 등 기존 값은 그대로). 팀 참고 설정 `config/airsim_2vehicle.json`
    에도 같은 값이 들어 있다
  - 숨긴 창 3 은 보이는 창과 다른 카메라(`back_center`)를 가리킨다. 같은 카메라면 AirSim 이 숨긴 창을 처리하면서 그
    카메라 촬영을 꺼 보이는 창이 멈춘다 (1.8.1 `SimHUD::updateWidgetSubwindowVisibility`. 예전 배치에서 3 키로 창 3 을
    켜야만 창 1 이 움직이던 증상)
- 따라갈 드론 바꾸기: `python3 track_drones.py view --follow drone-02` (settings.json 수정·백업, AirSim 재시작 필요)
- 두 드론 실제 속도는 같다: 수평 10 m/s, 상승 4·하강 3·착륙 마지막 1.0 m/s (`config/mavlink_drone.parm`·`config/dds_drone.parm`,
  `start_hetero_sim.sh` 가 넣는다). drone1(4.3)은 실제 속도가 설정의 약 0.555 배라 설정값이 다르다 (`INTERFACE.md` 3-2)
- 속도 확인: 비행 뒤 `python3 speed_check.py` (가장 최근 기록). 같은 길이 구간 시간·유지 속도를 드론별로 비교하고,
  다르면 drone1 값을 얼마로 고칠지 알려 준다

### 미니맵 (`minimap.py`)

AirSim 에는 2D 지도(미니맵) 기능이 없다 (화면에 겹쳐 띄울 수 있는 것은 기체 카메라 작은 창뿐). 그래서 지정한
섹터와 드론 위치만 그린 지도를 브라우저 창으로 띄운다. 게임 미니맵처럼 AirSim 화면 오른쪽 아래(숨긴 작은 창 3 자리)에 둔다.

- 켜기: `python3 minimap.py` (mission-101 섹터 4개 전부. `run_live.sh` 가 자동으로 켠다. 일부만은 `--sectors`)
- 켜면 그 컴퓨터에 브라우저 작은 창이 저절로 열린다 (Windows·WSL = Edge, Mac = Chrome·Edge, 리눅스 데스크톱 =
  Chrome·Chromium. 이미 열어 둔 화면이 있으면 새로 안 연다. 직접 열 때는 `http://localhost:8765`)
- **Windows·WSL 에서는 그 창을 주 화면 오른쪽 아래(숨긴 AirSim 작은 창 3 자리)에 항상 위로 저절로 고정한다**
  (PowerShell `SetWindowPos`, 크기 `--window 360x430`, 왼쪽 아래는 `--corner left`, 끄려면 `--no-pin`).
  버튼 줄 없이 지도와 데이터만 나온다
- 다른 곳(Mac 등)에서는 **항상 위에 띄우기** 버튼 → 작은 창을 오른쪽 아래로 끌어다 둔다
  (Document Picture-in-Picture, Chrome·Edge 116 이상)
- 화면 없는 서버(AWS 리눅스)에서 켜면 창 대신 `http://<서버 주소>:8765` 를 출력한다. AirSim 화면이 있는 컴퓨터의
  Edge 에서 그 주소를 한 번 연다 (AWS 보안 그룹에서 TCP 8765 허용, 또는 `ssh -L 8765:localhost:8765 <서버>`).
  프로그램은 자기가 도는 컴퓨터에만 창을 띄울 수 있다
- WSL 에서 창이 안 뜨고 `Exec format error` 가 나오면 Windows 프로그램 실행(interop)이 꺼진 것이다. systemd 를 켠
  WSL 에서 흔하다: `/usr/lib/binfmt.d/WSLInterop.conf` 에 `:WSLInterop:M::MZ::/init:PF` 를 넣고 PowerShell 에서
  `wsl --shutdown` 후 다시 켠다. 안 고쳐도 주소를 직접 열면 똑같이 쓴다
- 녹화(OBS): 브라우저 소스 `http://localhost:8765/?overlay=1` (배경 투명, 버튼 없음. 지도만은 `&data=0`)
- 화면은 지도 + 아래 데이터 줄 (위쪽 제목 줄 없음. 문제가 있을 때만 왼쪽 위에 주황 글씨)
  - N 화살표는 섹터 이름과 맞춰 스폰에서 '북' 섹터(sector-1-1) 쪽, 즉 오른쪽 위를 가리킨다 (지도 자체는 진북이 위)
  - 지도: 섹터(이름 서·북·남·동 + 탐색 고도, 색 북 빨강·남 파랑·동 주황·서 하늘 = AirSim 섹터 선과 같음)·지그재그
    수색 경로(점선)·격자·북쪽·축척·홈(H)·드론 핀(번호, drone-01 초록·drone-02 연노랑)과 지나간 길(가는 선). 비행 중인데 텔레메트리 기록이 3초 넘게
    끊기면 핀 둘레가 빨갛게 깜박인다
  - 아래 줄: 드론별 고도·배터리·링크 품질 = 미들웨어가 받은 값 (비행 중 끊기면 `끊김 N초`. 이륙 전·착륙 뒤에는
    미들웨어가 값을 안 읽으므로 마지막 값을 흐리게). **지도만 보기** 버튼 (작은 창에서는 지도 두 번 누르기,
    주소로는 `?data=0`) 으로 숨긴다
- 위치는 AirSim 참값이라 통신이 끊긴 동안에도 실제 위치가 보인다 (1초에 3번 읽는다. AirSim API 호출이 많으면
  화면이 끊겨 5번에서 줄였다). `--stream` 을 안 주면 가장 최근 텔레메트리 기록을 따라간다
- 시뮬레이터 없이 모양만 보기: `python3 minimap.py --demo`, 점검: `python3 minimap.py --selftest`
- 브라우저에서 안 열리면 `--bind 0.0.0.0` 으로 다시 켠다 (WSL 네트워크 설정에 따라 다르다)

## AirSim 과 SITL 이 다른 기계일 때 (AWS 등)

맵은 **LandscapeMountains** 다 (섹터·고도 검사가 전부 이 맵 기준). AirSim 공식 배포본 v1.8.1-windows 의
`LandscapeMountains.zip` 과 같은 파일이다:
https://github.com/microsoft/AirSim/releases/download/v1.8.1-windows/LandscapeMountains.zip

| 어디 | 무엇 |
|---|---|
| AirSim 기계 settings.json | `config/airsim_2vehicle.json` 에서 Copter1·Copter2 의 `UdpIp` = SITL 기계 주소, `LocalHostIp` = `0.0.0.0` (ArduPilot AirSim 문서의 "Run on different machines") |
| SITL 기계 | `export AIRSIM_HOST=<AirSim 기계 주소>` 후 `bash start_hetero_sim.sh` (팀 문서 `aws-deployment.md` 와 같은 이름. `SIM_ADDRESS` 도 된다) |
| 방화벽·보안 그룹 | UDP 9012·9022 (SITL → AirSim), UDP 9013·9023 (AirSim → SITL), TCP 41451 (SITL 쪽 도구 → AirSim API) |
| 섹터 그리기·드론 표시 | `export AIRSIM_HOST=<AirSim 기계 주소>` 후 `run_live.sh` (map_probe.py·track_drones.py·minimap.py 가 이 주소로 붙는다) |

## 주의

- **비상 착륙한 자리가 다음 이륙의 홈이 된다.** 미들웨어는 계획 시점의 드론 위치를 홈으로 쓰고, 탐색 고도는
  그 홈 기준이다. 2026-11-09 시험에서 웨이포인트 미도달로 비상 착륙한 drone-02는 홈 남쪽 언덕(출발점보다
  24m 높음)에 내려앉았다. 반대로 호수 쪽 낮은 곳에 내린 드론을 다시 띄우면 같은 고도 숫자라도 실제 높이가
  낮아져 홈 근처 지형에 가까워진다. **장애 시나리오 사이에는 AirSim과 SITL을 다시 켜서 출발점으로
  돌려놓는다.**
- 통신 두절은 미들웨어 #42가 처리한다 (goto 중 5초 무응답 → `LINK_LOST` → 20초 복구 대기). 시뮬레이션에서는
  두절 중 기체 failsafe(RTL)를 장애 주입이 흉내 낸다 (`INTERFACE.md` 2-9).
- 배터리 부족 귀환은 기체 배터리 failsafe(`BATT_FS_LOW_ACT=2` RTL)를 장애 주입이 흉내 낸다 (`--rtl-at`).
  ArduPilot 기본값은 "경고만"이라, 설정하지 않은 실기체는 스스로 돌아오지 않는다. `--rtl-at` 없이 `--drain` 만 쓰면
  예전처럼 0% 에서 그 자리 강제 착륙이다 (감소율은 임무 한 번에 맞춰 압축한 값, `INTERFACE.md` 2-9-1).
- 섹터 아래(호수 쪽)는 지형이 홈보다 20m 넘게 낮아서, 거기서 그 자리 착륙하면 내려오는 데 오래 걸린다
  (2026-11-09 실측: 30m→10m 는 약 1.6m/s, 홈 기준 10m 아래부터는 약 0.3m/s. 지금 착륙 속도 설정은 그 두 배 이상).
  그래서 장애 시나리오 2·3 은 그 자리에 내리지 않고 홈으로 돌아와 내린다.
- 장애가 난 드론은 미들웨어에서 `unavailable`로 남는다. 다음 계획을 내려면 서버를 다시 켠다.
