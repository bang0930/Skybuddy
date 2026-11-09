# 탐색 섹터와 장애 시나리오

담당: 이태우 (시뮬레이션). 미들웨어 코드는 바꾸지 않고, 설정 파일과 환경 변수로만 붙는다.

경로와 명령은 개발 환경(`~/skybuddy`, 평면 구조) 기준이다. 팀 저장소에서는 이 폴더가
`services/simulation/scenarios/`, 스크립트(`fault.py`, `map_probe.py`, `run_plan.py`, `track_drones.py`,
`scenario_check.sh`)가 `services/simulation/scripts/` 에 있다. `drones.sitl.json` 은 팀 저장소 사본에서
`simulation_app_path` 를 빼서 미들웨어 기본값(같은 저장소의 `services/simulation/app`)을 쓴다.

## 파일

| 파일 | 내용 |
|---|---|
| `missions/mission-101.json` | 산악맵 홈 북동쪽 2x2 섹터 (`make_sections.py`가 만든다) |
| `missions/mission-001.json` | 팀 저장소 원본 사본 (`check_mvp_pipeline.py`가 이 임무를 쓴다) |
| `drones.sitl.json` | 미들웨어 드론 설정. 지그재그 수색(`coverage`), 촬영 폭 60m, 장애 주입이 들어간 어댑터 경로 |

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
- AirSim 화면에 섹터 경계와 지그재그 경로 그리기: `python3 map_probe.py draw scenarios/missions/mission-101.json`
  (AirSim을 다시 켜면 지워지므로 시나리오를 돌릴 때마다 그린다. `scenario_check.sh`는 자동으로 그린다)

## 시나리오별 쓰임

장애 값은 텔레메트리 샘플마다(약 4Hz) 이어서 바뀐다. 미들웨어가 받는 값과 규칙은 `INTERFACE.md` 2-9.

| 시나리오 | 장애 걸기 (`fault.py`) | 섹터 | 샘플마다 바뀌는 값 |
|---|---|---|---|
| 배터리 소진 | `battery <드론> 0 --drain 120` (계획 전에) | `sector-0-0` | 비행 중에만 잔량·전압 감소, 0%면 강제 착륙 |
| 배터리 부족 (고정) | `battery <드론> 25` (계획 전에) | 가까운 `sector-0-0`과 먼 `sector-1-1` 중 고르게 한다 | 잔량 25%, 전압 11.25V |
| 통신 두절 (거리) | `link <드론> range --degraded-m 250 --down-m 330` | 가장 먼 `sector-1-1`을 맡긴다 | 링크 품질 100 → 0, 250m부터 샘플 드물게, 330m부터 끊김 |
| 통신 두절 (시간) | `link <드론> down --after-takeoff 15 --for 20` | 아무 섹터 | 샘플 없음 |
| 웨이포인트 미도달 | `stall <드론> --after-takeoff 5` | 실패한 드론의 섹터를 다른 드론에 다시 배정 | 위치가 목표 쪽으로 안 움직임 |

LLM 없이 리허설: `python3 run_plan.py mission-101 drone-01=sector-0-0 drone-02=sector-1-1`
(상태 줄에 미들웨어가 받은 배터리 값이 바뀔 때마다 나온다)

`scenario_check.sh` 로 한 번에 돌린다. 모드마다 미들웨어 서버를 띄우고, 섹터와 경로를 그리고, 드론 위치를 색으로
표시하고(`track_drones.py`), 끝나면 기록을 요약한다.

| 명령 | 내용 |
|---|---|
| `bash scenario_check.sh sections` | 장애 없이 sector-0-0·sector-0-1 지그재그 수색 (모두 성공이 정상) |
| `bash scenario_check.sh battery` | drone-01 배터리 비행 120초에 0% (섹터 가운데 30%, 마지막 줄 0% → 강제 착륙) |
| `bash scenario_check.sh link` | drone-02 가 sector-1-1 로 가며 링크 품질 하락 → 330m 두절 → failsafe RTL → 복구 후 귀환 |
| `bash scenario_check.sh faults` | mission-001 로 통신 두절 20초 + 웨이포인트 미도달 |

미들웨어에 넘긴 값 보기: `python3 fault.py stream --every 4` (약 1초에 1줄), 실시간은 `--follow`,
그래프용은 `--csv 파일`. 시나리오 기록 폴더에는 `telemetry_<시험>.jsonl` / `.csv` 로 남는다.

## AirSim 화면 (영상용)

- 섹터 경계·지그재그 경로: `python3 map_probe.py draw scenarios/missions/mission-101.json`
- 드론 위치: `python3 track_drones.py` — drone-01 자홍, drone-02 주황. 지나간 길이 선으로 남고, 드론 위로 기둥과
  이름표(배터리·링크 품질)가 따라다닌다. 멀리서 섹터 전체를 볼 때 두 드론을 구분하려는 것이다.
- AirSim 키 (1.8.1 소스 기준): `B` 뒤따라가기, `F` 1인칭, `\` 지상 관찰자, `M` 수동 카메라(화살표·W/A/S/D로 이동),
  `1`·`2`·`3` 작은 창. `B`·`F`는 FPV 기체 한 대만 따라가고 실행 중에 바꾸는 키는 없다. 기본은 첫 기체(drone-01).
- 따라갈 드론 바꾸기: `python3 track_drones.py view --follow drone-02` (settings.json 수정·백업, AirSim 재시작 필요)
- drone-01 을 따라가면서 drone-02 카메라를 작은 창에: `python3 track_drones.py view --follow drone-01 --subwindow drone-02`
  (재시작 후 `1` 키)

## 주의

- **비상 착륙한 자리가 다음 이륙의 홈이 된다.** 미들웨어는 계획 시점의 드론 위치를 홈으로 쓰고, 탐색 고도는
  그 홈 기준이다. 2026-11-09 시험에서 웨이포인트 미도달로 비상 착륙한 drone-02는 홈 남쪽 언덕(출발점보다
  24m 높음)에 내려앉았다. 반대로 호수 쪽 낮은 곳에 내린 드론을 다시 띄우면 같은 고도 숫자라도 실제 높이가
  낮아져 홈 근처 지형에 가까워진다. **장애 시나리오 사이에는 AirSim과 SITL을 다시 켜서 출발점으로
  돌려놓는다.**
- 통신 두절은 미들웨어 #42가 처리한다 (goto 중 5초 무응답 → `LINK_LOST` → 20초 복구 대기). 시뮬레이션에서는
  두절 중 기체 failsafe(RTL)를 장애 주입이 흉내 낸다 (`INTERFACE.md` 2-9).
- 배터리 0% 전에 귀환시키는 것은 아직 아무도 하지 않는다 (ArduPilot 배터리 failsafe 기본값은 경고만).
  지금 `battery` 시나리오가 강제 착륙으로 끝나는 것은 정상이고, 미들웨어·LLM 이 귀환을 붙이면 달라진다.
  감소율을 임무 한 번에 맞춰 압축했기 때문에 귀환에 드는 잔량 몫도 크다 (`INTERFACE.md` 2-9-1 계산 예).
- 섹터 아래(호수 쪽)는 지형이 홈보다 20m 넘게 낮아서, 거기서 강제 착륙하면 내려오는 데 2분 넘게 걸린다
  (2026-11-09 실측: 30m→10m 는 약 1.6m/s, 홈 기준 10m 아래부터는 약 0.3m/s). 그날은 그 사이 미들웨어 비상
  정지가 공중 disarm 을 보냈다. 어댑터 `land()` 가 내려가는 중이면 더 기다리도록 고쳤다 (`INTERFACE.md` 2-1-2).
- 장애가 난 드론은 미들웨어에서 `unavailable`로 남는다. 다음 계획을 내려면 서버를 다시 켠다.
