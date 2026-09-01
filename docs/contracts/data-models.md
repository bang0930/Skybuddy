# 핵심 데이터 계약

이 문서는 오케스트레이터, 미들웨어, 시뮬레이터 사이에서 교환하는 초기 데이터 형식을 정의합니다.
실행 가능한 원본 계약은 `services/middleware/app/schemas/mission.py`의 Pydantic 모델입니다.

## 계약 원칙

- 모든 식별자는 영문자 또는 숫자로 시작하며 영문자, 숫자, `_`, `-`만 사용합니다.
- 좌표는 WGS84 십진수 위도·경도를 사용하고 고도 단위는 미터입니다.
- 배터리는 `0`부터 `100`까지의 백분율입니다.
- 시각은 UTC 오프셋을 포함한 ISO 8601 형식이어야 합니다.
- 정의하지 않은 필드는 계약 오류로 거부합니다.
- 한 Mission Context 안의 드론과 탐색 구역 식별자는 각각 고유해야 합니다.
- 한 Mission Plan은 같은 드론이나 탐색 구역을 두 번 할당할 수 없습니다.

## Mission Context

미들웨어가 수집·정규화하고 검증한 뒤 오케스트레이터에 제공하는 입력입니다.

```json
{
  "mission_id": "mission-001",
  "instruction": "북쪽 능선을 우선 수색해 줘",
  "drones": [
    {
      "drone_id": "drone-01",
      "position": {
        "latitude": 37.45,
        "longitude": 127.12,
        "altitude_m": 220
      },
      "battery_percent": 82.5,
      "status": "available",
      "connection_status": "connected",
      "observed_at": "2026-08-31T12:00:00Z"
    }
  ],
  "search_areas": [
    {
      "area_id": "area-a",
      "boundary": [
        {"latitude": 37.45, "longitude": 127.12},
        {"latitude": 37.46, "longitude": 127.12},
        {"latitude": 37.45, "longitude": 127.13}
      ],
      "search_altitude_m": 80
    }
  ],
  "requested_at": "2026-08-31T12:00:00Z"
}
```

`DroneState.status`는 다음 값을 사용합니다.

| 값 | 의미 |
|---|---|
| `available` | 새 임무를 받을 수 있음 |
| `assigned` | 현재 임무가 할당됨 |
| `returning` | 복귀 중이므로 새 임무를 받을 수 없음 |
| `unavailable` | 고장 또는 안전상 이유로 임무를 수행할 수 없음 |

`DroneState.connection_status`는 운용 상태와 독립적으로 다음 값을 사용합니다.

| 값 | 의미 |
|---|---|
| `connected` | 미들웨어와 정상적으로 통신 중 |
| `disconnected` | 통신이 끊겼으며 재연결 또는 대기 판단이 필요함 |

통신이 끊긴 드론도 마지막으로 확인된 운용 상태는 유지합니다. 예를 들어 임무 수행 중 통신이
끊기면 `status`는 `assigned`, `connection_status`는 `disconnected`로 표현합니다. 이를 통해
고장이 확인된 `unavailable` 상태와 일시적인 통신 두절에 서로 다른 Fallback을 적용할 수 있습니다.

## Mission Plan

오케스트레이터가 생성하고 미들웨어가 검증하는 구조화 출력입니다. 이 단계에서는 상위 수준의
드론-구역 할당만 표현합니다. 비행 제어 명령 변환은 별도 계층에서 수행합니다.

`priority`는 `1`부터 `100`까지 사용하며 숫자가 높을수록 먼저 실행합니다. 기본값 `1`은 가장
낮은 우선순위입니다.

```json
{
  "mission_id": "mission-001",
  "assignments": [
    {
      "drone_id": "drone-01",
      "area_id": "area-a",
      "priority": 1
    }
  ],
  "generated_at": "2026-08-31T12:00:02Z"
}
```

### 오케스트레이터 연동 시 주의사항

- 한 계획에서 같은 드론 또는 탐색 구역을 두 번 할당하면 미들웨어 검증에 실패합니다.
- 프롬프트만으로 계약 준수를 보장하지 않고, 검증 실패 내용을 오케스트레이터에 전달해 제한된
  횟수만큼 다시 생성하는 흐름이 필요합니다.
- LLM 출력은 같은 입력에서도 달라질 수 있습니다. 재현성과 KPI 분석을 위해 입력, 모델 설정,
  원본 출력, 검증 오류, 재시도 횟수와 최종 결과를 함께 기록해야 합니다.

## JSON Schema 확인

저장소 루트에서 다음 명령으로 현재 모델의 JSON Schema를 확인할 수 있습니다.

```bash
PYTHONPATH=services/middleware python -c \
  'import json; from app.schemas import MissionPlan; print(json.dumps(MissionPlan.model_json_schema(), indent=2))'
```

## 아직 포함하지 않은 검증

- Mission Plan의 식별자가 해당 Mission Context에 실제로 존재하는지 확인하는 교차 검증
- 배터리 임계값에 따른 할당 거부 및 복귀 처리
- 탐색 구역을 MAVLink waypoint 또는 AirSim 명령으로 변환하는 규칙
- 통신 지연과 드론 이탈에 대한 Fallback 및 재할당 정책

위 항목은 이 계약을 입력으로 사용하는 API Gateway와 명령 변환 작업에서 단계적으로 추가합니다.
