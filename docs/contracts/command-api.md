# 명령 및 API 계약 초안

이 문서는 SkyBuddy 미들웨어가 프로토콜별 Adapter와 교환할 명령·결과 계약 및 향후
HTTP·MCP 경계를 정의합니다. 이번 단계에서는 계약만 정의하며 실제 API 라우트, Dispatcher,
MAVLink 및 AP_DDS 연결은 구현하지 않습니다.

실행 가능한 원본 계약은 다음 Pydantic 모델입니다.

- `services/middleware/app/schemas/mission.py`
- `services/middleware/app/schemas/command.py`

## 공통 명령

`DroneCommand`는 명령 식별자, 소속 임무와 작업, 대상 드론, 대상 프로토콜, 생성 시각과
명령별 payload를 포함합니다. payload는 `command_type`을 discriminator로 사용하는 JSON
Schema `oneOf` 계약입니다.

지원 명령은 다음과 같습니다.

| 명령 | payload | 의미 |
|---|---|---|
| `takeoff` | `altitude_m`, `altitude_reference` | 명시한 기준의 지정 고도로 이륙 |
| `goto` | `target` | 고도 기준이 포함된 WGS84 위치로 이동 |
| `land` | 없음 | 현재 위치에서 착륙 |
| `return_home` | 없음 | 홈으로 복귀 |
| `disarm` | 없음 | 비상 또는 실패 복구를 위한 시동 해제 |

```json
{
  "command_id": "command-001",
  "mission_id": "mission-001",
  "task_id": "task-001",
  "drone_id": "drone-01",
  "protocol": "mavlink",
  "payload": {
    "command_type": "goto",
    "target": {
      "latitude": 37.45,
      "longitude": 127.12,
      "altitude_m": 30,
      "altitude_reference": "home_relative"
    }
  },
  "created_at": "2026-09-19T10:00:00+09:00"
}
```

`MissionTask`는 동일한 mission, task, drone, protocol에 속하는 명령을 실행 순서대로
묶습니다. 서로 다른 대상·프로토콜의 명령이나 중복 `command_id`가 포함되면 계약 검증에
실패합니다.

## 명령 상태 및 결과

| 상태 | 의미 |
|---|---|
| `pending` | 아직 프로토콜 계층에 전달되지 않음 |
| `sent` | 메시지·토픽·서비스 요청을 전송함 |
| `accepted` | 프로토콜 계층에서 명시적인 수락 응답을 받음 |
| `executing` | 텔레메트리에서 실제 동작 시작을 확인함 |
| `succeeded` | 텔레메트리 또는 완료 응답으로 목표 달성을 확인함 |
| `failed` | 명시적인 거부 또는 실행 실패를 확인함 |
| `timed_out` | 정해진 시간 안에 응답 또는 완료를 확인하지 못함 |
| `unsupported` | 대상 프로토콜이나 기체가 명령을 지원하지 않음 |

`sent`, `accepted`, `succeeded`는 서로 같은 의미가 아닙니다. MAVLink `COMMAND_ACK` 또는
AP_DDS 서비스 응답은 명령 수락 근거가 될 수 있지만 실제 기체 동작 완료는 텔레메트리로
별도 확인해야 합니다. 명시적인 ACK가 없는 fire-and-forget 명령은 `sent`에서
`executing` 또는 `succeeded`로 이동할 수 있으며 `accepted`를 임의로 생성하지 않습니다.

실패·시간 초과·미지원 결과에는 `CommandError`가 필수이며, 그 밖의 상태에는 오류를 넣지
않습니다. 완료된 상태에서 실행 중 상태로 되돌아가는 등 잘못된 전이는 거부합니다.

```json
{
  "command_id": "command-001",
  "drone_id": "drone-01",
  "protocol": "mavlink",
  "previous_status": "sent",
  "status": "failed",
  "updated_at": "2026-09-19T10:00:02+09:00",
  "error": {
    "code": "COMMAND_REJECTED",
    "message": "Vehicle rejected the command",
    "retryable": false
  }
}
```

## HTTP API 초안

라우트 구현 시 아래 모델을 요청·응답 본문으로 재사용합니다.

| 작업 | 초안 Endpoint | 요청 | 성공 응답 |
|---|---|---|---|
| 드론 상태 조회 | `GET /drones/{drone_id}/state` | 없음 | `DroneState` |
| 임무 작업 제출 | `POST /missions/{mission_id}/tasks` | `MissionTask` | `202 Accepted`와 최초 `CommandResult` 목록 |
| 명령 결과 조회 | `GET /commands/{command_id}/result` | 없음 | `CommandResult` |

- URL의 `mission_id`, `drone_id`, `command_id`는 본문 또는 결과의 식별자와 일치해야 합니다.
- 입력 검증 실패는 `422`, 존재하지 않는 식별자는 `404`, 지원하지 않는 명령은 계약상
  `CommandResult(status="unsupported")`로 반환합니다.
- 실제 비동기 실행 방식과 저장소는 Dispatcher·Registry 작업에서 확정합니다.

## MCP Context Schema 초안

MCP Context는 기존 `MissionContext` JSON Schema를 기준으로 하며 각 `DroneState`에 다음을
포함합니다.

- `protocol`: `mavlink` 또는 `ap_dds`
- `capabilities.commands`: 대상이 실제 지원하는 공통 명령
- `capabilities.telemetry_fields`: 대상이 제공할 수 있는 텔레메트리 그룹
- 고도 기준이 명시된 `position`
- 값이 없을 수 있는 선택적 텔레메트리

LLM은 capabilities에 없는 명령을 생성하지 않아야 합니다. 다만 프롬프트 준수만 신뢰하지
않고 미들웨어가 `DroneCommand`와 capabilities를 다시 검증해야 합니다.

## JSON Schema 생성

저장소 루트에서 계약별 JSON Schema를 재현할 수 있습니다.

```bash
PYTHONPATH=services/middleware python -c \
  'import json; from app.schemas import MissionTask; print(json.dumps(MissionTask.model_json_schema(), indent=2))'

PYTHONPATH=services/middleware python -c \
  'import json; from app.schemas import CommandResult; print(json.dumps(CommandResult.model_json_schema(), indent=2))'
```

생성된 Schema는 Pydantic 모델을 원본으로 취급하며 별도의 수동 JSON Schema를 중복 관리하지
않습니다.

## 프로토콜 Adapter 계약

- [AP_DDS 상태 및 명령 변환](./ap-dds-adapter.md)
