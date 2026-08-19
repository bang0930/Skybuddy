# Docker 개발 환경

이 문서는 SkyBuddy의 초기 Docker Compose 개발 환경을 실행하고 확인하는 방법을 설명합니다.

현재 Compose 환경은 다음 두 서비스를 실행합니다.

| 서비스 | 호스트 포트 | 역할 |
|---|---:|---|
| `middleware` | `8000` | 오케스트레이터와 시뮬레이터 사이의 데이터 검증 및 변환 |
| `orchestrator` | `8001` | Mission Context를 구조화된 Mission Plan으로 변환 |

AirSim과 실제 LLM은 아직 Compose에 포함하지 않습니다. 현재 단계의 목적은 서비스별 컨테이너가
동일한 환경에서 기동되고 서로 통신할 수 있는 기반을 검증하는 것입니다.

## 사전 준비

- Docker Desktop 또는 Docker Engine
- Docker Compose v2

설치 확인:

```bash
docker --version
docker compose version
```

## 환경 변수 준비

로컬 환경 변수 파일을 생성합니다.

```bash
cp .env.example .env
```

`.env`에는 개인 환경에 맞는 값을 작성할 수 있지만 Git에는 포함하지 않습니다.

## 설정 검증

이미지를 빌드하기 전에 Compose 문법과 최종 적용 값을 확인합니다.

```bash
docker compose config
```

## 실행

```bash
docker compose up --build
```

백그라운드에서 실행하려면 다음 명령을 사용합니다.

```bash
docker compose up --build -d
```

## 동작 확인

```bash
curl http://localhost:8000/health
curl http://localhost:8001/health
docker compose ps
```

예상 응답:

```json
{"status":"ok","service":"middleware"}
```

```json
{"status":"ok","service":"orchestrator"}
```

FastAPI가 자동 생성한 API 문서는 다음 주소에서 확인할 수 있습니다.

- Middleware: <http://localhost:8000/docs>
- Orchestrator: <http://localhost:8001/docs>

## 로그 확인

전체 서비스 로그:

```bash
docker compose logs -f
```

서비스 하나의 로그:

```bash
docker compose logs -f middleware
```

## 종료

```bash
docker compose down
```

## 개발 중 코드 반영

Compose는 각 서비스의 `app` 디렉터리를 읽기 전용 볼륨으로 연결하고 Uvicorn을 `--reload` 모드로
실행합니다. 따라서 Python 파일을 수정하면 일반적으로 이미지를 다시 빌드하지 않아도 서비스가
자동으로 재시작됩니다.

의존성 파일인 `requirements.txt`를 변경한 경우에는 이미지를 다시 빌드해야 합니다.

```bash
docker compose up --build
```

## 현재 범위 밖의 구성

- AirSim 및 ArduPilot SITL 실행
- 실제 LLM API 연결
- MCP 서버
- Kafka 및 Spark
- 데이터베이스

위 구성은 기본 API 계약과 서비스 간 호출 흐름이 확정된 이후 별도 Issue와 브랜치에서 추가합니다.
