# syntax=docker/dockerfile:1

# Python 버전은 Compose의 build args 또는 docker build --build-arg로 바꿀 수 있다.
ARG PYTHON_VERSION=3.11-slim
FROM python:${PYTHON_VERSION}

# .pyc 생성을 막고 로그가 버퍼에 쌓이지 않도록 해 컨테이너 로그를 바로 확인한다.
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

WORKDIR /app

# 의존성 파일을 먼저 복사하면 앱 코드만 변경됐을 때 설치 레이어를 재사용할 수 있다.
COPY services/middleware/requirements.txt ./requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

COPY services/middleware/app ./app

# root가 아닌 별도 사용자로 애플리케이션을 실행한다.
RUN addgroup --system app && adduser --system --ingroup app app \
    && chown -R app:app /app
USER app

EXPOSE 8000

# Compose는 개발 환경에서 이 명령에 --reload를 추가해 덮어쓴다.
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]

