"""Entry point for the SkyBuddy orchestrator service."""

import logging
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from app.llm import run_command
from app.status import StatusUnavailable, read_status

logger = logging.getLogger(__name__)

_STATIC_DIR = Path(__file__).parent / "static"
_INDEX_HTML = _STATIC_DIR / "index.html"

app = FastAPI(
    title="SkyBuddy Orchestrator",
    description="Creates structured mission plans from mission context.",
    version="0.1.0",
)
# 관제 화면이 쓰는 로고 등 정적 파일 (/static/logo.jpg)
app.mount("/static", StaticFiles(directory=_STATIC_DIR), name="static")


@app.get("/", include_in_schema=False)
async def index() -> FileResponse:
    """관제 화면. 브라우저에서 http://127.0.0.1:8001/ 로 엽니다."""
    return FileResponse(_INDEX_HTML)


@app.get("/health", tags=["system"])
async def health() -> dict[str, str]:
    """Return the service health status for Docker and local development."""
    return {"status": "ok", "service": "orchestrator"}


@app.post("/command", tags=["llm"])
async def command_endpoint(message: str) -> dict:
    """자연어 명령을 받아 LLM을 통해 구조화된 드론 명령으로 변환합니다."""
    return await run_command(message)


@app.get("/status", tags=["ui"])
async def status_endpoint() -> dict:
    """LLM 없이 미들웨어에서 임무·드론·작업 상태를 읽어 옵니다. 관제 화면이 주기적으로 부릅니다."""
    try:
        return await read_status()
    except StatusUnavailable as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from None
    except Exception:
        logger.exception("status read failed")
        raise HTTPException(
            status_code=503, detail="미들웨어에 연결할 수 없습니다. 미들웨어 서버가 떠 있는지 확인하세요."
        ) from None
