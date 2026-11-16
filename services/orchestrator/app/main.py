"""Entry point for the SkyBuddy orchestrator service."""

import logging

from fastapi import FastAPI, HTTPException

from app.llm import run_command
from app.status import StatusUnavailable, read_status

logger = logging.getLogger(__name__)

app = FastAPI(
    title="SkyBuddy Orchestrator",
    description="Creates structured mission plans from mission context.",
    version="0.1.0",
)


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
