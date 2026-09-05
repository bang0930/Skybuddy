"""Entry point for the SkyBuddy orchestrator service."""

from fastapi import FastAPI

from app.llm import run_command

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

