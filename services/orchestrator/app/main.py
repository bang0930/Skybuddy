"""Entry point for the SkyBuddy orchestrator service."""

from fastapi import FastAPI

app = FastAPI(
    title="SkyBuddy Orchestrator",
    description="Creates structured mission plans from mission context.",
    version="0.1.0",
)


@app.get("/health", tags=["system"])
async def health() -> dict[str, str]:
    """Return the service health status for Docker and local development."""
    return {"status": "ok", "service": "orchestrator"}

