"""Entry point for the SkyBuddy middleware service."""

from fastapi import FastAPI

app = FastAPI(
    title="SkyBuddy Middleware",
    description="Validates and translates data between the orchestrator and simulator.",
    version="0.1.0",
)


@app.get("/health", tags=["system"])
async def health() -> dict[str, str]:
    """Return the service health status for Docker and local development."""
    return {"status": "ok", "service": "middleware"}

