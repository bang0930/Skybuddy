"""Append-only JSONL log of mission events for reproducibility and later KPI analysis."""

import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from pydantic import BaseModel


class EventLog:
    """Write one JSON object per line. ``path=None`` keeps events in memory only."""

    def __init__(self, path: Path | None = None) -> None:
        self.path = Path(path) if path is not None else None
        self._lock = threading.Lock()
        self.events: list[dict[str, Any]] = []
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, event: str, **fields: Any) -> dict[str, Any]:
        record = {
            "logged_at": datetime.now(timezone.utc).isoformat(),
            "event": event,
            **{key: _jsonable(value) for key, value in fields.items()},
        }
        line = json.dumps(record, ensure_ascii=False)
        with self._lock:
            self.events.append(record)
            if self.path is not None:
                with self.path.open("a", encoding="utf-8") as file:
                    file.write(line + "\n")
        return record


def _jsonable(value: Any) -> Any:
    if isinstance(value, BaseModel):
        return value.model_dump(mode="json")
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, datetime):
        return value.isoformat()
    if hasattr(value, "value") and isinstance(getattr(value, "value"), str):
        return value.value
    return value
