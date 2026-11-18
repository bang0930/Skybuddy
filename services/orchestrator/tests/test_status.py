"""관제 화면 상태 요약(_drone)이 미들웨어 드론 상태를 화면용으로 옮기는지 확인한다.

실행 (services/orchestrator에서): venv/bin/python -m pytest
"""

from app.status import _drone


def _middleware_drone(**overrides) -> dict:
    """get_mission_status 결과의 드론 한 대 (DroneState를 JSON으로 바꾼 모양)."""
    drone = {
        "drone_id": "drone-02",
        "protocol": "ap_dds",
        "status": "running",
        "connection_status": "connected",
        "battery_percent": 64.0,
        "link_quality_pct": 20.0,
        "position": {"latitude": -35.36, "longitude": 149.16, "altitude_m": 40.0},
    }
    drone.update(overrides)
    return drone


def test_drone_passes_link_quality_through():
    assert _drone(_middleware_drone())["link_quality_pct"] == 20.0


def test_drone_keeps_zero_link_quality():
    # 0은 링크 끊김이다. 값이 없는 것(None)과 섞이면 안 된다.
    assert _drone(_middleware_drone(link_quality_pct=0.0))["link_quality_pct"] == 0.0


def test_drone_link_quality_null_becomes_none():
    # 계약상 null은 temporarily_unavailable
    assert _drone(_middleware_drone(link_quality_pct=None))["link_quality_pct"] is None


def test_drone_link_quality_missing_becomes_none():
    # PR #51 이전 미들웨어처럼 필드가 없어도 오류 없이 None
    drone = _middleware_drone()
    del drone["link_quality_pct"]
    assert _drone(drone)["link_quality_pct"] is None


def test_drone_summary_fields():
    assert _drone(_middleware_drone()) == {
        "drone_id": "drone-02",
        "protocol": "ap_dds",
        "status": "running",
        "connection": "connected",
        "battery_percent": 64.0,
        "link_quality_pct": 20.0,
        "position": {"latitude": -35.36, "longitude": 149.16, "altitude_m": 40.0},
    }
