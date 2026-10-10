"""Shared fixtures for the runtime and MCP integration tests."""

from datetime import datetime, timezone

import pytest

from app.runtime import (
    Dispatcher,
    DispatcherConfig,
    DroneRegistry,
    EventLog,
    MissionScenario,
    MissionService,
    RegistryConfig,
)

HOME_LAT = -35.363261


def fast_registry_config(**overrides) -> RegistryConfig:
    """Two fake drones (one per protocol) that complete a mission in a few seconds."""
    fake = {
        "horizontal_speed_m_s": 50.0,
        "vertical_speed_m_s": 30.0,
        "telemetry_interval_s": 0.005,
    }
    config = {
        "telemetry_timeout_s": 3.0,
        "drones": [
            {
                "drone_id": "drone-01",
                "protocol": "mavlink",
                "mode": "fake",
                "fake": {
                    "home": {"latitude": HOME_LAT, "longitude": 149.165197},
                    "system_id": 7,
                    **fake,
                },
            },
            {
                "drone_id": "drone-02",
                "protocol": "ap_dds",
                "mode": "fake",
                "fake": {"home": {"latitude": HOME_LAT, "longitude": 149.165263}, **fake},
            },
        ],
    }
    config.update(overrides)
    return RegistryConfig.model_validate(config)


def scenario(**overrides) -> MissionScenario:
    data = {
        "mission_id": "mission-001",
        "instruction": "북쪽과 남쪽 구역을 나눠 수색해 줘",
        "search_areas": [
            {
                "area_id": "area-north",
                "boundary": [
                    {"latitude": -35.363126, "longitude": 149.16512},
                    {"latitude": -35.363126, "longitude": 149.16534},
                    {"latitude": -35.362947, "longitude": 149.16534},
                ],
                "search_altitude_m": 30,
                "search_altitude_reference": "home_relative",
            },
            {
                "area_id": "area-south",
                "boundary": [
                    {"latitude": -35.363575, "longitude": 149.16512},
                    {"latitude": -35.363575, "longitude": 149.16534},
                    {"latitude": -35.363396, "longitude": 149.16534},
                ],
                "search_altitude_m": 50,
                "search_altitude_reference": "home_relative",
            },
        ],
    }
    data.update(overrides)
    return MissionScenario.model_validate(data)


def plan(*assignments: tuple[str, str], mission_id: str = "mission-001") -> dict:
    return {
        "mission_id": mission_id,
        "assignments": [
            {"drone_id": drone_id, "area_id": area_id} for drone_id, area_id in assignments
        ],
        "generated_at": datetime.now(timezone.utc).isoformat(),
    }


@pytest.fixture
def registry():
    registry = DroneRegistry.from_config(fast_registry_config())
    registry.connect_all()
    yield registry
    registry.close_all()


@pytest.fixture
def event_log(tmp_path):
    return EventLog(tmp_path / "events.jsonl")


@pytest.fixture
def service(registry, event_log):
    dispatcher = Dispatcher(
        registry,
        event_log,
        config=DispatcherConfig(waypoint_timeout_s=10),
        telemetry_poll_timeout_s=0.2,
    )
    return MissionService(registry, [scenario()], dispatcher, event_log)
