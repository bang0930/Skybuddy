"""Runtime pieces that connect the middleware contracts to drones."""

from .config import DispatcherConfig
from .dispatcher import CommandTracker, Dispatcher, TaskExecution
from .event_log import EventLog
from .fake_drone import FakeDrone
from .planning import area_waypoints, build_mission_task, validate_plan
from .registry import (
    ClientFactory,
    DroneConfig,
    DroneHandle,
    DroneMode,
    DroneRegistry,
    FakeDroneOptions,
    RegistryConfig,
)
from .service import MissionScenario, MissionService, UnknownMissionError

__all__ = [
    "ClientFactory",
    "CommandTracker",
    "Dispatcher",
    "DispatcherConfig",
    "DroneConfig",
    "DroneHandle",
    "DroneMode",
    "DroneRegistry",
    "EventLog",
    "FakeDrone",
    "FakeDroneOptions",
    "MissionScenario",
    "MissionService",
    "RegistryConfig",
    "TaskExecution",
    "UnknownMissionError",
    "area_waypoints",
    "build_mission_task",
    "validate_plan",
]
