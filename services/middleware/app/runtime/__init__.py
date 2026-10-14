"""Runtime pieces that connect the middleware contracts to drones."""

from .config import DispatcherConfig, WaypointConfig
from .dispatcher import CommandTracker, Dispatcher, TaskExecution
from .event_log import EventLog
from .fake_drone import FakeDrone
from .planning import build_mission_task, build_mission_tasks, validate_plan
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
from .waypoints import (
    CoverageGenerator,
    VertexTraversalGenerator,
    WaypointCheckError,
    WaypointGenerator,
)

__all__ = [
    "ClientFactory",
    "CommandTracker",
    "CoverageGenerator",
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
    "VertexTraversalGenerator",
    "WaypointCheckError",
    "WaypointConfig",
    "WaypointGenerator",
    "build_mission_task",
    "build_mission_tasks",
    "validate_plan",
]
