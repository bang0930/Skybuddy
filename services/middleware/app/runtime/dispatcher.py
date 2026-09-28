"""Execute Mission Tasks on registered drones and record every command state transition.

Completion rules follow the simulation ``DroneInterface`` contract:

- ``takeoff()`` returns only after the target altitude is reached, so a normal return is
  evidence of completion (``sent -> succeeded``).
- ``goto()`` is fire-and-forget. The dispatcher polls telemetry: the first sample after
  sending marks ``executing``; arriving within ``reach_threshold_m`` marks ``succeeded``.
- ``land()``/``disarm()`` return whether the vehicle was confirmed disarmed.

``DroneInterface`` exposes no protocol ACK, so ``accepted`` is never fabricated.
"""

import contextlib
import math
import threading
import time
from collections.abc import Callable
from datetime import datetime, timezone

from app.adapters import (
    InterfaceMethod,
    UnsafeAltitudeError,
    UnsupportedDroneCommandError,
    to_drone_interface_request,
)
from app.schemas import (
    CommandError,
    CommandReport,
    CommandResult,
    CommandStatus,
    DroneCommand,
    DroneStatus,
    MissionTask,
    TaskReport,
    TaskState,
)

from .event_log import EventLog
from .fake_drone import METERS_PER_DEGREE_LAT
from .registry import DroneRegistry

TERMINAL_STATUSES = {
    CommandStatus.SUCCEEDED,
    CommandStatus.FAILED,
    CommandStatus.TIMED_OUT,
    CommandStatus.UNSUPPORTED,
}


class CommandTracker:
    """Hold one command's ordered ``CommandResult`` history."""

    def __init__(
        self,
        command: DroneCommand,
        *,
        on_transition: Callable[[CommandResult, float | None], None],
    ) -> None:
        self.command = command
        self.status = CommandStatus.PENDING
        self.history: list[CommandResult] = []
        self._on_transition = on_transition
        self._sent_at: float | None = None
        self._lock = threading.Lock()

    def transition(
        self, status: CommandStatus, error: CommandError | None = None
    ) -> CommandResult:
        with self._lock:
            # CommandResult rejects transitions the contract does not allow.
            result = CommandResult(
                command_id=self.command.command_id,
                drone_id=self.command.drone_id,
                protocol=self.command.protocol,
                previous_status=self.status,
                status=status,
                updated_at=datetime.now(timezone.utc),
                error=error,
            )
            now = time.monotonic()
            if status == CommandStatus.SENT:
                self._sent_at = now
            elapsed = None if self._sent_at is None else now - self._sent_at
            self.status = status
            self.history.append(result)
        self._on_transition(result, elapsed)
        return result

    def fail(self, code: str, message: str, *, retryable: bool = False) -> None:
        self.transition(
            CommandStatus.FAILED,
            CommandError(code=code, message=message[:1_000], retryable=retryable),
        )

    def report(self) -> CommandReport:
        with self._lock:
            return CommandReport(
                command_id=self.command.command_id,
                command_type=self.command.payload.command_type.value,
                status=self.status,
                history=list(self.history),
            )


class TaskExecution:
    """Runtime state of one Mission Task."""

    def __init__(self, task: MissionTask, area_id: str, trackers: list[CommandTracker]):
        self.task = task
        self.area_id = area_id
        self.trackers = trackers
        self.state = TaskState.RUNNING
        self.thread: threading.Thread | None = None

    def report(self) -> TaskReport:
        return TaskReport(
            task_id=self.task.task_id,
            drone_id=self.task.drone_id,
            area_id=self.area_id,
            state=self.state,
            commands=[tracker.report() for tracker in self.trackers],
        )


class Dispatcher:
    """Run each task on its own thread so drones fly concurrently."""

    def __init__(
        self,
        registry: DroneRegistry,
        event_log: EventLog,
        *,
        reach_threshold_m: float = 3.0,
        waypoint_timeout_s: float = 60.0,
        telemetry_poll_timeout_s: float = 2.0,
    ) -> None:
        self.registry = registry
        self.event_log = event_log
        self.reach_threshold_m = reach_threshold_m
        self.waypoint_timeout_s = waypoint_timeout_s
        self.telemetry_poll_timeout_s = telemetry_poll_timeout_s

    def start(self, task: MissionTask, area_id: str) -> TaskExecution:
        execution = TaskExecution(task, area_id, [])
        execution.trackers = [
            CommandTracker(command, on_transition=self._logger(task))
            for command in task.commands
        ]
        self.registry.set_status(task.drone_id, DroneStatus.ASSIGNED)
        execution.thread = threading.Thread(
            target=self._run,
            args=(execution,),
            name=f"task-{task.task_id}",
            daemon=True,
        )
        execution.thread.start()
        return execution

    # -------------------------------------------------------------------- execution

    def _run(self, execution: TaskExecution) -> None:
        task = execution.task
        airborne = False
        succeeded = True
        for tracker in execution.trackers:
            command_type = tracker.command.payload.command_type.value
            if command_type == "return_home" or _is_last_goto(execution, tracker):
                self.registry.set_status(task.drone_id, DroneStatus.RETURNING)
            ok = self._execute(tracker)
            if command_type == "takeoff" and tracker.history:
                airborne = True
            if command_type == "land" and ok:
                airborne = False
            if not ok:
                succeeded = False
                break

        if not succeeded:
            for tracker in execution.trackers:
                if tracker.status == CommandStatus.PENDING:
                    tracker.fail("TASK_ABORTED", "an earlier command in the task failed")
            if airborne:
                self._emergency_stop(task.drone_id)

        execution.state = TaskState.SUCCEEDED if succeeded else TaskState.FAILED
        # A failed task leaves the drone out of further allocation until checked.
        self.registry.set_status(
            task.drone_id,
            DroneStatus.AVAILABLE if succeeded else DroneStatus.UNAVAILABLE,
        )
        self.event_log.write(
            "task_finished",
            mission_id=task.mission_id,
            task_id=task.task_id,
            drone_id=task.drone_id,
            state=execution.state,
        )

    def _execute(self, tracker: CommandTracker) -> bool:
        handle = self.registry.get(tracker.command.drone_id)
        try:
            request = to_drone_interface_request(
                tracker.command,
                drone_id=handle.drone_id,
                protocol=handle.protocol,
                capabilities=handle.capabilities,
            )
        except UnsupportedDroneCommandError as exc:
            tracker.transition(
                CommandStatus.UNSUPPORTED,
                CommandError(code="COMMAND_NOT_SUPPORTED", message=str(exc)),
            )
            return False
        except (UnsafeAltitudeError, ValueError) as exc:
            tracker.fail("COMMAND_TRANSLATION_FAILED", str(exc))
            return False

        tracker.transition(CommandStatus.SENT)
        try:
            with handle.exclusive():
                outcome = request.invoke(handle.client)
        except Exception as exc:  # the simulation adapters signal failure by raising
            tracker.fail("DRONE_INTERFACE_ERROR", f"{type(exc).__name__}: {exc}")
            return False

        if request.method == InterfaceMethod.GOTO:
            return self._await_arrival(tracker, request.latitude, request.longitude)
        if request.method in (InterfaceMethod.LAND, InterfaceMethod.DISARM) and not outcome:
            tracker.transition(
                CommandStatus.TIMED_OUT,
                CommandError(
                    code=f"{request.method.value.upper()}_NOT_CONFIRMED",
                    message="vehicle was not confirmed disarmed before the deadline",
                    retryable=True,
                ),
            )
            return False
        tracker.transition(CommandStatus.SUCCEEDED)
        return True

    def _await_arrival(
        self, tracker: CommandTracker, latitude: float, longitude: float
    ) -> bool:
        handle = self.registry.get(tracker.command.drone_id)
        deadline = time.monotonic() + self.waypoint_timeout_s
        last_distance = None
        while time.monotonic() < deadline:
            with handle.exclusive():
                sample = handle.client.get_telemetry(timeout=self.telemetry_poll_timeout_s)
            if sample is None or sample.get("lat") is None or sample.get("lon") is None:
                continue
            if tracker.status == CommandStatus.SENT:
                tracker.transition(CommandStatus.EXECUTING)
            last_distance = _distance_m(
                sample["lat"], sample["lon"], latitude, longitude
            )
            if last_distance < self.reach_threshold_m:
                tracker.transition(CommandStatus.SUCCEEDED)
                return True
        detail = "no telemetry" if last_distance is None else f"{last_distance:.1f}m away"
        tracker.transition(
            CommandStatus.TIMED_OUT,
            CommandError(
                code="WAYPOINT_TIMEOUT",
                message=f"waypoint not reached within {self.waypoint_timeout_s}s ({detail})",
                retryable=True,
            ),
        )
        return False

    def _emergency_stop(self, drone_id: str) -> None:
        """Land first; disarm if landing cannot be confirmed (same policy as run_mission)."""
        handle = self.registry.get(drone_id)
        landed = False
        with contextlib.suppress(Exception), handle.exclusive():
            landed = bool(handle.client.land())
        if not landed:
            with contextlib.suppress(Exception), handle.exclusive():
                handle.client.disarm()
        self.event_log.write("emergency_stop", drone_id=drone_id, landed=landed)

    def _logger(self, task: MissionTask) -> Callable[[CommandResult, float | None], None]:
        types = {command.command_id: command.payload.command_type for command in task.commands}

        def log(result: CommandResult, elapsed_s: float | None) -> None:
            self.event_log.write(
                "command_transition",
                mission_id=task.mission_id,
                task_id=task.task_id,
                command_id=result.command_id,
                command_type=types[result.command_id],
                drone_id=result.drone_id,
                protocol=result.protocol,
                previous_status=result.previous_status,
                status=result.status,
                updated_at=result.updated_at,
                elapsed_since_sent_s=None if elapsed_s is None else round(elapsed_s, 3),
                error=result.error,
            )

        return log


def _is_last_goto(execution: TaskExecution, tracker: CommandTracker) -> bool:
    """The goto right before land is the temporary return-home leg."""
    trackers = execution.trackers
    index = trackers.index(tracker)
    return (
        tracker.command.payload.command_type.value == "goto"
        and index + 1 < len(trackers)
        and trackers[index + 1].command.payload.command_type.value == "land"
    )


def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Short-range planar distance, identical to ``DroneInterface.distance_m``."""
    dlat = (lat2 - lat1) * METERS_PER_DEGREE_LAT
    dlon = (lon2 - lon1) * METERS_PER_DEGREE_LAT * math.cos(math.radians(lat1))
    return math.hypot(dlat, dlon)
