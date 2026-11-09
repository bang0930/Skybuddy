"""Execute Mission Tasks on registered drones and record every command state transition.

Completion rules follow the simulation ``DroneInterface`` contract:

- ``takeoff()`` returns only after the target altitude is reached, so a normal return is
  evidence of completion (``sent -> succeeded``).
- ``goto()`` is fire-and-forget. The dispatcher polls telemetry: the first sample after
  sending marks ``executing``; arriving within ``reach_threshold_m`` marks ``succeeded``.
  The leg's time limit grows with its length (``DispatcherConfig.leg_timeout_s``).
- ``land()``/``disarm()`` return whether the vehicle was confirmed disarmed.

``DroneInterface`` exposes no protocol ACK, so ``accepted`` is never fabricated.

Lost link: while a ``goto`` leg is polled, ``DispatcherConfig.link_loss_timeout_s`` of
silence fails it with ``LINK_LOST``. Commands sent during the outage would not reach the
vehicle, so the middleware does not try to land it then; the vehicle's own failsafe
(ArduPilot ``FS_GCS_ENABLE``) is expected to act. The dispatcher waits up to
``link_recovery_timeout_s`` for telemetry; when it returns, the drone flies the task's
return leg and lands. Either way the task fails and the drone is left ``unavailable``.
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

from .config import DispatcherConfig
from .event_log import EventLog
from .fake_drone import METERS_PER_DEGREE_LAT
from .registry import DroneRegistry

LINK_LOST = "LINK_LOST"
# Below this altitude after a recovered link, the vehicle failsafe already landed it.
ON_GROUND_ALT_M = 1.0

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
        on_transition: Callable[[CommandResult, float | None, dict | None], None],
    ) -> None:
        self.command = command
        self.status = CommandStatus.PENDING
        self.history: list[CommandResult] = []
        self._on_transition = on_transition
        self._sent_at: float | None = None
        self._lock = threading.Lock()

    def transition(
        self,
        status: CommandStatus,
        error: CommandError | None = None,
        *,
        target: dict | None = None,
    ) -> CommandResult:
        """Record a transition. ``target`` names the client actually invoked (on ``sent``)."""
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
        self._on_transition(result, elapsed, target)
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
        config: DispatcherConfig | None = None,
        telemetry_poll_timeout_s: float = 2.0,
    ) -> None:
        self.registry = registry
        self.event_log = event_log
        self.config = config or DispatcherConfig()
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
        lost_link: CommandTracker | None = None
        for tracker in execution.trackers:
            command_type = tracker.command.payload.command_type.value
            if _is_return_leg(execution, tracker):
                self.registry.set_status(task.drone_id, DroneStatus.RETURNING)
            ok = self._execute(tracker)
            if command_type == "takeoff" and tracker.history:
                airborne = True
            if command_type == "land" and ok:
                airborne = False
            if not ok:
                succeeded = False
                if _error_code(tracker) == LINK_LOST:
                    lost_link = tracker
                break

        if not succeeded:
            reason = (
                "the telemetry link was lost"
                if lost_link is not None
                else "an earlier command in the task failed"
            )
            for tracker in execution.trackers:
                if tracker.status == CommandStatus.PENDING:
                    tracker.fail("TASK_ABORTED", reason)
            if lost_link is not None:
                self._recover_lost_link(execution, lost_link)
            elif airborne:
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

        leg_distance = None
        if request.method == InterfaceMethod.GOTO:
            start, _ = handle.latest_sample()
            if start and start.get("lat") is not None and start.get("lon") is not None:
                leg_distance = _distance_m(
                    start["lat"], start["lon"], request.latitude, request.longitude
                )

        # Log which registered client receives the call, so mixed-up identifiers are
        # visible in the event log rather than only in the vehicle's behaviour.
        tracker.transition(CommandStatus.SENT, target=handle.identity())
        try:
            with handle.exclusive():
                outcome = request.invoke(handle.client)
        except Exception as exc:  # the simulation adapters signal failure by raising
            tracker.fail("DRONE_INTERFACE_ERROR", f"{type(exc).__name__}: {exc}")
            return False

        if request.method == InterfaceMethod.GOTO:
            return self._await_arrival(
                tracker,
                request.latitude,
                request.longitude,
                timeout_s=self.config.leg_timeout_s(leg_distance),
            )
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
        self,
        tracker: CommandTracker,
        latitude: float,
        longitude: float,
        *,
        timeout_s: float,
    ) -> bool:
        def mark_executing() -> None:
            if tracker.status == CommandStatus.SENT:
                tracker.transition(CommandStatus.EXECUTING)

        outcome, detail = self._poll_arrival(
            tracker.command.drone_id,
            latitude,
            longitude,
            timeout_s=timeout_s,
            on_position=mark_executing,
        )
        if outcome == "reached":
            tracker.transition(CommandStatus.SUCCEEDED)
            return True
        if outcome == "link_lost":
            tracker.fail(LINK_LOST, detail)
            return False
        tracker.transition(
            CommandStatus.TIMED_OUT,
            CommandError(
                code="WAYPOINT_TIMEOUT",
                message=f"waypoint not reached within {timeout_s:.0f}s ({detail})",
                retryable=True,
            ),
        )
        return False

    def _poll_arrival(
        self,
        drone_id: str,
        latitude: float,
        longitude: float,
        *,
        timeout_s: float,
        on_position: Callable[[], None] | None = None,
    ) -> tuple[str, str]:
        """Poll until within ``reach_threshold_m``: ``reached``, ``timed_out`` or ``link_lost``."""
        handle = self.registry.get(drone_id)
        started = time.monotonic()
        deadline = started + timeout_s
        last_contact = started
        last_distance = None
        while time.monotonic() < deadline:
            with handle.exclusive():
                sample = handle.client.get_telemetry(timeout=self.telemetry_poll_timeout_s)
            now = time.monotonic()
            if sample is None:
                silent_s = now - last_contact
                if silent_s >= self.config.link_loss_timeout_s:
                    return "link_lost", (
                        f"no telemetry for {silent_s:.1f}s "
                        f"(limit {self.config.link_loss_timeout_s:g}s)"
                    )
                continue
            last_contact = now
            if sample.get("lat") is None or sample.get("lon") is None:
                continue
            if on_position is not None:
                on_position()
            last_distance = _distance_m(sample["lat"], sample["lon"], latitude, longitude)
            if last_distance < self.config.reach_threshold_m:
                return "reached", f"{last_distance:.1f}m away"
        detail = "no telemetry" if last_distance is None else f"{last_distance:.1f}m away"
        return "timed_out", detail

    # ------------------------------------------------------------------ lost link

    def _recover_lost_link(self, execution: TaskExecution, lost: CommandTracker) -> None:
        """Wait for the link; once back, fly the task's return leg and land."""
        task = execution.task
        drone_id = task.drone_id
        handle = self.registry.get(drone_id)
        ids = {"mission_id": task.mission_id, "task_id": task.task_id, "drone_id": drone_id}
        # The vehicle failsafe is expected to be bringing it home meanwhile.
        self.registry.set_status(drone_id, DroneStatus.RETURNING)
        self.event_log.write(
            "link_lost",
            **ids,
            command_id=lost.command.command_id,
            wait_s=self.config.link_recovery_timeout_s,
        )

        sample, outage_s = self._wait_for_link(handle)
        if sample is None:
            self.event_log.write("link_not_restored", **ids, waited_s=round(outage_s, 1))
            return
        self.event_log.write("link_restored", **ids, outage_s=round(outage_s, 1))

        altitude = sample.get("relative_alt_m")
        if altitude is not None and altitude < ON_GROUND_ALT_M:
            self.event_log.write(
                "link_recovery_finished", **ids, outcome="already_on_ground"
            )
            return

        returned = self._fly_return_leg(execution)
        if returned == "link_lost":
            # Lost again on the way home: commands would not arrive, leave it to failsafe.
            self.event_log.write("link_recovery_finished", **ids, outcome="link_lost_again")
            return
        landed = self._land_or_disarm(drone_id)
        self.event_log.write(
            "link_recovery_finished",
            **ids,
            outcome="landed" if landed else "disarmed",
            return_leg=returned,
        )

    def _wait_for_link(self, handle) -> tuple[dict | None, float]:
        """Poll until any telemetry sample arrives or ``link_recovery_timeout_s`` passes."""
        started = time.monotonic()
        deadline = started + self.config.link_recovery_timeout_s
        while time.monotonic() < deadline:
            with handle.exclusive():
                sample = handle.client.get_telemetry(timeout=self.telemetry_poll_timeout_s)
            if sample is not None:
                return sample, time.monotonic() - started
        return None, time.monotonic() - started

    def _fly_return_leg(self, execution: TaskExecution) -> str:
        """Re-issue the task's return command outside its tracker (it was aborted).

        Returns ``reached``, ``timed_out``, ``link_lost``, ``sent`` (return_home has no
        arrival check here) or ``failed``.
        """
        handle = self.registry.get(execution.task.drone_id)
        leg = next((t for t in execution.trackers if _is_return_leg(execution, t)), None)
        if leg is None:
            return "failed"
        try:
            request = to_drone_interface_request(
                leg.command,
                drone_id=handle.drone_id,
                protocol=handle.protocol,
                capabilities=handle.capabilities,
            )
            with handle.exclusive():
                request.invoke(handle.client)
        except Exception as exc:  # noqa: BLE001 - recovery must still try to land
            self.event_log.write(
                "link_recovery_return_failed",
                drone_id=handle.drone_id,
                error=f"{type(exc).__name__}: {exc}",
            )
            return "failed"
        if request.method != InterfaceMethod.GOTO:
            return "sent"
        start, _ = handle.latest_sample()
        distance = None
        if start and start.get("lat") is not None and start.get("lon") is not None:
            distance = _distance_m(
                start["lat"], start["lon"], request.latitude, request.longitude
            )
        outcome, _ = self._poll_arrival(
            handle.drone_id,
            request.latitude,
            request.longitude,
            timeout_s=self.config.leg_timeout_s(distance),
        )
        return outcome

    def _emergency_stop(self, drone_id: str) -> None:
        """Land first; disarm if landing cannot be confirmed (same policy as run_mission)."""
        landed = self._land_or_disarm(drone_id)
        self.event_log.write("emergency_stop", drone_id=drone_id, landed=landed)

    def _land_or_disarm(self, drone_id: str) -> bool:
        handle = self.registry.get(drone_id)
        landed = False
        with contextlib.suppress(Exception), handle.exclusive():
            landed = bool(handle.client.land())
        if not landed:
            with contextlib.suppress(Exception), handle.exclusive():
                handle.client.disarm()
        return landed

    def _logger(
        self, task: MissionTask
    ) -> Callable[[CommandResult, float | None, dict | None], None]:
        types = {command.command_id: command.payload.command_type for command in task.commands}

        def log(result: CommandResult, elapsed_s: float | None, target: dict | None) -> None:
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
                **({} if target is None else {"target": target}),
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


def _is_return_leg(execution: TaskExecution, tracker: CommandTracker) -> bool:
    return tracker.command.payload.command_type.value == "return_home" or _is_last_goto(
        execution, tracker
    )


def _error_code(tracker: CommandTracker) -> str | None:
    if not tracker.history or tracker.history[-1].error is None:
        return None
    return tracker.history[-1].error.code


def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Short-range planar distance, identical to ``DroneInterface.distance_m``."""
    dlat = (lat2 - lat1) * METERS_PER_DEGREE_LAT
    dlon = (lon2 - lon1) * METERS_PER_DEGREE_LAT * math.cos(math.radians(lat1))
    return math.hypot(dlat, dlon)
