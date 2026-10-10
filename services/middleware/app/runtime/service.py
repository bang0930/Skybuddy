"""Mission service: the state behind the three MCP tools."""

import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from pydantic import Field, model_validator
from typing_extensions import Self

from app.schemas import (
    AcceptedTask,
    MissionContext,
    MissionState,
    MissionStatusReport,
    PlanSubmissionResult,
    SearchArea,
    SubmissionStatus,
    TaskState,
    ValidationIssue,
)
from app.schemas.mission import ContractModel, Identifier, _require_unique

from .dispatcher import Dispatcher, TaskExecution
from .event_log import EventLog
from .planning import build_mission_task, validate_plan
from .registry import DroneRegistry


class MissionScenario(ContractModel):
    """Static mission input: the instruction and search areas of one mission."""

    mission_id: Identifier
    instruction: str = Field(min_length=1, max_length=2_000)
    search_areas: list[SearchArea] = Field(min_length=1)

    @model_validator(mode="after")
    def area_ids_must_be_unique(self) -> Self:
        _require_unique(
            [area.area_id for area in self.search_areas],
            "search_areas must have unique area_id values",
        )
        return self

    @classmethod
    def load(cls, path: Path) -> "MissionScenario":
        return cls.model_validate(json.loads(Path(path).read_text(encoding="utf-8")))


class UnknownMissionError(KeyError):
    """Raised when a tool references a mission that is not loaded."""


class MissionService:
    """Provide context, accept validated plans, and report progress per mission."""

    def __init__(
        self,
        registry: DroneRegistry,
        scenarios: list[MissionScenario],
        dispatcher: Dispatcher,
        event_log: EventLog,
    ) -> None:
        self.registry = registry
        self.scenarios = {scenario.mission_id: scenario for scenario in scenarios}
        self.dispatcher = dispatcher
        self.event_log = event_log
        self._lock = threading.Lock()
        self._attempts: dict[str, int] = {}
        self._executions: dict[str, list[TaskExecution]] = {}
        self._task_sequence = 0

    def mission_context(self, mission_id: str) -> MissionContext:
        scenario = self._scenario(mission_id)
        return MissionContext(
            mission_id=scenario.mission_id,
            instruction=scenario.instruction,
            drones=self.registry.states(),
            search_areas=scenario.search_areas,
            requested_at=datetime.now(timezone.utc),
        )

    def submit_plan(self, raw_plan: dict[str, Any]) -> PlanSubmissionResult:
        """Validate and start a plan. Returns immediately; flights continue in threads.

        An LLM cannot know the current time, so a missing ``generated_at`` is filled with
        the submission time. The event log keeps the plan exactly as submitted.
        """
        mission_id = str(raw_plan.get("mission_id", ""))
        with self._lock:
            attempt = self._attempts.get(mission_id, 0) + 1
            self._attempts[mission_id] = attempt
            filled_by_server = not raw_plan.get("generated_at")
            self.event_log.write(
                "plan_submitted",
                mission_id=mission_id,
                attempt=attempt,
                plan=raw_plan,
                generated_at_filled_by_server=filled_by_server,
            )
            if filled_by_server:
                raw_plan = {**raw_plan, "generated_at": datetime.now(timezone.utc)}

            if mission_id not in self.scenarios:
                return self._reject(
                    mission_id,
                    attempt,
                    [
                        ValidationIssue(
                            code="UNKNOWN_MISSION",
                            message=(
                                f"mission_id {mission_id!r} is not loaded; "
                                f"use one of {sorted(self.scenarios)}"
                            ),
                            location="mission_id",
                        )
                    ],
                )

            context = self.mission_context(mission_id)
            plan, issues = validate_plan(raw_plan, context)
            if plan is None:
                return self._reject(mission_id, attempt, issues)

            drones = {drone.drone_id: drone for drone in context.drones}
            areas = {area.area_id: area for area in context.search_areas}
            created_at = datetime.now(timezone.utc)
            planned = []
            # Higher priority starts first; drones then fly concurrently.
            for assignment in sorted(plan.assignments, key=lambda a: -a.priority):
                self._task_sequence += 1
                task = build_mission_task(
                    task_id=f"task-{self._task_sequence:03d}-{assignment.drone_id}",
                    mission_id=plan.mission_id,
                    drone=drones[assignment.drone_id],
                    area=areas[assignment.area_id],
                    created_at=created_at,
                )
                planned.append((task, assignment.area_id))

            accepted = [
                AcceptedTask(
                    task_id=task.task_id,
                    drone_id=task.drone_id,
                    area_id=area_id,
                    command_count=len(task.commands),
                )
                for task, area_id in planned
            ]
            self.event_log.write(
                "plan_accepted",
                mission_id=mission_id,
                attempt=attempt,
                tasks=accepted,
                commands=[task.commands for task, _ in planned],
            )
            self._executions.setdefault(mission_id, []).extend(
                self.dispatcher.start(task, area_id) for task, area_id in planned
            )
            return PlanSubmissionResult(
                mission_id=mission_id,
                status=SubmissionStatus.ACCEPTED,
                attempt=attempt,
                tasks=accepted,
            )

    def mission_status(self, mission_id: str) -> MissionStatusReport:
        self._scenario(mission_id)
        with self._lock:
            executions = list(self._executions.get(mission_id, []))
        tasks = [execution.report() for execution in executions]
        return MissionStatusReport(
            mission_id=mission_id,
            state=_mission_state([task.state for task in tasks]),
            drones=self.registry.states(),
            tasks=tasks,
        )

    def _scenario(self, mission_id: str) -> MissionScenario:
        try:
            return self.scenarios[mission_id]
        except KeyError:
            raise UnknownMissionError(
                f"unknown mission_id {mission_id!r}; loaded: {sorted(self.scenarios)}"
            ) from None

    def _reject(
        self, mission_id: str, attempt: int, issues: list[ValidationIssue]
    ) -> PlanSubmissionResult:
        self.event_log.write(
            "plan_rejected", mission_id=mission_id, attempt=attempt, errors=issues
        )
        return PlanSubmissionResult(
            mission_id=mission_id,
            status=SubmissionStatus.REJECTED,
            attempt=attempt,
            errors=issues,
        )


def _mission_state(states: list[TaskState]) -> MissionState:
    if not states:
        return MissionState.IDLE
    if any(state == TaskState.RUNNING for state in states):
        return MissionState.RUNNING
    if any(state == TaskState.FAILED for state in states):
        return MissionState.FAILED
    return MissionState.SUCCEEDED
