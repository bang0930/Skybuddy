"""Analyse a run's event log and tool responses for integration evidence.

Used by ``scripts/collect_integration_evidence.py``. Everything here works on plain
dictionaries read back from the JSONL event log and the MCP tool responses, so the same
checks apply to FakeDrone runs and real SITL runs.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

TERMINAL_STATUSES = {"succeeded", "failed", "timed_out", "unsupported"}


def read_events(path: Path) -> list[dict[str, Any]]:
    """Read a JSONL event log; blank lines are skipped."""
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    return [json.loads(line) for line in lines if line.strip()]


def registered_drones(events: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    """Drones as registered by the last ``server_started`` event, keyed by drone_id."""
    started = [event for event in events if event.get("event") == "server_started"]
    if not started:
        return {}
    return {drone["drone_id"]: drone for drone in started[-1].get("drones", [])}


def planned_commands(events: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    """Every command of every accepted plan, keyed by command_id."""
    commands = {}
    for event in events:
        if event.get("event") != "plan_accepted":
            continue
        for task_commands in event.get("commands", []):
            for command in task_commands:
                commands[command["command_id"]] = command
    return commands


def transition_rows(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """One row per ``command_transition`` event, flattened for CSV."""
    rows = []
    for event in events:
        if event.get("event") != "command_transition":
            continue
        error = event.get("error") or {}
        target = event.get("target") or {}
        rows.append(
            {
                "logged_at": event.get("logged_at"),
                "task_id": event.get("task_id"),
                "command_id": event.get("command_id"),
                "command_type": event.get("command_type"),
                "drone_id": event.get("drone_id"),
                "protocol": event.get("protocol"),
                "previous_status": event.get("previous_status"),
                "status": event.get("status"),
                "elapsed_since_sent_s": event.get("elapsed_since_sent_s"),
                "error_code": error.get("code"),
                "error_message": error.get("message"),
                "target_binding": _binding_label(target.get("binding")),
            }
        )
    return rows


def command_summaries(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """One row per command: transition path, final status, and time since ``sent``."""
    summaries: dict[str, dict[str, Any]] = {}
    for row in transition_rows(events):
        summary = summaries.setdefault(
            row["command_id"],
            {
                "task_id": row["task_id"],
                "command_id": row["command_id"],
                "drone_id": row["drone_id"],
                "protocol": row["protocol"],
                "command_type": row["command_type"],
                "transitions": "pending",
                "final_status": "pending",
                "sent_at": None,
                "finished_at": None,
                "elapsed_s": None,
                "error_code": None,
            },
        )
        summary["transitions"] += f">{row['status']}"
        summary["final_status"] = row["status"]
        if row["status"] == "sent":
            summary["sent_at"] = row["logged_at"]
        if row["status"] in TERMINAL_STATUSES:
            summary["finished_at"] = row["logged_at"]
            summary["elapsed_s"] = row["elapsed_since_sent_s"]
            summary["error_code"] = row["error_code"]
    return list(summaries.values())


@dataclass
class DroneIsolation:
    """Commands sent to one registered drone and how many reached the wrong client."""

    drone_id: str
    protocol: str | None
    binding: str
    sent_commands: int = 0
    mismatches: int = 0


@dataclass
class IsolationCheck:
    """Whether every command ran only on its registered drone, protocol, and binding."""

    failures: list[str] = field(default_factory=list)
    drones: list[DroneIsolation] = field(default_factory=list)

    @property
    def sent_commands(self) -> int:
        return sum(drone.sent_commands for drone in self.drones)

    @property
    def verdict(self) -> str:
        """``FAIL`` on any mismatch, ``N/A`` when no command was sent, else ``PASS``."""
        if self.failures:
            return "FAIL"
        return "PASS" if self.sent_commands else "N/A"


def check_identifier_isolation(
    events: list[dict[str, Any]], *, selected_drones: list[str] | None = None
) -> IsolationCheck:
    """Check from the event log that no command crossed to another drone.

    - Registered identities are distinct: no shared connection string, no shared
      MAVLink link + SYSID, no shared ROS 2 namespace.
    - Every planned command (``plan_accepted``) targets its task's drone with that
      drone's registered protocol, and only drones selected for the run were planned.
    - Every ``command_transition`` matches its planned command, and each ``sent``
      transition names the client that was actually invoked (``target``): it must be
      the same drone_id, protocol, and binding that ``server_started`` registered.
    """
    check = IsolationCheck()
    registry = registered_drones(events)
    if not registry:
        check.failures.append("server_started event not found")
        return check
    check.drones = [
        DroneIsolation(
            drone_id=drone_id,
            protocol=drone.get("protocol"),
            binding=_binding_label(drone.get("binding")),
        )
        for drone_id, drone in registry.items()
    ]
    per_drone = {drone.drone_id: drone for drone in check.drones}
    check.failures += _duplicate_identities(registry)

    planned = planned_commands(events)
    for event in events:
        if event.get("event") != "plan_accepted":
            continue
        task_drones = {task["task_id"]: task["drone_id"] for task in event.get("tasks", [])}
        for task_id, drone_id in task_drones.items():
            if drone_id not in registry:
                check.failures.append(f"{task_id}: planned for unregistered {drone_id}")
            elif selected_drones is not None and drone_id not in selected_drones:
                check.failures.append(f"{task_id}: planned for unselected {drone_id}")
        for task_commands in event.get("commands", []):
            for command in task_commands:
                command_id = command["command_id"]
                owner = task_drones.get(command["task_id"])
                if command["drone_id"] != owner:
                    check.failures.append(
                        f"{command_id}: targets {command['drone_id']}, task belongs to {owner}"
                    )
                registered = registry.get(command["drone_id"])
                if registered and command["protocol"] != registered.get("protocol"):
                    check.failures.append(
                        f"{command_id}: protocol {command['protocol']}, "
                        f"{command['drone_id']} is {registered.get('protocol')}"
                    )

    for event in events:
        if event.get("event") != "command_transition":
            continue
        command_id = event.get("command_id")
        command = planned.get(command_id)
        if command is None:
            check.failures.append(f"{command_id}: transition for a command never planned")
            continue
        for key in ("task_id", "drone_id", "protocol"):
            if event.get(key) != command.get(key):
                check.failures.append(
                    f"{command_id}: transition {key}={event.get(key)}, "
                    f"planned {command.get(key)}"
                )
        if event.get("status") != "sent":
            continue
        drone_id = command["drone_id"]
        target = event.get("target")
        if drone_id in per_drone:
            per_drone[drone_id].sent_commands += 1
        if target is None:
            check.failures.append(f"{command_id}: sent without a recorded target")
            _count_mismatch(per_drone, drone_id)
            continue
        registered = registry.get(drone_id, {})
        expected = {
            "drone_id": drone_id,
            "protocol": registered.get("protocol"),
            "binding": registered.get("binding"),
        }
        wrong = [key for key, value in expected.items() if target.get(key) != value]
        if wrong:
            details = ", ".join(
                f"{key} {_describe(target.get(key))} != {_describe(expected[key])}"
                for key in wrong
            )
            check.failures.append(f"{command_id}: sent to the wrong client ({details})")
            _count_mismatch(per_drone, drone_id)
    return check


def connection_gaps(
    observations: list[dict[str, Any]], drone_ids: list[str]
) -> list[dict[str, Any]]:
    """Find stretches where a drone was reported ``disconnected``.

    ``observations`` are tool responses in call order, each ``{"at", "elapsed_s",
    "response"}``. Responses with a ``drones`` list (context and status) count; the
    command stage comes from the status report's tasks. Consecutive disconnected
    observations of one drone form one gap.
    """
    gaps = []
    for drone_id in drone_ids:
        current = None
        for observation in observations:
            response = observation.get("response") or {}
            state = next(
                (d for d in response.get("drones", []) if d.get("drone_id") == drone_id),
                None,
            )
            if state is None:
                continue
            if state.get("connection_status") == "disconnected":
                stage = command_stage(response, drone_id)
                if current is None:
                    current = {
                        "drone_id": drone_id,
                        "start_at": observation["at"],
                        "start_elapsed_s": observation["elapsed_s"],
                        "end_at": observation["at"],
                        "end_elapsed_s": observation["elapsed_s"],
                        "observations": 0,
                        "stages": [],
                        "recovered": False,
                    }
                    gaps.append(current)
                current["end_at"] = observation["at"]
                current["end_elapsed_s"] = observation["elapsed_s"]
                current["observations"] += 1
                if stage not in current["stages"]:
                    current["stages"].append(stage)
            elif current is not None:
                current["recovered"] = True
                current = None
    return gaps


def command_stage(status_report: dict[str, Any], drone_id: str) -> str:
    """Name the command a drone is on, e.g. ``task-001-drone-01-c09 land (sent)``."""
    tasks = [
        task for task in status_report.get("tasks", []) if task.get("drone_id") == drone_id
    ]
    if not tasks:
        return "no task"
    task = tasks[-1]
    for command in task.get("commands", []):
        if command.get("status") not in TERMINAL_STATUSES:
            return f"{command['command_id']} {command['command_type']} ({command['status']})"
    return f"{task['task_id']} finished ({task.get('state')})"


def _duplicate_identities(registry: dict[str, dict[str, Any]]) -> list[str]:
    failures = []
    seen: dict[tuple, str] = {}
    for drone_id, drone in registry.items():
        keys = []
        if drone.get("connection_string"):
            keys.append(("connection string", drone["connection_string"]))
        binding = drone.get("binding") or {}
        if "namespace" in binding:
            keys.append(("ROS 2 namespace", binding["namespace"]))
        if "system_id" in binding:
            # Separate links may reuse SYSID 1 (ArduPilot default); one link must not.
            link = drone.get("connection_string") or drone_id
            keys.append(("MAVLink link and SYSID", f"{link} sysid={binding['system_id']}"))
        for key in keys:
            if key in seen:
                failures.append(f"{drone_id} and {seen[key]} share {key[0]} {key[1]}")
            else:
                seen[key] = drone_id
    return failures


def _count_mismatch(per_drone: dict[str, DroneIsolation], drone_id: str) -> None:
    if drone_id in per_drone:
        per_drone[drone_id].mismatches += 1


def _binding_label(binding: dict[str, Any] | None) -> str:
    if not binding:
        return "-"
    if "namespace" in binding:
        return f"namespace=/{binding['namespace']}"
    return f"sysid={binding.get('system_id')} compid={binding.get('component_id')}"


def _describe(value: Any) -> str:
    if isinstance(value, dict):
        return _binding_label(value)
    return str(value)
