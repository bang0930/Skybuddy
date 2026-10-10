"""End-to-end tests of plan submission and execution with fake drones."""

import json
import time

from conftest import plan

from app.runtime import Dispatcher, DispatcherConfig, MissionService
from app.schemas import (
    CommandStatus,
    DroneStatus,
    MissionState,
    SubmissionStatus,
    TaskState,
)
from app.schemas.command import _ALLOWED_STATUS_TRANSITIONS


def wait_for_completion(service, mission_id="mission-001", timeout_s=30.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        report = service.mission_status(mission_id)
        if report.state not in (MissionState.RUNNING, MissionState.IDLE):
            return report
        time.sleep(0.05)
    raise AssertionError("mission did not finish in time")


def test_accepted_plan_flies_both_drones_to_completion(service, event_log) -> None:
    result = service.submit_plan(
        plan(("drone-01", "area-north"), ("drone-02", "area-south"))
    )
    assert result.status == SubmissionStatus.ACCEPTED
    assert {task.drone_id for task in result.tasks} == {"drone-01", "drone-02"}
    # Both drones are assigned at the same time: they fly concurrently.
    assert service.mission_status("mission-001").state == MissionState.RUNNING
    assert {d.status for d in service.registry.states(refresh=False)} == {
        DroneStatus.ASSIGNED
    }

    report = wait_for_completion(service)

    assert report.state == MissionState.SUCCEEDED
    for task in report.tasks:
        assert task.state == TaskState.SUCCEEDED
        assert all(c.status == CommandStatus.SUCCEEDED for c in task.commands)
    assert all(d.status == DroneStatus.AVAILABLE for d in report.drones)
    landed = {d.drone_id: d.position.altitude_m for d in report.drones}
    assert all(altitude < 0.1 for altitude in landed.values())


def test_command_history_only_contains_contract_transitions(service) -> None:
    service.submit_plan(plan(("drone-02", "area-south")))
    report = wait_for_completion(service)

    for command in report.tasks[0].commands:
        previous = CommandStatus.PENDING
        for result in command.history:
            assert result.previous_status == previous
            assert result.status in _ALLOWED_STATUS_TRANSITIONS[previous]
            previous = result.status
        # DroneInterface has no ACK: accepted must never be fabricated.
        assert CommandStatus.ACCEPTED not in [r.status for r in command.history]

    goto = next(c for c in report.tasks[0].commands if c.command_type == "goto")
    assert [r.status for r in goto.history] == [
        CommandStatus.SENT,
        CommandStatus.EXECUTING,
        CommandStatus.SUCCEEDED,
    ]


def test_rejected_plans_are_counted_and_do_not_start_flights(service) -> None:
    first = service.submit_plan(plan(("drone-01", "area-west")))
    second = service.submit_plan(plan(("drone-01", "area-north")))

    assert first.status == SubmissionStatus.REJECTED
    assert first.errors[0].code == "UNKNOWN_AREA"
    assert (first.attempt, second.attempt) == (1, 2)
    assert second.status == SubmissionStatus.ACCEPTED
    wait_for_completion(service)


def test_busy_drone_cannot_be_assigned_twice(service) -> None:
    service.submit_plan(plan(("drone-01", "area-north")))
    again = service.submit_plan(plan(("drone-01", "area-south")))

    assert again.status == SubmissionStatus.REJECTED
    assert [issue.code for issue in again.errors] == ["DRONE_NOT_AVAILABLE"]
    wait_for_completion(service)


def test_unknown_mission_is_rejected(service) -> None:
    result = service.submit_plan(plan(("drone-01", "area-north"), mission_id="mission-404"))
    assert result.status == SubmissionStatus.REJECTED
    assert result.errors[0].code == "UNKNOWN_MISSION"


def test_lost_link_times_out_aborts_task_and_stops_drone(
    registry, event_log
) -> None:
    # A fast minimum speed keeps the distance-based leg limit short for this test.
    dispatcher = Dispatcher(
        registry,
        event_log,
        config=DispatcherConfig(waypoint_timeout_s=0.3, min_ground_speed_m_s=30),
        telemetry_poll_timeout_s=0.05,
    )
    from conftest import scenario

    service = MissionService(registry, [scenario()], dispatcher, event_log)
    client = registry.get("drone-01").client
    original_goto = client.goto

    def goto_then_drop_link(lat, lon, alt_m):
        original_goto(lat, lon, alt_m)
        client.link_up = False

    client.goto = goto_then_drop_link

    service.submit_plan(plan(("drone-01", "area-north")))
    report = wait_for_completion(service)

    task = report.tasks[0]
    statuses = [c.status for c in task.commands]
    assert task.state == TaskState.FAILED
    assert statuses[:2] == [CommandStatus.SUCCEEDED, CommandStatus.TIMED_OUT]
    assert task.commands[1].history[-1].error.code == "WAYPOINT_TIMEOUT"
    assert set(statuses[2:]) == {CommandStatus.FAILED}
    assert task.commands[2].history[-1].error.code == "TASK_ABORTED"
    assert registry.get("drone-01").status == DroneStatus.UNAVAILABLE
    assert [e["event"] for e in event_log.events].count("emergency_stop") == 1
    assert client._alt_m <= 0.05  # emergency landing brought it down


def test_event_log_records_submission_and_transitions(service, event_log) -> None:
    service.submit_plan(plan(("drone-01", "area-west")))
    service.submit_plan(plan(("drone-02", "area-south")))
    wait_for_completion(service)

    lines = [json.loads(line) for line in event_log.path.read_text().splitlines()]
    events = [line["event"] for line in lines]
    assert events[:4] == ["plan_submitted", "plan_rejected", "plan_submitted", "plan_accepted"]
    assert events[-1] == "task_finished"

    rejected = lines[1]
    assert rejected["attempt"] == 1
    assert rejected["errors"][0]["code"] == "UNKNOWN_AREA"
    assert lines[2]["plan"]["assignments"] == [
        {"drone_id": "drone-02", "area_id": "area-south"}
    ]

    transitions = [line for line in lines if line["event"] == "command_transition"]
    succeeded = [t for t in transitions if t["status"] == "succeeded"]
    assert all(t["elapsed_since_sent_s"] >= 0 for t in succeeded)
    assert {t["protocol"] for t in transitions} == {"ap_dds"}


def test_long_transit_legs_get_time_proportional_to_distance(registry, event_log) -> None:
    from conftest import scenario

    # Base limit 0.2 s would cut off a ~30 m leg; 30 m / 20 m/s = 1.5 s does not.
    dispatcher = Dispatcher(
        registry,
        event_log,
        config=DispatcherConfig(waypoint_timeout_s=0.2, min_ground_speed_m_s=20),
        telemetry_poll_timeout_s=0.05,
    )
    service = MissionService(registry, [scenario()], dispatcher, event_log)

    service.submit_plan(plan(("drone-01", "area-north")))
    report = wait_for_completion(service)

    assert report.state == MissionState.SUCCEEDED
