"""Waypoint not reached in time with the link up: return home, land, free the area for reassignment."""

import time

from conftest import plan, scenario

from app.runtime import Dispatcher, DispatcherConfig, MissionService
from app.schemas import CommandStatus, DroneStatus, MissionState, TaskState


def wait_for_completion(service, mission_id="mission-001", timeout_s=30.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        report = service.mission_status(mission_id)
        if report.state not in (MissionState.RUNNING, MissionState.IDLE):
            return report
        time.sleep(0.05)
    raise AssertionError("mission did not finish in time")


def build_service(registry, event_log) -> MissionService:
    # Fake drones fly 50 m/s, so normal legs take well under the 1 s limit.
    dispatcher = Dispatcher(
        registry,
        event_log,
        config=DispatcherConfig(waypoint_timeout_s=1.0, min_ground_speed_m_s=30),
        telemetry_poll_timeout_s=0.05,
    )
    return MissionService(registry, [scenario()], dispatcher, event_log)


def drop_second_goto(client) -> list:
    """The vehicle never gets the second area target (like a stalled leg)."""
    original_goto = client.goto
    calls = []

    def goto(lat, lon, alt_m):
        calls.append((lat, lon))
        if len(calls) == 2:
            return
        original_goto(lat, lon, alt_m)

    client.goto = goto
    return calls


def test_waypoint_timeout_returns_home_then_lands(registry, event_log) -> None:
    service = build_service(registry, event_log)
    client = registry.get("drone-01").client
    calls = drop_second_goto(client)

    service.submit_plan(plan(("drone-01", "area-north")))
    report = wait_for_completion(service)

    task = report.tasks[0]
    statuses = [c.status for c in task.commands]
    assert task.state == TaskState.FAILED
    assert statuses[:3] == [CommandStatus.SUCCEEDED, CommandStatus.SUCCEEDED, CommandStatus.TIMED_OUT]
    assert task.commands[2].history[-1].error.code == "WAYPOINT_TIMEOUT"
    assert all(c.history[-1].error.code == "TASK_ABORTED" for c in task.commands[3:])

    names = [e["event"] for e in event_log.events]
    assert "emergency_stop" not in names
    finished = next(e for e in event_log.events if e["event"] == "timeout_return_finished")
    assert finished["return_leg"] == "reached"
    assert finished["outcome"] == "landed"
    assert finished["command_id"] == task.commands[2].command_id
    assert names.index("timeout_return_finished") < names.index("task_finished")
    # The return leg was sent after the stalled one, and the drone is home on the ground.
    assert len(calls) == 3
    assert abs(client._north_m) < 3 and abs(client._east_m) < 3
    assert client._alt_m <= 0.05
    # Held out of allocation until checked, so the area goes to another drone.
    assert registry.get("drone-01").status == DroneStatus.UNAVAILABLE


def test_area_of_timed_out_task_can_go_to_another_drone(registry, event_log) -> None:
    service = build_service(registry, event_log)
    drop_second_goto(registry.get("drone-01").client)

    service.submit_plan(plan(("drone-01", "area-north")))
    wait_for_completion(service)
    rejected = service.submit_plan(plan(("drone-01", "area-north")))
    assert rejected.errors[0].code == "DRONE_NOT_AVAILABLE"

    accepted = service.submit_plan(plan(("drone-02", "area-north")))
    assert accepted.status.value == "accepted"
    report = wait_for_completion(service)
    reassigned = next(t for t in report.tasks if t.task_id == accepted.tasks[0].task_id)
    assert reassigned.state == TaskState.SUCCEEDED
