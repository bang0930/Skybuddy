"""Lost telemetry link during a task: detection, waiting, and return after recovery."""

import threading
import time

import pytest
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


def events(event_log) -> list[str]:
    return [e["event"] for e in event_log.events]


def event(event_log, name: str) -> dict:
    return next(e for e in event_log.events if e["event"] == name)


@pytest.fixture
def make_service(registry, event_log):
    def build(**config) -> MissionService:
        dispatcher = Dispatcher(
            registry,
            event_log,
            config=DispatcherConfig(
                waypoint_timeout_s=10, link_loss_timeout_s=0.2, **config
            ),
            telemetry_poll_timeout_s=0.05,
        )
        return MissionService(registry, [scenario()], dispatcher, event_log)

    return build


def drop_link_after_first_goto(client, *, restore_after_s=None, on_restore=None):
    """Cut telemetry right after the first area goto; optionally bring it back later."""
    original_goto = client.goto
    calls = []

    def goto(lat, lon, alt_m):
        original_goto(lat, lon, alt_m)
        calls.append((lat, lon))
        if len(calls) == 1:
            client.link_up = False
            if restore_after_s is not None:
                threading.Timer(restore_after_s, restore).start()

    def restore():
        if on_restore is not None:
            on_restore()
        client.link_up = True

    client.goto = goto


def assert_link_lost_task(report) -> None:
    task = report.tasks[0]
    statuses = [c.status for c in task.commands]
    assert task.state == TaskState.FAILED
    assert statuses[:2] == [CommandStatus.SUCCEEDED, CommandStatus.FAILED]
    assert task.commands[1].history[-1].error.code == "LINK_LOST"
    assert all(c.history[-1].error.code == "TASK_ABORTED" for c in task.commands[2:])
    assert "telemetry link was lost" in task.commands[2].history[-1].error.message


def test_link_restored_mid_flight_returns_home_and_lands(
    registry, event_log, make_service
) -> None:
    service = make_service(link_recovery_timeout_s=5)
    client = registry.get("drone-01").client
    drop_link_after_first_goto(client, restore_after_s=0.5)

    service.submit_plan(plan(("drone-01", "area-north")))
    report = wait_for_completion(service)

    assert_link_lost_task(report)
    names = events(event_log)
    assert names.index("link_lost") < names.index("link_restored")
    assert "emergency_stop" not in names
    finished = event(event_log, "link_recovery_finished")
    assert finished["outcome"] == "landed"
    assert finished["return_leg"] == "reached"
    # Back at home and on the ground, but held out of allocation until checked.
    assert abs(client._north_m) < 3 and abs(client._east_m) < 3
    assert client._alt_m <= 0.05
    assert registry.get("drone-01").status == DroneStatus.UNAVAILABLE


def test_link_never_restored_leaves_drone_to_vehicle_failsafe(
    registry, event_log, make_service
) -> None:
    service = make_service(link_recovery_timeout_s=0.3)
    client = registry.get("drone-01").client
    drop_link_after_first_goto(client)

    service.submit_plan(plan(("drone-01", "area-north")))
    report = wait_for_completion(service)

    assert_link_lost_task(report)
    names = events(event_log)
    assert "link_not_restored" in names
    assert "link_restored" not in names
    assert "emergency_stop" not in names
    # No land command was sent into a dead link.
    assert client._alt_m > 1
    assert registry.get("drone-01").status == DroneStatus.UNAVAILABLE


def test_link_restored_after_failsafe_landed_sends_nothing(
    registry, event_log, make_service
) -> None:
    service = make_service(link_recovery_timeout_s=5)
    client = registry.get("drone-01").client

    def failsafe_rtl_landed():
        with client._lock:
            client._north_m = client._east_m = client._alt_m = 0.0
            client._armed = False
            client._target = None

    drop_link_after_first_goto(client, restore_after_s=0.5, on_restore=failsafe_rtl_landed)
    sent_after_loss = []
    original_land = client.land
    client.land = lambda *a, **k: sent_after_loss.append("land") or original_land(*a, **k)

    service.submit_plan(plan(("drone-01", "area-north")))
    report = wait_for_completion(service)

    assert_link_lost_task(report)
    assert event(event_log, "link_recovery_finished")["outcome"] == "already_on_ground"
    assert sent_after_loss == []
    assert registry.get("drone-01").status == DroneStatus.UNAVAILABLE


def test_short_waypoint_timeout_still_wins_over_link_loss(
    registry, event_log, make_service
) -> None:
    # The leg limit expires before link_loss_timeout_s: the earlier timeout path applies.
    service = make_service(link_recovery_timeout_s=5)
    service.dispatcher.config = DispatcherConfig(
        waypoint_timeout_s=0.1, min_ground_speed_m_s=30, link_loss_timeout_s=5
    )
    client = registry.get("drone-01").client
    drop_link_after_first_goto(client)

    service.submit_plan(plan(("drone-01", "area-north")))
    report = wait_for_completion(service)

    assert report.tasks[0].commands[1].history[-1].error.code == "WAYPOINT_TIMEOUT"
    assert "link_lost" not in events(event_log)
    assert "emergency_stop" in events(event_log)
