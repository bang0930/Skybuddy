"""Tests for the integration evidence checks, run on real FakeDrone event logs."""

import copy

import pytest
from conftest import fast_registry_config, plan, scenario
from test_mission_service import wait_for_completion

from app.integration_evidence import (
    check_identifier_isolation,
    command_stage,
    command_summaries,
    connection_gaps,
)
from app.runtime import (
    Dispatcher,
    DispatcherConfig,
    DroneRegistry,
    EventLog,
    MissionService,
)


@pytest.fixture(scope="module")
def dual_run_log():
    """Event log of one two-drone FakeDrone mission, with its ``server_started`` record.

    The flight runs once per module; tests tamper with copies (``dual_run_events``).
    """
    registry = DroneRegistry.from_config(fast_registry_config())
    registry.connect_all()
    event_log = EventLog()
    event_log.write(
        "server_started", drones=[handle.describe() for handle in registry.handles()]
    )
    dispatcher = Dispatcher(
        registry,
        event_log,
        config=DispatcherConfig(waypoint_timeout_s=10),
        telemetry_poll_timeout_s=0.2,
    )
    service = MissionService(registry, [scenario()], dispatcher, event_log)
    try:
        result = service.submit_plan(
            plan(("drone-01", "area-north"), ("drone-02", "area-south"))
        )
        assert result.status.value == "accepted"
        assert wait_for_completion(service).state.value == "succeeded"
    finally:
        registry.close_all()
    return event_log.events


@pytest.fixture
def dual_run_events(dual_run_log):
    return copy.deepcopy(dual_run_log)


def _sent(events, drone_id):
    return next(
        event
        for event in events
        if event["event"] == "command_transition"
        and event["status"] == "sent"
        and event["drone_id"] == drone_id
    )


def test_server_started_records_connection_and_binding(registry) -> None:
    drones = {handle.drone_id: handle.describe() for handle in registry.handles()}

    assert set(drones["drone-01"]) == {
        "drone_id", "protocol", "mode", "connection_string", "connected",
        "connect_error", "binding",
    }
    assert drones["drone-01"]["binding"].system_id == 7
    assert drones["drone-02"]["binding"].namespace == "ap"
    assert drones["drone-02"]["connected"] is True


def test_dual_fake_run_passes_identifier_check(dual_run_events) -> None:
    check = check_identifier_isolation(
        dual_run_events, selected_drones=["drone-01", "drone-02"]
    )

    assert check.verdict == "PASS", check.failures
    sent = {drone.drone_id: drone.sent_commands for drone in check.drones}
    planned = sum(
        len(commands)
        for event in dual_run_events
        if event["event"] == "plan_accepted"
        for commands in event["commands"]
    )
    assert sent["drone-01"] > 0 and sent["drone-02"] > 0
    assert sum(sent.values()) == planned
    # Every sent transition names the client that was actually invoked.
    assert _sent(dual_run_events, "drone-01")["target"] == {
        "drone_id": "drone-01",
        "protocol": "mavlink",
        "binding": {"drone_id": "drone-01", "system_id": 7, "component_id": 1},
    }


def test_command_sent_to_another_drones_client_fails(dual_run_events) -> None:
    other = _sent(dual_run_events, "drone-02")["target"]
    _sent(dual_run_events, "drone-01")["target"] = other

    check = check_identifier_isolation(dual_run_events)

    assert check.verdict == "FAIL"
    assert any("sent to the wrong client" in failure for failure in check.failures)
    mismatches = {drone.drone_id: drone.mismatches for drone in check.drones}
    assert mismatches == {"drone-01": 1, "drone-02": 0}


def test_binding_change_after_registration_fails(dual_run_events) -> None:
    _sent(dual_run_events, "drone-01")["target"]["binding"]["system_id"] = 2

    check = check_identifier_isolation(dual_run_events)

    assert check.verdict == "FAIL"
    assert any("sysid=2" in failure for failure in check.failures)


def test_sent_without_target_fails(dual_run_events) -> None:
    del _sent(dual_run_events, "drone-02")["target"]

    check = check_identifier_isolation(dual_run_events)

    assert check.verdict == "FAIL"
    assert any("without a recorded target" in failure for failure in check.failures)


def test_planned_protocol_mismatch_fails(dual_run_events) -> None:
    accepted = next(e for e in dual_run_events if e["event"] == "plan_accepted")
    accepted["commands"][0][0]["protocol"] = "ap_dds"

    check = check_identifier_isolation(dual_run_events)

    assert check.verdict == "FAIL"
    assert any("protocol ap_dds" in failure for failure in check.failures)


def test_transition_logged_under_another_drone_fails(dual_run_events) -> None:
    event = next(
        e for e in dual_run_events
        if e["event"] == "command_transition" and e["drone_id"] == "drone-01"
        and e["status"] == "succeeded"
    )
    event["drone_id"] = "drone-02"

    check = check_identifier_isolation(dual_run_events)

    assert check.verdict == "FAIL"
    assert any("transition drone_id=drone-02" in failure for failure in check.failures)


def test_unselected_drone_in_plan_fails(dual_run_events) -> None:
    check = check_identifier_isolation(dual_run_events, selected_drones=["drone-01"])

    assert check.verdict == "FAIL"
    assert any("unselected drone-02" in failure for failure in check.failures)


def test_shared_namespace_or_connection_string_fails(dual_run_events) -> None:
    started = next(e for e in dual_run_events if e["event"] == "server_started")
    started["drones"][0]["connection_string"] = "udpin:127.0.0.1:14561"
    started["drones"][1]["connection_string"] = "udpin:127.0.0.1:14561"

    check = check_identifier_isolation(dual_run_events)

    assert check.verdict == "FAIL"
    assert any("share connection string" in failure for failure in check.failures)


def test_separate_mavlink_links_may_reuse_sysid() -> None:
    events = [
        {
            "event": "server_started",
            "drones": [
                {
                    "drone_id": drone_id,
                    "protocol": "mavlink",
                    "connection_string": link,
                    "binding": {"drone_id": drone_id, "system_id": 1, "component_id": 1},
                }
                for drone_id, link in (
                    ("drone-01", "udpin:127.0.0.1:14561"),
                    ("drone-03", "udpin:127.0.0.1:14571"),
                )
            ],
        }
    ]

    check = check_identifier_isolation(events)

    assert check.failures == []
    assert check.verdict == "N/A"  # nothing was sent


def test_missing_server_started_fails() -> None:
    assert check_identifier_isolation([]).verdict == "FAIL"


def test_command_summaries_follow_each_command(dual_run_events) -> None:
    rows = {row["command_id"]: row for row in command_summaries(dual_run_events)}

    takeoff = next(row for row in rows.values() if row["command_type"] == "takeoff")
    goto = next(row for row in rows.values() if row["command_type"] == "goto")
    assert takeoff["transitions"] == "pending>sent>succeeded"
    assert goto["transitions"] == "pending>sent>executing>succeeded"
    assert all(row["final_status"] == "succeeded" for row in rows.values())
    assert all(row["elapsed_s"] is not None for row in rows.values())


def _status(connection: str, command_status: str = "sent") -> dict:
    return {
        "drones": [{"drone_id": "drone-01", "connection_status": connection}],
        "tasks": [
            {
                "task_id": "task-001-drone-01",
                "drone_id": "drone-01",
                "state": "running",
                "commands": [
                    {"command_id": "c01", "command_type": "takeoff", "status": "succeeded"},
                    {"command_id": "c02", "command_type": "land", "status": command_status},
                ],
            }
        ],
    }


def test_connection_gaps_group_consecutive_disconnected_reports() -> None:
    reports = [
        _status("connected"),
        _status("disconnected"),
        _status("disconnected"),
        _status("connected", "succeeded"),
        _status("disconnected", "succeeded"),
    ]
    observations = [
        {"at": f"t{i}", "elapsed_s": float(i), "response": report}
        for i, report in enumerate(reports)
    ]

    gaps = connection_gaps(observations, ["drone-01", "drone-02"])

    assert [(g["start_at"], g["end_at"], g["observations"]) for g in gaps] == [
        ("t1", "t2", 2),
        ("t4", "t4", 1),
    ]
    assert gaps[0]["stages"] == ["c02 land (sent)"]
    assert gaps[0]["recovered"] is True
    assert gaps[1]["stages"] == ["task-001-drone-01 finished (running)"]
    assert gaps[1]["recovered"] is False


def test_command_stage_before_any_task() -> None:
    assert command_stage({"drones": [], "tasks": []}, "drone-01") == "no task"
