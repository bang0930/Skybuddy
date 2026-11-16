"""Tests for splitting a mission's search area through the mission service."""

import pytest
from conftest import plan, scenario
from test_mission_service import wait_for_completion

from app.runtime import Dispatcher, DispatcherConfig, MissionService, UnknownMissionError
from app.schemas import MissionState, SubmissionStatus


def _area_ids(service) -> list[str]:
    context = service.mission_context("mission-001")
    return [area.area_id for area in context.search_areas]


def test_split_replaces_the_area_with_its_pieces_in_place(service, event_log) -> None:
    result = service.split_area("mission-001", "area-north", 2)

    assert result["status"] == "split"
    assert [area["area_id"] for area in result["areas"]] == ["area-north-1", "area-north-2"]
    assert set(result["areas"][0]["center"]) == {"latitude", "longitude"}
    assert _area_ids(service) == ["area-north-1", "area-north-2", "area-south"]
    split = next(e for e in event_log.events if e["event"] == "area_split")
    assert split["area_ids"] == ["area-north-1", "area-north-2"]


def test_pieces_can_be_assigned_and_flown(service) -> None:
    service.split_area("mission-001", "area-north", 2)

    result = service.submit_plan(
        plan(("drone-01", "area-north-1"), ("drone-02", "area-north-2"))
    )

    assert result.status == SubmissionStatus.ACCEPTED
    assert wait_for_completion(service).state == MissionState.SUCCEEDED


def test_parent_area_cannot_be_assigned_after_split(service) -> None:
    service.split_area("mission-001", "area-north", 2)

    result = service.submit_plan(plan(("drone-01", "area-north")))

    assert result.status == SubmissionStatus.REJECTED
    assert result.errors[0].code == "UNKNOWN_AREA"


def test_rejects_unknown_area_and_keeps_areas(service, event_log) -> None:
    result = service.split_area("mission-001", "area-west", 2)

    assert result["status"] == "rejected"
    assert result["errors"][0]["code"] == "UNKNOWN_AREA"
    assert _area_ids(service) == ["area-north", "area-south"]
    assert any(e["event"] == "area_split_rejected" for e in event_log.events)


def test_rejects_invalid_piece_count(service) -> None:
    result = service.split_area("mission-001", "area-north", 1)

    assert result["errors"][0]["code"] == "INVALID_N_PIECES"
    assert _area_ids(service) == ["area-north", "area-south"]


def test_rejects_area_with_a_running_task(service) -> None:
    service.submit_plan(plan(("drone-01", "area-north")))

    result = service.split_area("mission-001", "area-north", 2)

    assert result["errors"][0]["code"] == "AREA_IN_USE"
    assert _area_ids(service) == ["area-north", "area-south"]
    wait_for_completion(service)


def test_rejects_piece_id_that_already_exists(registry, event_log) -> None:
    base = scenario()
    taken = base.search_areas[1].model_copy(update={"area_id": "area-north-1"})
    mission = scenario(
        search_areas=[base.search_areas[0].model_dump(), taken.model_dump()]
    )
    dispatcher = Dispatcher(registry, event_log, config=DispatcherConfig())
    service = MissionService(registry, [mission], dispatcher, event_log)

    result = service.split_area("mission-001", "area-north", 2)

    assert result["errors"][0]["code"] == "AREA_ID_TAKEN"


def test_unknown_mission_raises(service) -> None:
    with pytest.raises(UnknownMissionError):
        service.split_area("mission-404", "area-north", 2)
