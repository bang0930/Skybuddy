"""Keep the temporary FakeDrone aligned with the simulation DroneInterface contract."""

import importlib.util
import inspect
from pathlib import Path

import pytest

from app.runtime import FakeDrone
from app.runtime.fake_drone import landing_timeout_s

SIMULATION_INTERFACE = (
    Path(__file__).resolve().parents[2] / "simulation" / "app" / "drone_interface.py"
)
METHODS = ("connect", "takeoff", "goto", "land", "disarm", "get_telemetry", "close")


def load_simulation_interface():
    spec = importlib.util.spec_from_file_location("simulation_drone_interface", SIMULATION_INTERFACE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("method", METHODS)
def test_fake_drone_matches_drone_interface_signature(method) -> None:
    contract = load_simulation_interface().DroneInterface

    expected = inspect.signature(getattr(contract, method))
    actual = inspect.signature(getattr(FakeDrone, method))

    assert list(actual.parameters) == list(expected.parameters)
    for name, parameter in expected.parameters.items():
        assert actual.parameters[name].default == parameter.default, name


def test_fake_drone_telemetry_uses_simulation_fields() -> None:
    fields = load_simulation_interface().TELEMETRY_FIELDS
    drone = FakeDrone(home_lat=-35.363261, home_lon=149.165197, telemetry_interval_s=0.001)
    drone.connect()

    assert list(drone.get_telemetry()) == fields


@pytest.mark.parametrize("altitude", [None, 0.0, 10.0, 30.0, 50.0])
def test_landing_timeout_matches_simulation_rule(altitude) -> None:
    contract = load_simulation_interface().DroneInterface
    assert landing_timeout_s(altitude) == contract.landing_timeout_s(altitude)
