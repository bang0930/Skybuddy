"""Tests for the Drone Registry and its configuration."""

import json
from pathlib import Path

import pytest
from conftest import fast_registry_config
from pydantic import ValidationError

from app.adapters import ApDdsBinding, MavlinkDroneBinding
from app.runtime import DroneRegistry, FakeDrone, RegistryConfig
from app.schemas import (
    ConnectionStatus,
    DroneCommandType,
    DroneStatus,
    ProtocolType,
    TelemetryAvailability,
    TelemetryField,
)


class ManualClock:
    def __init__(self) -> None:
        self.now = 1_000_000.0

    def __call__(self) -> float:
        return self.now


def test_config_file_registers_two_fake_drones(tmp_path) -> None:
    path = tmp_path / "drones.json"
    path.write_text(fast_registry_config().model_dump_json(), encoding="utf-8")

    registry = DroneRegistry.from_config(RegistryConfig.load(path))
    registry.connect_all()

    drone_1, drone_2 = registry.handles()
    assert (drone_1.drone_id, drone_1.protocol) == ("drone-01", ProtocolType.MAVLINK)
    assert (drone_2.drone_id, drone_2.protocol) == ("drone-02", ProtocolType.AP_DDS)
    assert all(isinstance(handle.client, FakeDrone) for handle in registry.handles())
    assert all(handle.status == DroneStatus.AVAILABLE for handle in registry.handles())


def test_bundled_fake_config_is_valid() -> None:
    config = RegistryConfig.load(
        Path(__file__).resolve().parents[1] / "config" / "drones.fake.json"
    )
    assert [drone.drone_id for drone in config.drones] == ["drone-01", "drone-02"]


def test_connect_creates_protocol_bindings(registry) -> None:
    assert registry.get("drone-01").binding == MavlinkDroneBinding(
        drone_id="drone-01", system_id=7, component_id=1
    )
    assert registry.get("drone-02").binding == ApDdsBinding(
        drone_id="drone-02", namespace="ap"
    )


def test_state_is_normalized_per_protocol(registry) -> None:
    mavlink_state = registry.state("drone-01")
    ap_dds_state = registry.state("drone-02")

    assert mavlink_state.connection_status == ConnectionStatus.CONNECTED
    assert mavlink_state.satellites_visible == 12
    assert ap_dds_state.connection_status == ConnectionStatus.CONNECTED
    assert ap_dds_state.telemetry_availability(TelemetryField.VIBRATION) == (
        TelemetryAvailability.UNSUPPORTED
    )


def test_stale_telemetry_turns_disconnected() -> None:
    clock = ManualClock()
    registry = DroneRegistry.from_config(fast_registry_config(), clock=clock)
    registry.connect_all()
    assert registry.state("drone-01").connection_status == ConnectionStatus.CONNECTED

    registry.get("drone-01").client.link_up = False
    registry._refresh_timeout_s = 0.01
    clock.now += 3.5

    state = registry.state("drone-01")
    assert state.connection_status == ConnectionStatus.DISCONNECTED
    # The last known position is kept; only the link state changes.
    assert state.position is not None
    assert state.status == DroneStatus.AVAILABLE


def test_drone_without_any_telemetry_is_disconnected() -> None:
    registry = DroneRegistry.from_config(fast_registry_config())
    # Not connected: the fake returns no telemetry.
    registry._refresh_timeout_s = 0.01

    state = registry.state("drone-02")
    assert state.connection_status == ConnectionStatus.DISCONNECTED
    assert state.position is None


def test_status_changes_are_reflected_in_state(registry) -> None:
    registry.set_status("drone-02", DroneStatus.UNAVAILABLE)
    assert registry.state("drone-02").status == DroneStatus.UNAVAILABLE


def test_return_home_is_advertised_only_when_client_provides_it() -> None:
    class ReturningFake(FakeDrone):
        def return_home(self):
            return None

    def factory(config):
        return ReturningFake(home_lat=config.fake.home.latitude,
                             home_lon=config.fake.home.longitude)

    registry = DroneRegistry.from_config(fast_registry_config(), client_factory=factory)
    assert DroneCommandType.RETURN_HOME in registry.get("drone-01").capabilities.commands

    default = DroneRegistry.from_config(fast_registry_config())
    assert DroneCommandType.RETURN_HOME not in default.get("drone-01").capabilities.commands


def test_telemetry_read_inside_client_methods_updates_the_cache(registry) -> None:
    handle = registry.get("drone-01")
    before, _ = handle.latest_sample()

    handle.client.takeoff(30)  # internally polls self.get_telemetry()

    after, age = handle.latest_sample()
    assert after["relative_alt_m"] >= 28.5
    assert after["timestamp"] > before["timestamp"]
    assert age is not None


@pytest.mark.parametrize(
    "drone",
    [
        {"drone_id": "drone-01", "protocol": "mavlink", "mode": "fake"},
        {"drone_id": "drone-01", "protocol": "mavlink", "mode": "real"},
    ],
)
def test_config_requires_mode_specific_options(drone) -> None:
    with pytest.raises(ValidationError, match="requires"):
        RegistryConfig.model_validate({"drones": [drone]})


def test_config_rejects_duplicate_drone_ids() -> None:
    config = json.loads(fast_registry_config().model_dump_json())
    config["drones"][1]["drone_id"] = "drone-01"
    with pytest.raises(ValidationError, match="unique drone_id"):
        RegistryConfig.model_validate(config)


def test_fake_drone_heading_never_reaches_360() -> None:
    drone = FakeDrone(home_lat=-35.363261, home_lon=149.165197, telemetry_interval_s=0.001)
    drone.connect()
    drone._heading_deg = 359.997

    assert drone.get_telemetry()["heading_deg"] == 0.0


def test_invalid_sample_does_not_break_other_drones_or_the_report() -> None:
    clock = ManualClock()
    registry = DroneRegistry.from_config(fast_registry_config(), clock=clock)
    registry.connect_all()
    good = registry.state("drone-01")
    handle = registry.get("drone-01")

    # A sample outside the contract (battery above 100%) arrives from the adapter.
    bad = dict(handle.latest_sample()[0], battery_remaining_pct=150.0)
    handle._record(bad)

    states = {state.drone_id: state for state in registry.states(refresh=False)}

    assert set(states) == {"drone-01", "drone-02"}
    assert states["drone-01"].battery_percent == good.battery_percent  # last valid sample
    assert states["drone-01"].connection_status == ConnectionStatus.CONNECTED
    assert "battery_percent" in handle.sample_error

    # The rejected sample does not count as received: the link ages out.
    clock.now += 5
    assert registry.state("drone-01", refresh=False).connection_status == (
        ConnectionStatus.DISCONNECTED
    )


def test_failed_connection_keeps_server_running_with_other_drones() -> None:
    class UnreachableDrone(FakeDrone):
        def connect(self, timeout=30):
            raise RuntimeError("heartbeat not received")

        def get_telemetry(self, timeout=2):
            raise AssertionError("must not read telemetry from an unconnected drone")

    def factory(config):
        cls = UnreachableDrone if config.drone_id == "drone-02" else FakeDrone
        return cls(home_lat=config.fake.home.latitude, home_lon=config.fake.home.longitude,
                   telemetry_interval_s=0.001)

    registry = DroneRegistry.from_config(fast_registry_config(), client_factory=factory)
    errors = registry.connect_all()

    assert errors == {"drone-02": "RuntimeError: heartbeat not received"}
    assert registry.get("drone-01").connected
    assert not registry.get("drone-02").connected
    states = {state.drone_id: state for state in registry.states()}
    assert states["drone-01"].connection_status == ConnectionStatus.CONNECTED
    assert states["drone-02"].connection_status == ConnectionStatus.DISCONNECTED
    assert states["drone-02"].position is None


def test_dispatcher_settings_load_from_registry_config() -> None:
    config = RegistryConfig.model_validate(
        json.loads(fast_registry_config().model_dump_json())
        | {"dispatcher": {"reach_threshold_m": 5, "waypoint_timeout_s": 90,
                          "min_ground_speed_m_s": 2.5}}
    )
    assert config.dispatcher.reach_threshold_m == 5
    assert config.dispatcher.leg_timeout_s(None) == 90
    assert config.dispatcher.leg_timeout_s(100) == 90       # short leg: default applies
    assert config.dispatcher.leg_timeout_s(500) == 200      # long leg: 500m / 2.5m/s
    # Omitted section keeps the documented defaults (60 s, 3 m, 2.0 m/s).
    defaults = fast_registry_config().dispatcher
    assert (defaults.waypoint_timeout_s, defaults.reach_threshold_m,
            defaults.min_ground_speed_m_s) == (60, 3, 2.0)
