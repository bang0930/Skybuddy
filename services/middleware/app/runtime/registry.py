"""Drone Registry: build ``DroneInterface`` clients from configuration and track them.

Each registered drone owns one client. Every call into that client goes through
``DroneHandle`` so that

- only one thread talks to a client at a time (pymavlink and rclpy clients are not
  thread-safe for concurrent reads),
- every telemetry sample, including the ones a blocking ``takeoff()`` reads internally,
  refreshes the cached state used for ``connection_status``,
- print() output of the simulation adapters goes to stderr and cannot corrupt the MCP
  stdio channel.
"""

import contextlib
import json
import logging
import sys
import threading
import time
from collections.abc import Callable, Iterator
from enum import Enum
from pathlib import Path
from typing import Any, Literal

from pydantic import Field, ValidationError, model_validator
from typing_extensions import Self

from app.adapters import (
    AP_DDS_CAPABILITIES,
    MAVLINK_CAPABILITIES,
    ApDdsBinding,
    MavlinkDroneBinding,
    TelemetryStateMapper,
)
from app.schemas import (
    DroneCapabilities,
    DroneCommandType,
    DroneState,
    DroneStatus,
    GeoCoordinate,
    ProtocolType,
)
from app.schemas.mission import ContractModel, Identifier, _require_unique

from .config import DispatcherConfig
from .fake_drone import FakeDrone

logger = logging.getLogger(__name__)

# services/middleware/app/runtime/registry.py -> services/simulation/app
DEFAULT_SIMULATION_APP = Path(__file__).resolve().parents[3] / "simulation" / "app"


class DroneMode(str, Enum):
    FAKE = "fake"
    REAL = "real"


class FakeDroneOptions(ContractModel):
    """Parameters for the temporary kinematic FakeDrone."""

    home: GeoCoordinate
    horizontal_speed_m_s: float = Field(default=10.0, gt=0, le=50)
    vertical_speed_m_s: float = Field(default=6.0, gt=0, le=30)
    telemetry_interval_s: float = Field(default=0.05, gt=0, le=1)
    system_id: int = Field(default=1, ge=1, le=255)


class DroneConfig(ContractModel):
    """One drone entry of the registry configuration file."""

    drone_id: Identifier
    protocol: ProtocolType
    mode: DroneMode
    connection_string: str | None = Field(
        default=None,
        description="pymavlink string (udpin:127.0.0.1:14561) or ros2:<namespace>.",
    )
    fake: FakeDroneOptions | None = None

    @model_validator(mode="after")
    def mode_options_must_match(self) -> Self:
        if self.mode == DroneMode.FAKE and self.fake is None:
            raise ValueError("fake mode requires fake options")
        if self.mode == DroneMode.REAL and not self.connection_string:
            raise ValueError("real mode requires connection_string")
        return self


class RegistryConfig(ContractModel):
    drones: list[DroneConfig] = Field(min_length=1)
    telemetry_timeout_s: float = Field(default=3.0, gt=0, le=60)
    simulation_app_path: str | None = Field(
        default=None,
        description="Directory containing mavlink_drone.py and ros2_drone.py (real mode).",
    )
    dispatcher: DispatcherConfig = Field(default_factory=DispatcherConfig)

    @model_validator(mode="after")
    def drone_ids_must_be_unique(self) -> Self:
        _require_unique(
            [drone.drone_id for drone in self.drones],
            "drones must have unique drone_id values",
        )
        return self

    @classmethod
    def load(cls, path: Path) -> "RegistryConfig":
        return cls.model_validate(json.loads(Path(path).read_text(encoding="utf-8")))


class DroneHandle:
    """A registered drone: its client, identity, operational status, and last sample."""

    def __init__(
        self,
        config: DroneConfig,
        client: Any,
        *,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self.config = config
        self.drone_id = config.drone_id
        self.protocol = config.protocol
        self.client = client
        self.status = DroneStatus.AVAILABLE
        self.binding: MavlinkDroneBinding | ApDdsBinding | None = None
        self.capabilities = _capabilities_for(config.protocol, client)
        self.connected = False
        self.connect_error: str | None = None
        # Why the newest sample was rejected by the DroneState contract, if it was.
        self.sample_error: str | None = None
        self._clock = clock
        self._client_lock = threading.RLock()
        self._sample_lock = threading.Lock()
        self._latest: dict[str, Any] | None = None
        self._latest_at: float | None = None
        self._valid: dict[str, Any] | None = None
        self._valid_at: float | None = None
        _install_telemetry_tap(client, self._record)

    @contextlib.contextmanager
    def exclusive(self, *, blocking: bool = True) -> Iterator[bool]:
        """Serialize client access; yields False when ``blocking`` is off and it is busy."""
        acquired = self._client_lock.acquire(blocking=blocking)
        try:
            if not acquired:
                yield False
                return
            with _stdout_to_stderr():
                yield True
        finally:
            if acquired:
                self._client_lock.release()

    def connect(self, *, first_sample_timeout_s: float = 2.0) -> None:
        """Connect the client; on failure record why and re-raise."""
        try:
            with self.exclusive():
                self.client.connect()
                # Read one sample so the drone has a position and a link state immediately.
                self.client.get_telemetry(timeout=first_sample_timeout_s)
        except Exception as exc:
            self.connect_error = f"{type(exc).__name__}: {exc}"
            with contextlib.suppress(Exception), self.exclusive():
                self.client.close()
            raise
        self.connected = True
        self.connect_error = None
        self.binding = _binding_for(self)

    def close(self) -> None:
        with self.exclusive():
            self.client.close()
        self.connected = False

    def latest_sample(self) -> tuple[dict[str, Any] | None, float | None]:
        """Return the last telemetry sample and its age in seconds."""
        with self._sample_lock:
            if self._latest is None or self._latest_at is None:
                return None, None
            return dict(self._latest), max(0.0, self._clock() - self._latest_at)

    def snapshot(self) -> tuple[dict[str, Any] | None, float | None]:
        """Return the newest sample and when it was recorded."""
        with self._sample_lock:
            return (None if self._latest is None else dict(self._latest)), self._latest_at

    def last_valid(self) -> tuple[dict[str, Any] | None, float | None]:
        """Return the newest sample that passed the DroneState contract."""
        with self._sample_lock:
            return (None if self._valid is None else dict(self._valid)), self._valid_at

    def accept_sample(self, sample: dict[str, Any], recorded_at: float) -> None:
        with self._sample_lock:
            self._valid, self._valid_at = dict(sample), recorded_at
        self.sample_error = None

    def reject_sample(self, reason: str) -> None:
        if reason != self.sample_error:
            logger.warning("%s: telemetry sample rejected: %s", self.drone_id, reason)
        self.sample_error = reason

    def _record(self, sample: dict[str, Any]) -> None:
        with self._sample_lock:
            self._latest = dict(sample)
            self._latest_at = self._clock()


class DroneRegistry:
    """Registered drones addressed by ``drone_id``."""

    def __init__(
        self,
        handles: list[DroneHandle],
        *,
        telemetry_timeout_s: float = 3.0,
        refresh_timeout_s: float = 0.5,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self._handles = {handle.drone_id: handle for handle in handles}
        self._mapper = TelemetryStateMapper(telemetry_timeout_s=telemetry_timeout_s)
        self._refresh_timeout_s = refresh_timeout_s
        self._clock = clock
        self._status_lock = threading.Lock()

    @classmethod
    def from_config(
        cls,
        config: RegistryConfig,
        *,
        client_factory: Callable[[DroneConfig], Any] | None = None,
        clock: Callable[[], float] = time.time,
    ) -> "DroneRegistry":
        factory = client_factory or ClientFactory(config.simulation_app_path)
        handles = [
            DroneHandle(drone, factory(drone), clock=clock) for drone in config.drones
        ]
        return cls(handles, telemetry_timeout_s=config.telemetry_timeout_s, clock=clock)

    def connect_all(self) -> dict[str, str]:
        """Connect every drone; return ``{drone_id: error}`` for the ones that failed.

        A failed drone stays registered with no telemetry, so it is reported as
        disconnected and plan validation rejects assignments to it. The server keeps
        running with the drones that did connect.
        """
        errors = {}
        for handle in self._handles.values():
            try:
                handle.connect()
            except Exception:
                error = handle.connect_error or "unknown error"
                errors[handle.drone_id] = error
                logger.warning("%s: connect failed: %s", handle.drone_id, error)
        return errors

    def close_all(self) -> None:
        for handle in self._handles.values():
            with contextlib.suppress(Exception):
                handle.close()

    def get(self, drone_id: str) -> DroneHandle:
        try:
            return self._handles[drone_id]
        except KeyError:
            raise KeyError(f"unknown drone_id: {drone_id}") from None

    def __contains__(self, drone_id: object) -> bool:
        return drone_id in self._handles

    def handles(self) -> list[DroneHandle]:
        return list(self._handles.values())

    def set_status(self, drone_id: str, status: DroneStatus) -> None:
        with self._status_lock:
            self.get(drone_id).status = status

    def state(self, drone_id: str, *, refresh: bool = True) -> DroneState:
        """Build the current ``DroneState``, reading fresh telemetry when the client is idle.

        While a task holds the client (for example during a blocking takeoff), the cached
        sample is used instead of waiting.

        One drone's bad data never breaks the whole status report: a sample that violates
        the ``DroneState`` contract is treated as not received, and the last valid sample
        is reported instead (it turns disconnected once it is older than the timeout).
        """
        handle = self.get(drone_id)
        if refresh and handle.connected:
            with handle.exclusive(blocking=False) as acquired:
                if acquired:
                    try:
                        handle.client.get_telemetry(timeout=self._refresh_timeout_s)
                    except Exception as exc:  # report the cached state instead
                        logger.warning("%s: telemetry refresh failed: %s", drone_id, exc)

        sample, recorded_at = handle.snapshot()
        try:
            state = self._map(handle, sample, recorded_at)
        except ValueError as exc:
            handle.reject_sample(_describe_error(exc))
            return self._map(handle, *handle.last_valid())
        if sample is not None:
            handle.accept_sample(sample, recorded_at)
        return state

    def states(self, *, refresh: bool = True) -> list[DroneState]:
        return [self.state(drone_id, refresh=refresh) for drone_id in self._handles]

    def _map(
        self,
        handle: DroneHandle,
        sample: dict[str, Any] | None,
        recorded_at: float | None,
    ) -> DroneState:
        if sample is None or recorded_at is None:
            # No usable telemetry: report an empty, disconnected snapshot.
            sample, age = {"timestamp": self._clock()}, None
        else:
            age = max(0.0, self._clock() - recorded_at)
        return self._mapper.map(
            sample,
            drone_id=handle.drone_id,
            protocol=handle.protocol,
            capabilities=handle.capabilities,
            status=handle.status,
            telemetry_age_s=age,
        )


class ClientFactory:
    """Create a ``DroneInterface`` client for one configured drone."""

    def __init__(self, simulation_app_path: str | None = None) -> None:
        self._simulation_app_path = Path(simulation_app_path or DEFAULT_SIMULATION_APP)

    def __call__(self, config: DroneConfig) -> Any:
        if config.mode == DroneMode.FAKE:
            options = config.fake
            flavor: Literal["mavlink", "ap_dds"] = (
                "mavlink" if config.protocol == ProtocolType.MAVLINK else "ap_dds"
            )
            return FakeDrone(
                home_lat=options.home.latitude,
                home_lon=options.home.longitude,
                flavor=flavor,
                horizontal_speed_m_s=options.horizontal_speed_m_s,
                vertical_speed_m_s=options.vertical_speed_m_s,
                telemetry_interval_s=options.telemetry_interval_s,
                system_id=options.system_id,
            )
        # Real adapters live in the simulation service and use flat imports
        # (``from drone_interface import ...``), so their directory must be importable.
        path = str(self._simulation_app_path)
        if path not in sys.path:
            sys.path.insert(0, path)
        if config.protocol == ProtocolType.MAVLINK:
            from mavlink_drone import MavlinkDrone

            return MavlinkDrone(config.connection_string)
        from ros2_drone import Ros2Drone

        return Ros2Drone(config.connection_string, drone_id=config.drone_id)


def _describe_error(exc: ValueError) -> str:
    """Name the offending fields, e.g. ``battery_percent: Input should be ... 100``."""
    if isinstance(exc, ValidationError):
        detail = "; ".join(
            f"{'.'.join(str(part) for part in error['loc']) or 'sample'}: {error['msg']}"
            for error in exc.errors()
        )
    else:
        detail = str(exc)
    return detail[:500]


def _capabilities_for(protocol: ProtocolType, client: Any) -> DroneCapabilities:
    base = MAVLINK_CAPABILITIES if protocol == ProtocolType.MAVLINK else AP_DDS_CAPABILITIES
    commands = list(base.commands)
    # DroneInterface has no return_home() yet. Advertise it only when the client
    # actually provides one; until then the dispatcher returns via goto(home).
    if callable(getattr(client, "return_home", None)):
        commands.append(DroneCommandType.RETURN_HOME)
    return DroneCapabilities(commands=commands, telemetry_fields=base.telemetry_fields)


def _binding_for(handle: DroneHandle) -> MavlinkDroneBinding | ApDdsBinding | None:
    client = handle.client
    if handle.protocol == ProtocolType.MAVLINK:
        master = getattr(client, "master", None)
        system_id = getattr(master, "target_system", None) or getattr(client, "system_id", None)
        component_id = getattr(master, "target_component", None)
        if component_id is None:
            component_id = getattr(client, "component_id", 1)
        if system_id is None:
            return None
        return MavlinkDroneBinding(
            drone_id=handle.drone_id, system_id=system_id, component_id=component_id
        )
    namespace = getattr(client, "ns", None)
    if not namespace:
        return None
    return ApDdsBinding(drone_id=handle.drone_id, namespace=namespace)


def _install_telemetry_tap(
    client: Any, record: Callable[[dict[str, Any]], None]
) -> None:
    """Record every sample the client produces, including reads inside its own methods.

    The adapters call ``self.get_telemetry()`` from ``takeoff()``. Replacing the bound
    method on the instance makes those internal reads visible without changing the
    simulation code.
    """
    original = client.get_telemetry

    def tapped(*args: Any, **kwargs: Any) -> dict[str, Any] | None:
        sample = original(*args, **kwargs)
        if sample is not None:
            record(sample)
        return sample

    client.get_telemetry = tapped


_stdout_guard_lock = threading.Lock()
_stdout_guard_depth = 0
_saved_stdout: Any = None


@contextlib.contextmanager
def _stdout_to_stderr() -> Iterator[None]:
    """Redirect print() to stderr while any thread is inside a client call.

    The MCP stdio transport captured the original stdout when it started, so protocol
    messages are unaffected. A shared depth counter keeps concurrent drone threads from
    restoring stdout while another thread is still printing.
    """
    global _stdout_guard_depth, _saved_stdout
    with _stdout_guard_lock:
        if _stdout_guard_depth == 0:
            _saved_stdout = sys.stdout
            sys.stdout = sys.stderr
        _stdout_guard_depth += 1
    try:
        yield
    finally:
        with _stdout_guard_lock:
            _stdout_guard_depth -= 1
            if _stdout_guard_depth == 0:
                sys.stdout = _saved_stdout
                _saved_stdout = None
