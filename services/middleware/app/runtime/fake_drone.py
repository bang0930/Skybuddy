"""Temporary ``DroneInterface``-compatible vehicle for integration checks without AirSim.

AirSim does not run on macOS, so the MVP pipeline is verified against this simple
kinematic model. It follows the method signatures, completion rules, and ``Telemetry``
schema of ``services/simulation/app/drone_interface.py``. The official FakeDrone belongs
to the simulation part; replace the ``fake`` factory in ``registry.py`` once it exists.

The model moves linearly toward the latest target at fixed speeds and has no physics,
battery drain, or wind. It is not evidence that a real flight would succeed.
"""

import math
import threading
import time
from typing import Literal

METERS_PER_DEGREE_LAT = 111_320.0

FakeFlavor = Literal["mavlink", "ap_dds"]


class FakeDrone:
    """Kinematic stand-in exposing the seven ``DroneInterface`` methods."""

    def __init__(
        self,
        *,
        home_lat: float,
        home_lon: float,
        flavor: FakeFlavor = "mavlink",
        horizontal_speed_m_s: float = 10.0,
        vertical_speed_m_s: float = 6.0,
        telemetry_interval_s: float = 0.05,
        system_id: int = 1,
        component_id: int = 1,
        namespace: str = "ap",
    ) -> None:
        if horizontal_speed_m_s <= 0 or vertical_speed_m_s <= 0:
            raise ValueError("fake drone speeds must be positive")
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.flavor = flavor
        self.horizontal_speed_m_s = horizontal_speed_m_s
        self.vertical_speed_m_s = vertical_speed_m_s
        self.telemetry_interval_s = telemetry_interval_s
        # Identity a real adapter learns while connecting (MAVLink SYSID / ROS namespace).
        self.system_id = system_id
        self.component_id = component_id
        self.ns = namespace

        self.link_up = True
        self._lock = threading.Lock()
        self._connected = False
        self._armed = False
        self._north_m = 0.0
        self._east_m = 0.0
        self._alt_m = 0.0
        self._target: tuple[float, float, float] | None = None
        self._velocity = (0.0, 0.0, 0.0)
        self._heading_deg = 0.0
        self._updated_at = time.monotonic()

    # ------------------------------------------------------------------ connection

    def connect(self, timeout=30):
        self._connected = True
        self._updated_at = time.monotonic()
        return self

    def close(self):
        self._connected = False

    # ---------------------------------------------------------------- flight commands

    def takeoff(self, altitude_m):
        """Arm, climb, and return only after reaching 95% of the target altitude."""
        self._require_connected()
        with self._lock:
            self._advance()
            self._armed = True
            self._target = (self._north_m, self._east_m, float(altitude_m))
        deadline = time.monotonic() + self._travel_time(0, altitude_m) * 3 + 5
        while time.monotonic() < deadline:
            sample = self.get_telemetry(timeout=2)
            if sample is not None and sample["relative_alt_m"] >= altitude_m * 0.95:
                return
        raise RuntimeError(f"fake takeoff did not reach {altitude_m}m")

    def goto(self, lat, lon, alt_m):
        """Set the target and return immediately (fire-and-forget)."""
        self._require_connected()
        with self._lock:
            self._advance()
            if not self._armed or self._alt_m <= 0.5:
                raise RuntimeError("fake drone must take off before goto")
            north, east = self._offset(lat, lon)
            self._target = (north, east, float(alt_m))

    def land(self, timeout=60):
        """Descend in place and return True once disarmed on the ground."""
        self._require_connected()
        with self._lock:
            self._advance()
            self._target = (self._north_m, self._east_m, 0.0)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            # The vehicle descends on its own even if the telemetry link is down.
            time.sleep(self.telemetry_interval_s)
            with self._lock:
                self._advance()
                if self._alt_m <= 0.05:
                    self._armed = False
                    self._target = None
                    return True
        return False

    def disarm(self, timeout=5):
        with self._lock:
            self._advance()
            self._armed = False
            self._target = None
        return True

    # -------------------------------------------------------------------- telemetry

    def get_telemetry(self, timeout=2):
        if not self._connected or not self.link_up:
            time.sleep(timeout)
            return None
        time.sleep(min(self.telemetry_interval_s, timeout))
        with self._lock:
            self._advance()
            lat, lon = self._latlon(self._north_m, self._east_m)
            north_v, east_v, down_v = self._velocity
            heading = self._heading_deg
            altitude = self._alt_m
        is_mavlink = self.flavor == "mavlink"
        return {
            "lat": lat,
            "lon": lon,
            "relative_alt_m": round(altitude, 3),
            "vx": round(north_v, 3),
            "vy": round(east_v, 3),
            "vz": round(down_v, 3),
            "heading_deg": round(heading, 2),
            "gps_fix_type": 3,
            # AP_DDS provides no satellite count, vibration, or clipping topics.
            "satellites_visible": 12 if is_mavlink else None,
            "gps_eph": 0.8,
            "battery_voltage_v": 12.6,
            "battery_remaining_pct": 100.0,
            "vibration_x": 0.02 if is_mavlink else None,
            "vibration_y": 0.02 if is_mavlink else None,
            "vibration_z": 0.05 if is_mavlink else None,
            "clipping": 0 if is_mavlink else None,
            "timestamp": time.time(),
        }

    @staticmethod
    def distance_m(lat1, lon1, lat2, lon2):
        dlat = (lat2 - lat1) * METERS_PER_DEGREE_LAT
        dlon = (lon2 - lon1) * METERS_PER_DEGREE_LAT * math.cos(math.radians(lat1))
        return math.sqrt(dlat**2 + dlon**2)

    # --------------------------------------------------------------------- internals

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("fake drone is not connected")

    def _advance(self) -> None:
        """Move toward the target for the time elapsed since the last update."""
        now = time.monotonic()
        dt = now - self._updated_at
        self._updated_at = now
        if self._target is None or dt <= 0:
            self._velocity = (0.0, 0.0, 0.0)
            return
        target_north, target_east, target_alt = self._target
        d_north = target_north - self._north_m
        d_east = target_east - self._east_m
        d_alt = target_alt - self._alt_m

        horizontal = math.hypot(d_north, d_east)
        step = min(horizontal, self.horizontal_speed_m_s * dt)
        if horizontal > 0:
            self._north_m += d_north / horizontal * step
            self._east_m += d_east / horizontal * step
            self._heading_deg = math.degrees(math.atan2(d_east, d_north)) % 360
        climb = math.copysign(min(abs(d_alt), self.vertical_speed_m_s * dt), d_alt)
        self._alt_m = max(0.0, self._alt_m + climb)

        north_v = (d_north / horizontal * step / dt) if horizontal > 0 else 0.0
        east_v = (d_east / horizontal * step / dt) if horizontal > 0 else 0.0
        self._velocity = (north_v, east_v, -climb / dt)

    def _travel_time(self, from_alt: float, to_alt: float) -> float:
        return abs(to_alt - from_alt) / self.vertical_speed_m_s

    def _offset(self, lat: float, lon: float) -> tuple[float, float]:
        north = (lat - self.home_lat) * METERS_PER_DEGREE_LAT
        east = (
            (lon - self.home_lon)
            * METERS_PER_DEGREE_LAT
            * math.cos(math.radians(self.home_lat))
        )
        return north, east

    def _latlon(self, north: float, east: float) -> tuple[float, float]:
        lat = self.home_lat + north / METERS_PER_DEGREE_LAT
        lon = self.home_lon + east / (
            METERS_PER_DEGREE_LAT * math.cos(math.radians(self.home_lat))
        )
        return lat, lon
