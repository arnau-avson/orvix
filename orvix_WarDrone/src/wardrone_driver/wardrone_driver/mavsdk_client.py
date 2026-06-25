import asyncio
import math
from dataclasses import dataclass, field
from typing import Optional, Callable, Any
from enum import Enum


class FlightMode(Enum):
    HOLD = "HOLD"
    OFFBOARD = "OFFBOARD"
    RTL = "RTL"
    LAND = "LAND"
    MISSION = "MISSION"
    TAKEOFF = "TAKEOFF"
    MANUAL = "MANUAL"
    STABILIZED = "STABILIZED"
    UNKNOWN = "UNKNOWN"


@dataclass
class TelemetryData:
    latitude_deg: float = 0.0
    longitude_deg: float = 0.0
    absolute_altitude_m: float = 0.0
    relative_altitude_m: float = 0.0
    velocity_north_m_s: float = 0.0
    velocity_east_m_s: float = 0.0
    velocity_down_m_s: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    battery_voltage_v: float = 0.0
    battery_remaining_pct: float = 0.0
    gps_num_satellites: int = 0
    gps_fix_type: int = 0
    flight_mode: FlightMode = FlightMode.UNKNOWN
    is_armed: bool = False
    is_in_air: bool = False
    is_gyrometer_calibration_ok: bool = False
    is_accelerometer_calibration_ok: bool = False
    is_magnetometer_calibration_ok: bool = False
    is_local_position_ok: bool = False
    is_global_position_ok: bool = False
    is_home_position_ok: bool = False


class MavsdkClient:
    """Pure-Python async wrapper around MAVSDK for PX4 communication."""

    def __init__(self, connection_url: str = "udp://:14540", system_id: int = 1):
        self._connection_url = connection_url
        self._system_id = system_id
        self._drone = None
        self._connected = False
        self._telemetry = TelemetryData()
        self._telemetry_callbacks: list[Callable[[TelemetryData], Any]] = []

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def telemetry(self) -> TelemetryData:
        return self._telemetry

    def add_telemetry_callback(self, cb: Callable[[TelemetryData], Any]):
        self._telemetry_callbacks.append(cb)

    def _notify_telemetry(self):
        for cb in self._telemetry_callbacks:
            try:
                cb(self._telemetry)
            except Exception:
                pass

    async def connect(self) -> bool:
        from mavsdk import System
        self._drone = System(sysid=self._system_id)
        await self._drone.connect(system_address=self._connection_url)

        # Wait for connection
        async for state in self._drone.core.connection_state():
            if state.is_connected:
                self._connected = True
                break
        return self._connected

    async def start_telemetry_tasks(self):
        """Start all telemetry subscription tasks as concurrent coroutines."""
        tasks = [
            asyncio.ensure_future(self._watch_position()),
            asyncio.ensure_future(self._watch_attitude()),
            asyncio.ensure_future(self._watch_velocity()),
            asyncio.ensure_future(self._watch_battery()),
            asyncio.ensure_future(self._watch_flight_mode()),
            asyncio.ensure_future(self._watch_armed()),
            asyncio.ensure_future(self._watch_in_air()),
            asyncio.ensure_future(self._watch_health()),
            asyncio.ensure_future(self._watch_gps_info()),
        ]
        return tasks

    async def _watch_position(self):
        async for pos in self._drone.telemetry.position():
            self._telemetry.latitude_deg = pos.latitude_deg
            self._telemetry.longitude_deg = pos.longitude_deg
            self._telemetry.absolute_altitude_m = pos.absolute_altitude_m
            self._telemetry.relative_altitude_m = pos.relative_altitude_m
            self._notify_telemetry()

    async def _watch_attitude(self):
        async for att in self._drone.telemetry.attitude_euler():
            self._telemetry.roll_deg = att.roll_deg
            self._telemetry.pitch_deg = att.pitch_deg
            self._telemetry.yaw_deg = att.yaw_deg

    async def _watch_velocity(self):
        async for vel in self._drone.telemetry.velocity_ned():
            self._telemetry.velocity_north_m_s = vel.north_m_s
            self._telemetry.velocity_east_m_s = vel.east_m_s
            self._telemetry.velocity_down_m_s = vel.down_m_s

    async def _watch_battery(self):
        async for bat in self._drone.telemetry.battery():
            self._telemetry.battery_voltage_v = bat.voltage_v
            self._telemetry.battery_remaining_pct = bat.remaining_percent * 100.0

    async def _watch_flight_mode(self):
        async for mode in self._drone.telemetry.flight_mode():
            mode_str = str(mode).upper()
            try:
                self._telemetry.flight_mode = FlightMode(mode_str)
            except ValueError:
                self._telemetry.flight_mode = FlightMode.UNKNOWN

    async def _watch_armed(self):
        async for armed in self._drone.telemetry.armed():
            self._telemetry.is_armed = armed

    async def _watch_in_air(self):
        async for in_air in self._drone.telemetry.in_air():
            self._telemetry.is_in_air = in_air

    async def _watch_health(self):
        async for health in self._drone.telemetry.health():
            self._telemetry.is_gyrometer_calibration_ok = health.is_gyrometer_calibration_ok
            self._telemetry.is_accelerometer_calibration_ok = health.is_accelerometer_calibration_ok
            self._telemetry.is_magnetometer_calibration_ok = health.is_magnetometer_calibration_ok
            self._telemetry.is_local_position_ok = health.is_local_position_ok
            self._telemetry.is_global_position_ok = health.is_global_position_ok
            self._telemetry.is_home_position_ok = health.is_home_position_ok

    async def _watch_gps_info(self):
        async for gps in self._drone.telemetry.gps_info():
            self._telemetry.gps_num_satellites = gps.num_satellites
            self._telemetry.gps_fix_type = gps.fix_type.value

    # --- Commands ---

    async def arm(self) -> None:
        await self._drone.action.arm()

    async def disarm(self) -> None:
        await self._drone.action.disarm()

    async def takeoff(self, altitude_m: float = 10.0) -> None:
        await self._drone.action.set_takeoff_altitude(altitude_m)
        await self._drone.action.takeoff()

    async def land(self) -> None:
        await self._drone.action.land()

    async def return_to_launch(self) -> None:
        await self._drone.action.return_to_launch()

    async def set_flight_mode_hold(self) -> None:
        await self._drone.action.hold()

    async def goto_location(self, latitude_deg: float, longitude_deg: float,
                             absolute_altitude_m: float, yaw_deg: float) -> None:
        """Fly to a GPS coordinate using PX4 native goto (not offboard)."""
        await self._drone.action.goto_location(
            latitude_deg, longitude_deg, absolute_altitude_m, yaw_deg
        )

    async def set_maximum_speed(self, speed_m_s: float) -> None:
        """Set the maximum horizontal speed for goto commands."""
        await self._drone.action.set_maximum_speed(speed_m_s)

    # --- Offboard ---

    async def start_offboard(self) -> None:
        from mavsdk.offboard import VelocityBodyYawspeed
        # Must send a setpoint before starting offboard
        await self._drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        await self._drone.offboard.start()

    async def stop_offboard(self) -> None:
        await self._drone.offboard.stop()

    async def send_velocity_body(self, forward_m_s: float, right_m_s: float,
                                  down_m_s: float, yawspeed_deg_s: float) -> None:
        from mavsdk.offboard import VelocityBodyYawspeed
        await self._drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(forward_m_s, right_m_s, down_m_s, yawspeed_deg_s)
        )

    async def send_position_ned(self, north_m: float, east_m: float,
                                 down_m: float, yaw_deg: float) -> None:
        from mavsdk.offboard import PositionNedYaw
        await self._drone.offboard.set_position_ned(
            PositionNedYaw(north_m, east_m, down_m, yaw_deg)
        )

    # --- VIO ---

    async def send_vision_position_estimate(
        self,
        x_m: float, y_m: float, z_m: float,
        roll_rad: float, pitch_rad: float, yaw_rad: float,
        timestamp_us: int
    ) -> None:
        from mavsdk.mocap import VisionPositionEstimate, PositionBody, AngleBody, Covariance
        pose = VisionPositionEstimate(
            time_usec=timestamp_us,
            position_body=PositionBody(x_m, y_m, z_m),
            angle_body=AngleBody(roll_rad, pitch_rad, yaw_rad),
            pose_covariance=Covariance([float('nan')] * 21),
        )
        await self._drone.mocap.set_vision_position_estimate(pose)

    # --- Coordinate transforms ---

    @staticmethod
    def enu_to_ned(x_enu: float, y_enu: float, z_enu: float):
        """Convert ENU (East-North-Up) to NED (North-East-Down)."""
        return y_enu, x_enu, -z_enu

    @staticmethod
    def ned_to_enu(north: float, east: float, down: float):
        """Convert NED (North-East-Down) to ENU (East-North-Up)."""
        return east, north, -down

    @staticmethod
    def yaw_enu_to_ned(yaw_enu_rad: float) -> float:
        """Convert yaw from ENU convention to NED convention."""
        return math.pi / 2.0 - yaw_enu_rad

    @staticmethod
    def yaw_ned_to_enu(yaw_ned_rad: float) -> float:
        """Convert yaw from NED convention to ENU convention."""
        return math.pi / 2.0 - yaw_ned_rad
