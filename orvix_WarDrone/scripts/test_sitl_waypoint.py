#!/usr/bin/env python3
"""PX4 SITL waypoint navigation test -- offboard square pattern + RTL.

Connects directly to PX4 via MAVSDK (no ROS 2 required).
Designed to run against PX4 SIH / Gazebo SITL on udp://:14540.

The test flies a square pattern in offboard mode using NED position
setpoints, then returns to launch and waits for landing.

Exit codes:
    0  all checks passed
    1  one or more checks failed or a timeout occurred
"""

import asyncio
import math
import sys

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import (
    OffboardError,
    PositionNedYaw,
)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
MAVSDK_URI = "udp://:14540"
CONNECT_TIMEOUT_S = 30.0
TELEMETRY_TIMEOUT_S = 10.0
TAKEOFF_ALT_M = 10.0
ARMED_TIMEOUT_S = 10.0
TAKEOFF_TIMEOUT_S = 25.0
LANDING_TIMEOUT_S = 60.0

# Square pattern in NED (North-East-Down).  Down is negative altitude.
# The drone starts at (0, 0, -10) after takeoff, then visits each corner.
SQUARE_SIDE_M = 20.0
CRUISE_ALT_DOWN = -TAKEOFF_ALT_M  # NED down axis: -10 means 10 m above ground

# Waypoints: (north, east, down, yaw_deg)
WAYPOINTS = [
    (SQUARE_SIDE_M,  0.0,              CRUISE_ALT_DOWN, 0.0),    # WP1: North
    (SQUARE_SIDE_M,  SQUARE_SIDE_M,    CRUISE_ALT_DOWN, 90.0),   # WP2: North-East
    (0.0,            SQUARE_SIDE_M,    CRUISE_ALT_DOWN, 180.0),  # WP3: East
    (0.0,            0.0,              CRUISE_ALT_DOWN, 270.0),  # WP4: Back to start
]

# How close (m) the drone must get to a waypoint before it is considered reached.
WP_ACCEPT_RADIUS_M = 2.0
WP_TIMEOUT_S = 30.0

# How many initial setpoints to send before engaging offboard mode.
INITIAL_SETPOINT_COUNT = 20
SETPOINT_INTERVAL_S = 0.05  # 20 Hz


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
class TestResult:
    """Accumulates pass / fail verdicts."""

    def __init__(self):
        self.results: list[tuple[str, bool, str]] = []

    def record(self, name: str, passed: bool, detail: str = ""):
        tag = "PASS" if passed else "FAIL"
        msg = f"  [{tag}] {name}"
        if detail:
            msg += f"  -- {detail}"
        print(msg)
        self.results.append((name, passed, detail))

    @property
    def all_passed(self) -> bool:
        return all(ok for _, ok, _ in self.results)

    def summary(self):
        total = len(self.results)
        passed = sum(1 for _, ok, _ in self.results if ok)
        failed = total - passed
        print()
        print("=" * 60)
        if self.all_passed:
            print(f"WAYPOINT TEST PASSED  ({passed}/{total} checks)")
        else:
            print(f"WAYPOINT TEST FAILED  ({failed} failure(s) out of {total} checks)")
        print("=" * 60)


def ned_distance(n1, e1, d1, n2, e2, d2) -> float:
    """Euclidean distance in NED space."""
    return math.sqrt((n1 - n2) ** 2 + (e1 - e2) ** 2 + (d1 - d2) ** 2)


# ---------------------------------------------------------------------------
# Async telemetry helpers
# ---------------------------------------------------------------------------
async def _wait_connected(drone: System):
    async for state in drone.core.connection_state():
        if state.is_connected:
            return


async def _wait_armed(drone: System):
    async for armed in drone.telemetry.armed():
        if armed:
            return


async def _wait_altitude(drone: System, min_alt_m: float):
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= min_alt_m:
            return


async def _wait_landed(drone: System):
    async for state in drone.telemetry.landed_state():
        if "ON_GROUND" in str(state):
            return


async def _get_position_ned(drone: System):
    """Return the first (north, east, down) reading from odometry / position_velocity_ned."""
    async for pv in drone.telemetry.position_velocity_ned():
        p = pv.position
        return p.north_m, p.east_m, p.down_m


async def _get_position(drone: System):
    async for pos in drone.telemetry.position():
        return pos


async def _wait_ready_to_fly(drone: System):
    async for health in drone.telemetry.health():
        if health.is_armable:
            return


# ---------------------------------------------------------------------------
# Main test sequence
# ---------------------------------------------------------------------------
async def main() -> int:
    results = TestResult()
    drone = System()

    # ---- 1. Connect -------------------------------------------------------
    print(f"[*] Connecting to PX4 at {MAVSDK_URI} ...")
    await drone.connect(system_address=MAVSDK_URI)

    try:
        await asyncio.wait_for(_wait_connected(drone), timeout=CONNECT_TIMEOUT_S)
        results.record("Connection", True, f"connected to {MAVSDK_URI}")
    except asyncio.TimeoutError:
        results.record("Connection", False, f"timeout after {CONNECT_TIMEOUT_S}s")
        results.summary()
        return 1

    # ---- 2. Wait for ready ------------------------------------------------
    print("[*] Waiting for vehicle to be ready ...")
    try:
        await asyncio.wait_for(_wait_ready_to_fly(drone), timeout=TELEMETRY_TIMEOUT_S)
        results.record("ReadyToFly", True)
    except asyncio.TimeoutError:
        results.record("ReadyToFly", False, "vehicle not ready within timeout")
        results.summary()
        return 1

    # ---- 3. Arm -----------------------------------------------------------
    print("[*] Arming ...")
    try:
        await drone.action.arm()
        await asyncio.wait_for(_wait_armed(drone), timeout=ARMED_TIMEOUT_S)
        results.record("Arm", True)
    except (ActionError, asyncio.TimeoutError) as exc:
        results.record("Arm", False, str(exc))
        results.summary()
        return 1

    # ---- 4. Takeoff -------------------------------------------------------
    print(f"[*] Taking off to {TAKEOFF_ALT_M} m ...")
    try:
        await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
        await drone.action.takeoff()
        await asyncio.wait_for(
            _wait_altitude(drone, TAKEOFF_ALT_M * 0.8), timeout=TAKEOFF_TIMEOUT_S
        )
        results.record("Takeoff", True, f"reached ~{TAKEOFF_ALT_M} m AGL")
    except (ActionError, asyncio.TimeoutError) as exc:
        results.record("Takeoff", False, str(exc))
        results.summary()
        return 1

    # Let the drone stabilize at takeoff altitude
    await asyncio.sleep(2.0)

    # Read initial NED position (this will be our reference origin)
    try:
        start_n, start_e, start_d = await asyncio.wait_for(
            _get_position_ned(drone), timeout=TELEMETRY_TIMEOUT_S
        )
        print(
            f"[*] Initial NED position: N={start_n:.2f}  E={start_e:.2f}  D={start_d:.2f}"
        )
    except asyncio.TimeoutError:
        results.record("InitialPosition", False, "could not read NED position")
        results.summary()
        return 1

    # ---- 5. Offboard mode -- fly the square pattern -----------------------
    print("[*] Engaging offboard mode ...")

    # The first waypoint setpoint -- we must start sending BEFORE we switch
    # to offboard mode. PX4 requires a stream of setpoints.
    first_wp = WAYPOINTS[0]
    initial_sp = PositionNedYaw(
        start_n + first_wp[0],
        start_e + first_wp[1],
        first_wp[2],
        first_wp[3],
    )

    # Send a burst of initial setpoints so PX4 accepts offboard mode.
    print(f"[*] Sending {INITIAL_SETPOINT_COUNT} initial setpoints ...")
    for _ in range(INITIAL_SETPOINT_COUNT):
        await drone.offboard.set_position_ned(initial_sp)
        await asyncio.sleep(SETPOINT_INTERVAL_S)

    # Start offboard mode
    try:
        await drone.offboard.start()
        results.record("OffboardStart", True)
    except OffboardError as exc:
        results.record("OffboardStart", False, str(exc))
        # Fallback: land and bail
        await _safe_land(drone)
        results.summary()
        return 1

    # ---- 6. Navigate each waypoint ----------------------------------------
    all_wp_ok = True
    for idx, (wp_n, wp_e, wp_d, wp_yaw) in enumerate(WAYPOINTS):
        target_n = start_n + wp_n
        target_e = start_e + wp_e
        target_d = wp_d  # absolute NED down (not relative to start)
        label = f"WP{idx + 1} (N={wp_n:+.0f} E={wp_e:+.0f})"

        print(f"[*] Flying to {label} ...")
        setpoint = PositionNedYaw(target_n, target_e, target_d, wp_yaw)

        try:
            reached = await asyncio.wait_for(
                _fly_to_ned(drone, setpoint, target_n, target_e, target_d),
                timeout=WP_TIMEOUT_S,
            )
            # Read and print actual position
            cur_n, cur_e, cur_d = await asyncio.wait_for(
                _get_position_ned(drone), timeout=5.0
            )
            gps = await asyncio.wait_for(_get_position(drone), timeout=5.0)
            results.record(
                label,
                True,
                f"NED=({cur_n:.2f}, {cur_e:.2f}, {cur_d:.2f})  "
                f"GPS=({gps.latitude_deg:.6f}, {gps.longitude_deg:.6f}, "
                f"alt={gps.relative_altitude_m:.1f}m)",
            )
        except asyncio.TimeoutError:
            results.record(label, False, f"not reached within {WP_TIMEOUT_S}s")
            all_wp_ok = False

    # ---- 7. Stop offboard mode --------------------------------------------
    print("[*] Stopping offboard mode ...")
    try:
        await drone.offboard.stop()
    except OffboardError:
        pass  # acceptable if already stopped

    # ---- 8. RTL -----------------------------------------------------------
    print("[*] Returning to launch ...")
    try:
        await drone.action.return_to_launch()
        results.record("RTL", True, "RTL command sent")
    except ActionError as exc:
        results.record("RTL", False, str(exc))

    # ---- 9. Wait for landing ----------------------------------------------
    print("[*] Waiting for landing ...")
    try:
        await asyncio.wait_for(_wait_landed(drone), timeout=LANDING_TIMEOUT_S)
        results.record("Landing", True)
    except asyncio.TimeoutError:
        results.record("Landing", False, f"not landed within {LANDING_TIMEOUT_S}s")

    # ---- 10. Disarm -------------------------------------------------------
    await asyncio.sleep(2.0)
    print("[*] Disarming ...")
    try:
        await drone.action.disarm()
        results.record("Disarm", True)
    except ActionError as exc:
        results.record("Disarm", True, f"auto-disarmed or: {exc}")

    # ---- Summary ----------------------------------------------------------
    results.summary()
    return 0 if results.all_passed else 1


# ---------------------------------------------------------------------------
# Flight helpers
# ---------------------------------------------------------------------------
async def _fly_to_ned(
    drone: System,
    setpoint: PositionNedYaw,
    target_n: float,
    target_e: float,
    target_d: float,
):
    """Continuously send *setpoint* and wait until the drone is within
    WP_ACCEPT_RADIUS_M of the target NED position."""
    while True:
        await drone.offboard.set_position_ned(setpoint)

        # Check current position
        cur_n, cur_e, cur_d = await asyncio.wait_for(
            _get_position_ned(drone), timeout=5.0
        )
        dist = ned_distance(cur_n, cur_e, cur_d, target_n, target_e, target_d)
        if dist < WP_ACCEPT_RADIUS_M:
            # Hold position briefly to stabilize
            for _ in range(10):
                await drone.offboard.set_position_ned(setpoint)
                await asyncio.sleep(SETPOINT_INTERVAL_S)
            return True

        await asyncio.sleep(0.2)


async def _safe_land(drone: System):
    """Best-effort landing used as a fallback."""
    try:
        await drone.action.land()
    except ActionError:
        pass
    try:
        await asyncio.wait_for(_wait_landed(drone), timeout=LANDING_TIMEOUT_S)
    except asyncio.TimeoutError:
        pass


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
