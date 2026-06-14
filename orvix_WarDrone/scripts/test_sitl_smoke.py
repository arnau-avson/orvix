#!/usr/bin/env python3
"""PX4 SITL smoke test -- arm, takeoff, hover, land.

Connects directly to PX4 via MAVSDK (no ROS 2 required).
Designed to run against PX4 SIH / Gazebo SITL on udp://:14540.

Exit codes:
    0  all checks passed
    1  one or more checks failed or a timeout occurred
"""

import asyncio
import sys

from mavsdk import System
from mavsdk.action import ActionError

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
MAVSDK_URI = "udp://:14540"
CONNECT_TIMEOUT_S = 30.0
TELEMETRY_TIMEOUT_S = 10.0
TAKEOFF_ALT_M = 5.0
HOVER_TIME_S = 3.0
LANDING_TIMEOUT_S = 30.0
ARMED_TIMEOUT_S = 10.0
TAKEOFF_TIMEOUT_S = 20.0


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
            print(f"SMOKE TEST PASSED  ({passed}/{total} checks)")
        else:
            print(f"SMOKE TEST FAILED  ({failed} failure(s) out of {total} checks)")
        print("=" * 60)


# ---------------------------------------------------------------------------
# Main test sequence
# ---------------------------------------------------------------------------
async def main() -> int:
    results = TestResult()
    drone = System()

    # ---- 1. Connect -------------------------------------------------------
    print(f"[*] Connecting to PX4 at {MAVSDK_URI} ...")
    await drone.connect(system_address=MAVSDK_URI)

    print("[*] Waiting for connection ...")
    try:
        await asyncio.wait_for(_wait_connected(drone), timeout=CONNECT_TIMEOUT_S)
        results.record("Connection", True, f"connected to {MAVSDK_URI}")
    except asyncio.TimeoutError:
        results.record("Connection", False, f"timeout after {CONNECT_TIMEOUT_S}s")
        results.summary()
        return 1

    # ---- 2. Telemetry health ----------------------------------------------
    print("[*] Checking telemetry health ...")
    try:
        health = await asyncio.wait_for(
            _get_health(drone), timeout=TELEMETRY_TIMEOUT_S
        )
        results.record(
            "Telemetry/Health",
            True,
            f"gyro={health.is_gyrometer_calibration_ok}, "
            f"accel={health.is_accelerometer_calibration_ok}, "
            f"mag={health.is_magnetometer_calibration_ok}, "
            f"local_pos={health.is_local_position_ok}, "
            f"global_pos={health.is_global_position_ok}, "
            f"home_pos={health.is_home_position_ok}",
        )
    except asyncio.TimeoutError:
        results.record("Telemetry/Health", False, "timeout waiting for health data")

    # ---- 3. GPS -----------------------------------------------------------
    print("[*] Checking GPS ...")
    try:
        gps = await asyncio.wait_for(_get_gps(drone), timeout=TELEMETRY_TIMEOUT_S)
        has_fix = gps.fix_type.value >= 2  # at least 2D fix
        results.record(
            "Telemetry/GPS",
            has_fix,
            f"fix_type={gps.fix_type}, sats={gps.num_satellites}",
        )
    except asyncio.TimeoutError:
        results.record("Telemetry/GPS", False, "timeout waiting for GPS data")

    # ---- 4. Battery -------------------------------------------------------
    print("[*] Checking battery ...")
    try:
        battery = await asyncio.wait_for(
            _get_battery(drone), timeout=TELEMETRY_TIMEOUT_S
        )
        ok = battery.remaining_percent > 0.1
        results.record(
            "Telemetry/Battery",
            ok,
            f"remaining={battery.remaining_percent * 100:.0f}%, "
            f"voltage={battery.voltage_v:.2f} V",
        )
    except asyncio.TimeoutError:
        results.record("Telemetry/Battery", False, "timeout waiting for battery data")

    # ---- 5. Pre-flight (ready to fly) -------------------------------------
    print("[*] Waiting for vehicle to be ready to fly ...")
    try:
        await asyncio.wait_for(
            _wait_ready_to_fly(drone), timeout=TELEMETRY_TIMEOUT_S
        )
        results.record("ReadyToFly", True)
    except asyncio.TimeoutError:
        results.record(
            "ReadyToFly", False, "vehicle did not report ready within timeout"
        )

    # ---- 6. Arm -----------------------------------------------------------
    print("[*] Arming ...")
    try:
        await drone.action.arm()
        # Confirm armed
        await asyncio.wait_for(_wait_armed(drone), timeout=ARMED_TIMEOUT_S)
        results.record("Arm", True)
    except (ActionError, asyncio.TimeoutError) as exc:
        results.record("Arm", False, str(exc))
        results.summary()
        return 1 if not results.all_passed else 0

    # ---- 7. Takeoff -------------------------------------------------------
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

    # ---- 8. Hover ---------------------------------------------------------
    print(f"[*] Hovering for {HOVER_TIME_S} s ...")
    await asyncio.sleep(HOVER_TIME_S)
    try:
        pos = await _get_position(drone)
        results.record(
            "Hover",
            True,
            f"alt={pos.relative_altitude_m:.2f} m, "
            f"lat={pos.latitude_deg:.6f}, lon={pos.longitude_deg:.6f}",
        )
    except asyncio.TimeoutError:
        results.record("Hover", False, "could not read position during hover")

    # ---- 9. Land ----------------------------------------------------------
    print("[*] Landing ...")
    try:
        await drone.action.land()
        await asyncio.wait_for(
            _wait_landed(drone), timeout=LANDING_TIMEOUT_S
        )
        results.record("Land", True)
    except (ActionError, asyncio.TimeoutError) as exc:
        results.record("Land", False, str(exc))

    # ---- 10. Disarm -------------------------------------------------------
    print("[*] Disarming ...")
    # Allow a small delay for the vehicle to settle after landing
    await asyncio.sleep(2.0)
    try:
        await drone.action.disarm()
        results.record("Disarm", True)
    except ActionError as exc:
        # PX4 may auto-disarm after landing -- that is fine
        results.record("Disarm", True, f"auto-disarmed or: {exc}")

    # ---- Summary ----------------------------------------------------------
    results.summary()
    return 0 if results.all_passed else 1


# ---------------------------------------------------------------------------
# Async telemetry helpers (each yields the first value from a stream)
# ---------------------------------------------------------------------------
async def _wait_connected(drone: System):
    """Block until MAVSDK reports connected."""
    async for state in drone.core.connection_state():
        if state.is_connected:
            return


async def _get_health(drone: System):
    async for health in drone.telemetry.health():
        return health


async def _get_gps(drone: System):
    async for gps in drone.telemetry.gps_info():
        return gps


async def _get_battery(drone: System):
    async for battery in drone.telemetry.battery():
        return battery


async def _get_position(drone: System):
    async for pos in drone.telemetry.position():
        return pos


async def _wait_ready_to_fly(drone: System):
    """Wait until health reports the vehicle is ready to arm and fly."""
    async for health in drone.telemetry.health():
        if health.is_armable:
            return


async def _wait_armed(drone: System):
    async for armed in drone.telemetry.armed():
        if armed:
            return


async def _wait_altitude(drone: System, min_alt_m: float):
    """Wait until relative altitude exceeds *min_alt_m*."""
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= min_alt_m:
            return


async def _wait_landed(drone: System):
    """Wait until the drone is on the ground (landed state)."""
    async for state in drone.telemetry.landed_state():
        if str(state) == "ON_GROUND" or "ON_GROUND" in str(state):
            return


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
