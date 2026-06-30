"""Tests for enhanced preflight checks (F4).

Since the actual _do_preflight() method lives in MissionControllerNode (requires rclpy),
we replicate the check logic inline to verify correctness without ROS dependencies.
"""

import pytest


# ---------------------------------------------------------------------------
# Replicated preflight check functions (mirror _do_preflight logic)
# ---------------------------------------------------------------------------

def preflight_check_gps(gps_fix: int, gps_sats: int, require_3d: bool = True, min_sats: int = 6):
    """Replicate GPS preflight logic."""
    ok = True
    failures = []
    if require_3d and gps_fix < 3:
        ok = False
        failures.append(f"GPS fix {gps_fix} < 3")
    if gps_sats < min_sats:
        ok = False
        failures.append(f"GPS sats {gps_sats} < {min_sats}")
    return ok, failures


def preflight_check_battery(battery_pct: float, min_pct: float = 25.0):
    """Replicate battery preflight logic."""
    ok = battery_pct > min_pct
    failures = [] if ok else [f"Battery {battery_pct:.0f}% <= {min_pct:.0f}%"]
    return ok, failures


def preflight_check_health(gyro: bool, accel: bool, mag: bool,
                           local_pos: bool, global_pos: bool, home_pos: bool):
    """Replicate health flags preflight logic."""
    checks = {
        "Gyro": gyro,
        "Accel": accel,
        "Mag": mag,
        "LocalPos": local_pos,
        "GlobalPos": global_pos,
        "HomePos": home_pos,
    }
    failures = [f"{name} NOT OK" for name, val in checks.items() if not val]
    return len(failures) == 0, failures


def preflight_check_mission(mission_file: str, mission_type: str = "navigate_and_track"):
    """Replicate mission loaded check."""
    if mission_type == "track_only":
        return True, []
    has_mission = bool(mission_file)
    if not has_mission:
        return False, ["No mission file for navigation"]
    return True, []


def run_full_preflight(gps_fix: int, gps_sats: int, battery_pct: float,
                       gyro: bool, accel: bool, mag: bool,
                       local_pos: bool, global_pos: bool, home_pos: bool,
                       mission_file: str,
                       require_3d: bool = True, min_sats: int = 6,
                       min_battery_pct: float = 25.0,
                       mission_type: str = "navigate_and_track"):
    """Run all preflight checks and return (all_ok, failures)."""
    all_ok = True
    all_failures = []

    ok, failures = preflight_check_gps(gps_fix, gps_sats, require_3d, min_sats)
    if not ok:
        all_ok = False
        all_failures.extend(failures)

    ok, failures = preflight_check_battery(battery_pct, min_battery_pct)
    if not ok:
        all_ok = False
        all_failures.extend(failures)

    ok, failures = preflight_check_health(gyro, accel, mag, local_pos, global_pos, home_pos)
    if not ok:
        all_ok = False
        all_failures.extend(failures)

    ok, failures = preflight_check_mission(mission_file, mission_type)
    if not ok:
        all_ok = False
        all_failures.extend(failures)

    return all_ok, all_failures


# ---------------------------------------------------------------------------
# GPS preflight tests
# ---------------------------------------------------------------------------

class TestPreflightGPS:
    """Tests for GPS preflight check logic."""

    def test_3d_fix_and_enough_sats_ok(self):
        ok, failures = preflight_check_gps(gps_fix=3, gps_sats=10)
        assert ok is True
        assert failures == []

    def test_3d_fix_with_exactly_min_sats_ok(self):
        ok, failures = preflight_check_gps(gps_fix=3, gps_sats=6)
        assert ok is True
        assert failures == []

    def test_fix_below_3_fails(self):
        ok, failures = preflight_check_gps(gps_fix=2, gps_sats=10)
        assert ok is False
        assert len(failures) == 1
        assert "GPS fix 2 < 3" in failures[0]

    def test_fix_zero_fails(self):
        ok, failures = preflight_check_gps(gps_fix=0, gps_sats=10)
        assert ok is False
        assert "GPS fix 0 < 3" in failures[0]

    def test_sats_below_min_fails(self):
        ok, failures = preflight_check_gps(gps_fix=3, gps_sats=4)
        assert ok is False
        assert len(failures) == 1
        assert "GPS sats 4 < 6" in failures[0]

    def test_both_fix_and_sats_fail(self):
        ok, failures = preflight_check_gps(gps_fix=1, gps_sats=2)
        assert ok is False
        assert len(failures) == 2
        assert any("GPS fix" in f for f in failures)
        assert any("GPS sats" in f for f in failures)

    def test_require_3d_false_allows_fix_2(self):
        ok, failures = preflight_check_gps(gps_fix=2, gps_sats=10, require_3d=False)
        assert ok is True
        assert failures == []

    def test_require_3d_false_still_checks_sats(self):
        ok, failures = preflight_check_gps(gps_fix=2, gps_sats=3, require_3d=False)
        assert ok is False
        assert len(failures) == 1
        assert "GPS sats" in failures[0]

    def test_custom_min_sats(self):
        ok, failures = preflight_check_gps(gps_fix=3, gps_sats=8, min_sats=10)
        assert ok is False
        assert "GPS sats 8 < 10" in failures[0]

    def test_custom_min_sats_met(self):
        ok, failures = preflight_check_gps(gps_fix=3, gps_sats=10, min_sats=10)
        assert ok is True
        assert failures == []

    def test_sats_one_below_min_fails(self):
        ok, failures = preflight_check_gps(gps_fix=3, gps_sats=5)
        assert ok is False
        assert "GPS sats 5 < 6" in failures[0]


# ---------------------------------------------------------------------------
# Battery preflight tests
# ---------------------------------------------------------------------------

class TestPreflightBattery:
    """Tests for battery preflight check logic."""

    def test_high_battery_ok(self):
        ok, failures = preflight_check_battery(90.0)
        assert ok is True
        assert failures == []

    def test_just_above_min_ok(self):
        ok, failures = preflight_check_battery(26.0)
        assert ok is True
        assert failures == []

    def test_equal_to_min_fails(self):
        """Battery must be strictly greater than min_pct."""
        ok, failures = preflight_check_battery(25.0)
        assert ok is False
        assert len(failures) == 1
        assert "Battery 25% <= 25%" in failures[0]

    def test_below_min_fails(self):
        ok, failures = preflight_check_battery(10.0)
        assert ok is False
        assert "Battery 10% <= 25%" in failures[0]

    def test_zero_battery_fails(self):
        ok, failures = preflight_check_battery(0.0)
        assert ok is False
        assert "Battery 0% <= 25%" in failures[0]

    def test_full_battery_ok(self):
        ok, failures = preflight_check_battery(100.0)
        assert ok is True
        assert failures == []

    def test_custom_min_threshold(self):
        ok, failures = preflight_check_battery(30.0, min_pct=50.0)
        assert ok is False
        assert "Battery 30% <= 50%" in failures[0]

    def test_custom_min_threshold_met(self):
        ok, failures = preflight_check_battery(51.0, min_pct=50.0)
        assert ok is True
        assert failures == []

    def test_barely_above_default_min(self):
        ok, failures = preflight_check_battery(25.1)
        assert ok is True
        assert failures == []


# ---------------------------------------------------------------------------
# Health preflight tests
# ---------------------------------------------------------------------------

class TestPreflightHealth:
    """Tests for individual health flag preflight checks."""

    def test_all_healthy_ok(self):
        ok, failures = preflight_check_health(
            gyro=True, accel=True, mag=True,
            local_pos=True, global_pos=True, home_pos=True,
        )
        assert ok is True
        assert failures == []

    def test_gyro_false_fails(self):
        ok, failures = preflight_check_health(
            gyro=False, accel=True, mag=True,
            local_pos=True, global_pos=True, home_pos=True,
        )
        assert ok is False
        assert len(failures) == 1
        assert "Gyro NOT OK" in failures

    def test_accel_false_fails(self):
        ok, failures = preflight_check_health(
            gyro=True, accel=False, mag=True,
            local_pos=True, global_pos=True, home_pos=True,
        )
        assert ok is False
        assert "Accel NOT OK" in failures

    def test_mag_false_fails(self):
        ok, failures = preflight_check_health(
            gyro=True, accel=True, mag=False,
            local_pos=True, global_pos=True, home_pos=True,
        )
        assert ok is False
        assert "Mag NOT OK" in failures

    def test_local_pos_false_fails(self):
        ok, failures = preflight_check_health(
            gyro=True, accel=True, mag=True,
            local_pos=False, global_pos=True, home_pos=True,
        )
        assert ok is False
        assert "LocalPos NOT OK" in failures

    def test_global_pos_false_fails(self):
        ok, failures = preflight_check_health(
            gyro=True, accel=True, mag=True,
            local_pos=True, global_pos=False, home_pos=True,
        )
        assert ok is False
        assert "GlobalPos NOT OK" in failures

    def test_home_pos_false_fails(self):
        ok, failures = preflight_check_health(
            gyro=True, accel=True, mag=True,
            local_pos=True, global_pos=True, home_pos=False,
        )
        assert ok is False
        assert "HomePos NOT OK" in failures

    def test_all_false_gives_six_failures(self):
        ok, failures = preflight_check_health(
            gyro=False, accel=False, mag=False,
            local_pos=False, global_pos=False, home_pos=False,
        )
        assert ok is False
        assert len(failures) == 6
        expected = {"Gyro NOT OK", "Accel NOT OK", "Mag NOT OK",
                    "LocalPos NOT OK", "GlobalPos NOT OK", "HomePos NOT OK"}
        assert set(failures) == expected

    def test_multiple_failures_reports_all(self):
        ok, failures = preflight_check_health(
            gyro=False, accel=True, mag=False,
            local_pos=True, global_pos=True, home_pos=True,
        )
        assert ok is False
        assert len(failures) == 2
        assert "Gyro NOT OK" in failures
        assert "Mag NOT OK" in failures

    def test_failure_order_matches_check_order(self):
        """Failures should be reported in the order sensors are checked."""
        ok, failures = preflight_check_health(
            gyro=False, accel=False, mag=False,
            local_pos=False, global_pos=False, home_pos=False,
        )
        assert failures == [
            "Gyro NOT OK", "Accel NOT OK", "Mag NOT OK",
            "LocalPos NOT OK", "GlobalPos NOT OK", "HomePos NOT OK",
        ]


# ---------------------------------------------------------------------------
# Mission loaded preflight tests
# ---------------------------------------------------------------------------

class TestPreflightMission:
    """Tests for mission file loaded preflight check."""

    def test_mission_file_set_ok(self):
        ok, failures = preflight_check_mission("/path/to/mission.yaml")
        assert ok is True
        assert failures == []

    def test_empty_mission_file_fails(self):
        ok, failures = preflight_check_mission("")
        assert ok is False
        assert "No mission file for navigation" in failures[0]

    def test_track_only_type_with_empty_file_ok(self):
        """TRACK_ONLY missions do not require a mission file."""
        ok, failures = preflight_check_mission("", mission_type="track_only")
        assert ok is True
        assert failures == []

    def test_track_only_type_with_file_set_ok(self):
        ok, failures = preflight_check_mission("/some/file.yaml", mission_type="track_only")
        assert ok is True
        assert failures == []

    def test_navigate_and_track_without_file_fails(self):
        ok, failures = preflight_check_mission("", mission_type="navigate_and_track")
        assert ok is False
        assert len(failures) == 1

    def test_navigate_only_without_file_fails(self):
        ok, failures = preflight_check_mission("", mission_type="navigate_only")
        assert ok is False
        assert "No mission file for navigation" in failures[0]

    def test_navigate_only_with_file_ok(self):
        ok, failures = preflight_check_mission("/path/mission.yaml", mission_type="navigate_only")
        assert ok is True
        assert failures == []


# ---------------------------------------------------------------------------
# Full preflight integration tests
# ---------------------------------------------------------------------------

class TestFullPreflight:
    """Tests combining all preflight checks into one pass/fail result."""

    # Default healthy values for convenience
    HEALTHY_DEFAULTS = dict(
        gps_fix=3, gps_sats=10, battery_pct=90.0,
        gyro=True, accel=True, mag=True,
        local_pos=True, global_pos=True, home_pos=True,
        mission_file="/path/to/mission.yaml",
    )

    def _run(self, **overrides):
        params = {**self.HEALTHY_DEFAULTS, **overrides}
        return run_full_preflight(**params)

    def test_all_ok_passes(self):
        ok, failures = self._run()
        assert ok is True
        assert failures == []

    def test_gps_failure_causes_overall_fail(self):
        ok, failures = self._run(gps_fix=1)
        assert ok is False
        assert any("GPS fix" in f for f in failures)

    def test_battery_failure_causes_overall_fail(self):
        ok, failures = self._run(battery_pct=20.0)
        assert ok is False
        assert any("Battery" in f for f in failures)

    def test_health_failure_causes_overall_fail(self):
        ok, failures = self._run(mag=False)
        assert ok is False
        assert any("Mag NOT OK" in f for f in failures)

    def test_mission_failure_causes_overall_fail(self):
        ok, failures = self._run(mission_file="")
        assert ok is False
        assert any("No mission file" in f for f in failures)

    def test_multiple_subsystem_failures_collected(self):
        ok, failures = self._run(gps_fix=0, battery_pct=10.0, accel=False, mission_file="")
        assert ok is False
        # Expect failures from GPS, battery, health, and mission
        assert any("GPS fix" in f for f in failures)
        assert any("Battery" in f for f in failures)
        assert any("Accel NOT OK" in f for f in failures)
        assert any("No mission file" in f for f in failures)
        assert len(failures) >= 4

    def test_track_only_skips_mission_file_check(self):
        ok, failures = self._run(mission_file="", mission_type="track_only")
        assert ok is True
        assert failures == []

    def test_single_sensor_failure_in_otherwise_healthy(self):
        """A single unhealthy sensor should fail the entire preflight."""
        ok, failures = self._run(home_pos=False)
        assert ok is False
        assert len(failures) == 1
        assert "HomePos NOT OK" in failures[0]

    def test_gps_sats_failure_in_otherwise_healthy(self):
        ok, failures = self._run(gps_sats=3)
        assert ok is False
        assert any("GPS sats" in f for f in failures)

    def test_custom_parameters(self):
        """Full preflight with non-default thresholds."""
        ok, failures = self._run(
            gps_sats=8, min_sats=10,
            battery_pct=45.0, min_battery_pct=50.0,
        )
        assert ok is False
        assert any("GPS sats 8 < 10" in f for f in failures)
        assert any("Battery 45% <= 50%" in f for f in failures)

    def test_all_subsystems_failing(self):
        ok, failures = self._run(
            gps_fix=0, gps_sats=0,
            battery_pct=0.0,
            gyro=False, accel=False, mag=False,
            local_pos=False, global_pos=False, home_pos=False,
            mission_file="",
        )
        assert ok is False
        # GPS (2) + Battery (1) + Health (6) + Mission (1) = 10 failures
        assert len(failures) == 10
