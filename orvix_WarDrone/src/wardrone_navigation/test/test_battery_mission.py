"""Tests for mission battery feasibility estimation."""

import pytest

from wardrone_navigation.mission_loader import (
    estimate_mission_feasibility,
    MissionData,
    WaypointData,
)
from wardrone_navigation.geo_utils import haversine_distance


# Reference point: Zurich area
REF_LAT = 47.3977
REF_LON = 8.5456


def _make_mission(waypoints, mission_id="test"):
    return MissionData(mission_id=mission_id, waypoints=waypoints)


class TestBatteryFeasibility:
    def test_feasible_mission(self):
        """Short mission with 90% battery should be feasible."""
        # Two waypoints ~100m apart
        wp1 = WaypointData(REF_LAT + 0.0005, REF_LON, 10.0)
        wp2 = WaypointData(REF_LAT + 0.001, REF_LON, 10.0)
        mission = _make_mission([wp1, wp2])

        feasible, detail, total_dist, est_time, batt_needed = (
            estimate_mission_feasibility(mission, REF_LAT, REF_LON, 90.0)
        )
        assert feasible is True
        assert batt_needed < 70.0  # should need much less than 70%

    def test_infeasible_mission(self):
        """Long mission with 30% battery should be infeasible."""
        # Waypoints far apart (~10km)
        wp1 = WaypointData(REF_LAT + 0.05, REF_LON, 10.0)
        wp2 = WaypointData(REF_LAT + 0.1, REF_LON, 10.0)
        mission = _make_mission([wp1, wp2])

        feasible, detail, total_dist, est_time, batt_needed = (
            estimate_mission_feasibility(mission, REF_LAT, REF_LON, 30.0)
        )
        assert feasible is False

    def test_empty_waypoints(self):
        """Mission with no waypoints should be infeasible."""
        mission = _make_mission([])

        feasible, detail, total_dist, est_time, batt_needed = (
            estimate_mission_feasibility(mission, REF_LAT, REF_LON, 90.0)
        )
        assert feasible is False
        assert "No waypoints" in detail

    def test_loiter_adds_time(self):
        """Waypoints with loiter should require more time and battery."""
        wp_no_loiter = WaypointData(REF_LAT + 0.0005, REF_LON, 10.0, loiter_time_s=0.0)
        wp_loiter = WaypointData(REF_LAT + 0.0005, REF_LON, 10.0, loiter_time_s=60.0)

        mission_no_loiter = _make_mission([wp_no_loiter])
        mission_loiter = _make_mission([wp_loiter])

        _, _, _, time_no_loiter, batt_no_loiter = (
            estimate_mission_feasibility(mission_no_loiter, REF_LAT, REF_LON, 90.0)
        )
        _, _, _, time_loiter, batt_loiter = (
            estimate_mission_feasibility(mission_loiter, REF_LAT, REF_LON, 90.0)
        )

        assert time_loiter > time_no_loiter
        assert batt_loiter > batt_no_loiter
        # The difference should be approximately 60 seconds of loiter
        assert abs((time_loiter - time_no_loiter) - 60.0) < 1.0

    def test_rtl_distance_included(self):
        """Total distance should include return-to-launch segment."""
        wp1 = WaypointData(REF_LAT + 0.001, REF_LON, 10.0)
        wp2 = WaypointData(REF_LAT + 0.002, REF_LON, 10.0)
        mission = _make_mission([wp1, wp2])

        _, _, total_dist, _, _ = (
            estimate_mission_feasibility(mission, REF_LAT, REF_LON, 90.0)
        )

        # Sum of just waypoint-to-waypoint distances (no RTL)
        d_start_wp1 = haversine_distance(REF_LAT, REF_LON,
                                         wp1.latitude_deg, wp1.longitude_deg)
        d_wp1_wp2 = haversine_distance(wp1.latitude_deg, wp1.longitude_deg,
                                       wp2.latitude_deg, wp2.longitude_deg)
        waypoint_only_dist = d_start_wp1 + d_wp1_wp2

        # Total should be greater because it includes RTL from last waypoint
        assert total_dist > waypoint_only_dist

        # RTL distance from last waypoint back to start
        rtl_dist = haversine_distance(wp2.latitude_deg, wp2.longitude_deg,
                                      REF_LAT, REF_LON)
        assert abs(total_dist - (waypoint_only_dist + rtl_dist)) < 1.0

    def test_battery_reserve_affects_feasibility(self):
        """Higher reserve requirement should reduce available battery."""
        wp = WaypointData(REF_LAT + 0.001, REF_LON, 10.0)
        mission = _make_mission([wp])

        feasible_low_reserve, _, _, _, _ = (
            estimate_mission_feasibility(mission, REF_LAT, REF_LON, 25.0,
                                         battery_reserve_pct=10.0)
        )
        feasible_high_reserve, _, _, _, _ = (
            estimate_mission_feasibility(mission, REF_LAT, REF_LON, 25.0,
                                         battery_reserve_pct=24.0)
        )

        # With 25% battery and 10% reserve, 15% available -> likely feasible
        assert feasible_low_reserve is True
        # With 25% battery and 24% reserve, only 1% available -> likely infeasible
        assert feasible_high_reserve is False
