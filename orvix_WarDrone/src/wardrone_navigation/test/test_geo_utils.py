"""Tests for geo_utils module."""

import math
import pytest

from wardrone_navigation.geo_utils import (
    haversine_distance,
    compute_bearing,
    normalize_angle,
    offset_position,
    compute_reroute_waypoint,
    is_point_near_path,
    estimate_obstacle_position,
)


class TestHaversineDistance:
    def test_same_point_is_zero(self):
        d = haversine_distance(47.3977, 8.5456, 47.3977, 8.5456)
        assert d == 0.0

    def test_one_degree_latitude(self):
        """One degree of latitude is approximately 111 km."""
        d = haversine_distance(0.0, 0.0, 1.0, 0.0)
        assert abs(d - 111_195) < 500  # within 500m of expected

    def test_symmetry(self):
        d1 = haversine_distance(47.3977, 8.5456, 48.0, 9.0)
        d2 = haversine_distance(48.0, 9.0, 47.3977, 8.5456)
        assert abs(d1 - d2) < 1e-6

    def test_short_distance(self):
        """Two points ~100m apart should give a reasonable result."""
        lat1, lon1 = 47.3977, 8.5456
        lat2, lon2 = 47.3986, 8.5456  # roughly 100m north
        d = haversine_distance(lat1, lon1, lat2, lon2)
        assert 90 < d < 110


class TestComputeBearing:
    def test_north(self):
        bearing = compute_bearing(47.0, 8.0, 48.0, 8.0)
        assert abs(bearing - 0.0) < 1.0

    def test_east(self):
        bearing = compute_bearing(0.0, 8.0, 0.0, 9.0)
        assert abs(bearing - 90.0) < 1.0

    def test_south(self):
        bearing = compute_bearing(48.0, 8.0, 47.0, 8.0)
        assert abs(bearing - 180.0) < 1.0 or abs(bearing + 180.0) < 1.0

    def test_west(self):
        bearing = compute_bearing(0.0, 9.0, 0.0, 8.0)
        assert abs(bearing - (-90.0)) < 1.0 or abs(bearing - 270.0) < 1.0


class TestNormalizeAngle:
    def test_zero(self):
        assert normalize_angle(0.0) == 0.0

    def test_270_to_neg90(self):
        assert abs(normalize_angle(270.0) - (-90.0)) < 1e-9

    def test_neg270_to_90(self):
        assert abs(normalize_angle(-270.0) - 90.0) < 1e-9

    def test_360_to_zero(self):
        assert abs(normalize_angle(360.0)) < 1e-9

    def test_neg180(self):
        assert abs(normalize_angle(-180.0) - (-180.0)) < 1e-9

    def test_180(self):
        assert abs(normalize_angle(180.0) - 180.0) < 1e-9

    def test_large_positive(self):
        assert abs(normalize_angle(720.0)) < 1e-9

    def test_large_negative(self):
        assert abs(normalize_angle(-720.0)) < 1e-9


class TestOffsetPosition:
    def test_north_increases_lat(self):
        lat, lon = offset_position(47.0, 8.0, 0.0, 100.0)
        assert lat > 47.0
        assert abs(lon - 8.0) < 1e-6

    def test_east_increases_lon(self):
        lat, lon = offset_position(47.0, 8.0, 90.0, 100.0)
        assert lon > 8.0
        assert abs(lat - 47.0) < 1e-4  # small numerical drift acceptable

    def test_south_decreases_lat(self):
        lat, lon = offset_position(47.0, 8.0, 180.0, 100.0)
        assert lat < 47.0

    def test_round_trip(self):
        """Offset north then south should return near the original position."""
        lat1, lon1 = offset_position(47.0, 8.0, 0.0, 100.0)
        lat2, lon2 = offset_position(lat1, lon1, 180.0, 100.0)
        assert abs(lat2 - 47.0) < 1e-5
        assert abs(lon2 - 8.0) < 1e-5

    def test_zero_distance(self):
        lat, lon = offset_position(47.0, 8.0, 45.0, 0.0)
        assert abs(lat - 47.0) < 1e-9
        assert abs(lon - 8.0) < 1e-9


class TestComputeRerouteWaypoint:
    def test_obstacle_blocking_returns_waypoint(self):
        """Obstacle straight ahead (same bearing as path) should return a waypoint."""
        current_lat, current_lon = 47.3977, 8.5456
        target_lat, target_lon = 47.3987, 8.5456  # due north
        obstacle_bearing = 0.0  # obstacle also due north
        result = compute_reroute_waypoint(
            current_lat, current_lon, target_lat, target_lon, obstacle_bearing
        )
        assert result is not None
        assert len(result) == 2

    def test_obstacle_not_blocking_returns_none(self):
        """Obstacle >60 degrees off path should return None."""
        current_lat, current_lon = 47.3977, 8.5456
        target_lat, target_lon = 47.3987, 8.5456  # due north
        obstacle_bearing = 90.0  # obstacle due east, path is north
        result = compute_reroute_waypoint(
            current_lat, current_lon, target_lat, target_lon, obstacle_bearing
        )
        assert result is None

    def test_obstacle_at_59_degrees_returns_waypoint(self):
        """Obstacle at 59 degrees off path (within 60) should still trigger reroute."""
        current_lat, current_lon = 47.3977, 8.5456
        target_lat, target_lon = 47.3987, 8.5456  # due north
        obstacle_bearing = 59.0  # within 60 degrees of north
        result = compute_reroute_waypoint(
            current_lat, current_lon, target_lat, target_lon, obstacle_bearing
        )
        assert result is not None

    def test_obstacle_at_61_degrees_returns_none(self):
        """Obstacle at 61 degrees off path should not trigger reroute."""
        current_lat, current_lon = 47.3977, 8.5456
        target_lat, target_lon = 47.3987, 8.5456  # due north
        obstacle_bearing = 61.0
        result = compute_reroute_waypoint(
            current_lat, current_lon, target_lat, target_lon, obstacle_bearing
        )
        assert result is None


class TestIsPointNearPath:
    def test_point_on_path(self):
        """A point on the path should be near it."""
        start_lat, start_lon = 47.3977, 8.5456
        end_lat, end_lon = 47.3987, 8.5456
        # Midpoint is on the path
        mid_lat = (start_lat + end_lat) / 2
        mid_lon = (start_lon + end_lon) / 2
        assert is_point_near_path(start_lat, start_lon, end_lat, end_lon,
                                  mid_lat, mid_lon, 5.0)

    def test_point_far_away(self):
        """A point far from the path should not be near it."""
        start_lat, start_lon = 47.3977, 8.5456
        end_lat, end_lon = 47.3987, 8.5456
        far_lat, far_lon = 47.3977, 8.56  # ~400m east
        assert not is_point_near_path(start_lat, start_lon, end_lat, end_lon,
                                      far_lat, far_lon, 10.0)

    def test_degenerate_start_equals_end(self):
        """When start and end are the same, distance to that single point is checked."""
        lat, lon = 47.3977, 8.5456
        # Point very close to start/end
        assert is_point_near_path(lat, lon, lat, lon, lat + 0.000001, lon, 5.0)
        # Point far away
        assert not is_point_near_path(lat, lon, lat, lon, lat + 0.01, lon, 5.0)

    def test_endpoint_proximity(self):
        """A point near the start should be counted as near the path."""
        start_lat, start_lon = 47.3977, 8.5456
        end_lat, end_lon = 47.3987, 8.5456
        # 1m from start point
        near_lat, near_lon = offset_position(start_lat, start_lon, 90.0, 1.0)
        assert is_point_near_path(start_lat, start_lon, end_lat, end_lon,
                                  near_lat, near_lon, 5.0)


class TestEstimateObstaclePosition:
    def test_zero_bearing_zero_yaw_goes_north(self):
        """Bearing 0, yaw 0 -> obstacle is due north."""
        lat, lon = estimate_obstacle_position(47.0, 8.0, 0.0, 0.0, 100.0)
        assert lat > 47.0
        assert abs(lon - 8.0) < 1e-5

    def test_90_bearing_zero_yaw_goes_east(self):
        """Bearing 90, yaw 0 -> obstacle is due east."""
        lat, lon = estimate_obstacle_position(47.0, 8.0, 0.0, 90.0, 100.0)
        assert lon > 8.0

    def test_yaw_rotates_bearing(self):
        """Yaw adds to bearing: bearing 0 + yaw 90 = obstacle east."""
        lat, lon = estimate_obstacle_position(47.0, 8.0, 90.0, 0.0, 100.0)
        assert lon > 8.0

    def test_distance_correct(self):
        """The obstacle should be approximately obstacle_distance_m away."""
        origin_lat, origin_lon = 47.0, 8.0
        obs_lat, obs_lon = estimate_obstacle_position(origin_lat, origin_lon, 0.0, 0.0, 50.0)
        d = haversine_distance(origin_lat, origin_lon, obs_lat, obs_lon)
        assert abs(d - 50.0) < 1.0
