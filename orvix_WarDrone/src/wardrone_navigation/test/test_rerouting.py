"""Tests for rerouting logic from geo_utils."""

import math
import pytest

from wardrone_navigation.geo_utils import (
    compute_reroute_waypoint,
    compute_bearing,
    haversine_distance,
    offset_position,
    normalize_angle,
)


class TestRerouting:
    def test_obstacle_blocking_path(self):
        """Obstacle straight ahead on the path should trigger a reroute waypoint."""
        current_lat, current_lon = 47.3977, 8.5456
        target_lat, target_lon = 47.3987, 8.5456  # due north

        # Obstacle bearing = path bearing (due north = ~0 degrees)
        result = compute_reroute_waypoint(
            current_lat, current_lon, target_lat, target_lon,
            obstacle_bearing_deg=0.0, offset_distance_m=20.0,
        )
        assert result is not None

    def test_obstacle_not_blocking(self):
        """Obstacle more than 60 degrees off path should return None."""
        current_lat, current_lon = 47.3977, 8.5456
        target_lat, target_lon = 47.3987, 8.5456  # due north

        # Obstacle is due east (90 degrees off from north path)
        result = compute_reroute_waypoint(
            current_lat, current_lon, target_lat, target_lon,
            obstacle_bearing_deg=90.0, offset_distance_m=20.0,
        )
        assert result is None

    def test_offset_distance(self):
        """The reroute waypoint should be approximately offset_distance_m from current pos."""
        current_lat, current_lon = 47.3977, 8.5456
        target_lat, target_lon = 47.3987, 8.5456  # due north
        offset_dist = 25.0

        result = compute_reroute_waypoint(
            current_lat, current_lon, target_lat, target_lon,
            obstacle_bearing_deg=0.0, offset_distance_m=offset_dist,
        )
        assert result is not None
        reroute_lat, reroute_lon = result
        actual_dist = haversine_distance(current_lat, current_lon,
                                         reroute_lat, reroute_lon)
        assert abs(actual_dist - offset_dist) < 2.0

    def test_reroute_goes_sideways(self):
        """The reroute waypoint should be roughly perpendicular to the path direction."""
        current_lat, current_lon = 47.3977, 8.5456
        target_lat, target_lon = 47.3987, 8.5456  # due north

        result = compute_reroute_waypoint(
            current_lat, current_lon, target_lat, target_lon,
            obstacle_bearing_deg=0.0, offset_distance_m=30.0,
        )
        assert result is not None
        reroute_lat, reroute_lon = result

        # Bearing from current to reroute should be roughly perpendicular to path
        path_bearing = compute_bearing(current_lat, current_lon,
                                       target_lat, target_lon)
        reroute_bearing = compute_bearing(current_lat, current_lon,
                                          reroute_lat, reroute_lon)

        angle_diff = abs(normalize_angle(reroute_bearing - path_bearing))
        # Should be close to 90 degrees (perpendicular)
        assert abs(angle_diff - 90.0) < 5.0

    def test_obstacle_right_of_path_reroutes_left(self):
        """When obstacle is to the right of path, reroute should go left."""
        current_lat, current_lon = 47.3977, 8.5456
        target_lat, target_lon = 47.3987, 8.5456  # due north
        # Obstacle slightly right of path (positive angle_diff)
        obstacle_bearing = 30.0

        result = compute_reroute_waypoint(
            current_lat, current_lon, target_lat, target_lon,
            obstacle_bearing_deg=obstacle_bearing, offset_distance_m=20.0,
        )
        assert result is not None
        reroute_lat, reroute_lon = result

        # Reroute should go left (negative bearing relative to path, i.e. west)
        reroute_bearing = compute_bearing(current_lat, current_lon,
                                          reroute_lat, reroute_lon)
        # Path bearing is ~0 (north), reroute should be ~-90 (west)
        angle_diff = normalize_angle(reroute_bearing - 0.0)
        assert angle_diff < 0  # left of path (negative = west)

    def test_obstacle_left_of_path_reroutes_right(self):
        """When obstacle is to the left of path, reroute should go right."""
        current_lat, current_lon = 47.3977, 8.5456
        target_lat, target_lon = 47.3987, 8.5456  # due north
        # Obstacle slightly left of path (negative angle_diff)
        obstacle_bearing = -30.0

        result = compute_reroute_waypoint(
            current_lat, current_lon, target_lat, target_lon,
            obstacle_bearing_deg=obstacle_bearing, offset_distance_m=20.0,
        )
        assert result is not None
        reroute_lat, reroute_lon = result

        # Reroute should go right (positive bearing relative to path, i.e. east)
        reroute_bearing = compute_bearing(current_lat, current_lon,
                                          reroute_lat, reroute_lon)
        angle_diff = normalize_angle(reroute_bearing - 0.0)
        assert angle_diff > 0  # right of path (positive = east)
