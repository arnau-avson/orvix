"""Tests for waypoint_navigator_node module."""

import pytest
from wardrone_navigation.waypoint_navigator_node import haversine_distance


class TestHaversineDistance:
    def test_same_point(self):
        d = haversine_distance(47.3977, 8.5456, 47.3977, 8.5456)
        assert d == pytest.approx(0.0, abs=0.01)

    def test_known_distance(self):
        # Zurich to Bern ~95 km
        d = haversine_distance(47.3769, 8.5417, 46.9480, 7.4474)
        assert 90000 < d < 110000

    def test_short_distance(self):
        # ~111m per 0.001 deg latitude at equator
        d = haversine_distance(0.0, 0.0, 0.001, 0.0)
        assert 100 < d < 120

    def test_symmetry(self):
        d1 = haversine_distance(47.0, 8.0, 48.0, 9.0)
        d2 = haversine_distance(48.0, 9.0, 47.0, 8.0)
        assert d1 == pytest.approx(d2, rel=1e-6)


class TestWaypointNavigatorImport:
    def test_import(self):
        from wardrone_navigation.waypoint_navigator_node import WaypointNavigatorNode
        assert WaypointNavigatorNode is not None
