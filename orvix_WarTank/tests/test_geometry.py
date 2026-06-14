"""Pure-function tests for geometry helpers."""
import math

import pytest

from delivery_robot.geometry import (
    bearing_deg,
    haversine_m,
    project_onto_segment,
)


class TestBearing:
    def test_due_north(self):
        assert bearing_deg(0.0, 0.0, 1.0, 0.0) == pytest.approx(0.0, abs=0.5)

    def test_due_east(self):
        assert bearing_deg(0.0, 0.0, 0.0, 1.0) == pytest.approx(90.0, abs=0.5)

    def test_due_south(self):
        assert bearing_deg(0.0, 0.0, -1.0, 0.0) == pytest.approx(180.0, abs=0.5)

    def test_due_west(self):
        assert bearing_deg(0.0, 0.0, 0.0, -1.0) == pytest.approx(270.0, abs=0.5)

    def test_returns_in_zero_to_360(self):
        for lat, lon in [(1.0, 1.0), (-1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)]:
            b = bearing_deg(0, 0, lat, lon)
            assert 0 <= b < 360


class TestHaversine:
    def test_zero_distance(self):
        assert haversine_m(41.0, 2.0, 41.0, 2.0) == pytest.approx(0.0, abs=0.01)

    def test_barcelona_to_madrid(self):
        # Reference: ~505 km
        d = haversine_m(41.3879, 2.1700, 40.4168, -3.7038)
        assert 500_000 < d < 510_000

    def test_one_degree_lat_at_equator(self):
        # 1 degree of latitude is ~111 km
        d = haversine_m(0.0, 0.0, 1.0, 0.0)
        assert 110_000 < d < 112_000


class TestProjectOntoSegment:
    def test_point_at_segment_start(self):
        lat, lon, t, dist = project_onto_segment(0.0, 0.0, 0.0, 0.0, 0.001, 0.0)
        assert t == pytest.approx(0.0, abs=1e-6)
        assert dist == pytest.approx(0.0, abs=0.01)

    def test_point_at_segment_end(self):
        lat, lon, t, dist = project_onto_segment(0.001, 0.0, 0.0, 0.0, 0.001, 0.0)
        assert t == pytest.approx(1.0, abs=1e-6)
        assert dist == pytest.approx(0.0, abs=0.01)

    def test_point_at_midpoint(self):
        lat, lon, t, dist = project_onto_segment(0.0005, 0.0, 0.0, 0.0, 0.001, 0.0)
        assert t == pytest.approx(0.5, abs=1e-3)
        assert dist == pytest.approx(0.0, abs=0.01)

    def test_perpendicular_distance(self):
        # Point lies at lat=0.0, lon=0.0005 (perpendicular to a north-going segment)
        # The east-west offset of 0.0005 deg ≈ 55 m at the equator.
        lat, lon, t, dist = project_onto_segment(0.0, 0.0005, 0.0, 0.0, 0.001, 0.0)
        assert t == pytest.approx(0.0, abs=1e-3)
        assert 50 < dist < 60

    def test_clamps_before_segment(self):
        # Point west of the start of an east-going segment.
        _, _, t, _ = project_onto_segment(0.0, -0.001, 0.0, 0.0, 0.0, 0.001)
        assert t == 0.0

    def test_clamps_after_segment(self):
        # Point east of the end.
        _, _, t, _ = project_onto_segment(0.0, 0.002, 0.0, 0.0, 0.0, 0.001)
        assert t == 1.0

    def test_degenerate_segment(self):
        # Zero-length segment: returns start point.
        lat, lon, t, dist = project_onto_segment(0.0001, 0.0, 0.0, 0.0, 0.0, 0.0)
        assert t == 0.0
        assert dist > 0
