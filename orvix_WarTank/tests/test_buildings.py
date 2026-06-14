"""Building-intersection validator tests with hand-built polygons."""
import pytest
import shapely.geometry as sg

from delivery_robot.buildings import (
    BuildingIntersection,
    find_route_inside_buildings,
    snap_outside_buildings,
)
from delivery_robot.models import Point, Route, Step


def _route_east(lat=0.0):
    a = Point(lat=lat, lon=0.0)
    b = Point(lat=lat, lon=0.001)
    return Route(
        origin=a, destination=b,
        steps=[Step(start=a, end=b, length_m=111.0, geometry=[a, b])],
    )


class TestFindRouteInsideBuildings:
    def test_no_buildings_returns_empty(self):
        assert find_route_inside_buildings(_route_east(), None) == []

    def test_route_outside_building_no_hit(self):
        building = sg.box(0.0002, 0.001, 0.0008, 0.002)  # well north of route
        assert find_route_inside_buildings(_route_east(), building) == []

    def test_route_through_building_flagged(self):
        # Building straddles the route latitude
        building = sg.box(0.0003, -0.0001, 0.0007, 0.0001)
        out = find_route_inside_buildings(_route_east(), building)
        assert len(out) == 1
        assert out[0].overlap_length_m > 30  # ~44m overlap (0.0007-0.0003 lon)

    def test_overlap_below_inset_not_flagged(self):
        # A building wall touching the route line but only 0.5 m wide should
        # be inset away. (Width in degrees: ~5e-6 lat ≈ 0.5m)
        building = sg.box(0.0005, -0.0000045, 0.0005005, 0.0000045)
        out = find_route_inside_buildings(_route_east(), building)
        assert out == []

    def test_records_segment_index(self):
        # Multi-segment route: only segment 1 crosses the building.
        a = Point(0.0, 0.0)
        b = Point(0.0, 0.0005)
        c = Point(0.0, 0.001)
        route = Route(origin=a, destination=c, steps=[
            Step(start=a, end=b, length_m=55.0, geometry=[a, b]),
            Step(start=b, end=c, length_m=55.0, geometry=[b, c]),
        ])
        building = sg.box(0.0006, -0.0001, 0.0009, 0.0001)
        out = find_route_inside_buildings(route, building)
        assert len(out) == 1
        assert out[0].segment_index == 1


class TestSnapOutsideBuildings:
    def test_point_outside_unchanged(self):
        building = sg.box(0.0, 0.0, 0.001, 0.001)
        p = Point(lat=0.002, lon=0.002)
        assert snap_outside_buildings(p, building) == p

    def test_no_buildings_unchanged(self):
        p = Point(lat=0.5, lon=0.5)
        assert snap_outside_buildings(p, None) == p

    def test_inside_point_snaps_to_boundary(self):
        # Building covers a 0.001x0.001 deg square (~110m on a side).
        building = sg.box(0.0, 0.0, 0.001, 0.001)
        # Point at center
        p = Point(lat=0.0005, lon=0.0005)
        snapped = snap_outside_buildings(p, building)
        # Should be outside the building
        assert not building.contains(sg.Point(snapped.lon, snapped.lat))
        # Should be roughly within ~60m of the original
        from delivery_robot.geometry import haversine_m
        assert haversine_m(p.lat, p.lon, snapped.lat, snapped.lon) < 80
