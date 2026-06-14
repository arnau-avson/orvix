"""Localization tests — RouteSimulator and RouteTracker, no network needed."""
import pytest

from delivery_robot.localization import RouteSimulator, RouteTracker
from delivery_robot.localization.models import Pose
from delivery_robot.models import Point, Route, Step
from delivery_robot.traffic_lights import TrafficLight


def _make_route():
    """Two-segment east-going route, 50m + 50m = 100m total."""
    a = Point(lat=0.0, lon=0.0)
    b = Point(lat=0.0, lon=0.000449)   # ≈50m east
    c = Point(lat=0.0, lon=0.000898)   # ≈100m east
    steps = [
        Step(start=a, end=b, length_m=50.0, geometry=[a, b]),
        Step(start=b, end=c, length_m=50.0, geometry=[b, c]),
    ]
    return Route(origin=a, destination=c, steps=steps)


class TestRouteSimulator:
    def test_first_pose_at_origin(self):
        route = _make_route()
        sim = RouteSimulator(route, speed_mps=1.0, timestep_s=1.0)
        p = sim.get_pose()
        assert p.point.lat == pytest.approx(0.0, abs=1e-6)
        assert p.point.lon == pytest.approx(0.0, abs=1e-6)

    def test_advances_with_each_call(self):
        route = _make_route()
        sim = RouteSimulator(route, speed_mps=1.0, timestep_s=10.0)
        p1 = sim.get_pose()
        p2 = sim.get_pose()
        # 10s at 1m/s = 10m east
        assert p2.point.lon > p1.point.lon

    def test_finishes_at_destination(self):
        route = _make_route()
        sim = RouteSimulator(route, speed_mps=10.0, timestep_s=20.0)
        # Walk past total distance (100m) — first call is at 0, second at
        # 200m which exceeds total → finishes.
        sim.get_pose()
        last = sim.get_pose()
        assert last is not None
        # And the next call returns None (done)
        assert sim.get_pose() is None

    def test_heading_is_eastbound(self):
        route = _make_route()
        sim = RouteSimulator(route, speed_mps=1.0, timestep_s=1.0)
        p = sim.get_pose()
        # East = 90° in compass
        assert p.heading_deg == pytest.approx(90.0, abs=1.0)

    def test_rejects_short_polyline(self):
        single = Point(lat=0, lon=0)
        # No steps → polyline empty → ValueError
        with pytest.raises(ValueError):
            RouteSimulator(Route(origin=single, destination=single, steps=[]))


class TestRouteTracker:
    def test_progress_zero_at_origin(self):
        route = _make_route()
        tracker = RouteTracker(route)
        pose = Pose(point=Point(0.0, 0.0), heading_deg=90.0,
                    speed_mps=0, accuracy_m=2.0, timestamp_s=0.0)
        s = tracker.update(pose)
        assert s.progress_m == pytest.approx(0.0, abs=0.5)
        assert s.remaining_m == pytest.approx(100.0, abs=1.0)
        assert not s.is_off_route

    def test_progress_at_midpoint(self):
        route = _make_route()
        tracker = RouteTracker(route)
        pose = Pose(point=Point(0.0, 0.000449), heading_deg=90.0,
                    speed_mps=0, accuracy_m=2.0, timestamp_s=0.0)
        s = tracker.update(pose)
        assert s.progress_m == pytest.approx(50.0, abs=2.0)

    def test_off_route_detection(self):
        route = _make_route()
        tracker = RouteTracker(route, off_route_threshold_m=10.0)
        # 100m north of route → way off
        pose = Pose(point=Point(0.0009, 0.000449), heading_deg=90.0,
                    speed_mps=0, accuracy_m=2.0, timestamp_s=0.0)
        s = tracker.update(pose)
        assert s.is_off_route
        assert s.off_route_distance_m > 10.0

    def test_attached_lights_within_radius(self):
        route = _make_route()
        tracker = RouteTracker(route, approach_radius_m=10.0)
        light = TrafficLight(
            node_id=1,
            point=Point(0.0, 0.000449),
            kind="pedestrian",
            step_index=1,
            approach_bearing=90.0,
            exit_bearing=90.0,
            crossing_bearing=90.0,
        )
        tracker.attach_lights([light])

        # Pose at the light → should be in approach radius
        s = tracker.update(Pose(
            point=Point(0.0, 0.000449), heading_deg=90.0,
            speed_mps=0, accuracy_m=2.0, timestamp_s=0.0,
        ))
        assert len(s.approaching_lights) == 1

        # Pose far away → not in radius
        s = tracker.update(Pose(
            point=Point(0.0, 0.0), heading_deg=90.0,
            speed_mps=0, accuracy_m=2.0, timestamp_s=0.0,
        ))
        assert len(s.approaching_lights) == 0
