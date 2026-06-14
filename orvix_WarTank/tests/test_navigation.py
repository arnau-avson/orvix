"""NavigationOrchestrator tests with mocked perception + localization."""
from typing import List, Optional, Tuple

import numpy as np
import pytest

from delivery_robot.localization import RouteTracker
from delivery_robot.localization.models import Pose
from delivery_robot.models import Point, Route, Step
from delivery_robot.navigation import (
    NavigationAction,
    NavigationOrchestrator,
    NavigationState,
)
from delivery_robot.perception.obstacles import Obstacle
from delivery_robot.traffic_lights import TrafficLight


def _route():
    a = Point(lat=0.0, lon=0.0)
    b = Point(lat=0.0, lon=0.000449)
    c = Point(lat=0.0, lon=0.000898)
    return Route(
        origin=a,
        destination=c,
        steps=[
            Step(start=a, end=b, length_m=50.0, geometry=[a, b]),
            Step(start=b, end=c, length_m=50.0, geometry=[b, c]),
        ],
    )


def _pose(lon: float, t: float = 0.0):
    return Pose(point=Point(lat=0.0, lon=lon), heading_deg=90.0,
                speed_mps=1.4, accuracy_m=2.0, timestamp_s=t)


class _MockLight:
    def __init__(self, state="unknown"):
        self.state = state
        self.observed = 0

    def observe(self, frame):
        self.observed += 1
        return self.state

    def fused_state(self):
        return self.state

    def is_green(self, light):
        return self.state == "green"

    def reset(self):
        pass


class _MockGate:
    def __init__(self, blocker: Optional[Obstacle] = None):
        self.blocker = blocker
        self.latest_obstacles: List = []

    def observe(self, frame):
        return ([self.blocker] if self.blocker else []), self.blocker

    def current_blocker(self):
        return self.blocker

    def reset(self):
        pass


@pytest.fixture
def make_orch():
    """Factory: returns (orchestrator, light_mock, gate_mock)."""
    def _build(light_state="unknown", blocker=None, with_signal=False):
        route = _route()
        tracker = RouteTracker(route, approach_radius_m=10.0)
        if with_signal:
            tracker.attach_lights([TrafficLight(
                node_id=1, point=Point(0.0, 0.000449), kind="pedestrian",
                step_index=1, approach_bearing=90.0, exit_bearing=90.0,
                crossing_bearing=90.0,
            )])
        light = _MockLight(light_state)
        gate = _MockGate(blocker)
        orch = NavigationOrchestrator(
            route=route, tracker=tracker, light_sensor=light,
            obstacle_gate=gate, arrived_radius_m=2.0,
        )
        return orch, light, gate
    return _build


class TestOrchestrator:
    def test_walking_at_start(self, make_orch):
        orch, *_ = make_orch()
        d = orch.tick(_pose(0.0), np.zeros((10, 10, 3), dtype=np.uint8))
        assert d.state == NavigationState.WALKING
        assert d.action == NavigationAction.GO

    def test_arrived_at_end(self, make_orch):
        orch, *_ = make_orch()
        # Pose right at destination
        d = orch.tick(_pose(0.000898), np.zeros((10, 10, 3), dtype=np.uint8))
        assert d.state == NavigationState.ARRIVED
        assert d.action == NavigationAction.STOP
        assert d.is_terminal

    def test_obstacle_stops(self, make_orch):
        blocker = Obstacle(class_name="person", cls_id=0,
                           x1=0, y1=0, x2=10, y2=20, confidence=0.9)
        orch, *_ = make_orch(blocker=blocker)
        d = orch.tick(_pose(0.0), np.zeros((10, 10, 3), dtype=np.uint8))
        assert d.state == NavigationState.STOPPED_FOR_OBSTACLE
        assert d.action == NavigationAction.STOP
        assert d.blocker is blocker

    def test_red_light_waits(self, make_orch):
        orch, *_ = make_orch(light_state="red", with_signal=True)
        # Pose near the signal
        d = orch.tick(_pose(0.000449), np.zeros((10, 10, 3), dtype=np.uint8))
        assert d.state == NavigationState.WAITING_AT_CROSSING
        assert d.action == NavigationAction.WAIT
        assert d.light_state == "red"

    def test_green_light_crosses(self, make_orch):
        orch, *_ = make_orch(light_state="green", with_signal=True)
        d = orch.tick(_pose(0.000449), np.zeros((10, 10, 3), dtype=np.uint8))
        assert d.state == NavigationState.CROSSING
        assert d.action == NavigationAction.GO
        assert d.light_state == "green"

    def test_unknown_light_holds(self, make_orch):
        orch, *_ = make_orch(light_state="unknown", with_signal=True)
        d = orch.tick(_pose(0.000449), np.zeros((10, 10, 3), dtype=np.uint8))
        assert d.state == NavigationState.APPROACHING_CROSSING
        assert d.action == NavigationAction.WAIT

    def test_obstacle_takes_priority_over_green_light(self, make_orch):
        blocker = Obstacle(class_name="person", cls_id=0,
                           x1=0, y1=0, x2=10, y2=20, confidence=0.9)
        orch, *_ = make_orch(light_state="green", blocker=blocker, with_signal=True)
        d = orch.tick(_pose(0.000449), np.zeros((10, 10, 3), dtype=np.uint8))
        assert d.state == NavigationState.STOPPED_FOR_OBSTACLE

    def test_cleared_light_does_not_re_trigger(self, make_orch):
        orch, light, _ = make_orch(light_state="green", with_signal=True)
        # First tick at the light: CROSSING
        d1 = orch.tick(_pose(0.000449, t=0), np.zeros((10, 10, 3), dtype=np.uint8))
        assert d1.state == NavigationState.CROSSING

        # Second tick still near the light, but the light has been cleared.
        # State should be WALKING, not CROSSING again.
        d2 = orch.tick(_pose(0.000449, t=1), np.zeros((10, 10, 3), dtype=np.uint8))
        assert d2.state == NavigationState.WALKING
