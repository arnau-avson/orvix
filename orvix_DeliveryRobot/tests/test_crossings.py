"""Geometric crossing detection tests with hand-built graphs."""
import networkx as nx
import pytest

from delivery_robot.crossings import Crossing, find_road_crossings
from delivery_robot.models import Point, Route, Step
from delivery_robot.traffic_lights import TrafficLight


def _route_east(lat=0.0):
    """Two-point route going east at the given latitude."""
    a = Point(lat=lat, lon=0.0)
    b = Point(lat=lat, lon=0.001)
    return Route(
        origin=a, destination=b,
        steps=[Step(start=a, end=b, length_m=111.0, geometry=[a, b])],
    )


def _road_graph_with_north_south_road(at_lon: float, highway: str = "secondary"):
    """One-edge graph: a road segment running north-south at `at_lon`."""
    g = nx.MultiDiGraph()
    g.add_node(1, x=at_lon, y=-0.001)
    g.add_node(2, x=at_lon, y=0.001)
    g.add_edge(1, 2, highway=highway)
    return g


class TestFindRoadCrossings:
    def test_one_crossing_perpendicular(self):
        route = _route_east()
        graph = _road_graph_with_north_south_road(at_lon=0.0005)
        out = find_road_crossings(route, graph)
        assert len(out) == 1
        assert out[0].point.lon == pytest.approx(0.0005, abs=1e-6)
        assert out[0].road_type == "secondary"
        assert out[0].crossing_bearing == pytest.approx(90.0, abs=1.0)
        assert out[0].step_index == 0

    def test_no_crossing_when_road_outside_route(self):
        route = _route_east()
        # Road at lon=0.005 but route only goes to lon=0.001
        graph = _road_graph_with_north_south_road(at_lon=0.005)
        assert find_road_crossings(route, graph) == []

    def test_multiple_roads_yield_multiple_crossings(self):
        route = _route_east()
        graph = nx.MultiDiGraph()
        # Two parallel north-south roads
        for i, (lon, hw) in enumerate([(0.00025, "primary"), (0.00075, "tertiary")]):
            graph.add_node(2 * i + 1, x=lon, y=-0.001)
            graph.add_node(2 * i + 2, x=lon, y=0.001)
            graph.add_edge(2 * i + 1, 2 * i + 2, highway=hw)
        out = find_road_crossings(route, graph)
        assert len(out) == 2
        types = {c.road_type for c in out}
        assert types == {"primary", "tertiary"}

    def test_footway_in_graph_is_ignored(self):
        # A footway parallel to our route shouldn't count as a road crossing.
        route = _route_east()
        graph = nx.MultiDiGraph()
        graph.add_node(1, x=0.0005, y=-0.001)
        graph.add_node(2, x=0.0005, y=0.001)
        graph.add_edge(1, 2, highway="footway")
        assert find_road_crossings(route, graph) == []

    def test_signal_attached_when_within_radius(self):
        route = _route_east()
        graph = _road_graph_with_north_south_road(at_lon=0.0005)
        # Signal node ~10m north of the crossing
        sig = TrafficLight(
            node_id=99, point=Point(lat=0.0001, lon=0.0005),
            kind="pedestrian", step_index=0,
            approach_bearing=90.0, exit_bearing=90.0, crossing_bearing=90.0,
        )
        out = find_road_crossings(route, graph, signal_nodes=[sig])
        assert len(out) == 1
        assert out[0].is_signaled
        assert out[0].signal is sig

    def test_signal_too_far_is_not_attached(self):
        route = _route_east()
        graph = _road_graph_with_north_south_road(at_lon=0.0005)
        # Signal ~110m away (way more than 25m threshold)
        sig = TrafficLight(
            node_id=99, point=Point(lat=0.001, lon=0.0005),
            kind="pedestrian", step_index=0,
            approach_bearing=90.0, exit_bearing=90.0, crossing_bearing=90.0,
        )
        out = find_road_crossings(route, graph, signal_nodes=[sig])
        assert len(out) == 1
        assert not out[0].is_signaled
        assert out[0].signal is None

    def test_dedupes_overlapping_intersections(self):
        # Two parallel roads at the same lon shouldn't produce duplicate crossings
        route = _route_east()
        graph = nx.MultiDiGraph()
        for i in range(2):
            graph.add_node(2 * i + 1, x=0.0005, y=-0.001)
            graph.add_node(2 * i + 2, x=0.0005, y=0.001)
            graph.add_edge(2 * i + 1, 2 * i + 2, highway="residential")
        out = find_road_crossings(route, graph)
        assert len(out) == 1  # deduped
