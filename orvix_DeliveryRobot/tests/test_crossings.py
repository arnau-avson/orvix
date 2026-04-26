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
        c = out[0]
        assert c.point.lon == pytest.approx(0.0005, abs=1e-6)
        assert c.road_type == "secondary"
        assert c.crossing_bearing == pytest.approx(90.0, abs=1.0)
        assert c.step_index == 0
        # Default secondary width = 11 m. Perpendicular crossing → length = width.
        assert c.road_width_m == pytest.approx(11.0, abs=0.1)
        assert c.crossing_length_m == pytest.approx(11.0, abs=0.5)
        # Entry should be ~5.5m west of midpoint, exit ~5.5m east.
        assert c.entry_point.lon < c.point.lon
        assert c.exit_point.lon > c.point.lon

    def test_entry_and_exit_are_symmetric_around_midpoint(self):
        route = _route_east()
        graph = _road_graph_with_north_south_road(at_lon=0.0005, highway="primary")
        c = find_road_crossings(route, graph)[0]
        # Distances from midpoint to entry vs exit should be equal.
        from delivery_robot.geometry import haversine_m
        d_entry = haversine_m(c.point.lat, c.point.lon,
                              c.entry_point.lat, c.entry_point.lon)
        d_exit = haversine_m(c.point.lat, c.point.lon,
                             c.exit_point.lat, c.exit_point.lon)
        assert d_entry == pytest.approx(d_exit, abs=0.1)
        assert d_entry == pytest.approx(c.crossing_length_m / 2, abs=0.5)

    def test_road_width_from_lanes_tag(self):
        route = _route_east()
        graph = nx.MultiDiGraph()
        graph.add_node(1, x=0.0005, y=-0.001)
        graph.add_node(2, x=0.0005, y=0.001)
        # Explicit lanes=4 → 4 * 3.25 = 13m, overriding the residential default of 6m.
        graph.add_edge(1, 2, highway="residential", lanes="4")
        c = find_road_crossings(route, graph)[0]
        assert c.road_width_m == pytest.approx(13.0, abs=0.1)

    def test_road_width_from_explicit_width_tag(self):
        route = _route_east()
        graph = nx.MultiDiGraph()
        graph.add_node(1, x=0.0005, y=-0.001)
        graph.add_node(2, x=0.0005, y=0.001)
        graph.add_edge(1, 2, highway="residential", width="8.5 m")
        c = find_road_crossings(route, graph)[0]
        assert c.road_width_m == pytest.approx(8.5, abs=0.1)

    def test_crossing_length_uses_real_meters_not_degrees(self):
        # At lat=41° (Barcelona), 1° longitude is ~84 km, not 111 km.
        # A perpendicular crossing of a 10m-wide road at lat=41 should give
        # crossing_length=10m, not 10/cos(41) = ~13m.
        a = Point(lat=41.0, lon=0.0)
        b = Point(lat=41.0, lon=0.001)
        route = Route(origin=a, destination=b, steps=[
            Step(start=a, end=b, length_m=84.0, geometry=[a, b]),
        ])
        graph = nx.MultiDiGraph()
        graph.add_node(1, x=0.0005, y=40.999)
        graph.add_node(2, x=0.0005, y=41.001)
        graph.add_edge(1, 2, highway="residential", width="10")
        c = find_road_crossings(route, graph)[0]
        assert c.crossing_length_m == pytest.approx(10.0, abs=0.5)

    def test_oblique_crossing_length_correct(self):
        # Route running NE (45°) crossing a north-south road of width 10m.
        # Crossing length along route = 10 / sin(45°) ≈ 14.14 m
        a = Point(lat=0.0, lon=0.0)
        b = Point(lat=0.001, lon=0.001)  # ~111m NE-ish
        route = Route(origin=a, destination=b, steps=[
            Step(start=a, end=b, length_m=156.0, geometry=[a, b]),
        ])
        graph = nx.MultiDiGraph()
        graph.add_node(1, x=0.0005, y=-0.001)
        graph.add_node(2, x=0.0005, y=0.001)
        graph.add_edge(1, 2, highway="residential", width="10")
        c = find_road_crossings(route, graph)[0]
        assert c.crossing_length_m == pytest.approx(14.14, abs=0.5)

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
