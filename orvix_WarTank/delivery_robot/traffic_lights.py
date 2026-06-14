"""Traffic light awareness for the routing layer.

This module identifies traffic-signal nodes along a computed route and
records the direction in which the robot will cross them. The actual
green/red detection lives in the perception layer (camera + CV); here we
only define the interface (`TrafficLightSensor`) and the logic that decides
*which* signal applies given the robot's heading.

OSM tagging reference used:
- `highway=traffic_signals` — generic signal node on a way
- `crossing=traffic_signals` — pedestrian crossing with signals
- `highway=crossing` + `crossing=traffic_signals` — pedestrian-controlled crossing
- `traffic_signals:direction=forward|backward` — which way the signal faces
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from math import atan2, cos, degrees, radians, sin
from typing import Iterable, List, Optional

import networkx as nx

from .models import Point, Route


_SIGNAL_HIGHWAY_TAGS = {"traffic_signals"}
_SIGNAL_CROSSING_TAGS = {"traffic_signals", "signals"}


def _node_has_signal(graph: nx.MultiDiGraph, node_id: int) -> Optional[str]:
    """Return the signal kind at this node ('vehicular', 'pedestrian') or None."""
    data = graph.nodes[node_id]

    highway = data.get("highway")
    if isinstance(highway, list):
        highway = next((h for h in highway if h in _SIGNAL_HIGHWAY_TAGS), None)
    if highway in _SIGNAL_HIGHWAY_TAGS:
        crossing = data.get("crossing")
        if crossing in _SIGNAL_CROSSING_TAGS:
            return "pedestrian"
        return "vehicular"

    crossing = data.get("crossing")
    if isinstance(crossing, list):
        crossing = next((c for c in crossing if c in _SIGNAL_CROSSING_TAGS), None)
    if crossing in _SIGNAL_CROSSING_TAGS:
        return "pedestrian"

    return None


def _bearing(a: Point, b: Point) -> float:
    """Compass bearing from `a` to `b` in degrees (0=N, 90=E)."""
    lat1, lat2 = radians(a.lat), radians(b.lat)
    dlon = radians(b.lon - a.lon)
    x = sin(dlon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    return (degrees(atan2(x, y)) + 360.0) % 360.0


@dataclass
class TrafficLight:
    """A signal node sitting along the route."""
    node_id: int
    point: Point
    kind: str                    # 'pedestrian' or 'vehicular'
    step_index: int              # index of the step that *exits* this node
    approach_bearing: float      # direction we arrive from (degrees)
    exit_bearing: float          # direction we head toward after the signal
    crossing_bearing: float      # bearing of the crossing — this is what the
                                 # perception layer uses to pick which light to read

    @property
    def must_yield(self) -> bool:
        """A pedestrian-controlled signal always governs the robot.

        Vehicular-only signals don't apply to a robot on the sidewalk unless
        the route geometry forces it across the roadway — which, on a `walk`
        graph, only happens at crossings. So if we land on a vehicular signal
        node within a walk graph, treat it as advisory (false)."""
        return self.kind == "pedestrian"


class TrafficLightSensor(ABC):
    """Interface for the perception layer.

    The router calls `is_green(light)` when the robot is queued at a signal.
    Implementations decide which physical light to read from `light.crossing_bearing`.
    """

    @abstractmethod
    def is_green(self, light: TrafficLight) -> bool: ...


class AlwaysGoSensor(TrafficLightSensor):
    """Stub used to exercise routing logic without a camera in the loop."""

    def is_green(self, light: TrafficLight) -> bool:  # noqa: ARG002
        return True


def find_traffic_lights(route: Route, graph: nx.MultiDiGraph) -> List[TrafficLight]:
    """Scan the route for signal nodes and annotate each with crossing direction.

    Direction-aware logic: the `crossing_bearing` is the bearing of the edge
    *exiting* the signal node — i.e., the direction the robot will travel
    across the crossing. The perception layer can use this to align the
    camera or pick the correct physical traffic light face to read.
    """
    if not route.steps:
        return []

    src_node = _nearest_node(graph, route.origin)
    nodes_on_path = [src_node]
    for step in route.steps:
        nodes_on_path.append(_nearest_node(graph, step.end))

    lights: List[TrafficLight] = []
    for i, node_id in enumerate(nodes_on_path):
        kind = _node_has_signal(graph, node_id)
        if kind is None:
            continue
        if i == 0 or i == len(nodes_on_path) - 1:
            # Origin and destination signals don't gate crossing decisions.
            continue

        prev_step = route.steps[i - 1]
        next_step = route.steps[i] if i < len(route.steps) else None
        if next_step is None:
            continue

        approach = _bearing(prev_step.start, prev_step.end)
        exit_b = _bearing(next_step.start, next_step.end)

        lights.append(
            TrafficLight(
                node_id=node_id,
                point=Point(lat=graph.nodes[node_id]["y"], lon=graph.nodes[node_id]["x"]),
                kind=kind,
                step_index=i,
                approach_bearing=approach,
                exit_bearing=exit_b,
                crossing_bearing=exit_b,
            )
        )

    return lights


def _nearest_node(graph: nx.MultiDiGraph, point: Point) -> int:
    import osmnx as ox
    return ox.distance.nearest_nodes(graph, X=point.lon, Y=point.lat)


def should_proceed(light: TrafficLight, sensor: TrafficLightSensor) -> bool:
    """Return True if the robot may cross this signal right now."""
    if not light.must_yield:
        return True
    return sensor.is_green(light)


def plan_with_signals(
    route: Route,
    graph: nx.MultiDiGraph,
) -> "AnnotatedRoute":
    """Attach traffic-light annotations to a route."""
    return AnnotatedRoute(route=route, lights=find_traffic_lights(route, graph))


@dataclass
class AnnotatedRoute:
    route: Route
    lights: List[TrafficLight]

    def lights_at_step(self, step_index: int) -> Iterable[TrafficLight]:
        return (l for l in self.lights if l.step_index == step_index)
