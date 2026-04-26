"""Detect where the pedestrian route crosses vehicle roads.

Geometric approach: intersect the route polyline with the vehicle road
network. Independent of OSM `highway=crossing` tagging (which is wildly
inconsistent in coverage), so we don't miss crossings just because a
mapper forgot to add the tag.

Each detected `Crossing` carries:
- `point` — lat/lon of the intersection.
- `road_type` — OSM highway tag of the road being crossed
  (residential, secondary, primary, ...).
- `crossing_bearing` — direction of travel at the crossing point
  (perpendicular to the road by definition of a crossing).
- `step_index` — index of the route step containing this crossing.
- `signal` — nearest `TrafficLight` from OSM (None if unsignaled).

Combined with the visual semáforo detector, this gives a complete picture:
geometry says where the cruce is, vision says go/wait.
"""
from dataclasses import dataclass
from typing import List, Optional, Set, Tuple

import networkx as nx

try:
    import shapely.geometry as sg
except ImportError as e:  # pragma: no cover
    raise ImportError("shapely is required (installed as an osmnx dependency)") from e

from .geometry import bearing_deg, haversine_m
from .models import Point, Route
from .traffic_lights import TrafficLight


# Highway tags considered "vehicle road" for crossing detection. These are
# the surfaces a pedestrian must yield to / cross with caution.
_ROAD_TAGS = {
    "motorway", "motorway_link",
    "trunk", "trunk_link",
    "primary", "primary_link",
    "secondary", "secondary_link",
    "tertiary", "tertiary_link",
    "unclassified",
    "residential",
    "service",
}

# Maximum distance (meters) between a crossing and an OSM signal node for the
# signal to be considered governing this crossing.
_SIGNAL_NEAREST_M = 25.0

# Rounding precision for deduplicating intersection points (≈11 cm at the equator).
_DEDUPE_DECIMALS = 6


@dataclass
class Crossing:
    point: Point
    road_type: str
    crossing_bearing: float
    step_index: int
    signal: Optional[TrafficLight] = None

    @property
    def is_signaled(self) -> bool:
        return self.signal is not None


def _highway_tag(data: dict) -> Optional[str]:
    h = data.get("highway")
    if isinstance(h, list):
        return h[0] if h else None
    return h


def _edge_geometry_coords(graph: nx.MultiDiGraph, u: int, v: int, data: dict):
    geom = data.get("geometry")
    if geom is not None:
        return list(geom.coords)
    return [
        (graph.nodes[u]["x"], graph.nodes[u]["y"]),
        (graph.nodes[v]["x"], graph.nodes[v]["y"]),
    ]


def _extract_points(geom) -> List:
    """Pull `shapely.Point` objects out of any intersection result type."""
    if geom.is_empty:
        return []
    gt = geom.geom_type
    if gt == "Point":
        return [geom]
    if gt == "MultiPoint":
        return list(geom.geoms)
    # When route segment overlaps the road exactly (very rare), take the
    # endpoints of the overlap as the crossing locations.
    from shapely.geometry import Point as SPoint
    if gt == "LineString":
        coords = list(geom.coords)
        return [SPoint(coords[0]), SPoint(coords[-1])] if coords else []
    if gt == "MultiLineString":
        out = []
        for g in geom.geoms:
            coords = list(g.coords)
            if coords:
                out.extend([SPoint(coords[0]), SPoint(coords[-1])])
        return out
    return []


def find_road_crossings(
    route: Route,
    road_graph: nx.MultiDiGraph,
    signal_nodes: Optional[List[TrafficLight]] = None,
) -> List[Crossing]:
    """Return the list of points where the route polyline crosses a road.

    Parameters
    ----------
    route : Route
        The pedestrian route to analyze.
    road_graph : nx.MultiDiGraph
        Graph containing the vehicle road geometry. Best results when this is
        a `network_type='drive'` graph (only roads, no footways), so the
        route's pedestrian segments only intersect roads at actual crossings.
    signal_nodes : list of TrafficLight, optional
        OSM signal nodes (from `find_traffic_lights`). Each crossing gets the
        nearest signal within 25 m attached, if any.
    """
    road_lines: List[Tuple[sg.LineString, str]] = []
    for u, v, data in road_graph.edges(data=True):
        ht = _highway_tag(data)
        if ht not in _ROAD_TAGS:
            continue
        coords = _edge_geometry_coords(road_graph, u, v, data)
        if len(coords) < 2:
            continue
        road_lines.append((sg.LineString(coords), ht))

    crossings: List[Crossing] = []
    seen: Set[Tuple[float, float]] = set()
    polyline = route.full_polyline

    for i in range(len(polyline) - 1):
        a = polyline[i]
        b = polyline[i + 1]
        seg = sg.LineString([(a.lon, a.lat), (b.lon, b.lat)])
        seg_bearing = bearing_deg(a.lat, a.lon, b.lat, b.lon)

        for road_line, road_type in road_lines:
            if not seg.intersects(road_line):
                continue
            for p in _extract_points(seg.intersection(road_line)):
                key = (round(p.y, _DEDUPE_DECIMALS), round(p.x, _DEDUPE_DECIMALS))
                if key in seen:
                    continue
                seen.add(key)
                crossings.append(Crossing(
                    point=Point(lat=p.y, lon=p.x),
                    road_type=road_type,
                    crossing_bearing=seg_bearing,
                    step_index=i,
                ))

    if signal_nodes:
        for c in crossings:
            best, best_d = None, float("inf")
            for sig in signal_nodes:
                d = haversine_m(c.point.lat, c.point.lon,
                                sig.point.lat, sig.point.lon)
                if d < best_d and d <= _SIGNAL_NEAREST_M:
                    best, best_d = sig, d
            c.signal = best

    return crossings
