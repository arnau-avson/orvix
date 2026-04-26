"""Detect where the pedestrian route crosses vehicle roads — and how wide
each crossing is.

Geometric approach:
1. Intersect the route polyline with the vehicle road network. Each
   intersection is a single midpoint of a crossing.
2. For each midpoint, look up the road's local bearing (from its OSM
   geometry) and width (from its `width`/`lanes` tag, with sensible
   per-highway-type defaults if missing).
3. Compute the crossing length along the route as
       crossing_length = road_width / sin(angle_between_route_and_road)
   so an oblique crossing is correctly longer than a perpendicular one.
4. Offset along the route bearing by ±crossing_length/2 to get
   `entry_point` (curb the robot leaves) and `exit_point` (curb it
   arrives at).
5. Optionally enrich each crossing with the nearest OSM signal node.

Independent of OSM `highway=crossing` tagging (which is wildly inconsistent),
so we don't miss crossings just because mappers forgot to tag them.
"""
import re
from dataclasses import dataclass
from typing import List, Optional, Set, Tuple

import networkx as nx

try:
    import shapely.geometry as sg
except ImportError as e:  # pragma: no cover
    raise ImportError("shapely is required (installed as an osmnx dependency)") from e

from .geometry import bearing_deg, destination_point, haversine_m
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

# Default road widths in meters, by OSM `highway` tag. Used when the edge
# has no explicit `width` or `lanes` tag. Conservative real-world averages
# for Spanish urban streets.
_DEFAULT_WIDTH_M = {
    "motorway": 22.0,
    "motorway_link": 8.0,
    "trunk": 16.0,
    "trunk_link": 7.0,
    "primary": 14.0,
    "primary_link": 7.0,
    "secondary": 11.0,
    "secondary_link": 6.0,
    "tertiary": 9.0,
    "tertiary_link": 5.0,
    "unclassified": 7.0,
    "residential": 6.0,
    "service": 4.0,
}
_LANE_WIDTH_M = 3.25  # Standard urban lane width.

# Maximum distance (meters) between a crossing and an OSM signal node for the
# signal to be considered governing this crossing.
_SIGNAL_NEAREST_M = 25.0

_DEDUPE_DECIMALS = 6


@dataclass
class Crossing:
    point: Point                  # Midpoint of the crossing.
    entry_point: Point            # Where the robot LEAVES its current curb.
    exit_point: Point             # Where the robot ARRIVES at the far curb.
    road_type: str
    road_width_m: float
    crossing_bearing: float       # Direction along the route at the crossing.
    crossing_length_m: float      # Distance from entry to exit (along route).
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


def _parse_first_number(value) -> Optional[float]:
    """Parse '5', '5 m', '5.5', '5;6', '5-6' → first numeric value, or None."""
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, list):
        for v in value:
            n = _parse_first_number(v)
            if n is not None:
                return n
        return None
    m = re.search(r"\d+(?:\.\d+)?", str(value))
    return float(m.group()) if m else None


def _road_width_m(data: dict, highway: str) -> float:
    """Best-effort road width from OSM tags, falling back to type defaults."""
    explicit = _parse_first_number(data.get("width"))
    if explicit is not None and explicit > 0:
        return explicit
    lanes = _parse_first_number(data.get("lanes"))
    if lanes is not None and lanes > 0:
        return lanes * _LANE_WIDTH_M
    return _DEFAULT_WIDTH_M.get(highway, 7.0)


def _edge_geometry_coords(graph: nx.MultiDiGraph, u: int, v: int, data: dict):
    geom = data.get("geometry")
    if geom is not None:
        return list(geom.coords)
    return [
        (graph.nodes[u]["x"], graph.nodes[u]["y"]),
        (graph.nodes[v]["x"], graph.nodes[v]["y"]),
    ]


def _bearing_of_segment_nearest(road_line: sg.LineString, p: sg.Point) -> float:
    """Local compass bearing of the road LineString at the segment closest to p."""
    coords = list(road_line.coords)
    if len(coords) < 2:
        return 0.0
    best_i, best_d = 0, float("inf")
    for i in range(len(coords) - 1):
        seg = sg.LineString([coords[i], coords[i + 1]])
        d = seg.distance(p)
        if d < best_d:
            best_d = d
            best_i = i
    a = coords[best_i]
    b = coords[best_i + 1]
    return bearing_deg(a[1], a[0], b[1], b[0])


def _angle_between_bearings_deg(a: float, b: float) -> float:
    """Smallest angle between two compass bearings, in degrees [0, 90]."""
    diff = abs(a - b) % 180.0
    return min(diff, 180.0 - diff)


def _extract_points(geom) -> List:
    if geom.is_empty:
        return []
    gt = geom.geom_type
    if gt == "Point":
        return [geom]
    if gt == "MultiPoint":
        return list(geom.geoms)
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
    """Return the list of road crossings the route traverses, with entry,
    midpoint, and exit coordinates computed.
    """
    road_lines: List[Tuple[sg.LineString, str, float]] = []
    for u, v, data in road_graph.edges(data=True):
        ht = _highway_tag(data)
        if ht not in _ROAD_TAGS:
            continue
        coords = _edge_geometry_coords(road_graph, u, v, data)
        if len(coords) < 2:
            continue
        road_lines.append((sg.LineString(coords), ht, _road_width_m(data, ht)))

    crossings: List[Crossing] = []
    seen: Set[Tuple[float, float]] = set()
    polyline = route.full_polyline

    for i in range(len(polyline) - 1):
        a = polyline[i]
        b = polyline[i + 1]
        seg = sg.LineString([(a.lon, a.lat), (b.lon, b.lat)])
        route_bearing = bearing_deg(a.lat, a.lon, b.lat, b.lon)

        for road_line, road_type, road_width in road_lines:
            if not seg.intersects(road_line):
                continue
            for p in _extract_points(seg.intersection(road_line)):
                key = (round(p.y, _DEDUPE_DECIMALS), round(p.x, _DEDUPE_DECIMALS))
                if key in seen:
                    continue
                seen.add(key)

                road_bearing = _bearing_of_segment_nearest(road_line, p)
                angle = _angle_between_bearings_deg(route_bearing, road_bearing)
                # Avoid div-by-zero when the route runs parallel to the road
                # (shouldn't happen in pedestrian-strict mode, but guard anyway).
                import math
                sin_angle = math.sin(math.radians(max(angle, 5.0)))
                crossing_length = road_width / sin_angle
                half = crossing_length / 2.0

                mid = Point(lat=p.y, lon=p.x)
                entry_lat, entry_lon = destination_point(
                    mid.lat, mid.lon, (route_bearing + 180) % 360, half
                )
                exit_lat, exit_lon = destination_point(
                    mid.lat, mid.lon, route_bearing, half
                )

                crossings.append(Crossing(
                    point=mid,
                    entry_point=Point(lat=entry_lat, lon=entry_lon),
                    exit_point=Point(lat=exit_lat, lon=exit_lon),
                    road_type=road_type,
                    road_width_m=road_width,
                    crossing_bearing=route_bearing,
                    crossing_length_m=crossing_length,
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
