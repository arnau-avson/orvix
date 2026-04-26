"""Detect where the pedestrian route crosses vehicle roads — and exactly
where each crossing starts and ends on each curb.

Method: for each route segment, project it and the candidate road centerline
to a local equirectangular meter frame, then take the geometric intersection
of the route with the road's buffer (radius = road_width / 2). Each
overlapping LineString piece is one crossing:

    overlap = route_segment ∩ buffer(road_centerline, width/2)

The first and last points of `overlap` are exactly where the robot leaves
its current curb and arrives at the far one. The midpoint and length come
out for free, with no trigonometric assumption about straight roads or
perpendicular crossings.

Width is sourced (in priority order) from OSM `width`, `lanes` × 3.25 m,
or per-highway-type defaults. Each crossing optionally carries the nearest
OSM `traffic_signals` node within 25 m (`signal`).
"""
import math
import re
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

_DEFAULT_WIDTH_M = {
    "motorway": 22.0, "motorway_link": 8.0,
    "trunk": 16.0, "trunk_link": 7.0,
    "primary": 14.0, "primary_link": 7.0,
    "secondary": 11.0, "secondary_link": 6.0,
    "tertiary": 9.0, "tertiary_link": 5.0,
    "unclassified": 7.0,
    "residential": 6.0,
    "service": 4.0,
}
_LANE_WIDTH_M = 3.25

_SIGNAL_NEAREST_M = 25.0
_DEDUPE_DECIMALS = 6
_MIN_OVERLAP_M = 0.5


@dataclass
class Crossing:
    point: Point                  # Midpoint of entry-exit segment.
    entry_point: Point            # Curb the robot leaves (start of crossing).
    exit_point: Point             # Curb the robot arrives at (end of crossing).
    road_type: str
    road_width_m: float
    crossing_bearing: float       # Bearing from entry to exit.
    crossing_length_m: float      # Distance entry -> exit.
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


# --- Local equirectangular projection ---------------------------------------
# Converts (lon, lat) to (x, y) in meters relative to a reference point.
# Accurate to <0.1 % over distances under 10 km — vastly better than naive
# degree buffering at non-equatorial latitudes.

def _project_to_m(lonlat: tuple, ref_lat: float, ref_lon: float) -> tuple:
    cos_lat = math.cos(math.radians(ref_lat))
    return (
        (lonlat[0] - ref_lon) * 111_000.0 * cos_lat,
        (lonlat[1] - ref_lat) * 111_000.0,
    )


def _project_to_lonlat(xy: tuple, ref_lat: float, ref_lon: float) -> tuple:
    cos_lat = math.cos(math.radians(ref_lat))
    return (
        ref_lon + xy[0] / (111_000.0 * cos_lat),
        ref_lat + xy[1] / 111_000.0,
    )


def _project_geom_to_m(geom: sg.LineString, ref_lat: float, ref_lon: float) -> sg.LineString:
    return sg.LineString([_project_to_m(c, ref_lat, ref_lon) for c in geom.coords])


def _extract_lines(geom) -> List[sg.LineString]:
    if geom.is_empty:
        return []
    if geom.geom_type == "LineString":
        return [geom]
    if geom.geom_type == "MultiLineString":
        return list(geom.geoms)
    return []


def find_road_crossings(
    route: Route,
    road_graph: nx.MultiDiGraph,
    signal_nodes: Optional[List[TrafficLight]] = None,
) -> List[Crossing]:
    """Return every crossing the route makes through a vehicle road, with
    entry/exit coordinates derived geometrically from the road buffer.
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
        seg_lonlat = sg.LineString([(a.lon, a.lat), (b.lon, b.lat)])

        # Project around the segment midpoint so meter scales are correct
        # locally for both N-S and E-W.
        ref_lat = (a.lat + b.lat) / 2
        ref_lon = (a.lon + b.lon) / 2
        seg_m = _project_geom_to_m(seg_lonlat, ref_lat, ref_lon)

        for road_line, road_type, road_width in road_lines:
            if not seg_lonlat.intersects(road_line):
                continue

            road_m = _project_geom_to_m(road_line, ref_lat, ref_lon)
            # Flat caps so the buffer doesn't extend a half-disc past either
            # end of the road segment we happen to have geometry for.
            buf_m = road_m.buffer(road_width / 2, cap_style=2)
            overlap_m = seg_m.intersection(buf_m)

            for piece in _extract_lines(overlap_m):
                length_m = piece.length
                if length_m < _MIN_OVERLAP_M:
                    continue

                coords_m = list(piece.coords)
                entry_lon, entry_lat = _project_to_lonlat(coords_m[0], ref_lat, ref_lon)
                exit_lon, exit_lat = _project_to_lonlat(coords_m[-1], ref_lat, ref_lon)
                mid_x = (coords_m[0][0] + coords_m[-1][0]) / 2
                mid_y = (coords_m[0][1] + coords_m[-1][1]) / 2
                mid_lon, mid_lat = _project_to_lonlat((mid_x, mid_y), ref_lat, ref_lon)

                key = (round(mid_lat, _DEDUPE_DECIMALS),
                       round(mid_lon, _DEDUPE_DECIMALS))
                if key in seen:
                    continue
                seen.add(key)

                crossings.append(Crossing(
                    point=Point(lat=mid_lat, lon=mid_lon),
                    entry_point=Point(lat=entry_lat, lon=entry_lon),
                    exit_point=Point(lat=exit_lat, lon=exit_lon),
                    road_type=road_type,
                    road_width_m=road_width,
                    crossing_bearing=bearing_deg(entry_lat, entry_lon,
                                                 exit_lat, exit_lon),
                    crossing_length_m=length_m,
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
