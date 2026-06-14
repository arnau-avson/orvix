"""Validate that the planned route doesn't pass through any building.

Even with a strict pedestrian OSM filter, a route can still nick a building
because of:
- Mistaken or missing tags on a footway (`indoor=yes` not set, but the way
  is in fact inside an arcade).
- Polyline simplification cutting a corner.
- Two distant nodes connected by a straight edge that visually clips a
  building footprint.

This module downloads building footprints from OSM and checks every route
segment against them. It's run after routing as a sanity check; segments
that are flagged should either be filtered out at the routing stage or
manually inspected.
"""
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

try:
    import shapely.geometry as sg
    from shapely import wkb
except ImportError as e:  # pragma: no cover
    raise ImportError("shapely is required (installed as an osmnx dependency)") from e

from .models import Point, Route


_CACHE_DIR = Path(__file__).resolve().parent.parent / ".graph_cache"
_CACHE_DIR.mkdir(exist_ok=True)

# How much (in meters) we shrink the building polygons before checking
# intersections. OSM building outlines often touch the adjacent sidewalk
# geometry; an unshrunk overlap would yield false positives every time the
# route walks alongside a wall. ~0.7 m inset gives a real-world margin.
_BUILDING_INSET_M = 0.7
# Shapely buffer is in input units (degrees here). 1 m ≈ 9e-6 degrees of
# latitude; convert via local cos for longitude. We use the latitude scale
# only — slight conservative bias is fine.
_M_TO_DEG_LAT = 1 / 111_000


@dataclass
class BuildingIntersection:
    segment_index: int
    entry_point: Point
    exit_point: Point
    overlap_length_m: float


# How far outside a building wall to push a snapped destination, to ensure
# the subsequent `nearest_nodes` lookup picks a node outside the building.
_OUTWARD_OFFSET_M = 3.0


def snap_outside_buildings(point: Point, buildings) -> Point:
    """If `point` is inside a building polygon, return the nearest point on
    that building's boundary, pushed `_OUTWARD_OFFSET_M` meters outward.
    Otherwise return the point unchanged.

    Use case: a geocoded destination inside a mall (e.g. L'illa Diagonal)
    routes the robot into the building. Snap it to the entrance area before
    routing, so the delivery ends at the door.
    """
    if buildings is None or buildings.is_empty:
        return point
    p = sg.Point(point.lon, point.lat)
    if not buildings.contains(p):
        return point

    boundary = buildings.boundary
    nearest = boundary.interpolate(boundary.project(p))
    # Vector from inside point -> wall, extended outward.
    dx = nearest.x - p.x
    dy = nearest.y - p.y
    norm = (dx * dx + dy * dy) ** 0.5
    if norm < 1e-12:
        return Point(lat=nearest.y, lon=nearest.x)
    offset_deg = _OUTWARD_OFFSET_M * _M_TO_DEG_LAT
    out_lon = nearest.x + (dx / norm) * offset_deg
    out_lat = nearest.y + (dy / norm) * offset_deg
    return Point(lat=out_lat, lon=out_lon)


def load_buildings(
    center: Point,
    radius_m: float = 1500.0,
    use_cache: bool = True,
):
    """Return a Shapely geometry that is the union of all building polygons
    within `radius_m` of `center`. None if no buildings.
    """
    cache = _CACHE_DIR / (
        f"buildings_{center.lat:.5f}_{center.lon:.5f}_r{int(radius_m)}.wkb"
    )
    if use_cache and cache.exists():
        return wkb.loads(cache.read_bytes())

    import osmnx as ox  # lazy import — module is heavy

    try:
        gdf = ox.features.features_from_point(
            center.as_tuple(),
            tags={"building": True},
            dist=radius_m,
        )
    except Exception:
        # No features in area, or upstream service hiccup
        return None

    if gdf is None or gdf.empty:
        return None

    union = gdf.geometry.unary_union
    # Many OSM polygons are not topologically clean; buffer(0) heals them.
    union = union.buffer(0)
    if use_cache:
        cache.write_bytes(wkb.dumps(union))
    return union


def find_route_inside_buildings(
    route: Route,
    buildings,
    inset_m: float = _BUILDING_INSET_M,
) -> List[BuildingIntersection]:
    """Return every route segment that physically crosses a building footprint.

    The buildings are inset by `inset_m` first to avoid false positives where
    an OSM-tagged sidewalk touches a wall. A flagged segment is one that
    actually penetrates the building interior beyond that margin.
    """
    if buildings is None or buildings.is_empty:
        return []

    inset_deg = -inset_m * _M_TO_DEG_LAT
    inset = buildings.buffer(inset_deg)
    if inset.is_empty:
        return []

    polyline = route.full_polyline
    issues: List[BuildingIntersection] = []
    for i in range(len(polyline) - 1):
        a = polyline[i]
        b = polyline[i + 1]
        seg = sg.LineString([(a.lon, a.lat), (b.lon, b.lat)])
        if not seg.intersects(inset):
            continue

        inter = seg.intersection(inset)
        # Convert overlap length from degrees to approximate meters.
        # `inter.length` is a sum of LineString lengths in degrees.
        if inter.geom_type in ("LineString", "MultiLineString"):
            overlap_deg = inter.length
        elif hasattr(inter, "length"):
            overlap_deg = inter.length
        else:
            overlap_deg = 0
        overlap_m = overlap_deg / _M_TO_DEG_LAT

        bounds = inter.bounds  # (minx, miny, maxx, maxy)
        issues.append(BuildingIntersection(
            segment_index=i,
            entry_point=Point(lat=bounds[1], lon=bounds[0]),
            exit_point=Point(lat=bounds[3], lon=bounds[2]),
            overlap_length_m=overlap_m,
        ))
    return issues
